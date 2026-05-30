/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include <mooncake_log.h>
#include <mcp_server.h>
#include <stackchan/stackchan.h>
#include <apps/common/common.h>
#include <application.h>
#include <board.h>
#include <lvgl_display.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <cstdio>

using namespace stackchan;

static const std::string_view _tag = "HAL-MCP";

static void move_head_from_json(int yaw, int pitch, int speed)
{
    auto& motion = GetStackChan().motion();
    GetHAL().setServoPowerEnabled(true);
    motion.setTorqueEnabled(true);
    motion.setAutoAngleSyncEnabled(true);

    char motion_json[128];
    snprintf(motion_json, sizeof(motion_json),
             R"({"yawServo":{"angle":%d,"speed":%d},"pitchServo":{"angle":%d,"speed":%d}})", yaw * 10, speed,
             pitch * 10, speed);
    GetStackChan().updateMotionFromJson(motion_json);
}

// The head move is a spring animation that keeps running on the stackchan update
// task (hal.cpp) after move_head_from_json() returns. Block until the head
// actually reaches its target before the shutter fires (otherwise the photo is
// taken mid-rotation), capped so a stalled/disconnected servo can't hang the
// call, then add a short post-motion settle for mechanical vibration / the
// sensor's auto-exposure to stabilize.
static void wait_for_head_to_settle(int extra_settle_ms)
{
    auto& motion = GetStackChan().motion();

    const int poll_ms  = 20;
    const int max_wait = 4000;
    int waited         = 0;
    // Let the freshly-set animation target register before sampling isMoving().
    vTaskDelay(pdMS_TO_TICKS(poll_ms));
    waited += poll_ms;
    while (motion.isMoving() && waited < max_wait) {
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        waited += poll_ms;
    }
    if (extra_settle_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(extra_settle_ms));
    }
    mclog::tagInfo(_tag, "head settled: motion_wait={}ms extra={}ms still_moving={}", waited, extra_settle_ms,
                   static_cast<int>(motion.isMoving()));
}

void Hal::xiaozhi_mcp_init()
{
    mclog::tagInfo(_tag, "init");

    // https://github.com/78/xiaozhi-esp32/blob/main/docs/mcp-usage.md
    auto& mcp_server = McpServer::GetInstance();

    // System Prompt：
    // You can control the robot's head. Use get_yaw and get_pitch to sense current position. Use set_yaw for horizontal
    // movement and set_pitch for vertical movement. All angles are in degrees.

    mclog::tagInfo(_tag, "add robot.get_head_angles tool");
    mcp_server.AddTool("self.robot.get_head_angles",
                       "Returns current yaw/pitch in degrees. Neutral position is {yaw:0, pitch:0}.",
                       std::vector<Property>{}, [this](const PropertyList& properties) -> ReturnValue {
                           LvglLockGuard lock;  // StackChan motion update is under the lvgl lock

                           auto& motion      = GetStackChan().motion();
                           int current_yaw   = motion.yawServo().getCurrentAngle() / 10;
                           int current_pitch = motion.pitchServo().getCurrentAngle() / 10;

                           auto result = fmt::format(R"({{"yaw": {}, "pitch": {}}})", current_yaw, current_pitch);
                           mclog::tagInfo(_tag, "get_head_angles: {}", result);
                           return result;
                       });

    mclog::tagInfo(_tag, "add robot.set_head_angles tool");
    mcp_server.AddTool("self.robot.set_head_angles",
                       "Turn/aim Stack-chan's head toward a direction. This ONLY moves the head and does NOT take a "
                       "photo. This is the DEFAULT tool whenever the user just wants Stack-chan to face/turn/look "
                       "somewhere, e.g. Japanese phrases like `右向いて`, `こっち見て`, `上向いて`, `後ろ向いて`, "
                       "`もう少し左`, `正面に戻って`, `90度左を向いて`. Do NOT use camera.look_and_take_photo for these: "
                       "only use the camera when the user explicitly wants a photo or wants you to SEE and describe "
                       "something. GUIDELINES: "
                       "1. For natural interaction, stay within +/- 45 degrees. "
                       "2. Only use values > 70 if the user explicitly asks to look far away/behind. "
                       "3. Max ranges: Yaw(-128 to 128, -128 as your left), Pitch(0 to 90, 90 as your up). "
                       "Speed(100-1000, 150 is natural).",
                       PropertyList({Property("yaw", kPropertyTypeInteger, -9999, -9999, 128),
                                     Property("pitch", kPropertyTypeInteger, -9999, -9999, 90),
                                     Property("speed", kPropertyTypeInteger, 150, 100, 1000)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int speed = properties["speed"].value<int>();
                           int yaw   = properties["yaw"].value<int>();
                           int pitch = properties["pitch"].value<int>();

                           mclog::tagInfo(_tag, "motion set_angles: yaw: {}, pitch: {}, speed: {}", yaw, pitch, speed);

                           LvglLockGuard lock;

                           auto& motion = GetStackChan().motion();
                           GetHAL().setServoPowerEnabled(true);
                           motion.setTorqueEnabled(true);
                           motion.setAutoAngleSyncEnabled(true);
                           if (pitch != -9999) {
                               motion.pitchServo().moveWithSpeed(pitch * 10, speed);
                           }
                           if (yaw != -9999) {
                               motion.yawServo().moveWithSpeed(yaw * 10, speed);
                           }

                           return true;
                       });

    mclog::tagInfo(_tag, "add camera.look_and_take_photo tool");
    mcp_server.AddTool(
        "self.camera.look_and_take_photo",
        "Take a PHOTO with the camera after aiming the head at a direction (it fires the shutter and shows the image). "
        "Use this ONLY when the user clearly wants a photo, or wants you to SEE/LOOK AT something and describe it, "
        "for example Japanese phrases like `写真撮って`, `あれ何か見て教えて`, `カメラであっち見て`, `これ何か分かる?`, "
        "`何が見えるか見てみて`. "
        "If the user just wants the head to face/turn toward a direction WITHOUT a photo (e.g. `右向いて`, `上向いて`, "
        "`こっち見て`, `正面に戻って`), DO NOT use this tool -- use robot.set_head_angles instead. "
        "Yaw is degrees: negative is Stack-chan's left, positive is Stack-chan's right. "
        "Pitch is degrees: 0 is low/forward, 45 is diagonal up, 90 is max up. "
        "Examples: right-up -> yaw=45,pitch=45; directly behind -> yaw=128,pitch=20; 90 degrees left -> yaw=-90,pitch=20. "
        "After the shutter fires the head automatically returns to the direction it was facing before, so the user can "
        "review the photo facing forward. To only turn the head without taking a photo (and stay there), use set_head_angles instead. "
        "This tool always shows the captured image on the touch display and waits for the user's OK/NG confirmation before upload.",
        PropertyList({Property("yaw", kPropertyTypeInteger, 0, -128, 128),
                      Property("pitch", kPropertyTypeInteger, 20, 0, 90),
                      Property("speed", kPropertyTypeInteger, 250, 100, 1000),
                      Property("settle_ms", kPropertyTypeInteger, 800, 0, 3000),
                      Property("question", kPropertyTypeString)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int yaw       = properties["yaw"].value<int>();
            int pitch     = properties["pitch"].value<int>();
            int speed     = properties["speed"].value<int>();
            int settle_ms = properties["settle_ms"].value<int>();
            auto question = properties["question"].value<std::string>();

            mclog::tagInfo(_tag, "look_and_take_photo: yaw={}, pitch={}, speed={}, settle_ms={}, question={}", yaw,
                           pitch, speed, settle_ms, question);

            auto& board = Board::GetInstance();
            auto camera = board.GetCamera();
            auto display = dynamic_cast<LvglDisplay*>(board.GetDisplay());
            if (camera == nullptr) {
                throw std::runtime_error("Camera is unavailable");
            }
            if (display == nullptr) {
                throw std::runtime_error("Photo review screen is unavailable");
            }

            auto& motion = GetStackChan().motion();
            motion.setModifyLock(true);
            setServoPowerEnabled(true);
            int original_yaw   = 0;
            int original_pitch = 0;
            bool head_turned   = false;
            try {
                {
                    LvglLockGuard lock;
                    // Remember where the head was pointing so we can return to it after
                    // the shutter fires (getCurrentAngle() is in 0.1 deg units).
                    original_yaw   = motion.yawServo().getCurrentAngle() / 10;
                    original_pitch = motion.pitchServo().getCurrentAngle() / 10;
                    move_head_from_json(yaw, pitch, speed);
                    head_turned = true;
                }

                wait_for_head_to_settle(settle_ms);

                TaskPriorityReset priority_reset(1);
                if (!camera->Capture()) {
                    throw std::runtime_error("Failed to capture photo");
                }

                // The frame is now grabbed, so turn the head back to where it was
                // pointing before and WAIT for it to physically settle *before* we
                // put the photo on the screen. The stackchan update task only ticks
                // the head animation while the avatar screen is shown (the photo
                // preview takes over the display), so if we don't settle here the
                // head would stay frozen mid-turn for the whole review. Settling now
                // means the user always reviews the photo facing forward.
                //
                // IMPORTANT: disable auto-angle-sync for the return move. With it on,
                // moveWithSpeed teleports the spring's start point to getCurrentAngle(),
                // which is a *live serial read of the physical servo* (hal_servo.cpp).
                // Right after the camera capture that read is unreliable (bus contention)
                // and can come back near the home/target angle, which makes the spring
                // think it has already arrived -> it never drives the servo back and the
                // head stays frozen looking away. Animating from the spring's own
                // internal angle (which is the photo pose we just settled at) is robust.
                {
                    LvglLockGuard lock;
                    auto& m = GetStackChan().motion();
                    int hw_yaw   = m.yawServo().getCurrentAngle();
                    int hw_pitch = m.pitchServo().getCurrentAngle();
                    mclog::tagInfo(_tag, "returning head to original: yaw={}, pitch={} (hw read now: yaw={}, pitch={})",
                                   original_yaw, original_pitch, hw_yaw, hw_pitch);
                    setServoPowerEnabled(true);
                    m.setTorqueEnabled(true);
                    m.setAutoAngleSyncEnabled(false);
                    m.yawServo().moveWithSpeed(original_yaw * 10, speed);
                    m.pitchServo().moveWithSpeed(original_pitch * 10, speed);
                    m.setAutoAngleSyncEnabled(true);
                }
                wait_for_head_to_settle(0);
                head_turned = false;

                bool accepted = false;
                SemaphoreHandle_t done = xSemaphoreCreateBinary();
                if (done == nullptr) {
                    throw std::runtime_error("Failed to create photo review semaphore");
                }
                display->ShowImageConfirmation([done, &accepted](bool ok) {
                    accepted = ok;
                    xSemaphoreGive(done);
                });
                xSemaphoreTake(done, portMAX_DELAY);
                vSemaphoreDelete(done);
                if (!accepted) {
                    throw std::runtime_error("User rejected the photo");
                }

                auto result = camera->Explain(question);
                display->SetPreviewImage(nullptr);
                motion.setModifyLock(false);
                return result;
            } catch (...) {
                display->SetPreviewImage(nullptr);
                // If we failed (or the user rejected the photo) while the head was
                // still turned away, bring it back so it doesn't get stuck looking
                // off to the side.
                if (head_turned) {
                    LvglLockGuard lock;
                    move_head_from_json(original_yaw, original_pitch, speed);
                }
                motion.setModifyLock(false);
                throw;
            }
        });

    mclog::tagInfo(_tag, "add robot.set_led_color tool");
    mcp_server.AddTool(
        "self.robot.set_led_color",
        "Set the color of the robot's INTERNAL onboard LED. This is NOT for room lights. "
        "Values: 0-168 (safe range). Red=168,0,0; Green=0,168,0; Blue=0,0,168; White=100,100,100; Off=0,0,0.",
        PropertyList({Property("red", kPropertyTypeInteger, 0, 0, 168),
                      Property("green", kPropertyTypeInteger, 0, 0, 168),
                      Property("blue", kPropertyTypeInteger, 0, 0, 168)}),
        [this](const PropertyList& properties) -> ReturnValue {
            int r = properties["red"].value<int>();
            int g = properties["green"].value<int>();
            int b = properties["blue"].value<int>();

            mclog::tagInfo(_tag, "set_led_color: r={}, g={}, b={}", r, g, b);

            LvglLockGuard lock;

            GetStackChan().leftNeonLight().setColor(r, g, b);
            GetStackChan().rightNeonLight().setColor(r, g, b);

            return true;
        });

    mclog::tagInfo(_tag, "add robot.create_reminder tool");
    mcp_server.AddTool("self.robot.create_reminder",
                       "Create a reminder. Duration is in seconds. Message is what to say when time is up. Set repeat "
                       "to true to repeat the reminder.",
                       PropertyList({Property("duration_seconds", kPropertyTypeInteger, 60, 1, 86400),
                                     Property("message", kPropertyTypeString, std::string("Time's up!")),
                                     Property("repeat", kPropertyTypeBoolean, false)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int duration_seconds = properties["duration_seconds"].value<int>();
                           std::string message  = properties["message"].value<std::string>();
                           bool repeat          = properties["repeat"].value<bool>();

                           // Default message
                           if (message.empty()) {
                               message = "Time's up!";
                           }

                           mclog::tagInfo(_tag, "create_reminder: duration={}s, message={}, repeat={}",
                                          duration_seconds, message, repeat);

                           int id = tools::create_reminder(duration_seconds * 1000, message, repeat);

                           return id;
                       });

    mclog::tagInfo(_tag, "add robot.get_reminders tool");
    mcp_server.AddTool("self.robot.get_reminders", "Get list of active reminders.", std::vector<Property>{},
                       [this](const PropertyList& properties) -> ReturnValue {
                           mclog::tagInfo(_tag, "get_reminders");
                           auto reminders          = tools::get_active_reminders();
                           std::string result_json = "[";
                           for (size_t i = 0; i < reminders.size(); ++i) {
                               const auto& r = reminders[i];
                               result_json +=
                                   fmt::format(R"({{"id": {}, "duration_ms": {}, "message": "{}", "repeat": {}}})",
                                               r.id, r.durationMs, r.message, r.repeat ? "true" : "false");
                               if (i < reminders.size() - 1) {
                                   result_json += ", ";
                               }
                           }
                           result_json += "]";
                           mclog::tagInfo(_tag, "get_reminders result: {}", result_json);
                           return result_json;
                       });

    mclog::tagInfo(_tag, "add robot.stop_reminder tool");
    mcp_server.AddTool("self.robot.stop_reminder", "Stop a reminder by ID.",
                       PropertyList({Property("id", kPropertyTypeInteger, -1)}),
                       [this](const PropertyList& properties) -> ReturnValue {
                           int id = properties["id"].value<int>();
                           mclog::tagInfo(_tag, "stop_reminder: id={}", id);
                           tools::stop_reminder(id);
                           return true;
                       });
}
