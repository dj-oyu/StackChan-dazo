/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "hal.h"
#include <memory>
#include <mooncake_log.h>
#include <nvs_flash.h>
#include <cmath>
#include <esp_random.h>

static std::unique_ptr<Hal> _hal_instance;
static const std::string_view _tag = "HAL";

// h in [0,360), s/v in [0,1] -> r/g/b 0..255. Used for the beat LED (hue = tempo).
static void hsv_to_rgb(float h, float s, float v, uint8_t* r, uint8_t* g, uint8_t* b)
{
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float rr = 0, gg = 0, bb = 0;
    if (h < 60)       { rr = c; gg = x; }
    else if (h < 120) { rr = x; gg = c; }
    else if (h < 180) { gg = c; bb = x; }
    else if (h < 240) { gg = x; bb = c; }
    else if (h < 300) { rr = x; bb = c; }
    else              { rr = c; bb = x; }
    *r = (uint8_t)((rr + m) * 255.0f);
    *g = (uint8_t)((gg + m) * 255.0f);
    *b = (uint8_t)((bb + m) * 255.0f);
}

Hal& GetHAL()
{
    if (!_hal_instance) {
        mclog::tagInfo(_tag, "creating hal instance");
        _hal_instance = std::make_unique<Hal>();
    }
    return *_hal_instance.get();
}

void Hal::init()
{
    mclog::tagInfo(_tag, "init");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xiaozhi_board_init();
    xiaozhi_mcp_init();
    head_touch_init();
    io_expander_init();
    rtc_init();
    imu_init();
    servo_init();
    lvgl_init();
}

/* -------------------------------------------------------------------------- */
/*                                   System                                   */
/* -------------------------------------------------------------------------- */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <system_info.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_mac.h>

void Hal::delay(std::uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

std::uint32_t Hal::millis()
{
    return esp_timer_get_time() / 1000;
}

void Hal::feedTheDog()
{
    vTaskDelay(1);
}

std::array<uint8_t, 6> Hal::getFactoryMac()
{
    std::array<uint8_t, 6> mac;
    esp_efuse_mac_get_default(mac.data());
    return mac;
}

std::string Hal::getFactoryMacString(std::string divider)
{
    auto mac = getFactoryMac();
    return fmt::format("{:02X}{}{:02X}{}{:02X}{}{:02X}{}{:02X}{}{:02X}", mac[0], divider, mac[1], divider, mac[2],
                       divider, mac[3], divider, mac[4], divider, mac[5]);
}

void Hal::reboot()
{
    esp_restart();
}

static void _confirm_ota_image_if_stable()
{
    constexpr uint32_t ota_confirm_delay_ms = 20000;
    static bool ota_confirm_checked         = false;
    if (ota_confirm_checked || GetHAL().millis() < ota_confirm_delay_ms) {
        return;
    }
    ota_confirm_checked = true;

    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running == nullptr) {
        mclog::tagError(_tag, "failed to get running partition for ota confirmation");
        return;
    }

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) != ESP_OK) {
        mclog::tagError(_tag, "failed to get ota state for partition: {}", running->label);
        return;
    }

    mclog::tagInfo(_tag, "ota confirm check: partition={}, state={}", running->label, static_cast<int>(ota_state));
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        mclog::tagInfo(_tag, "ota image is stable, marking current app valid");
        esp_ota_mark_app_valid_cancel_rollback();
    }
}

void Hal::updateHeapStatusLog()
{
    _confirm_ota_image_if_stable();

    static uint32_t last_log_tick = 0;
    if (millis() - last_log_tick < 10000) {
        return;
    }
    last_log_tick = millis();
    SystemInfo::PrintHeapStats();
}

/* -------------------------------------------------------------------------- */
/*                                   Xiaozhi                                  */
/* -------------------------------------------------------------------------- */
#include "board/hal_bridge.h"
#include <agent_runtime/gpt_agent_runtime.h>
#include <agent_runtime/grok_agent_runtime.h>
#include <agent_runtime/xiaozhi_agent_runtime.h>
#include <stackchan/stackchan.h>
#include <apps/common/common.h>
#include <assets/assets.h>

void Hal::xiaozhi_board_init()
{
    mclog::tagInfo(_tag, "xiaozhi board init");

    hal_bridge::xiaozhi_board_init();
}

class BeatReactiveAnimator {
public:
    void update(const hal_bridge::BeatState& beat)
    {
        update_neon(beat);
        update_avatar(beat);
    }

private:
    void update_neon(const hal_bridge::BeatState& beat)
    {
        if (beat.locked && beat.period_us > 0) {
            int64_t k = (esp_timer_get_time() - beat.anchor_us) / beat.period_us;
            if (k != beat_k_) {
                beat_k_ = k;
                beat_intensity_ = 1.0f;

                float bpm = 60000000.0f / (float)beat.period_us;
                float t = (bpm - 60.0f) / 120.0f;
                if (t < 0) t = 0;
                if (t > 1) t = 1;
                float hue = (1.0f - t) * 240.0f;

                int lvl = (int)(beat.confidence * 4.0f);
                if (lvl < 0) lvl = 0;
                if (lvl > 3) lvl = 3;
                float peak = (60.0f + lvl * (40.0f / 3.0f)) / 255.0f;
                hsv_to_rgb(hue, 1.0f, peak, &beat_r_, &beat_g_, &beat_b_);
            }
        } else {
            beat_k_ = -1;
        }

        if (beat_intensity_ <= 0.02f) {
            return;
        }
        uint8_t r = (uint8_t)(beat_r_ * beat_intensity_);
        uint8_t g = (uint8_t)(beat_g_ * beat_intensity_);
        uint8_t b = (uint8_t)(beat_b_ * beat_intensity_);
        for (int i = 0; i < 12; ++i) {
            GetHAL().setRgbColor(i, r, g, b);
        }
        GetHAL().refreshRgb();
        beat_intensity_ *= 0.78f;
    }

    void update_avatar(const hal_bridge::BeatState& beat)
    {
        using Emotion = stackchan::avatar::Emotion;
        if (beat.locked && beat.period_us > 0 && GetStackChan().hasAvatar()) {
            auto& avatar = GetStackChan().avatar();
            int64_t k = (esp_timer_get_time() - beat.anchor_us) / beat.period_us;
            if (k != headbang_k_) {
                headbang_k_ = k;
                headbang_ = 1.0f;
                if ((esp_random() % 100) < 15) {
                    uint32_t e = esp_random() % 100;
                    Emotion em = (e < 55) ? Emotion::Happy
                               : (e < 85) ? Emotion::Neutral
                                          : Emotion::Doubt;
                    avatar.setEmotion(em);
                }
            }

            if (headbang_ <= 0.02f) {
                reset_avatar_offset();
                return;
            }

            int dy = (int)(headbang_ * 38.0f);
            avatar.leftEye().setPosition(uitk::Vector2i(0, dy));
            avatar.rightEye().setPosition(uitk::Vector2i(0, dy));
            avatar.mouth().setPosition(uitk::Vector2i(0, dy));
            headbang_ *= 0.82f;
            headbang_active_ = true;
            return;
        }

        if (!headbang_active_) {
            return;
        }
        reset_avatar_offset(true);
    }

    void reset_avatar_offset(bool reset_phase = false)
    {
        if (reset_phase) {
            headbang_k_ = -1;
        }
        if (!headbang_active_) {
            return;
        }
        headbang_active_ = false;
        if (GetStackChan().hasAvatar()) {
            auto& avatar = GetStackChan().avatar();
            avatar.leftEye().setPosition(uitk::Vector2i(0, 0));
            avatar.rightEye().setPosition(uitk::Vector2i(0, 0));
            avatar.mouth().setPosition(uitk::Vector2i(0, 0));
            avatar.setEmotion(stackchan::avatar::Emotion::Neutral);
        }
    }

    int64_t beat_k_ = -1;
    float beat_intensity_ = 0.0f;
    uint8_t beat_r_ = 0;
    uint8_t beat_g_ = 0;
    uint8_t beat_b_ = 0;
    int64_t headbang_k_ = -1;
    float headbang_ = 0.0f;
    bool headbang_active_ = false;
};

static void _stackchan_update_task(void* param)
{
    bool is_setup_done = false;
    BeatReactiveAnimator beat_animator;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        tools::update_reminders();

        LvglLockGuard lock;

        if (!hal_bridge::is_xiaozhi_idle()) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        GetStackChan().update();

        beat_animator.update(hal_bridge::beat_get());

        if (!hal_bridge::is_xiaozhi_ready()) {
            continue;
        }

        if (!is_setup_done) {
            // Setup when xiaozhi ready
            GetHAL().startSntp();
            view::create_home_indicator([]() { GetHAL().requestWarmReboot(0); }, 0x81DBBD, 0x134233);
            view::create_status_bar(0x81DBBD, 0x134233);
            is_setup_done = true;
        }

        view::update_home_indicator();
        view::update_status_bar();
    }
}

void Hal::startXiaozhi()
{
    mclog::tagInfo(_tag, "start xiaozhi");

    auto& motion = GetStackChan().motion();
    motion.setAutoAngleSyncEnabled(true);
    motion.setAutoTorqueReleaseEnabled(true);

    // Setup reminder handler
    tools::on_reminder_triggered().clear();
    tools::on_reminder_triggered().connect([](int id, std::string_view msg) {
        mclog::tagInfo(_tag, "reminder triggered: id: {}, msg: {}", id, msg);
        {
            LvglLockGuard lock;
            auto& avatar = GetStackChan().avatar();
            avatar.addDecorator(std::make_unique<view::ReminderView>(lv_screen_active(), msg));
        }
        hal_bridge::app_play_sound(OGG_NEW_NOTIFICATION);
    });

    // Start stackchan update task
    xTaskCreatePinnedToCore(_stackchan_update_task, "stackchan", 4096, NULL, 3, NULL, 1);

    hal_bridge::start_xiaozhi_app();
}

void Hal::startRequestedAgentRuntime()
{
    switch (_requested_agent_runtime) {
        case AgentRuntimeKind::Gpt: {
            GptAgentRuntime runtime;
            runtime.start();
            break;
        }
        case AgentRuntimeKind::Grok: {
            GrokAgentRuntime runtime;
            runtime.start();
            break;
        }
        case AgentRuntimeKind::Xiaozhi: {
            XiaozhiAgentRuntime runtime;
            runtime.start();
            break;
        }
        case AgentRuntimeKind::None:
        default:
            mclog::tagWarn(_tag, "agent runtime start requested without selected runtime");
            break;
    }
}

XiaozhiConfig_t Hal::getXiaozhiConfig()
{
    auto bridge_config = hal_bridge::get_xiaozhi_config();
    return XiaozhiConfig_t{
        .idleShutdownTimeSeconds   = bridge_config.idleShutdownTimeSeconds,
        .allowShutdownWhenCharging = bridge_config.allowShutdownWhenCharging,
        .idleRandomMovementLevel   = bridge_config.idleRandomMovementLevel,
    };
}

void Hal::setXiaozhiConfig(XiaozhiConfig_t config)
{
    hal_bridge::set_xiaozhi_config({
        .idleShutdownTimeSeconds   = config.idleShutdownTimeSeconds,
        .allowShutdownWhenCharging = config.allowShutdownWhenCharging,
        .idleRandomMovementLevel   = config.idleRandomMovementLevel,
    });
}

uint8_t Hal::getBatteryLevel()
{
    return hal_bridge::board_get_battery_level();
}

bool Hal::isBatteryCharging()
{
    return hal_bridge::board_is_battery_charging();
}

void Hal::factoryReset()
{
    mclog::tagInfo(_tag, "start factory reset");
    ESP_ERROR_CHECK(nvs_flash_erase());
    reboot();
}

/* -------------------------------------------------------------------------- */
/*                                   Display                                  */
/* -------------------------------------------------------------------------- */
#include "board/hal_bridge.h"

void Hal::lvglLock()
{
    hal_bridge::disply_lvgl_lock();
}

void Hal::lvglUnlock()
{
    hal_bridge::disply_lvgl_unlock();
}

void Hal::setBackLightBrightness(uint8_t brightness, bool permanent)
{
    hal_bridge::board_set_backlight_brightness(brightness, permanent);
}

uint8_t Hal::getBackLightBrightness()
{
    return hal_bridge::board_get_backlight_brightness();
}

void Hal::setSpeakerVolume(uint8_t volume, bool permanent)
{
    hal_bridge::board_set_speaker_volume(volume, permanent);
}

uint8_t Hal::getSpeakerVolume()
{
    return hal_bridge::board_get_speaker_volume();
}

/* -------------------------------------------------------------------------- */
/*                                    Lvgl                                    */
/* -------------------------------------------------------------------------- */
#include "board/hal_bridge.h"
#include <stackchan/stackchan.h>

static void lvgl_read_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
    hal_bridge::lock();
    auto& bridge_data = hal_bridge::get_data();

    // mclog::tagInfo(_tag, "touchpoint: {}, x: {}, y: {}", bridge_data.touchPoint.num, bridge_data.touchPoint.x,
    //                bridge_data.touchPoint.y);

    if (bridge_data.touchPoint.num == 0) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = bridge_data.touchPoint.x;
        data->point.y = bridge_data.touchPoint.y;
    }

    hal_bridge::unlock();
}

void Hal::lvgl_init()
{
    mclog::tagInfo(_tag, "lvgl init");

    hal_bridge::disply_lvgl_lock();

    mclog::tagInfo(_tag, "create lvgl touchpad indev");
    lvTouchpad = lv_indev_create();
    lv_indev_set_type(lvTouchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvTouchpad, lvgl_read_cb);
    lv_indev_set_group(lvTouchpad, lv_group_get_default());
    lv_indev_set_display(lvTouchpad, hal_bridge::display_get_lvgl_display());

    hal_bridge::disply_lvgl_unlock();
}

/* -------------------------------------------------------------------------- */
/*                                 Warm Reboot                                */
/* -------------------------------------------------------------------------- */
#include <settings.h>
#include <string_view>

static std::string_view _warm_boot_nvs_ns  = "warm_boot";
static std::string_view _warm_boot_nvs_key = "app_index";

void Hal::requestWarmReboot(int appIndex)
{
    mclog::tagInfo(_tag, "warm reboot request to app index: {}", appIndex);

    {
        Settings settings(_warm_boot_nvs_ns.data(), true);
        settings.SetInt(_warm_boot_nvs_key.data(), appIndex);
    }

    delay(100);
    esp_restart();
}

int Hal::getWarmRebootTarget()
{
    Settings settings(_warm_boot_nvs_ns.data(), false);
    return settings.GetInt(_warm_boot_nvs_key.data(), -1);
}

void Hal::clearWarmRebootRequest()
{
    mclog::tagInfo(_tag, "clear warm reboot request");

    Settings settings(_warm_boot_nvs_ns.data(), true);
    settings.SetInt(_warm_boot_nvs_key.data(), -1);
}
