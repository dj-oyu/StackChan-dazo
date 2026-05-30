/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <cstdint>
#include <lvgl.h>
#include <driver/i2c_master.h>
#include <string_view>

class StackChanCamera;

namespace hal_bridge {

struct TouchPoint_t {
    int num = 0;
    int x   = -1;
    int y   = -1;
};

struct Data_t {
    TouchPoint_t touchPoint;
    bool isXiaozhiMode              = false;
    bool isXiaozhiModeToggleEnabled = false;
};

struct XiaozhiConfig_t {
    uint32_t idleShutdownTimeSeconds = 600;
    bool allowShutdownWhenCharging   = false;
    uint8_t idleRandomMovementLevel  = 2;
};

void lock();
void unlock();
Data_t& get_data();

void set_touch_point(int num, int x, int y);
TouchPoint_t get_touch_point();

bool is_xiaozhi_mode();
void set_xiaozhi_mode(bool mode);
void toggle_xiaozhi_chat_state();

void disply_lvgl_lock();
void disply_lvgl_unlock();
lv_disp_t* display_get_lvgl_display();

void xiaozhi_board_init();
void start_xiaozhi_app();
bool is_xiaozhi_ready();
bool is_xiaozhi_idle();
XiaozhiConfig_t get_xiaozhi_config();
void set_xiaozhi_config(const XiaozhiConfig_t& config);

i2c_master_bus_handle_t board_get_i2c_bus();
StackChanCamera* board_get_camera();
int board_get_battery_level();
bool board_is_battery_charging();
void board_set_backlight_brightness(uint8_t brightness, bool permanent = false);
uint8_t board_get_backlight_brightness();
void board_set_speaker_volume(uint8_t volume, bool permanent = false);
uint8_t board_get_speaker_volume();

void app_play_sound(const std::string_view& sound);

// True while the head servos are mid-move. Lets audio-side code (e.g. the idle
// music survey / future beat detector) ignore frames contaminated by servo noise
// using the known motion state instead of trying to detect it acoustically.
bool servo_is_moving();

// Beat-detector state shared from the audio task (producer) to the stackchan update
// task (consumer, drives the LED pulse). Beats occur at anchor_us + k*period_us.
struct BeatState {
    int64_t anchor_us = 0;   // time of a recent beat (phase reference, esp_timer us)
    int64_t period_us = 0;   // beat period in microseconds
    bool locked = false;     // true while a stable tempo is tracked
    float confidence = 0.0f; // 0..1 tempo lock confidence (drives LED brightness)
};
void beat_publish(int64_t anchor_us, int64_t period_us, bool locked, float confidence);
BeatState beat_get();

}  // namespace hal_bridge
