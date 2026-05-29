/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "default.h"

#include <cstring>
#include <string>

using namespace uitk;
using namespace uitk::lvgl_cpp;
using namespace stackchan::avatar;

// The transparent container frames the bubble. The bubble is anchored to its
// bottom edge (~y234) and grows upward, so multi-line text never runs off the
// bottom of the screen.
static const Vector2i _container_pos  = Vector2i(0, 46);
static const Vector2i _container_size = Vector2i(320, 160);
static const int _bubble_width        = 300;  // fixed width; height grows with text
static const int _bubble_pad_x        = 14;
static const int _bubble_pad_y        = 6;
static const int _bubble_max_lines    = 2;   // grow up to this, then scroll/loop
static const int _bubble_bottom_ofs   = -12;  // bubble bottom relative to container bottom
static const int _scroll_speed_pps    = 30;  // auto-scroll speed (px/sec) when looping

// Drives the looping vertical auto-scroll of overflowing text.
static void bubble_scroll_anim_cb(void* obj, int32_t y)
{
    lv_obj_scroll_to_y((lv_obj_t*)obj, y, LV_ANIM_OFF);
}

DefaultSpeechBubble::DefaultSpeechBubble(lv_obj_t* parent, lv_color_t primaryColor, lv_color_t secondaryColor,
                                         const lv_font_t* font)
{
    _container = std::make_unique<Container>(parent);
    _container->setRadius(0);
    _container->setAlign(LV_ALIGN_CENTER);
    _container->setBorderWidth(0);
    _container->setBgOpa(0);
    _container->removeFlag(LV_OBJ_FLAG_SCROLLABLE);
    _container->setSize(_container_size.x, _container_size.y);
    _container->setPos(_container_pos.x, _container_pos.y);
    _container->setPadding(0, 0, 0, 0);
    // The bubble grows taller than the container for multi-line text; don't clip it.
    lv_obj_add_flag(_container->get(), LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    _bubble = std::make_unique<Container>(_container->get());
    _bubble->setRadius(LV_RADIUS_CIRCLE);
    // Anchor to the bottom and grow upward so multi-line text never runs off-screen.
    _bubble->setAlign(LV_ALIGN_BOTTOM_MID);
    _bubble->setBorderWidth(0);
    _bubble->setBgColor(primaryColor);
    _bubble->setWidth(_bubble_width);
    _bubble->setPos(0, _bubble_bottom_ofs);
    {
        // Height tracks the text exactly (no manual measurement that could clip the
        // last line); capped via max_height in setSpeech. Allow vertical scrolling so
        // text beyond _bubble_max_lines stays readable.
        lv_obj_t* bubble = _bubble->get();
        lv_obj_set_height(bubble, LV_SIZE_CONTENT);
        lv_obj_set_style_pad_left(bubble, _bubble_pad_x, 0);
        lv_obj_set_style_pad_right(bubble, _bubble_pad_x, 0);
        lv_obj_set_style_pad_top(bubble, _bubble_pad_y, 0);
        lv_obj_set_style_pad_bottom(bubble, _bubble_pad_y, 0);
        lv_obj_add_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(bubble, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(bubble, LV_SCROLLBAR_MODE_OFF);
    }

    _text = std::make_unique<Label>(_bubble->get());
    _text->setTextColor(secondaryColor);
    _text->setTextFont(font);
    _text->setTextAlign(LV_TEXT_ALIGN_CENTER);
    _text->setAlign(LV_ALIGN_TOP_MID);
    _text->setPos(0, 0);
    _text->setWidth(_bubble_width - _bubble_pad_x * 2);
    // WRAP renders real line breaks ('\n') and wraps long lines to multiple rows.
    _text->setLongMode(LV_LABEL_LONG_MODE_WRAP);

    clearSpeech();
}

DefaultSpeechBubble::~DefaultSpeechBubble()
{
    _text.reset();
    _bubble.reset();
    _container.reset();
}

void DefaultSpeechBubble::setSpeech(std::string_view text)
{
    if (text.empty()) {
        clearSpeech();
        return;
    }

    // Streaming re-sends the accumulated text; skip all work (LVGL text re-alloc,
    // layout, animation) when nothing changed.
    if (_shown.size() == text.size() && memcmp(_shown.data(), text.data(), text.size()) == 0) {
        return;
    }
    // Reuse one owned buffer instead of allocating a fresh std::string per delta,
    // keeping churn off the scarce internal heap.
    _shown.assign(text.data(), text.size());

    // Keep real newlines: WRAP renders them as line breaks. The bubble height grows
    // with the text up to _bubble_max_lines; longer text scrolls vertically so the
    // whole message can be read (the text is streamed as it is spoken).
    _text->setText(_shown.c_str());

    const lv_font_t* font = _text->getTextFont();
    int line_h     = lv_font_get_line_height(font);
    int line_space = lv_obj_get_style_text_line_space(_text->get(), LV_PART_MAIN);

    lv_obj_t* bubble = _bubble->get();
    // Bubble height auto-fits the text (LV_SIZE_CONTENT); cap the visible area at an
    // EXACT integer number of lines so the scroll window always lands on line
    // boundaries (no half-line slivers clipped at the top/bottom of the bubble).
    int max_h = line_h * _bubble_max_lines + line_space * (_bubble_max_lines - 1) + _bubble_pad_y * 2;
    lv_obj_set_style_max_height(bubble, max_h, 0);

    lv_obj_update_layout(bubble);
    _overflow = lv_obj_get_height(_text->get()) - lv_obj_get_content_height(bubble);
    if (_overflow < 0) {
        _overflow = 0;
    }

    // While the text is still streaming, don't loop (it would keep jumping back to
    // the top). Just follow the latest line by keeping the bottom in view. The
    // read-through loop is started from onSpeechComplete() once the reply is done.
    lv_anim_delete(bubble, bubble_scroll_anim_cb);
    lv_obj_scroll_to_y(bubble, _overflow, LV_ANIM_OFF);

    setVisible(true);
}

void DefaultSpeechBubble::onSpeechComplete()
{
    if (_overflow <= 0) {
        return;  // fits in the bubble; nothing to scroll
    }
    // Reply finished: scroll the whole message from the top, looping, so it can be
    // read in full (down to the end and back to the start, with pauses).
    lv_obj_t* bubble = _bubble->get();
    lv_anim_delete(bubble, bubble_scroll_anim_cb);
    lv_obj_scroll_to_y(bubble, 0, LV_ANIM_OFF);

    int duration = _overflow * 1000 / _scroll_speed_pps;
    if (duration < 300) {
        duration = 300;
    }
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, bubble);
    lv_anim_set_exec_cb(&a, bubble_scroll_anim_cb);
    lv_anim_set_values(&a, 0, _overflow);
    lv_anim_set_time(&a, duration);
    lv_anim_set_playback_time(&a, duration);
    lv_anim_set_repeat_delay(&a, 1500);    // pause at the top before repeating
    lv_anim_set_playback_delay(&a, 1500);  // pause at the bottom before going back
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
}

void DefaultSpeechBubble::clearSpeech()
{
    lv_anim_delete(_bubble->get(), bubble_scroll_anim_cb);
    _text->setText("");
    _shown.clear();
    _overflow = 0;
    setVisible(false);
}

void DefaultSpeechBubble::setVisible(bool visible)
{
    SpeechBubble::setVisible(visible);

    _container->setHidden(!visible);
}

void DefaultSpeechBubble::setTextFont(void* font)
{
    if (_text && font) {
        _text->setTextFont((lv_font_t*)font);
    }
}
