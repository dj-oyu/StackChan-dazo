/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include "element.h"
#include <string_view>
#include <string>

namespace stackchan::avatar {

/**
 * @brief Speech bubble base class
 *
 */
class SpeechBubble : public Element {
public:
    virtual ~SpeechBubble() = default;

    virtual void setSpeech(std::string_view text)
    {
    }

    // Called once when the streamed message is complete; the default bubble uses
    // this to start the read-through scroll loop (it doesn't loop while streaming).
    virtual void onSpeechComplete()
    {
    }

    virtual void clearSpeech()
    {
    }

    virtual void setTextFont(void* font)
    {
    }
};

}  // namespace stackchan::avatar
