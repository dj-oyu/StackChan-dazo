/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "gpt_agent_runtime.h"
#include <hal/hal.h>
#include <mooncake_log.h>
#include <settings.h>
#include <string_view>

static const std::string_view _tag = "GPT_AGENT";

void GptAgentRuntime::start()
{
    mclog::tagInfo(_tag, "start gpt runtime");

    {
        Settings settings("agent", true);
        settings.SetString("session_provider", "gpt");
    }

    GetHAL().startXiaozhi();
}
