/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "grok_agent_runtime.h"
#include <hal/hal.h>
#include <mooncake_log.h>
#include <settings.h>
#include <string_view>

static const std::string_view _tag = "GROK_AGENT";

void GrokAgentRuntime::start()
{
    mclog::tagInfo(_tag, "start grok runtime");

    {
        Settings settings("agent", true);
        settings.SetString("provider", "grok");
    }

    GetHAL().startXiaozhi();
}
