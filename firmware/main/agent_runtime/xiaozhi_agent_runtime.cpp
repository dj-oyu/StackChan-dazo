/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "xiaozhi_agent_runtime.h"
#include <hal/hal.h>
#include <settings.h>

void XiaozhiAgentRuntime::start()
{
    {
        Settings settings("agent", true);
        settings.SetString("session_provider", "xiaozhi");
    }

    GetHAL().startXiaozhi();
}
