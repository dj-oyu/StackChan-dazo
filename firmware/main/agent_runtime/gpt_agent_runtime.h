/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "agent_runtime.h"

class GptAgentRuntime : public AgentRuntime {
public:
    void start() override;
};

