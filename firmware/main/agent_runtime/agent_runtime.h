/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

class AgentRuntime {
public:
    virtual ~AgentRuntime() = default;
    virtual void start()    = 0;
};

