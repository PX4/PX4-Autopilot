/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CanForwardTestHelpers.hpp
 * DroneCAN CAN ID builders shared by the can_forward tests.
 */

#pragma once

#include <stdint.h>

namespace can_forward_test
{

static constexpr uint32_t Eff = 1U << 31;

// DroneCAN data type IDs used by node discovery and firmware update
static constexpr uint16_t NodeStatus = 341;          // message
static constexpr uint16_t Allocation = 1;            // message (dynamic node ID allocation)
static constexpr uint16_t GetNodeInfo = 1;           // service
static constexpr uint16_t BeginFirmwareUpdate = 40;  // service
static constexpr uint16_t FileGetInfo = 45;          // service
static constexpr uint16_t FileRead = 48;             // service
static constexpr uint16_t ParamGetSet = 11;          // service

/// message frame: priority | type ID (16 bit) | 0 | source node
static constexpr uint32_t messageId(uint16_t type_id, uint8_t src, uint8_t priority = 16)
{
	return Eff | (uint32_t(priority) << 24) | (uint32_t(type_id) << 8) | (src & 0x7FU);
}

/// service frame: priority | type ID (8 bit) | request | destination | 1 | source node
static constexpr uint32_t serviceId(uint8_t type_id, bool request, uint8_t dst, uint8_t src, uint8_t priority = 16)
{
	return Eff | (uint32_t(priority) << 24) | (uint32_t(type_id) << 16) | (uint32_t(request ? 1 : 0) << 15)
	       | (uint32_t(dst & 0x7FU) << 8) | 0x80U | (src & 0x7FU);
}

/// anonymous message frame: priority | discriminator | type ID (2 bit) | 0 | source 0
static constexpr uint32_t anonymousId(uint16_t discriminator, uint8_t type_id_low2, uint8_t priority = 16)
{
	return Eff | (uint32_t(priority) << 24) | (uint32_t(discriminator & 0x3FFFU) << 10)
	       | (uint32_t(type_id_low2 & 0x3U) << 8);
}

} // namespace can_forward_test
