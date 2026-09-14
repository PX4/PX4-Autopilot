/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include "../TdkPacketConfig.hpp"

namespace tdk_icm45686_config
{
using AddressSpace = tdk_packet::AddressSpace;
using RegisterConfig = tdk_packet::RegisterConfig;
constexpr uint8_t kRegisterCount { 15 };
constexpr uint8_t kDirectFirst { 2 };
constexpr uint8_t kDirectCount { 13 };
constexpr uint8_t kIndirectFirst { 0 };
constexpr uint8_t kIndirectCount { 2 };

/**
 * @brief Restore the fixed configuration and fill its dynamic fields.
 * @param[in] fifo_watermark Watermark in records; zero or an unrepresentable value is invalid.
 * @param[in] clock_input Select external CLKIN on INT2; false selects the internal clock.
 * @param[out] config Exact-size destination; unchanged when the watermark is invalid.
 * @return kRegisterCount on success, or UINT8_MAX for an invalid watermark.
 */
[[nodiscard]] uint8_t load(
	uint16_t fifo_watermark,
	bool clock_input,
	RegisterConfig(&config)[kRegisterCount]);

} // namespace tdk_icm45686_config
