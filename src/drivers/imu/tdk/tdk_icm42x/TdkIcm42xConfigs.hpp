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

namespace tdk_icm42x_config
{
using Variant = tdk_packet::Variant;
using AddressSpace = tdk_packet::AddressSpace;
using RegisterConfig = tdk_packet::RegisterConfig;
using Context = tdk_packet::Context;
using tdk_packet::kMaxRegisterConfigs;

/**
 * @brief Fill a complete register configuration without losing the destination capacity.
 * @param[in] variant Exact chip/register dialect.
 * @param[in] context Watermark in the variant's register units and reference-clock selection.
 * @param[out] config Fixed-capacity destination; only the returned valid prefix may be used.
 * @return Register count, zero for an unsupported variant, or UINT8_MAX for invalid masks, watermark or overflow.
 * @note On error, discard the partial configuration instead of writing it to the sensor.
 */
[[nodiscard]] uint8_t load(
	Variant variant,
	const Context &context,
	RegisterConfig(&config)[kMaxRegisterConfigs]);

} // namespace tdk_icm42x_config
