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

#include "BoschDirectFifo.hpp"
#include "../../common/SpiFamily.hpp"

namespace bosch_direct_model
{
enum class Variant : uint8_t {
	kBmi055Accel,
	kBmi055Gyro,
	kBmi08xAccel,
	kBmi08xGyro,
	kBmi270,
};

enum class Temperature : uint8_t {
	kNone,
	kSigned8,
	kBmi08x11,
	kBmi27016,
};

/** Immutable wire, sampling and conversion policy for one board-registered SPI endpoint. */
struct Profile {
	imu::SpiModel device;
	Variant       variant;
	bosch_direct_fifo::Format format;
	uint8_t     whoami;
	uint8_t     alternate_whoami;
	uint16_t    rate_hz;
	uint16_t    fifo_capacity_bytes;
	uint8_t     reset_reg;
	uint8_t     count_reg;
	uint8_t     fifo_reg;
	uint8_t     watermark_reg;
	uint8_t     temperature_reg;
	Temperature temperature;
	uint32_t    reset_wait_us;
	uint32_t    configure_wait_us;
	float       accel_range; ///< m/s^2.
	float       accel_scale; ///< (m/s^2) per decoded count.
	float       gyro_range; ///< rad/s.
	float       gyro_scale; ///< (rad/s) per decoded count.
	bool        interrupt_enabled;
	bool        reject_all_minimum;
};

struct RegisterConfig {
	uint8_t reg;
	uint8_t set_bits;
	uint8_t clear_bits;
};

constexpr uint8_t kMaxRegisterConfigs { 16 };

/** Returns zero on invalid/unsupported input, never a truncated configuration. */
uint8_t loadRegisters(
	const Profile &profile,
	uint16_t watermark,
	RegisterConfig(&out)[kMaxRegisterConfigs]);

} // namespace bosch_direct_model
