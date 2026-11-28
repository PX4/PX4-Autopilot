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

/**
 * @file ICM42xConfigs.hpp
 * @brief Model profiles and bounded banked register-table construction for TDK ICM42x devices.
 */

#pragma once

#include <lib/drivers/imu/SpiFamily.hpp>

#include <cstdint>

namespace icm42x_config
{

enum class Variant : uint8_t {
	kIcm40609D,
	kIcm42605,
	kIcm42688P,
	kIcm42686P,
	kIim42652,
	kIim42653,
};

enum class AddressSpace : uint8_t {
	kBank0 = 0,
	kBank1 = 1,
	kBank2 = 2,
};

enum class PacketFormat : uint8_t {
	kStandard16,
	kHighRes20,
};

struct RegisterConfig {
	AddressSpace space      { AddressSpace::kBank0 };
	uint16_t     reg        { 0 };
	uint8_t      set_bits   { 0 };
	uint8_t      clear_bits { 0 };
};

struct Context {
	uint16_t fifo_watermark { 0 };     ///< Register watermark: records for ICM40609D, bytes for other variants.
	bool     clock_input    { false }; ///< Enable external CLKIN; false selects the internal clock.
	uint16_t aaf_hz         { 0 };     ///< Anti-alias filter preset [Hz] (0 selects the 585 Hz chip default).
};

static constexpr uint8_t  kMaxRegisterConfigs { 32 };
static constexpr uint8_t  kFifoTransferPrefix { 4 }; ///< Command + INT_STATUS + FIFO_COUNTH + FIFO_COUNTL [bytes].
static constexpr uint16_t kOutputDataRateHz   { 8000 };

/** Immutable wire layout and conversion policy for one banked 8 kHz model. */
struct Profile {
	imu::SpiModel device {};
	Variant       variant               { Variant::kIcm42688P };
	uint8_t       whoami                { 0 };
	PacketFormat  packet_format         { PacketFormat::kHighRes20 };
	uint8_t       packet_size           { 20 };    ///< Complete wire record size excluding the SPI prefix [bytes].
	float         accel_range_g         { 16.f };  ///< Accelerometer full-scale range [g].
	float         gyro_range_dps        { 2000.f }; ///< Gyroscope full-scale range [deg/s].
	bool          check_reset_done      { true };  ///< Require INT_STATUS::RESET_DONE_INT after soft reset.
	bool          fifo_count_is_records { false }; ///< ICM-40609-D reports records; others report bytes.
	bool          clock_input           { true };  ///< External CLKIN supported on Pin 9.
};

template<typename T>
void add(
	RegisterConfig(&config)[kMaxRegisterConfigs],
	uint8_t &count,
	AddressSpace space,
	T reg,
	uint8_t set_bits,
	uint8_t clear_bits)
{
	if (count < kMaxRegisterConfigs && (set_bits & clear_bits) == 0) {
		config[count++] = {space, static_cast<uint16_t>(reg), set_bits, clear_bits};

	} else {
		count = UINT8_MAX;
	}
}

template<typename T>
void addWatermark(
	RegisterConfig(&config)[kMaxRegisterConfigs],
	uint8_t &count,
	T low_register,
	T high_register,
	uint16_t watermark,
	uint8_t high_mask)
{
	const uint8_t low  = watermark & 0xff;
	const uint8_t high = watermark >> 8;

	if (watermark == 0 || (high & ~high_mask) != 0) {
		count = UINT8_MAX;
		return;
	}

	add(config, count, AddressSpace::kBank0, low_register, low, static_cast<uint8_t>(~low));
	add(config, count, AddressSpace::kBank0, high_register, high, high_mask & ~high);
}

/**
 * @brief Fill a complete register configuration without losing the destination capacity.
 * @return Register count, zero for an unsupported variant, or UINT8_MAX for invalid masks, watermark, AAF or overflow.
 */
[[nodiscard]] uint8_t load(
	Variant variant,
	const Context &context,
	RegisterConfig(&config)[kMaxRegisterConfigs]);

} // namespace icm42x_config

struct ICM42xOptions {
	const icm42x_config::Profile *profile { nullptr };
	uint16_t aaf_hz { 0 };
};
