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
 * @file ICM45xConfigs.hpp
 * @brief Model profiles and BANK_0 / IREG configuration tables for ICM-45686 and ICM-56686.
 */

#pragma once

#include <lib/drivers/imu/SpiFamily.hpp>

#include <cstdint>

namespace icm45x_config
{

enum class Variant : uint8_t {
	kIcm45686,
	kIcm56686,
};

struct Bank0Config {
	uint8_t reg        { 0 };
	uint8_t set_bits   { 0 };
	uint8_t clear_bits { 0 };
};

struct IregConfig {
	uint16_t reg        { 0 };
	uint8_t  set_bits   { 0 };
	uint8_t  clear_bits { 0 };
};

struct Context {
	uint16_t fifo_watermark { 0 };
	bool     clock_input    { false };
	bool     int2           { false };
	bool     has_drdy_gpio  { false };
};

static constexpr uint8_t kMaxBank0Configs { 16 };
static constexpr uint8_t kMaxIregConfigs  { 10 };

struct Profile {
	imu::SpiModel device {};
	Variant       variant                 { Variant::kIcm45686 };
	uint8_t       whoami                  { 0 };
	uint16_t      fifo_capacity           { 0 };     ///< Hardware FIFO capacity [bytes].
	uint16_t      output_data_rate_hz     { 6400 };  ///< Nominal accel/gyro output data rate [Hz].
	uint8_t       packet_size             { 16 };    ///< Wire record size excluding the SPI prefix [bytes].
	float         accel_range_g           { 32.f };  ///< Accelerometer full-scale range [g].
	float         gyro_range_dps          { 4000.f }; ///< Gyroscope full-scale range [deg/s].
	float         temperature_sensitivity { 128.f }; ///< Temperature sensitivity [LSB/degC].
	float         temperature_offset      { 25.f };  ///< Temperature offset at zero raw count [degC].
	uint8_t       fifo_count_reg          { 0 };
	uint8_t       fifo_data_reg           { 0 };
	uint8_t       temperature_reg         { 0 };
	bool          double_read_fifo_count  { false };
	bool          fifo_temperature        { false };
	bool          periodic_register_check { false };
	bool          clock_input             { false };
	bool          supports_int2           { false };
};

[[nodiscard]] uint8_t loadBank0(
	Variant variant,
	const Context &context,
	Bank0Config(&config)[kMaxBank0Configs]);

[[nodiscard]] uint8_t loadIreg(
	Variant variant,
	const Context &context,
	IregConfig(&config)[kMaxIregConfigs]);

} // namespace icm45x_config

struct ICM45xOptions {
	const icm45x_config::Profile *profile { nullptr };
	bool int2 { false };
};
