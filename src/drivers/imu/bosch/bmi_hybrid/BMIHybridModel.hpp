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
 * @file BMIHybridModel.hpp
 * @brief Model profiles and bounded register configuration for Bosch SPI IMUs.
 */

#pragma once

#include "BMIHybridFifo.hpp"
#include "BMIHybridRegisters.hpp"

#include <lib/drivers/imu/SpiFamily.hpp>

namespace bmi_hybrid_model
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

constexpr uint8_t resetReg(bmi_hybrid_fifo::Format format)
{
	return bmi_hybrid_fifo::tagged(format)
	       ? bmi_hybrid_registers::tagged::Command
	       : bmi_hybrid_registers::fixed::Reset;
}

constexpr uint8_t countReg(bmi_hybrid_fifo::Format format)
{
	return bmi_hybrid_fifo::tagged(format)
	       ? bmi_hybrid_registers::tagged::FifoLength
	       : bmi_hybrid_registers::fixed::FifoStatus;
}

constexpr uint8_t fifoReg(bmi_hybrid_fifo::Format format)
{
	return format == bmi_hybrid_fifo::Format::kTaggedAccel ? bmi_hybrid_registers::tagged::FifoLength
	       : format == bmi_hybrid_fifo::Format::kTaggedImu ? bmi_hybrid_registers::tagged::FifoData
	       : bmi_hybrid_registers::fixed::FifoData;
}

constexpr uint8_t watermarkReg(bmi_hybrid_fifo::Format format)
{
	return bmi_hybrid_fifo::tagged(format) ? bmi_hybrid_registers::tagged::WatermarkLow
	       : format == bmi_hybrid_fifo::Format::kFixed12Accel ? bmi_hybrid_registers::fixed::AccelWatermark
	       : bmi_hybrid_registers::fixed::GyroWatermark;
}

constexpr uint8_t temperatureReg(Temperature temperature)
{
	return temperature == Temperature::kSigned8
	       ? bmi_hybrid_registers::fixed::Temperature
	       : bmi_hybrid_registers::tagged::Temperature;
}

/** Immutable wire, sampling and conversion policy for one board-registered SPI endpoint. */
struct Profile {
	imu::SpiModel           device {};
	Variant                 variant             { Variant::kBmi08xAccel };
	bmi_hybrid_fifo::Format format              { bmi_hybrid_fifo::Format::kTaggedAccel };
	uint8_t                 whoami              { 0 };
	uint8_t                 alternate_whoami    { 0 };
	uint16_t                rate_hz             { 2000 }; ///< Nominal output data rate [Hz].
	uint16_t                fifo_capacity_bytes { 600 };  ///< Hardware FIFO capacity [bytes].
	Temperature             temperature         { Temperature::kNone };
	uint32_t                reset_wait_us       { 1000 };  ///< Soft-reset recovery wait [us].
	uint32_t                configure_wait_us   { 10000 }; ///< Post-configuration startup settle wait [us].
	float                   accel_range         { 0.f };   ///< Accelerometer full-scale range [m/s^2].
	float                   accel_scale         { 0.f };   ///< Accelerometer scale factor [(m/s^2)/LSB].
	float                   gyro_range          { 0.f };   ///< Gyroscope full-scale range [rad/s].
	float                   gyro_scale          { 0.f };   ///< Gyroscope scale factor [(rad/s)/LSB].
	bool                    interrupt_enabled   { true };
	bool                    reject_all_minimum  { false };
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

} // namespace bmi_hybrid_model
