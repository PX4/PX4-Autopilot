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

#include "../common/SpiFamily.hpp"

/** Wire and conversion descriptions shared by flat-register TDK endpoints. */
namespace tdk_flat
{
enum class Variant : uint8_t {
	kMpu6000,
	kMpu6500,
	kMpu9250,
	kIcm20602,
	kIcm20608G,
	kIcm20689,
	kIam20680HP,
};

/** Immutable wire layout, scaling and initialization policy for one exact model. */
struct Profile {
	imu::SpiModel device;
	Variant       variant;
	uint8_t       whoami;
	uint16_t      fifo_size; ///< Conservative software FIFO bound in bytes; may be smaller than the hardware capacity.
	uint8_t       fifo_packet_size; ///< Wire frame size in bytes, excluding the SPI command.
	uint8_t       gyro_offset; ///< Byte offset of gyro X within a frame.
	uint8_t       samples_per_transfer; ///< Gyro frames per new accel sample, not the total SPI transfer count.
	float    temperature_sensitivity; ///< Raw counts per degree Celsius.
	float    temperature_offset; ///< Degrees Celsius added after dividing raw temperature by sensitivity.
	uint8_t  reset_pwr_value;
	uint32_t reset_wait_us;
	uint32_t configure_wait_us;
	bool     check_reset_pwr;
	bool     check_reset_config;
	bool     has_factory_accel_offsets;
	bool     has_fifo_temperature;
};


} // namespace tdk_flat
