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
#include "TdkPacketConfig.hpp"

/** Descriptors and bounded register-table construction for TDK packet-FIFO devices. */
namespace tdk_packet
{

enum class Protocol : uint8_t {
	kBanked,
	kMreg,
	kDirect456,
};

enum class PacketFormat : uint8_t {
	kStandard16,
	kHighRes20,
	kStandard16LittleEndian,
};

/** Immutable protocol, wire layout and conversion policy for one exact model. */
struct Profile {
	imu::SpiModel device;
	Variant       variant;
	uint8_t       whoami;
	Protocol      protocol;
	PacketFormat  packet_format;
	uint16_t      fifo_capacity; ///< Hardware FIFO capacity in bytes, even when its count register reports records.
	uint16_t      output_data_rate_hz; ///< Configured nominal accel/gyro sample rate in Hz.
	uint8_t       packet_size; ///< Complete wire record size in bytes, excluding the SPI prefix.
	float         accel_range_g; ///< Positive full-scale acceleration in standard gravity units.
	float         gyro_range_dps; ///< Positive full-scale angular rate in degrees per second.
	float   temperature_sensitivity; ///< Register/high-resolution temperature counts per degree Celsius.
	float   temperature_offset; ///< Degrees Celsius added after dividing raw temperature by sensitivity.
	uint8_t whoami_reg;
	uint8_t reset_reg;
	uint8_t reset_bit;
	uint8_t reset_status_reg;
	uint8_t reset_status_bit;
	uint8_t power_reg;
	uint8_t int_status_reg;
	uint8_t fifo_full_bit;
	uint8_t fifo_count_reg;
	uint8_t fifo_data_reg;
	uint8_t fifo_transfer_prefix; ///< Bytes before the first FIFO record, including command/status/count.
	uint8_t signal_path_reset_reg;
	uint8_t fifo_flush_bit;
	uint8_t temperature_reg;
	bool    fifo_count_is_records; ///< True: count register reports records; false: bytes. Capacity remains in bytes.
	bool    fifo_count_little_endian;
	bool    fifo_temperature;
	bool    data_ready_interrupt;
	bool    clock_input;
};


} // namespace tdk_packet
