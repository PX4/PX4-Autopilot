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

#include <cstdint>

/** Register descriptions remain independent of SPI lifecycle and device objects. */
namespace tdk_packet
{

enum class Variant : uint8_t {
	kIcm40609D,
	kIcm42605,
	kIcm42670P,
	kIcm42688P,
	kIcm42686P,
	kIim42652,
	kIim42653,
	kIcm45686,
};

enum class AddressSpace : uint8_t {
	kBank0 = 0,
	kBank1 = 1,
	kBank2 = 2,
	kMreg1 = 0x11,
	kIreg = 0x12,
};

struct RegisterConfig {
	AddressSpace space      { AddressSpace::kBank0 };
	uint16_t     reg        { 0 };
	uint8_t      set_bits   { 0 };
	uint8_t      clear_bits { 0 };
};

struct Context {
	uint16_t fifo_watermark { 0 }; ///< Register watermark: records for ICM40609D/ICM45686, bytes for the other variants.
	bool     clock_input    { false }; ///< Enable the model's external reference-clock input; false selects its internal clock.
};

static constexpr uint8_t kMaxRegisterConfigs { 32 };

// Store required set/clear masks for both configuration and periodic readback checks.
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
		count = UINT8_MAX; // Reject overflow or contradictory required bits.
	}
}

// Watermarks are complete field values, not independent enable bits. Keep
// expected zeroes in the readback contract while preserving reserved bits.
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

} // namespace tdk_packet
