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

#include "TdkIcm42670PConfigs.hpp"
#include "TdkIcm42670PRegisters.hpp"

#include <cstring>

namespace tdk_icm42670p_config
{
namespace
{
using namespace tdk_icm42670p_registers;

// Fixed masks and address-space partitions are checked at compile time.
template<typename Register>
constexpr RegisterConfig entry(AddressSpace space, Register reg, uint8_t set_bits, uint8_t clear_bits)
{
	return {space, static_cast<uint16_t>(reg), set_bits, clear_bits};
}

constexpr RegisterConfig kDefaults[] {
	entry(AddressSpace::kBank0,
	      Register::BANK_0::INT_CONFIG,
	      static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	      | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	      static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::PWR_MGMT0,
	      static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	      | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	      0),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::GYRO_CONFIG0,
	      static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS_SET)
	      | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_1600HZ_SET),
	      static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS_CLEAR)
	      | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_1600HZ_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::ACCEL_CONFIG0,
	      static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_16G_SET)
	      | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_1600HZ_SET),
	      static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_16G_CLEAR)
	      | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_1600HZ_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::GYRO_CONFIG1,
	      0,
	      static_cast<uint8_t>(GYRO_CONFIG1_BIT::GYRO_UI_FILT_BW_BYPASSED_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::ACCEL_CONFIG1,
	      0,
	      static_cast<uint8_t>(ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_BW_BYPASSED_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::FIFO_CONFIG1,
	      static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_MODE_STOP_ON_FULL),
	      static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_BYPASS)),
	entry(AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG2, 0, 0), // Watermark low byte; filled by load().
	entry(AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG3, 0, 0), // Watermark high byte; filled by load().
	entry(AddressSpace::kBank0,
	      Register::BANK_0::INT_SOURCE0,
	      static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	      0),
	entry(AddressSpace::kMreg1,
	      Register::MREG1::FIFO_CONFIG5,
	      static_cast<uint8_t>(FIFO_CONFIG5_BIT::FIFO_GYRO_EN)
	      | static_cast<uint8_t>(FIFO_CONFIG5_BIT::FIFO_ACCEL_EN),
	      0),
	entry(AddressSpace::kMreg1,
	      Register::MREG1::INT_CONFIG0,
	      static_cast<uint8_t>(INT_CONFIG0_BIT::FIFO_THS_INT_CLEAR),
	      0),
};

constexpr bool validDefaults()
{
	for (unsigned i = 0; i < kRegisterCount; ++i) {
		const auto &config = kDefaults[i];
		const bool direct = i >= kDirectFirst && i < kDirectFirst + kDirectCount;

		if ((config.set_bits & config.clear_bits) != 0
		    || config.space != (direct ? AddressSpace::kBank0 : AddressSpace::kMreg1)) {
			return false;
		}
	}

	return true;
}

static_assert(sizeof(kDefaults) / sizeof(kDefaults[0]) == kRegisterCount, "Complete fixed register configuration");
static_assert(validDefaults(), "Disjoint masks and contiguous address-space partitions required");
constexpr uint8_t kWatermarkLow { 7 };
constexpr uint8_t kWatermarkHigh { 8 };
constexpr uint8_t kWatermarkHighMask { 0x0f };

static_assert(kDirectCount + kIndirectCount == kRegisterCount, "Complete address-space partitions");
static_assert(
	(kDirectFirst == 0 && kIndirectFirst == kDirectCount)
	|| (kIndirectFirst == 0 && kDirectFirst == kIndirectCount),
	"Non-overlapping contiguous address-space partitions");
static_assert(kDefaults[kWatermarkLow].reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG2),
	      "Watermark low-byte index");
static_assert(kDefaults[kWatermarkHigh].reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG3),
	      "Watermark high-byte index");

} // namespace

uint8_t load(uint16_t fifo_watermark, RegisterConfig(&config)[kRegisterCount])
{
	const uint8_t low = fifo_watermark & 0xff;
	const uint8_t high = fifo_watermark >> 8;

	if (fifo_watermark == 0 || (high & ~kWatermarkHighMask) != 0) {
		return UINT8_MAX;
	}

	memcpy(config, kDefaults, sizeof(kDefaults));

	// Watermarks are complete field values; verify zero bits as well as set bits.
	config[kWatermarkLow].set_bits = low;
	config[kWatermarkLow].clear_bits = static_cast<uint8_t>(~low);
	config[kWatermarkHigh].set_bits = high;
	config[kWatermarkHigh].clear_bits = kWatermarkHighMask & ~high;

	return kRegisterCount;
}

} // namespace tdk_icm42670p_config
