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

#include "TdkIcm45686Configs.hpp"
#include "TdkIcm45686Registers.hpp"

#include <cstring>

namespace tdk_icm45686_config
{
namespace
{
using namespace tdk_icm45686_registers;

// Fixed masks and address-space partitions are checked at compile time.
template<typename Register>
constexpr RegisterConfig entry(AddressSpace space, Register reg, uint8_t set_bits, uint8_t clear_bits)
{
	return {space, static_cast<uint16_t>(reg), set_bits, clear_bits};
}

constexpr RegisterConfig kDefaults[] {
	entry(AddressSpace::kIreg,
	      Register::IREG::IPREG_SYS1_REG_166,
	      static_cast<uint8_t>(IPREG_SYS1_REG_166_BIT::GYRO_SRC_CTRL_INTERP_AAF_SET),
	      static_cast<uint8_t>(IPREG_SYS1_REG_166_BIT::GYRO_SRC_CTRL_INTERP_AAF_CLEAR)),
	entry(AddressSpace::kIreg,
	      Register::IREG::IPREG_SYS2_REG_123,
	      static_cast<uint8_t>(IPREG_SYS2_REG_123_BIT::ACCEL_SRC_CTRL_INTERP_AAF_SET),
	      static_cast<uint8_t>(IPREG_SYS2_REG_123_BIT::ACCEL_SRC_CTRL_INTERP_AAF_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::INT1_CONFIG0,
	      static_cast<uint8_t>(INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS),
	      static_cast<uint8_t>(~static_cast<uint8_t>(INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS))),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::INT1_CONFIG2,
	      0,
	      static_cast<uint8_t>(INT1_CONFIG2_BIT::INT1_DRIVE)
	      | static_cast<uint8_t>(INT1_CONFIG2_BIT::INT1_MODE)
	      | static_cast<uint8_t>(INT1_CONFIG2_BIT::INT1_POLARITY)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::PWR_MGMT0,
	      static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	      | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	      0),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::GYRO_CONFIG0,
	      static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_SET)
	      | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_SET),
	      static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_CLEAR)
	      | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::ACCEL_CONFIG0,
	      static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_SET)
	      | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_SET),
	      static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_CLEAR)
	      | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_CLEAR)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::FIFO_CONFIG4,
	      static_cast<uint8_t>(FIFO_CONFIG4_BIT::FIFO_TMST_FSYNC_EN),
	      static_cast<uint8_t>(FIFO_CONFIG4_BIT::FIFO_COMP_EN)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::FIFO_CONFIG0,
	      static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET)
	      | static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET),
	      static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLEAR)
	      | static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_CLEAR)),
	entry(AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1_0, 0, 0), // Watermark low byte; filled by load().
	entry(AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1_1, 0, 0), // Watermark high byte; filled by load().
	entry(AddressSpace::kBank0,
	      Register::BANK_0::FIFO_CONFIG2,
	      static_cast<uint8_t>(FIFO_CONFIG2_BIT::FIFO_WR_WM_EQ_OR_GT_TH),
	      0),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::FIFO_CONFIG3,
	      static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_GYRO_EN)
	      | static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_ACCEL_EN)
	      | static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_IF_EN),
	      static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_HIRES_EN)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::RTC_CONFIG,
	      0,
	      static_cast<uint8_t>(RTC_CONFIG_BIT::RTC_MODE)),
	entry(AddressSpace::kBank0,
	      Register::BANK_0::IOC_PAD_SCENARIO_OVRD,
	      0,
	      0),
};

constexpr bool validDefaults()
{
	for (unsigned i = 0; i < kRegisterCount; ++i) {
		const auto &config = kDefaults[i];
		const bool direct = i >= kDirectFirst && i < kDirectFirst + kDirectCount;

		if ((config.set_bits & config.clear_bits) != 0
		    || config.space != (direct ? AddressSpace::kBank0 : AddressSpace::kIreg)) {
			return false;
		}
	}

	return true;
}

static_assert(sizeof(kDefaults) / sizeof(kDefaults[0]) == kRegisterCount, "Complete fixed register configuration");
static_assert(validDefaults(), "Disjoint masks and contiguous address-space partitions required");
constexpr uint8_t kWatermarkLow { 9 };
constexpr uint8_t kWatermarkHigh { 10 };
constexpr uint8_t kWatermarkHighMask { 0xff };

static_assert(kDirectCount + kIndirectCount == kRegisterCount, "Complete address-space partitions");
static_assert(
	(kDirectFirst == 0 && kIndirectFirst == kDirectCount)
	|| (kIndirectFirst == 0 && kDirectFirst == kIndirectCount),
	"Non-overlapping contiguous address-space partitions");
static_assert(kDefaults[kWatermarkLow].reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG1_0),
	      "Watermark low-byte index");
static_assert(kDefaults[kWatermarkHigh].reg == static_cast<uint16_t>(Register::BANK_0::FIFO_CONFIG1_1),
	      "Watermark high-byte index");

} // namespace

uint8_t load(uint16_t fifo_watermark, bool clock_input, RegisterConfig(&config)[kRegisterCount])
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

	constexpr uint8_t kRtcConfig { 13 };
	constexpr uint8_t kPadConfig { 14 };
	constexpr uint8_t kRtcMode { static_cast<uint8_t>(RTC_CONFIG_BIT::RTC_MODE) };
	constexpr uint8_t kClockPad {
		static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD)
		| static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD_CLKIN)
	};

	static_assert(kDefaults[kRtcConfig].reg == static_cast<uint16_t>(Register::BANK_0::RTC_CONFIG),
		      "Reference-clock register index");
	static_assert(kDefaults[kPadConfig].reg == static_cast<uint16_t>(Register::BANK_0::IOC_PAD_SCENARIO_OVRD),
		      "Clock-input pad register index");

	config[kRtcConfig].set_bits = clock_input ? kRtcMode : 0;
	config[kRtcConfig].clear_bits = clock_input ? 0 : kRtcMode;
	config[kPadConfig].set_bits = clock_input ? kClockPad : 0;

	return kRegisterCount;
}

} // namespace tdk_icm45686_config
