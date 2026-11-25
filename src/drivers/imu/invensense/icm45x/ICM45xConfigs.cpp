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
 * @file ICM45xConfigs.cpp
 * @brief BANK_0 and IREG configuration builders for ICM-45686 and ICM-56686.
 */

#include "ICM45xConfigs.hpp"
#include "ICM45xRegisters.hpp"

#include <px4_platform_common/px4_config.h>

namespace icm45x_config
{
namespace
{

template<typename Reg>
void addBank0(
	Bank0Config(&config)[kMaxBank0Configs],
	uint8_t &count,
	Reg reg,
	uint8_t set_bits,
	uint8_t clear_bits)
{
	if (count < kMaxBank0Configs && (set_bits & clear_bits) == 0) {
		config[count++] = {static_cast<uint8_t>(reg), set_bits, clear_bits};

	} else {
		count = UINT8_MAX;
	}
}

template<typename Reg>
void addBank0Masked(
	Bank0Config(&config)[kMaxBank0Configs],
	uint8_t &count,
	Reg reg,
	uint8_t value,
	uint8_t mask)
{
	addBank0(config, count, reg, value & mask, static_cast<uint8_t>(~value) & mask);
}

void addIreg(
	IregConfig(&config)[kMaxIregConfigs],
	uint8_t &count,
	uint16_t reg,
	uint8_t set_bits,
	uint8_t clear_bits)
{
	if (count < kMaxIregConfigs && (set_bits & clear_bits) == 0) {
		config[count++] = {reg, set_bits, clear_bits};

	} else {
		count = UINT8_MAX;
	}
}

uint8_t loadIcm45686Bank0(const Context &context, Bank0Config(&config)[kMaxBank0Configs])
{
	if (context.fifo_watermark == 0 || context.int2) {
		return UINT8_MAX;
	}

	using namespace icm45x_registers;
	using namespace icm45x_registers::icm45686;

	uint8_t count = 0;

	const uint8_t int_sources = context.has_drdy_gpio
				    ? static_cast<uint8_t>(INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS)
				    : 0;
	const uint8_t wm_low  = context.fifo_watermark & 0xFF;
	const uint8_t wm_high = (context.fifo_watermark >> 8) & 0xFF;

	// Route FIFO watermark interrupt to INT1: push-pull, pulsed, active-low.
	addBank0Masked(config, count, BANK_0::INT1_CONFIG0, int_sources, 0xFF);
	addBank0(config, count, BANK_0::INT1_CONFIG2,
		 0,
		 INT1_CONFIG2_BIT::INT1_DRIVE | INT1_CONFIG2_BIT::INT1_MODE | INT1_CONFIG2_BIT::INT1_POLARITY);
	addBank0(config, count, BANK_0::PWR_MGMT0,
		 common::PWR_MGMT0_LOW_NOISE,
		 0);
	addBank0(config, count, BANK_0::GYRO_CONFIG0,
		 GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_SET | GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_SET,
		 GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_CLEAR | GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_CLEAR);
	addBank0(config, count, BANK_0::ACCEL_CONFIG0,
		 ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_SET | ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_SET,
		 ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_CLEAR | ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_CLEAR);
	addBank0(config, count, BANK_0::FIFO_CONFIG4,
		 FIFO_CONFIG4_BIT::FIFO_TMST_FSYNC_EN,
		 FIFO_CONFIG4_BIT::FIFO_COMP_EN);
	// DS-000577 Section 17.31: FIFO_WR_WM_GT_TH must be written while FIFO_MODE is bypass.
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG1_0, wm_low, 0xFF);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG1_1, wm_high, 0xFF);
	addBank0(config, count, BANK_0::FIFO_CONFIG2,
		 FIFO_CONFIG2_BIT::FIFO_WR_WM_EQ_OR_GT_TH,
		 0);
	addBank0(config, count, BANK_0::FIFO_CONFIG0,
		 FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET | FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET,
		 FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLR | FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_CLEAR);
	addBank0(config, count, BANK_0::FIFO_CONFIG3,
		 FIFO_CONFIG3_BIT::FIFO_GYRO_EN | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN | FIFO_CONFIG3_BIT::FIFO_IF_EN,
		 FIFO_CONFIG3_BIT::FIFO_HIRES_EN);
	addBank0(config, count, BANK_0::RTC_CONFIG,
		 context.clock_input ? static_cast<uint8_t>(RTC_CONFIG_BIT::RTC_MODE) : 0,
		 context.clock_input ? 0 : static_cast<uint8_t>(RTC_CONFIG_BIT::RTC_MODE));
	addBank0(config, count, BANK_0::IOC_PAD_SCENARIO_OVRD,
		 context.clock_input
		 ? static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD
					| IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD_CLKIN)
		 : 0,
		 context.clock_input
		 ? static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD_FSYNC)
		 : static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD
					| IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD_CLKIN
					| IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD_FSYNC));

	return count;
}

uint8_t loadIcm45686Ireg(const Context &context, IregConfig(&config)[kMaxIregConfigs])
{
	using namespace icm45x_registers::icm45686;

	uint8_t count = 0;

	addIreg(config, count, IREG::SREG_CTRL, 0, SREG_CTRL_BIT::SREG_DATA_ENDIAN_SEL);
	// DS-000489 Sections 16.37, 19.47: ACCEL_LP_CLK_SEL (0xA258 bit 4) must be 1 whenever RTC_MODE is enabled.
	addIreg(config, count, IREG::SMC_CONTROL_0,
		context.clock_input ? static_cast<uint8_t>(SMC_CONTROL_0_BIT::ACCEL_LP_CLK_SEL) : 0,
		context.clock_input ? 0 : static_cast<uint8_t>(SMC_CONTROL_0_BIT::ACCEL_LP_CLK_SEL));
	addIreg(config, count, IREG::IPREG_SYS1_REG_166,
		IPREG_SYS1_REG_166_BIT::GYRO_SRC_CTRL_INTERP_AAF_SET,
		IPREG_SYS1_REG_166_BIT::GYRO_SRC_CTRL_INTERP_AAF_CLEAR);
	addIreg(config, count, IREG::IPREG_SYS2_REG_123,
		IPREG_SYS2_REG_123_BIT::ACCEL_SRC_CTRL_INTERP_AAF_SET,
		IPREG_SYS2_REG_123_BIT::ACCEL_SRC_CTRL_INTERP_AAF_CLEAR);

	return count;
}

uint8_t loadIcm56686Bank0(const Context &context, Bank0Config(&config)[kMaxBank0Configs])
{
	if (context.fifo_watermark == 0 || context.clock_input) {
		return UINT8_MAX;
	}

	using namespace icm45x_registers;
	using namespace icm45x_registers::icm56686;

	uint8_t count = 0;

	const BANK_0 int_cfg0 = context.int2 ? BANK_0::INT2_CONFIG0 : BANK_0::INT1_CONFIG0;
	const BANK_0 int_cfg2 = context.int2 ? BANK_0::INT2_CONFIG2 : BANK_0::INT1_CONFIG2;

	const uint8_t int_sources = context.has_drdy_gpio
				    ? static_cast<uint8_t>(INT_CONFIG0_BIT::STATUS_EN_FIFO_THS | INT_CONFIG0_BIT::STATUS_EN_FIFO_FULL)
				    : 0;
	const uint8_t wm_low  = context.fifo_watermark & 0xFF;
	const uint8_t wm_high = (context.fifo_watermark >> 8) & 0xFF;

	addBank0Masked(config, count, int_cfg0, int_sources, 0xFF);
	addBank0Masked(config, count, int_cfg2, INT_CONFIG2_BIT::ACTIVE_HIGH, INT_CONFIG2_BIT::CONFIG_MASK);
	addBank0Masked(config, count, BANK_0::PWR_MGMT0, common::PWR_MGMT0_LOW_NOISE, 0x0F);
	addBank0Masked(config, count, BANK_0::ACCEL_CONFIG0,
		       ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G | ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ,
		       ACCEL_CONFIG0_BIT::ACCEL_CONFIG0_MASK);
	addBank0Masked(config, count, BANK_0::GYRO_CONFIG0,
		       GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS | GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ,
		       GYRO_CONFIG0_BIT::GYRO_CONFIG0_MASK);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG0,
		       FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL | FIFO_CONFIG0_BIT::FIFO_DEPTH_2K,
		       0xFF);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG1_0, wm_low, 0xFF);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG1_1, wm_high, 0xFF);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG2,
		       FIFO_CONFIG2_BIT::FIFO_WR_WM_GT_TH | FIFO_CONFIG2_BIT::FIFO_RESERVED_RESET,
		       FIFO_CONFIG2_BIT::FIFO_CONFIG2_MASK);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG3,
		       FIFO_CONFIG3_BIT::FIFO_HIRES_EN | FIFO_CONFIG3_BIT::FIFO_GYRO_EN
		       | FIFO_CONFIG3_BIT::FIFO_ACCEL_EN | FIFO_CONFIG3_BIT::FIFO_IF_EN,
		       FIFO_CONFIG3_BIT::FIFO_CONFIG3_MASK);
	addBank0Masked(config, count, BANK_0::FIFO_CONFIG4,
		       FIFO_CONFIG4_BIT::FIFO_TMST_FSYNC_EN,
		       FIFO_CONFIG4_BIT::FIFO_CONFIG4_MASK);

	return count;
}

uint8_t loadIcm56686Ireg(const Context &context, IregConfig(&config)[kMaxIregConfigs])
{
	using namespace icm45x_registers::icm56686;

	uint8_t count = 0;

	const uint16_t pulse_min_on  = context.int2 ? IREG::INT_PULSE_MIN_ON_INTF1 : IREG::INT_PULSE_MIN_ON_INTF0;
	const uint16_t pulse_min_off = context.int2 ? IREG::INT_PULSE_MIN_OFF_INTF1 : IREG::INT_PULSE_MIN_OFF_INTF0;

	addIreg(config, count, IREG::SREG_CTRL,
		SREG_CTRL_BIT::SREG_SIFS_20BITS_EN,
		SREG_CTRL_BIT::SREG_DATA_ENDIAN_BIG);
	addIreg(config, count, IREG::SMC_CONTROL_0,
		SMC_CONTROL_0_BIT::TMST_EN,
		0);
	addIreg(config, count, pulse_min_on,
		INTF0_PULSE_BIT::DURATION_8_US,
		static_cast<uint8_t>(INTF0_PULSE_BIT::DURATION_MASK & ~INTF0_PULSE_BIT::DURATION_8_US));
	addIreg(config, count, pulse_min_off,
		INTF0_PULSE_BIT::DURATION_8_US,
		static_cast<uint8_t>(INTF0_PULSE_BIT::DURATION_MASK & ~INTF0_PULSE_BIT::DURATION_8_US));
	addIreg(config, count, IREG::GYRO_SRC_CTRL,
		GYRO_SRC_CTRL_BIT::GYRO_SRC_CTRL_SRC_ON_PREFILTER_ON,
		static_cast<uint8_t>(GYRO_SRC_CTRL_BIT::GYRO_SRC_CTRL_MASK
				     & ~GYRO_SRC_CTRL_BIT::GYRO_SRC_CTRL_SRC_ON_PREFILTER_ON));
	addIreg(config, count, IREG::GYRO_UI_LPF_CFG,
		GYRO_UI_LPF_CFG_BIT::GYRO_UI_LPFBW_ODR_DIV_2,
		GYRO_UI_LPF_CFG_BIT::GYRO_UI_LPFBW_MASK);
	addIreg(config, count, IREG::ACCEL_SRC_CTRL,
		ACCEL_SRC_CTRL_BIT::ACCEL_SRC_CTRL_SRC_ON_PREFILTER_ON,
		static_cast<uint8_t>(ACCEL_SRC_CTRL_BIT::ACCEL_SRC_CTRL_MASK
				     & ~ACCEL_SRC_CTRL_BIT::ACCEL_SRC_CTRL_SRC_ON_PREFILTER_ON));
	addIreg(config, count, IREG::ACCEL_UI_LPF_CFG,
		ACCEL_UI_LPF_CFG_BIT::ACCEL_UI_LPFBW_ODR_DIV_2,
		ACCEL_UI_LPF_CFG_BIT::ACCEL_UI_LPFBW_MASK);

	return count;
}

} // namespace

uint8_t loadBank0(Variant variant, const Context &context, Bank0Config(&config)[kMaxBank0Configs])
{
	switch (variant) {
	case Variant::kIcm45686: {
			return loadIcm45686Bank0(context, config);
		}

	case Variant::kIcm56686: {
			return loadIcm56686Bank0(context, config);
		}

	default: {
			return 0;
		}
	}
}

uint8_t loadIreg(Variant variant, const Context &context, IregConfig(&config)[kMaxIregConfigs])
{
	switch (variant) {
	case Variant::kIcm45686: {
			return loadIcm45686Ireg(context, config);
		}

	case Variant::kIcm56686: {
			return loadIcm56686Ireg(context, config);
		}

	default: {
			return 0;
		}
	}
}

} // namespace icm45x_config
