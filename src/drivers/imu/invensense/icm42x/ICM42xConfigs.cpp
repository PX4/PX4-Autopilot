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
 * @file ICM42xConfigs.cpp
 * @brief Bounded register configuration tables for TDK ICM42x SPI IMU models.
 */

#include "ICM42xConfigs.hpp"

#include "../InvenSense_AAF.hpp"
#include "ICM42xRegisters.hpp"

namespace icm42x_config
{
namespace
{

bool applyAntiAliasFilter(
	uint16_t aaf_hz,
	RegisterConfig(&config)[kMaxRegisterConfigs],
	uint8_t count)
{
	if (aaf_hz == 0) {
		return true;
	}

	const InvenSense_AAF::Preset *p = InvenSense_AAF::preset(aaf_hz);

	if (p == nullptr) {
		return false;
	}

	using namespace icm42x_registers;

	const auto set = [](RegisterConfig & entry, uint8_t value, uint8_t mask) {
		entry.set_bits   = value & mask;
		entry.clear_bits = static_cast<uint8_t>(~value) & mask;
	};

	for (uint8_t i = 0; i < count; ++i) {
		RegisterConfig &r = config[i];

		if (r.space == AddressSpace::kBank0) {
			switch (static_cast<Register::BANK_0>(r.reg)) {
			case Register::BANK_0::GYRO_CONFIG1:
				set(r, Bit3, static_cast<uint8_t>(GYRO_CONFIG1_BIT::GYRO_UI_FILT_ORD)); // 3rd order
				break;

			case Register::BANK_0::ACCEL_CONFIG1:
				set(r, Bit4, static_cast<uint8_t>(ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_ORD)); // 3rd order
				break;

			case Register::BANK_0::GYRO_ACCEL_CONFIG0:
				set(r,
				    static_cast<uint8_t>((p->ui_filt_bw << 4) | p->ui_filt_bw),
				    static_cast<uint8_t>(GYRO_ACCEL_CONFIG0_BIT::ACCEL_UI_FILT_BW)
				    | static_cast<uint8_t>(GYRO_ACCEL_CONFIG0_BIT::GYRO_UI_FILT_BW));
				break;

			default:
				break;
			}

		} else if (r.space == AddressSpace::kBank1) {
			switch (static_cast<Register::BANK_1>(r.reg)) {
			case Register::BANK_1::GYRO_CONFIG_STATIC3:
				set(r, p->delt, 0x3F);
				break;

			case Register::BANK_1::GYRO_CONFIG_STATIC4:
				set(r, static_cast<uint8_t>(p->deltsqr & 0xFF), 0xFF);
				break;

			case Register::BANK_1::GYRO_CONFIG_STATIC5:
				set(r, static_cast<uint8_t>((p->bitshift << 4) | ((p->deltsqr >> 8) & 0x0F)), 0xFF);
				break;

			default:
				break;
			}

		} else if (r.space == AddressSpace::kBank2) {
			switch (static_cast<Register::BANK_2>(r.reg)) {
			case Register::BANK_2::ACCEL_CONFIG_STATIC2:
				set(r, static_cast<uint8_t>(p->delt << 1), 0x7F); // bit 0 (AAF_DIS) stays cleared
				break;

			case Register::BANK_2::ACCEL_CONFIG_STATIC3:
				set(r, static_cast<uint8_t>(p->deltsqr & 0xFF), 0xFF);
				break;

			case Register::BANK_2::ACCEL_CONFIG_STATIC4:
				set(r, static_cast<uint8_t>((p->bitshift << 4) | ((p->deltsqr >> 8) & 0x0F)), 0xFF);
				break;

			default:
				break;
			}
		}
	}

	return true;
}

uint8_t loadHighResolution(
	Variant variant,
	const Context &context,
	RegisterConfig(&config)[kMaxRegisterConfigs])
{
	using namespace icm42x_registers;

	uint8_t count = 0;

	// Active-low, push-pull, latched INT1; the FIFO watermark drives acquisition.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG,
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG,
	    static_cast<uint8_t>(FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL),
	    0);

	add(config, count, AddressSpace::kBank0, Register::BANK_0::INTF_CONFIG0,
	    static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_ENDIAN)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::SENSOR_DATA_ENDIAN)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::UI_SIFS_CFG_DISABLE_I2C),
	    static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_HOLD_LAST_DATA_EN)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_REC));

	// IIM42653 also selects the clock source explicitly when using the external clock input.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INTF_CONFIG1,
	    static_cast<uint8_t>(INTF_CONFIG1_BIT::AFSR_SET)
	    | (context.clock_input ? static_cast<uint8_t>(INTF_CONFIG1_BIT::RTC_MODE) : 0)
	    | ((context.clock_input && variant == Variant::kIim42653) ? static_cast<uint8_t>(INTF_CONFIG1_BIT::CLKSEL) : 0),
	    static_cast<uint8_t>(INTF_CONFIG1_BIT::AFSR_CLEAR)
	    | (context.clock_input ? 0 : static_cast<uint8_t>(INTF_CONFIG1_BIT::RTC_MODE))
	    | ((context.clock_input && variant == Variant::kIim42653) ? static_cast<uint8_t>(INTF_CONFIG1_BIT::CLKSEL_CLEAR) : 0));

	// Match both channel rates and ranges to the high-resolution packet profile.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::PWR_MGMT0,
	    static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	    | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG0,
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_FS_SEL_MAX)
	    | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_SET),
	    Bit7 | Bit6 | Bit5 | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG0,
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_FS_SEL_MAX)
	    | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_SET),
	    Bit7 | Bit6 | Bit5 | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG1,
	    0,
	    static_cast<uint8_t>(GYRO_CONFIG1_BIT::GYRO_UI_FILT_ORD));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_ACCEL_CONFIG0,
	    0,
	    static_cast<uint8_t>(GYRO_ACCEL_CONFIG0_BIT::ACCEL_UI_FILT_BW)
	    | static_cast<uint8_t>(GYRO_ACCEL_CONFIG0_BIT::GYRO_UI_FILT_BW));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG1,
	    0,
	    static_cast<uint8_t>(ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_ORD));

	// Include delta timestamps and temperature in the high-resolution inertial packets.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::TMST_CONFIG,
	    static_cast<uint8_t>(TMST_CONFIG_BIT::TMST_EN)
	    | static_cast<uint8_t>(TMST_CONFIG_BIT::TMST_DELTA_EN)
	    | static_cast<uint8_t>(TMST_CONFIG_BIT::TMST_TO_REGS_EN)
	    | static_cast<uint8_t>(TMST_CONFIG_BIT::TMST_RES),
	    static_cast<uint8_t>(TMST_CONFIG_BIT::TMST_FSYNC_EN));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1,
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_WM_GT_TH)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_HIRES_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_TEMP_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_GYRO_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_ACCEL_EN),
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_TMST_FSYNC_EN));

	// The watermark is expressed in bytes for these profiles.
	addWatermark(config, count, Register::BANK_0::FIFO_CONFIG2, Register::BANK_0::FIFO_CONFIG3,
		     context.fifo_watermark, 0x0f); // FIFO_WM[11:8] occupies the low nibble.

	// Clear the latched interrupt on FIFO reads, disable async reset per datasheet, and route the watermark to INT1.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG0,
	    static_cast<uint8_t>(INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG1,
	    0,
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_ASYNC_RESET));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_SOURCE0,
	    static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	    0);

	// Keep the gyro notch and anti-alias filters enabled with the default 585 Hz configuration.
	add(config, count, AddressSpace::kBank1, Register::BANK_1::GYRO_CONFIG_STATIC2,
	    0,
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC2_BIT::GYRO_NF_DIS)
	    | static_cast<uint8_t>(GYRO_CONFIG_STATIC2_BIT::GYRO_AAF_DIS));
	add(config, count, AddressSpace::kBank1, Register::BANK_1::GYRO_CONFIG_STATIC3,
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC3_BIT::GYRO_AAF_DELT_585HZ_SET),
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC3_BIT::GYRO_AAF_DELT_585HZ_CLEAR));
	add(config, count, AddressSpace::kBank1, Register::BANK_1::GYRO_CONFIG_STATIC4,
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC4_BIT::GYRO_AAF_DELTSQR_LSB_585HZ_SET),
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC4_BIT::GYRO_AAF_DELTSQR_LSB_585HZ_CLEAR));
	add(config, count, AddressSpace::kBank1, Register::BANK_1::GYRO_CONFIG_STATIC5,
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_BITSHIFT_585HZ_SET)
	    | static_cast<uint8_t>(GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_DELTSQR_MSB_585HZ_SET),
	    static_cast<uint8_t>(GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_BITSHIFT_585HZ_CLEAR)
	    | static_cast<uint8_t>(GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_DELTSQR_MSB_585HZ_CLEAR));

	// Route Pin 9 to CLKIN when external clock is enabled, otherwise keep Pin 9 at default INT2 (00).
	add(config, count, AddressSpace::kBank1, Register::BANK_1::INTF_CONFIG5,
	    context.clock_input ? static_cast<uint8_t>(INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_SET) : 0,
	    context.clock_input
	    ? static_cast<uint8_t>(INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_CLEAR)
	    : static_cast<uint8_t>(INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_SET)
	    | static_cast<uint8_t>(INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_CLEAR));

	// Apply the matching accelerometer anti-alias filter settings.
	add(config, count, AddressSpace::kBank2, Register::BANK_2::ACCEL_CONFIG_STATIC2,
	    static_cast<uint8_t>(ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DELT_585HZ_SET),
	    static_cast<uint8_t>(ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DELT_585HZ_CLEAR)
	    | static_cast<uint8_t>(ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DIS));
	add(config, count, AddressSpace::kBank2, Register::BANK_2::ACCEL_CONFIG_STATIC3,
	    static_cast<uint8_t>(ACCEL_CONFIG_STATIC3_BIT::ACCEL_AAF_DELTSQR_LSB_585HZ_SET),
	    static_cast<uint8_t>(ACCEL_CONFIG_STATIC3_BIT::ACCEL_AAF_DELTSQR_LSB_585HZ_CLEAR));
	add(config, count, AddressSpace::kBank2, Register::BANK_2::ACCEL_CONFIG_STATIC4,
	    static_cast<uint8_t>(ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_BITSHIFT_585HZ_SET)
	    | static_cast<uint8_t>(ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_DELTSQR_MSB_SET),
	    static_cast<uint8_t>(ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_BITSHIFT_585HZ_CLEAR)
	    | static_cast<uint8_t>(ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_DELTSQR_MSB_CLEAR));

	if (variant == Variant::kIim42653) {
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_CONFIG1,
		    0,
		    static_cast<uint8_t>(AUX1_CONFIG1_BIT::AUX1_ACCEL_LP_CLK_SEL)
		    | static_cast<uint8_t>(AUX1_CONFIG1_BIT::GYRO_AUX1_EN)
		    | static_cast<uint8_t>(AUX1_CONFIG1_BIT::ACCEL_AUX1_EN));
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_CONFIG2,
		    static_cast<uint8_t>(AUX1_CONFIG2_BIT::GYRO_AUX1_HPF_DIS),
		    0);
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_SPI_REG1,
		    static_cast<uint8_t>(AUX1_SPI_REG1_BIT::AUX1_SPI_REG1_SET),
		    static_cast<uint8_t>(AUX1_SPI_REG1_BIT::AUX1_SPI_REG1_CLEAR));
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_SPI_REG2,
		    static_cast<uint8_t>(AUX1_SPI_REG2_BIT::AUX1_SPI_REG2_SET),
		    static_cast<uint8_t>(AUX1_SPI_REG2_BIT::AUX1_SPI_REG2_CLEAR));
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_SPI_REG3,
		    static_cast<uint8_t>(AUX1_SPI_REG3_BIT::AUX1_SPI_REG3_SET),
		    static_cast<uint8_t>(AUX1_SPI_REG3_BIT::AUX1_SPI_REG3_CLEAR));
	}

	if (count > kMaxRegisterConfigs || !applyAntiAliasFilter(context.aaf_hz, config, count)) {
		return UINT8_MAX;
	}

	return count;
}

uint8_t loadStandard16(
	Variant variant,
	const Context &context,
	RegisterConfig(&config)[kMaxRegisterConfigs])
{
	if (context.aaf_hz != 0) {
		return UINT8_MAX;
	}

	using namespace icm42x_registers;

	uint8_t count = 0;

	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG,
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY));

	const bool count_records = variant == Variant::kIcm40609D;
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INTF_CONFIG0,
	    (count_records ? static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_REC) : 0)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_ENDIAN)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::SENSOR_DATA_ENDIAN)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::UI_SIFS_CFG_DISABLE_I2C),
	    static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_HOLD_LAST_DATA_EN)
	    | (count_records ? 0 : static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_REC)));

	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG,
	    static_cast<uint8_t>(FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL),
	    0);

	// Use the 8 kHz low-noise inertial stream without FIFO temperature.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::PWR_MGMT0,
	    static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	    | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG0,
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_SET),
	    Bit7 | Bit6 | Bit5 | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_8KHZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG0,
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_SET),
	    Bit7 | Bit6 | Bit5 | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_8KHZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1,
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_WM_GT_TH)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_GYRO_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_ACCEL_EN),
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_TEMP_EN));

	addWatermark(config, count, Register::BANK_0::FIFO_CONFIG2, Register::BANK_0::FIFO_CONFIG3,
		     context.fifo_watermark, 0x0f);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG0,
	    static_cast<uint8_t>(INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ),
	    0);

	if (variant == Variant::kIcm42605) {
		add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG1,
		    0,
		    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_ASYNC_RESET));
	}

	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_SOURCE0,
	    static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	    0);

	return count;
}

} // namespace

uint8_t load(Variant variant, const Context &context, RegisterConfig(&config)[kMaxRegisterConfigs])
{
	switch (variant) {
	case Variant::kIcm40609D:
	case Variant::kIcm42605: {
			return loadStandard16(variant, context, config);
		}

	case Variant::kIcm42688P:
	case Variant::kIcm42686P:
	case Variant::kIim42652:
	case Variant::kIim42653: {
			return loadHighResolution(variant, context, config);
		}

	default: {
			return 0;
		}
	}
}

} // namespace icm42x_config
