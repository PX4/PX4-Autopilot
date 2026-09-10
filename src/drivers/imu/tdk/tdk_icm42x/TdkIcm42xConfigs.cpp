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

#include "TdkIcm42xConfigs.hpp"
#include <px4_platform_common/px4_config.h>
#include "TdkIcm42xRegisters.hpp"
#include "registers/TdkICM40609DRegisters.hpp"
#include "registers/TdkICM42605Registers.hpp"
#include "registers/TdkICM42670PRegisters.hpp"
#include "registers/TdkICM45686Registers.hpp"

namespace tdk_icm42x_config
{
namespace
{

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

#if defined(CONFIG_TDK_ICM42X_ICM42688P) \
	|| defined(CONFIG_TDK_ICM42X_ICM42686P) \
	|| defined(CONFIG_TDK_ICM42X_IIM42652) \
	|| defined(CONFIG_TDK_ICM42X_IIM42653)
uint8_t loadHighResolution(
	Variant variant,
	const Context &context,
	RegisterConfig(&config)[kMaxRegisterConfigs])
{
	using namespace tdk_icm42x_registers;

	uint8_t count = 0;

	// Active-low, push-pull, latched INT1; the FIFO watermark drives acquisition.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG,
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG,
	    static_cast<uint8_t>(FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL),
	    0);

	if (variant == Variant::kIcm42688P || variant == Variant::kIcm42686P) {
		add(config, count, AddressSpace::kBank0, Register::BANK_0::INTF_CONFIG0,
		    static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_ENDIAN)
		    | static_cast<uint8_t>(INTF_CONFIG0_BIT::SENSOR_DATA_ENDIAN)
		    | static_cast<uint8_t>(INTF_CONFIG0_BIT::UI_SIFS_CFG_DISABLE_I2C),
		    0);
	}

	// IIM42653 also selects the clock source explicitly when using the external clock input.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INTF_CONFIG1,
	    static_cast<uint8_t>(INTF_CONFIG1_BIT::AFSR_SET)
	    | (context.clock_input ? static_cast<uint8_t>(INTF_CONFIG1_BIT::RTC_MODE) : 0)
	    | ((context.clock_input && variant == Variant::kIim42653) ? static_cast<uint8_t>(INTF_CONFIG1_BIT::CLKSEL) : 0),
	    static_cast<uint8_t>(INTF_CONFIG1_BIT::AFSR_CLEAR)
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

	// Clear the latched interrupt on FIFO reads and route the watermark to INT1.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG0,
	    static_cast<uint8_t>(INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG1,
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_TPULSE_DURATION)
	    | static_cast<uint8_t>(INT_CONFIG1_BIT::INT_TDEASSERT_DISABLE),
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_ASYNC_RESET));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_SOURCE0,
	    static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	    0);

	// Keep the gyro notch and anti-alias filters enabled with the existing 585 Hz configuration.
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

	// Route the optional external clock input without changing the internal-clock pin configuration.
	add(config, count, AddressSpace::kBank1, Register::BANK_1::INTF_CONFIG5,
	    context.clock_input ? static_cast<uint8_t>(INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_SET) : 0,
	    context.clock_input ? static_cast<uint8_t>(INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_CLEAR) : 0);

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
	}

	if (variant == Variant::kIim42653) {
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_CONFIG2,
		    static_cast<uint8_t>(AUX1_CONFIG2_BIT::GYRO_AUX1_HPF_DIS),
		    0);
	}

	if (variant == Variant::kIim42653) {
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_SPI_REG1,
		    static_cast<uint8_t>(AUX1_SPI_REG1_BIT::AUX1_SPI_REG1_SET),
		    static_cast<uint8_t>(AUX1_SPI_REG1_BIT::AUX1_SPI_REG1_CLEAR));
	}

	if (variant == Variant::kIim42653) {
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_SPI_REG2,
		    static_cast<uint8_t>(AUX1_SPI_REG2_BIT::AUX1_SPI_REG2_SET),
		    static_cast<uint8_t>(AUX1_SPI_REG2_BIT::AUX1_SPI_REG2_CLEAR));
	}

	if (variant == Variant::kIim42653) {
		add(config, count, AddressSpace::kBank2, Register::BANK_2::AUX1_SPI_REG3,
		    static_cast<uint8_t>(AUX1_SPI_REG3_BIT::AUX1_SPI_REG3_SET),
		    static_cast<uint8_t>(AUX1_SPI_REG3_BIT::AUX1_SPI_REG3_CLEAR));
	}

	return count;
}
#endif // high-resolution model selections

#if defined(CONFIG_TDK_ICM42X_ICM40609D)
uint8_t loadIcm40609D(const Context &context, RegisterConfig(&config)[kMaxRegisterConfigs])
{
	using namespace tdk_icm40609d_registers;

	uint8_t count = 0;

	// Configure INT1 and report FIFO counts in records.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG,
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INTF_CONFIG0,
	    static_cast<uint8_t>(INTF_CONFIG0_BIT::FIFO_COUNT_REC)
	    | static_cast<uint8_t>(INTF_CONFIG0_BIT::UI_SIFS_CFG_DISABLE_I2C),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG,
	    static_cast<uint8_t>(FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL),
	    0);

	// Use the 8 kHz low-noise inertial stream without FIFO temperature.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::PWR_MGMT0,
	    static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	    | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG0,
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_8kHz),
	    Bit7 | Bit6 | Bit5 | Bit3 | Bit2);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG0,
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_8kHz),
	    Bit7 | Bit6 | Bit5 | Bit3 | Bit2);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1,
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_WM_GT_TH)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_GYRO_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_ACCEL_EN),
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_TEMP_EN));

	// This model uses a record-count watermark, not a byte-count watermark.
	addWatermark(config, count, Register::BANK_0::FIFO_CONFIG2, Register::BANK_0::FIFO_CONFIG3,
		     context.fifo_watermark, 0x0f);

	// Reading the FIFO clears the latched watermark interrupt.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG0,
	    static_cast<uint8_t>(INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG1,
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_TPULSE_DURATION)
	    | static_cast<uint8_t>(INT_CONFIG1_BIT::INT_TDEASSERT_DISABLE),
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_ASYNC_RESET));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_SOURCE0,
	    static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	    0);

	return count;
}
#endif // CONFIG_TDK_ICM42X_ICM40609D

#if defined(CONFIG_TDK_ICM42X_ICM42605)
uint8_t loadIcm42605(const Context &context, RegisterConfig(&config)[kMaxRegisterConfigs])
{
	using namespace tdk_icm42605_registers;

	uint8_t count = 0;

	// Configure the latched INT1 signal and stop-on-full FIFO mode.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG,
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG,
	    static_cast<uint8_t>(FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL),
	    0);

	// Use the 8 kHz low-noise inertial stream without FIFO temperature.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::PWR_MGMT0,
	    static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	    | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG0,
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_8kHz),
	    Bit7 | Bit6 | Bit5 | Bit3 | Bit2);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG0,
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_8kHz),
	    Bit7 | Bit6 | Bit5 | Bit3 | Bit2);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1,
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_WM_GT_TH)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_GYRO_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_ACCEL_EN),
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_TEMP_EN));

	// Program the byte-count watermark before enabling its interrupt.
	addWatermark(config, count, Register::BANK_0::FIFO_CONFIG2, Register::BANK_0::FIFO_CONFIG3,
		     context.fifo_watermark, 0x0f);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG0,
	    static_cast<uint8_t>(INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG1,
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_TPULSE_DURATION)
	    | static_cast<uint8_t>(INT_CONFIG1_BIT::INT_TDEASSERT_DISABLE),
	    static_cast<uint8_t>(INT_CONFIG1_BIT::INT_ASYNC_RESET));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_SOURCE0,
	    static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	    0);

	return count;
}
#endif // CONFIG_TDK_ICM42X_ICM42605

#if defined(CONFIG_TDK_ICM42X_ICM42670P)
uint8_t loadIcm42670P(const Context &context, RegisterConfig(&config)[kMaxRegisterConfigs])
{
	using namespace tdk_icm42670p_registers;

	uint8_t count = 0;

	// Configure the latched INT1 signal.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_CONFIG,
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT),
	    static_cast<uint8_t>(INT_CONFIG_BIT::INT1_POLARITY));

	// Select the 1600 Hz low-noise inertial stream and bypass the UI filters.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::PWR_MGMT0,
	    static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	    | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG0,
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS_SET)
	    | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_1600HZ_SET),
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS_CLEAR)
	    | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_1600HZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG0,
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_16G_SET)
	    | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_1600HZ_SET),
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_16G_CLEAR)
	    | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_1600HZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG1,
	    0,
	    static_cast<uint8_t>(GYRO_CONFIG1_BIT::GYRO_UI_FILT_BW_BYPASSED_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG1,
	    0,
	    static_cast<uint8_t>(ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_BW_BYPASSED_CLEAR));

	// Use stop-on-full mode with a byte-count watermark.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG1,
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_MODE_STOP_ON_FULL),
	    static_cast<uint8_t>(FIFO_CONFIG1_BIT::FIFO_BYPASS));
	addWatermark(config, count, Register::BANK_0::FIFO_CONFIG2, Register::BANK_0::FIFO_CONFIG3,
		     context.fifo_watermark, 0x0f);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT_SOURCE0,
	    static_cast<uint8_t>(INT_SOURCE0_BIT::FIFO_THS_INT1_EN),
	    0);

	// FIFO channel selection and interrupt clearing reside in the indirect MREG space.
	add(config, count, AddressSpace::kMreg1, Register::MREG1::FIFO_CONFIG5,
	    static_cast<uint8_t>(FIFO_CONFIG5_BIT::FIFO_GYRO_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG5_BIT::FIFO_ACCEL_EN),
	    0);
	add(config, count, AddressSpace::kMreg1, Register::MREG1::INT_CONFIG0,
	    static_cast<uint8_t>(INT_CONFIG0_BIT::FIFO_THS_INT_CLEAR),
	    0);

	return count;
}
#endif // CONFIG_TDK_ICM42X_ICM42670P

#if defined(CONFIG_TDK_ICM42X_ICM45686)
uint8_t loadIcm45686(const Context &context, RegisterConfig(&config)[kMaxRegisterConfigs])
{
	using namespace tdk_icm45686_registers;

	uint8_t count = 0;

	// Configure gyro and accel interpolation/anti-alias filtering through IREG.
	add(config, count, AddressSpace::kIreg, Register::IREG::IPREG_SYS1_REG_166,
	    static_cast<uint8_t>(IPREG_SYS1_REG_166_BIT::GYRO_SRC_CTRL_INTERP_AAF_SET),
	    static_cast<uint8_t>(IPREG_SYS1_REG_166_BIT::GYRO_SRC_CTRL_INTERP_AAF_CLEAR));
	add(config, count, AddressSpace::kIreg, Register::IREG::IPREG_SYS2_REG_123,
	    static_cast<uint8_t>(IPREG_SYS2_REG_123_BIT::ACCEL_SRC_CTRL_INTERP_AAF_SET),
	    static_cast<uint8_t>(IPREG_SYS2_REG_123_BIT::ACCEL_SRC_CTRL_INTERP_AAF_CLEAR));

	// Route only the FIFO watermark to INT1.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT1_CONFIG0,
	    static_cast<uint8_t>(INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS),
	    static_cast<uint8_t>(~static_cast<uint8_t>(INT1_CONFIG0_BIT::INT1_STATUS_EN_FIFO_THS)));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::INT1_CONFIG2,
	    0,
	    static_cast<uint8_t>(INT1_CONFIG2_BIT::INT1_DRIVE)
	    | static_cast<uint8_t>(INT1_CONFIG2_BIT::INT1_MODE)
	    | static_cast<uint8_t>(INT1_CONFIG2_BIT::INT1_POLARITY));

	// Match the 6400 Hz channel configuration to the standard packet profile.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::PWR_MGMT0,
	    static_cast<uint8_t>(PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE)
	    | static_cast<uint8_t>(PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::GYRO_CONFIG0,
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_SET)
	    | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_SET),
	    static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_UI_FS_SEL_4000_DPS_CLEAR)
	    | static_cast<uint8_t>(GYRO_CONFIG0_BIT::GYRO_ODR_6400_HZ_CLEAR));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::ACCEL_CONFIG0,
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_SET)
	    | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_SET),
	    static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_UI_FS_SEL_32_G_CLEAR)
	    | static_cast<uint8_t>(ACCEL_CONFIG0_BIT::ACCEL_ODR_6400_HZ_CLEAR));

	// Use uncompressed standard packets with timestamps; high-resolution packets remain disabled.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG4,
	    static_cast<uint8_t>(FIFO_CONFIG4_BIT::FIFO_TMST_FSYNC_EN),
	    static_cast<uint8_t>(FIFO_CONFIG4_BIT::FIFO_COMP_EN));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG0,
	    static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_SET)
	    | static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_SET),
	    static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_MODE_STOP_ON_FULL_CLEAR)
	    | static_cast<uint8_t>(FIFO_CONFIG0_BIT::FIFO_DEPTH_8K_CLEAR));

	// ICM45686 uses a record-count watermark.
	addWatermark(config, count, Register::BANK_0::FIFO_CONFIG1_0, Register::BANK_0::FIFO_CONFIG1_1,
		     context.fifo_watermark, 0xff);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG2,
	    static_cast<uint8_t>(FIFO_CONFIG2_BIT::FIFO_WR_WM_EQ_OR_GT_TH),
	    0);
	add(config, count, AddressSpace::kBank0, Register::BANK_0::FIFO_CONFIG3,
	    static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_GYRO_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_ACCEL_EN)
	    | static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_IF_EN),
	    static_cast<uint8_t>(FIFO_CONFIG3_BIT::FIFO_HIRES_EN));

	// Use the optional external clock on INT2 only when requested.
	add(config, count, AddressSpace::kBank0, Register::BANK_0::RTC_CONFIG,
	    context.clock_input ? static_cast<uint8_t>(RTC_CONFIG_BIT::RTC_MODE) : 0,
	    context.clock_input ? 0 : static_cast<uint8_t>(RTC_CONFIG_BIT::RTC_MODE));
	add(config, count, AddressSpace::kBank0, Register::BANK_0::IOC_PAD_SCENARIO_OVRD,
	    context.clock_input
	    ? (static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD)
	       | static_cast<uint8_t>(IOC_PAD_SCENARIO_OVRD_BIT::PADS_INT2_CFG_OVRD_CLKIN)) : 0,
	    0);

	return count;
}
#endif // CONFIG_TDK_ICM42X_ICM45686

} // namespace

uint8_t load(Variant variant, const Context &context, RegisterConfig(&config)[kMaxRegisterConfigs])
{
	switch (variant) {
#if defined(CONFIG_TDK_ICM42X_ICM40609D)

	case Variant::kIcm40609D: {
			return loadIcm40609D(context, config);
		}

#endif // CONFIG_TDK_ICM42X_ICM40609D

#if defined(CONFIG_TDK_ICM42X_ICM42605)

	case Variant::kIcm42605: {
			return loadIcm42605(context, config);
		}

#endif // CONFIG_TDK_ICM42X_ICM42605

#if defined(CONFIG_TDK_ICM42X_ICM42670P)

	case Variant::kIcm42670P: {
			return loadIcm42670P(context, config);
		}

#endif // CONFIG_TDK_ICM42X_ICM42670P

#if defined(CONFIG_TDK_ICM42X_ICM45686)

	case Variant::kIcm45686: {
			return loadIcm45686(context, config);
		}

#endif // CONFIG_TDK_ICM42X_ICM45686

#if defined(CONFIG_TDK_ICM42X_ICM42688P)

	case Variant::kIcm42688P: {
			return loadHighResolution(variant, context, config);
		}

#endif // CONFIG_TDK_ICM42X_ICM42688P

#if defined(CONFIG_TDK_ICM42X_ICM42686P)

	case Variant::kIcm42686P: {
			return loadHighResolution(variant, context, config);
		}

#endif // CONFIG_TDK_ICM42X_ICM42686P

#if defined(CONFIG_TDK_ICM42X_IIM42652)

	case Variant::kIim42652: {
			return loadHighResolution(variant, context, config);
		}

#endif // CONFIG_TDK_ICM42X_IIM42652

#if defined(CONFIG_TDK_ICM42X_IIM42653)

	case Variant::kIim42653: {
			return loadHighResolution(variant, context, config);
		}

#endif // CONFIG_TDK_ICM42X_IIM42653

	default: {
			return 0;
		}
	}
}

} // namespace tdk_icm42x_config
