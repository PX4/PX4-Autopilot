/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

#include "BMI088.hpp"

#define BMI088_DEVICE_PATH_GYRO		"/dev/bmi088_gyro"
#define BMI088_DEVICE_PATH_GYRO_EXT	"/dev/bmi088_gyro_ext"

// BMI088 Gyro registers
#define BMI088_GYR_CHIP_ID          0x00
#define BMI088_GYR_X_L              0x02
#define BMI088_GYR_X_H              0x03
#define BMI088_GYR_Y_L              0x04
#define BMI088_GYR_Y_H              0x05
#define BMI088_GYR_Z_L              0x06
#define BMI088_GYR_Z_H              0x07
#define BMI088_GYR_INT_STATUS_0     0x09
#define BMI088_GYR_INT_STATUS_1     0x0A
#define BMI088_GYR_INT_STATUS_2     0x0B
#define BMI088_GYR_INT_STATUS_3     0x0C
#define BMI088_GYR_FIFO_STATUS      0x0E
#define BMI088_GYR_RANGE            0x0F
#define BMI088_GYR_BW               0x10
#define BMI088_GYR_LPM1             0x11
#define BMI088_GYR_LPM2             0x12
#define BMI088_GYR_RATE_HBW         0x13
#define BMI088_GYR_SOFTRESET        0x14
#define BMI088_GYR_INT_EN_0         0x15
#define BMI088_GYR_INT_EN_1         0x16
#define BMI088_GYR_INT_MAP_0        0x17
#define BMI088_GYR_INT_MAP_1        0x18
#define BMI088_GYR_INT_MAP_2        0x19
#define BMI088_GYRO_0_REG           0x1A
#define BMI088_GYRO_1_REG           0x1B
#define BMI088_GYRO_2_REG           0x1C
#define BMI088_GYRO_3_REG           0x1E
#define BMI088_GYR_INT_LATCH        0x21
#define BMI088_GYR_INT_LH_0         0x22
#define BMI088_GYR_INT_LH_1         0x23
#define BMI088_GYR_INT_LH_2         0x24
#define BMI088_GYR_INT_LH_3         0x25
#define BMI088_GYR_INT_LH_4         0x26
#define BMI088_GYR_INT_LH_5         0x27
#define BMI088_GYR_SOC              0x31
#define BMI088_GYR_A_FOC            0x32
#define BMI088_GYR_TRIM_NVM_CTRL    0x33
#define BMI088_BGW_SPI3_WDT         0x34
#define BMI088_GYR_OFFSET_COMP      0x36
#define BMI088_GYR_OFFSET_COMP_X    0x37
#define BMI088_GYR_OFFSET_COMP_Y    0x38
#define BMI088_GYR_OFFSET_COMP_Z    0x39
#define BMI088_GYR_TRIM_GPO         0x3A
#define BMI088_GYR_TRIM_GP1         0x3B
#define BMI088_GYR_SELF_TEST        0x3C
#define BMI088_GYR_FIFO_CONFIG_0    0x3D
#define BMI088_GYR_FIFO_CONFIG_1    0x3E
#define BMI088_GYR_FIFO_DATA        0x3F

// BMI088 Gyroscope Chip-Id
#define BMI088_GYR_WHO_AM_I         0x0F

//ODR & DLPF filter bandwidth settings (they are coupled)
#define BMI088_GYRO_RATE_100        (0<<3) | (1<<2) | (1<<1) | (1<<0)
#define BMI088_GYRO_RATE_200        (0<<3) | (1<<2) | (1<<1) | (0<<0)
#define BMI088_GYRO_RATE_400        (0<<3) | (0<<2) | (1<<1) | (1<<0)
#define BMI088_GYRO_RATE_1000       (0<<3) | (0<<2) | (1<<1) | (0<<0)
#define BMI088_GYRO_RATE_2000       (0<<3) | (0<<2) | (0<<1) | (1<<0)

//BMI088_GYR_LPM1      0x11
#define BMI088_GYRO_NORMAL          (0<<7) | (0<<5)
#define BMI088_GYRO_DEEP_SUSPEND    (0<<7) | (1<<5)
#define BMI088_GYRO_SUSPEND         (1<<7) | (0<<5)

//BMI088_GYR_RANGE        0x0F
#define BMI088_GYRO_RANGE_2000_DPS  (0<<2) | (0<<1) | (0<<0)
#define BMI088_GYRO_RANGE_1000_DPS  (0<<2) | (0<<1) | (1<<0)
#define BMI088_GYRO_RANGE_500_DPS   (0<<2) | (1<<1) | (0<<0)
#define BMI088_GYRO_RANGE_250_DPS   (0<<2) | (1<<1) | (1<<0)
#define BMI088_GYRO_RANGE_125_DPS   (1<<2) | (0<<1) | (0<<0)

//BMI088_GYR_INT_EN_0         0x15
#define BMI088_GYR_DRDY_INT_EN      (1<<7)

//BMI088_GYR_INT_MAP_1     0x18
#define BMI088_GYR_DRDY_INT1        (1<<0)

// Default and Max values
#define BMI088_GYRO_DEFAULT_RANGE_DPS		2000
#define BMI088_GYRO_DEFAULT_RATE            1000
#define BMI088_GYRO_MAX_RATE                1000
#define BMI088_GYRO_MAX_PUBLISH_RATE        280

#define BMI088_GYRO_DEFAULT_DRIVER_FILTER_FREQ  50

/* Mask definitions for Gyro bandwidth */
#define BMI088_GYRO_BW_MASK                  0x0F

class BMI088_gyro : public BMI088
{
public:
	BMI088_gyro(I2CSPIBusOption bus_option, int bus, const char *path_accel, uint32_t device, enum Rotation rotation,
		    int bus_frequency, spi_mode_e spi_mode);
	virtual ~BMI088_gyro();

	int     init() override;

	// Start automatic measurement.
	void            start() override;

	void            print_status() override;

	void            print_registers() override;

	// deliberately cause a sensor error
	void            test_error() override;

	void RunImpl() override;
protected:

	int     probe() override;

private:

	PX4Gyroscope	_px4_gyro;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define BMI088_GYRO_NUM_CHECKED_REGISTERS 7
	static const uint8_t    _checked_registers[BMI088_GYRO_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI088_GYRO_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI088_GYRO_NUM_CHECKED_REGISTERS];

	// last temperature reading for print_info()
	float           _last_temperature;

	/**
	     * Reset chip.
	     *
	     * Resets the chip and measurements ranges, but not scale and offset.
	     */
	int         reset();

	/**
	     * Static trampoline from the hrt_call context; because we don't have a
	     * generic hrt wrapper yet.
	     *
	     * Called by the HRT in interrupt context at the specified rate if
	     * automatic polling is enabled.
	     *
	     * @param arg       Instance pointer for the driver that is polling.
	     */
	static void     measure_trampoline(void *arg);

	/**
	     * Modify a register in the BMI088_gyro
	     *
	     * Bits are cleared before bits are set.
	     *
	     * @param reg       The register to modify.
	     * @param clearbits Bits in the register to clear.
	     * @param setbits   Bits in the register to set.
	     */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	     * Write a register in the BMI088_gyro, updating _checked_values
	     *
	     * @param reg       The register to write.
	     * @param value     The new value to write.
	     */
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	     * Set the BMI088_gyro measurement range.
	     *
	     * @param max_dps   The maximum DPS value the range must support.
	     * @return      OK if the value can be supported, -EINVAL otherwise.
	     */
	int         set_gyro_range(unsigned max_dps);

	/*
	     * set gyro sample rate
	     */
	int gyro_set_sample_rate(float desired_sample_rate_hz);

	/*
	     * check that key registers still have the right value
	     */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI088_gyro(const BMI088_gyro &);
	BMI088_gyro operator=(const BMI088_gyro &);

#pragma pack(push, 1)
	/**
	     * Report conversation within the BMI088_gyro, including command byte and
	     * interrupt status.
	     */
	struct BMI_GyroReport {
		uint8_t     cmd;
		uint8_t     chip_id;	// Read to check if bus is workig
		uint8_t     ununsed;
		int16_t     gyro_x;
		int16_t     gyro_y;
		int16_t     gyro_z;
	};
#pragma pack(pop)

};
