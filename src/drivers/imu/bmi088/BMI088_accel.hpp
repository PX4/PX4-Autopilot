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

#include "BMI088.hpp"

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/conversion/rotation.h>

#define BMI088_DEVICE_PATH_ACCEL	"/dev/bmi088_accel"
#define BMI088_DEVICE_PATH_ACCEL_EXT	"/dev/bmi088_accel_ext"

// BMI088 Accel registers
#define BMI088_ACC_CHIP_ID            0x00

#define BMI088_ACC_ERR_REG            0x02
#define BMI088_ACC_STATUS             0x03
#define BMI088_ACC_X_L                0x12
#define BMI088_ACC_X_H                0x13
#define BMI088_ACC_Y_L                0x14
#define BMI088_ACC_Y_H                0x15
#define BMI088_ACC_Z_L                0x16
#define BMI088_ACC_Z_H                0x17
#define BMI088_ACC_SENSORTIME_0       0x18
#define BMI088_ACC_SENSORTIME_1       0x19
#define BMI088_ACC_SENSORTIME_2       0x1A
#define BMI088_ACC_INT_STAT_1         0x1D
#define BMI088_ACC_TEMP_H             0x22
#define BMI088_ACC_TEMP_L             0x23
#define BMI088_ACC_CONF               0x40
#define BMI088_ACC_RANGE              0x41
#define BMI088_ACC_INT1_IO_CONF       0x53
#define BMI088_ACC_INT2_IO_CONF       0x54
#define BMI088_ACC_INT1_INT2_MAP_DATA 0x58
#define BMI088_ACC_SELF_TEST          0x6D
#define BMI088_ACC_PWR_CONF           0x7C
#define BMI088_ACC_PWR_CTRL           0x7D
#define BMI088_ACC_SOFTRESET          0x7E

// BMI088 Accelerometer Chip-Id
#define BMI088_ACC_WHO_AM_I         0x1E

// BMI088_ACC_ERR_REG       0x02
#define BMI088_ACC_ERR_REG_NO_ERROR (0x00<<2)
#define BMI088_ACC_ERR_REG_ERROR    (0x01<<2)

#define BMI088_ACC_ERR_REG_FATAL_ERROR    (0x01<<0)

// BMI088_ACC_STATUS       0x03
#define BMI088_ACC_STATUS_DRDY      (0x01<<7)

// BMI088_ACC_INT_STAT_1   0x01D
#define BMI088_ACC_INT_STAT_1_DRDY  (0x01<<7)

// BMI088_ACC_CONF           0x40
#define BMI088_ACC_CONF_BWP_4       (0x08<<4)
#define BMI088_ACC_CONF_BWP_2       (0x09<<4)
#define BMI088_ACC_CONF_BWP_NORMAL  (0x0A<<4)

#define BMI088_ACC_CONF_ODR_12_5    (0x05<<0)
#define BMI088_ACC_CONF_ODR_25      (0x06<<0)
#define BMI088_ACC_CONF_ODR_50      (0x07<<0)
#define BMI088_ACC_CONF_ODR_100     (0x08<<0)
#define BMI088_ACC_CONF_ODR_200     (0x09<<0)
#define BMI088_ACC_CONF_ODR_400     (0x0A<<0)
#define BMI088_ACC_CONF_ODR_800     (0x0B<<0)
#define BMI088_ACC_CONF_ODR_1600    (0x0C<<0)

// BMI088_ACC_RANGE        0x41
#define BMI088_ACCEL_RANGE_3_G      (0x00<<0)
#define BMI088_ACCEL_RANGE_6_G      (0x01<<0)
#define BMI088_ACCEL_RANGE_12_G     (0x02<<0)
#define BMI088_ACCEL_RANGE_24_G     (0x03<<0)

// BMI088_ACC_INT1_IO_CONF      0x53
#define BMI088_ACC_INT1_IO_CONF_INT1_IN     (0x01<<4)

#define BMI088_ACC_INT1_IO_CONF_INT1_OUT    (0x01<<3)

#define BMI088_ACC_INT1_IO_CONF_PP          (0x00<<2)
#define BMI088_ACC_INT1_IO_CONF_OD          (0x01<<2)

#define BMI088_ACC_INT1_IO_CONF_ACTIVE_LOW  (0x00<<1)
#define BMI088_ACC_INT1_IO_CONF_ACTIVE_HIGH (0x01<<1)

// BMI088_ACC_INT2_IO_CONF      0x54
#define BMI088_ACC_INT2_IO_CONF_INT1_IN     (0x01<<4)

#define BMI088_ACC_INT2_IO_CONF_INT1_OUT    (0x01<<3)

#define BMI088_ACC_INT2_IO_CONF_PP          (0x00<<2)
#define BMI088_ACC_INT2_IO_CONF_OD          (0x01<<2)

#define BMI088_ACC_INT2_IO_CONF_ACTIVE_LOW  (0x00<<1)
#define BMI088_ACC_INT2_IO_CONF_ACTIVE_HIGH (0x01<<1)

// BMI088_ACC_INT1_INT2_MAP_DATA  0x54
#define BMI088_ACC_INT1_INT2_MAP_DATA_INT2_DRDY  (0x01<<6)
#define BMI088_ACC_INT1_INT2_MAP_DATA_INT1_DRDY  (0x01<<2)

// BMI088_ACC_SELF_TEST            0x6D
#define BMI088_ACC_SELF_TEST_OFF        (0x00<<0)
#define BMI088_ACC_SELF_TEST_POSITIVE   (0x0D<<0)
#define BMI088_ACC_SELF_TEST_NEGATIVE   (0x09<<0)

// BMI088_ACC_PWR_CONF            0x7C
#define BMI088_ACC_PWR_CONF_SUSPEND     (0x03<<0)
#define BMI088_ACC_PWR_CONF_ACTIVE      (0x00<<0)

// BMI088_ACC_PWR_CTRL            0x7D
#define BMI088_ACC_PWR_CTRL_EN          (0x04<<0)


///////
// To Do check these defaults and maks below
///

// Default and Max values
#define BMI088_ACCEL_DEFAULT_RANGE_G		    24
#define BMI088_ACCEL_DEFAULT_RATE           800
#define BMI088_ACCEL_MAX_RATE               800
#define BMI088_ACCEL_MAX_PUBLISH_RATE       800

#define BMI088_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 50

class BMI088_accel : public BMI088
{
public:
	BMI088_accel(I2CSPIBusOption bus_option, int bus, const char *path_accel, uint32_t device, enum Rotation rotation,
		     int bus_frequency, spi_mode_e spi_mode);
	virtual ~BMI088_accel();

	int     init() override;

	// Start automatic measurement.
	void            start();

	// We need to override the read_reg function from the BMI088 base class, because the accelerometer requires a dummy byte read before each read operation
	uint8_t   read_reg(unsigned reg) override;

	// We need to override the read_reg16 function from the BMI088 base class, because the accelerometer requires a dummy byte read before each read operation
	uint16_t read_reg16(unsigned reg) override;

	void            print_status() override;

	void            print_registers();

	// deliberately cause a sensor error
	void            test_error();

	void     RunImpl() override;
protected:

	int     probe() override;

private:

	PX4Accelerometer	_px4_accel;

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _bad_registers;
	perf_counter_t      _duplicates;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define BMI088_ACCEL_NUM_CHECKED_REGISTERS 7
	static const uint8_t    _checked_registers[BMI088_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_values[BMI088_ACCEL_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI088_ACCEL_NUM_CHECKED_REGISTERS];

	bool            _got_duplicate;

	/**
	     * Reset chip.
	     *
	     * Resets the chip and measurements ranges, but not scale and offset.
	     */
	int         reset();

	/**
	     * Modify a register in the BMI088_accel
	     *
	     * Bits are cleared before bits are set.
	     *
	     * @param reg       The register to modify.
	     * @param clearbits Bits in the register to clear.
	     * @param setbits   Bits in the register to set.
	     */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	     * Write a register in the BMI088_accel, updating _checked_values
	     *
	     * @param reg       The register to write.
	     * @param value     The new value to write.
	     */
	void            write_checked_reg(unsigned reg, uint8_t value);

	/**
	     * Set the BMI088_accel measurement range.
	     *
	     * @param max_g     The maximum G value the range must support.
	     * @return      OK if the value can be supported, -EINVAL otherwise.
	     */
	int         set_accel_range(unsigned max_g);

	/**
	     * Set accel sample rate
	     */
	int accel_set_sample_rate(float desired_sample_rate_hz);

	/*
	     * check that key registers still have the right value
	     */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	BMI088_accel(const BMI088_accel &);
	BMI088_accel operator=(const BMI088_accel &);

};
