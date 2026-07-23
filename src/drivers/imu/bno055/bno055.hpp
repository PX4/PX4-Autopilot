/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <drivers/device/i2c.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <systemlib/conversions.h>

using namespace time_literals;


class BNO055 : public device::I2C, public I2CSPIDriver<BNO055>
{
public:

	BNO055(const I2CSPIDriverConfig &config);
	virtual ~BNO055();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config,
					     int runtime_instance);


	int init() override;

	/**
	 * Starts polling the sensor
	 */
	void start();

	/**
	 * The function that polls the sensors and updates their values
	 */
	void RunImpl();

	static void print_usage();

	/**
	 * Completely resets chip
	 *
	 * @note init() must have been called before, and since the chip is now reset, init() should be called again
	 */
	int reset();

	struct three_d { double x; double y; double z; };

private:

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;
	PX4Magnetometer _px4_mag;

	// most recent data
	struct three_d accel_xyz;
	struct three_d gyro_xyz;
	struct three_d mag_xyz;

	// some select addresses and config values from the Bosch driver
	const uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3F;
	const uint8_t BNO055_CHIP_ID_ADDR = 0x0;
	const uint8_t BNO055_ACCEL_CONFIG_ADDR = 0x8;
	const uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
	const uint8_t BNO055_GYRO_CONFIG_ADDR = 0x0A;
	const uint8_t BNO055_MAG_CONFIG_ADDR = 0x09;
	const uint8_t BNO055_PAGE_ID_ADDR = 0x07; // 7 in both pages
	const uint8_t BNO055_UNIT_SEL_ADDR = 0x3B;
	const uint8_t BNO055_SYS_ERR_ADDR = 0x3A;

	const uint8_t BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08;
	const uint8_t BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09;
	const uint8_t BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A;
	const uint8_t BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B;
	const uint8_t BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C;
	const uint8_t BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D;
	const uint8_t BNO055_MAG_DATA_X_LSB_ADDR = 0X0E;
	const uint8_t BNO055_MAG_DATA_X_MSB_ADDR = 0X0F;
	const uint8_t BNO055_MAG_DATA_Y_LSB_ADDR = 0X10;
	const uint8_t BNO055_MAG_DATA_Y_MSB_ADDR = 0X11;
	const uint8_t BNO055_MAG_DATA_Z_LSB_ADDR = 0X12;
	const uint8_t BNO055_MAG_DATA_Z_MSB_ADDR = 0X13;
	const uint8_t BNO055_GYRO_DATA_X_LSB_ADDR = 0X14;
	const uint8_t BNO055_GYRO_DATA_X_MSB_ADDR = 0X15;
	const uint8_t BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16;
	const uint8_t BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17;
	const uint8_t BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18;
	const uint8_t BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19;


	const uint8_t BNO055_ACCEL_BW_POS = 0x2;
	const uint8_t BNO055_ACCEL_BW_MSK = 0x1C;
	const uint8_t BNO055_GYRO_BW_POS = 0x3;
	const uint8_t BNO055_GYRO_BW_MSK = 0x38;
	const uint8_t BNO055_MAG_DATA_OUTPUT_RATE_MSK = 0x7;
	const uint8_t BNO055_SYS_RST_MSK = 0x20;
	const uint8_t BNO055_OPERATION_MODE_CONFIG = 0x0;
	const uint8_t BNO055_OPERATION_MODE_AMG = 0x7;

	// 0x28 is the first address, 0x29 is an alternative
	const uint8_t BNO055_I2C_ADDR1 = 0x28;

	// TODO: check (and tweak) those values
	// they seem to work fine for a FW, I don't have a MC at hand right now
	// cutoff freqs for the internal low pass filters
	const uint8_t ACCEL_BW_REGVAL = 0x04; // 125 Hz
	const uint8_t GYRO_BW_REGVAL = 0x02; // 116 Hz
	// update freq for the magnetometer
	const uint8_t MAG_RATE_REGVAL = 0x07; // 30 Hz (maximum freq for the magnetometer)
	// poll at 250 Hz to match other sensors, 2x the low pass bandwidth kind of "feels right"
	const uint32_t POLLING_INTERVAL_US = uint32_t(1000000 / 250);
	// the magnetometer is slower than the other sensors, so don't fetch it every time
	const uint32_t MAG_INTERVAL_US = uint32_t(1000000 / 30);
	// save the last time the magnetometer was read
	uint32_t mag_last_read = 0;


	// internal values for talking with the chip, taken from the bno055_t of the Bosch driver
	uint8_t chip_id = 0; /**< chip_id of bno055 */
	uint8_t page_id = 0;
	uint8_t dev_addr = BNO055_I2C_ADDR1;

	/**
	 * Switches the sensor register page, if necessary, and updates page_id
	 *
	 * @param p 0 (data and basic config) or 1 (some extra config)
	*/
	int set_page_id(uint8_t p);

	/**
	 * Helper function for the get_<x>
	 *
	 * @param out already-initialized, floating point result
	 * @param addr start address for the 6 byte read
	 * @param divide_by 16bit sensor values will be divided by this, result stored in out
	*/
	int get_sensor(BNO055::three_d *out, uint8_t addr, double divide_by);

	/**
	 * @param out already-initialized ptr to struct, output in m/s^2
	*/
	int get_accel(three_d *out);

	/**
	 * @param out already-initialized ptr to struct, output in rad/s
	*/
	int get_gyro(three_d *out);

	/**
	 * @param out already-initialized ptr to struct, output in uT
	*/
	int get_mag(three_d *out);

	int read_reg(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
	int write_reg(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);

};
