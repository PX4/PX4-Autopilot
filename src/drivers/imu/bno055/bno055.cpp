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

#include "bno055.hpp"

BNO055::BNO055(const I2CSPIDriverConfig &config):
	I2C(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id()),
	_px4_gyro(get_device_id()),
	_px4_mag(get_device_id())
{
}

BNO055::~BNO055()
{
}

int BNO055::init()
{
	// do I2C init
	int ret = I2C::init();

	uint8_t data = 0;

	if (ret != OK) {
		PX4_ERR("I2C setup failed");
		return ret;
	}

	// a bit hacky, but seems to do the trick
	set_device_address(dev_addr);

	// force-write the page id
	page_id = 255;
	set_page_id(0);

	// read chip id
	ret = read_reg(BNO055_CHIP_ID_ADDR, &chip_id, 1);

	if (ret != OK) {
		PX4_ERR("Cannot read chip ID: 0x%x", ret);
		return ret;
	}

	if (chip_id != 0xA0) {
		PX4_ERR("Chip ID is wrong, need 0xA0, got 0x%x", chip_id);
		return -1;
	}

	// reset sensor, so it will be in default config
	ret = reset();

	if (ret != OK) {
		PX4_ERR("Can't reset sensor");
		return ret;
	}

	// wait some time for the sensor to reset
	// TODO: I know this is kind of ugly, but it's only during startup, and it seems to work just fine
	usleep(700 * 1000);

	// force-set the page id to 0 after reset, just in case
	page_id = 255;
	set_page_id(0);

	// we should now be in config mode after the reset
	ret = read_reg(BNO055_OPR_MODE_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Sensor did not come online after reset");
		return ret;
	}

	if ((data & 0x0F) != BNO055_OPERATION_MODE_CONFIG) {
		PX4_ERR("Sensor did not reset correctly, mode should be 0 for config, but is 0x%x", data);
		return -1;
	}

	// while we are here, check sensor error code
	data = 255;
	ret = read_reg(BNO055_SYS_ERR_ADDR, &data, 1);

	if (ret != OK || data != 0) {
		PX4_ERR("Sensor error, code 0x%x", data);
		return -1;
	}

	// next, configure the sensor bandwidths

	// all of those are in page 1
	set_page_id(1);

	ret = read_reg(BNO055_ACCEL_CONFIG_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot read accel config");
		return ret;
	}

	data = (data & ~BNO055_ACCEL_BW_MSK) | (ACCEL_BW_REGVAL << BNO055_ACCEL_BW_POS);
	ret = write_reg(BNO055_ACCEL_CONFIG_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot write accel config");
		return ret;
	}

	ret = read_reg(BNO055_GYRO_CONFIG_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot read gyro config");
		return ret;
	}

	data = (data & ~BNO055_GYRO_BW_MSK) | (GYRO_BW_REGVAL << BNO055_GYRO_BW_POS);
	ret = write_reg(BNO055_GYRO_CONFIG_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot write gyro config");
		return ret;
	}

	ret = read_reg(BNO055_MAG_CONFIG_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot read mag config");
		return ret;
	}

	data = (data & ~BNO055_MAG_DATA_OUTPUT_RATE_MSK) | MAG_RATE_REGVAL;
	ret = write_reg(BNO055_MAG_CONFIG_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot write mag config");
		return ret;
	}

	// configure output formats: m/s^2, rad/s, uT
	set_page_id(0);
	data = 0b00000010;
	ret = write_reg(BNO055_UNIT_SEL_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot write unit config");
		return ret;
	}


	// switch to AMG mode: all sensors, no onboard fusion
	data = BNO055_OPERATION_MODE_AMG;
	ret = write_reg(BNO055_OPR_MODE_ADDR, &data, 1);

	if (ret != OK) {
		PX4_ERR("Cannot enter run mode");
		return ret;
	}

	// 20 ms delay for mode switch
	usleep(20 * 1000);


	// schedule sensor polling
	// TODO: make a separate step? Or just integrate it here?
	start();

	return ret;
}

int BNO055::reset()
{
	uint8_t data = 0;

	if (set_page_id(0) != OK) { return -1; }

	// read sys register, set reset bit, write back
	if (read_reg(BNO055_SYS_TRIGGER_ADDR, &data, 1) != OK) { return -2; }

	data = data | BNO055_SYS_RST_MSK;

	if (write_reg(BNO055_SYS_TRIGGER_ADDR, &data, 1) != OK) { return -3; }

	// we're now reset
	return 0;
}

int BNO055::set_page_id(uint8_t p)
{
	// only 0 and 1 are valid page values
	if (p != 0 && p != 1) { return -1; }

	// try to avoid unnecessary transfers
	if (page_id == p) { return 0; }

	int ret = write_reg(BNO055_PAGE_ID_ADDR, &p, 1);

	if (ret == OK) {
		// only set if this was successful, so it won't skip a second attempt
		page_id = p;
		return OK;

	} else {
		return -2;
	}
}

int BNO055::read_reg(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	return transfer(&reg_addr, 1, reg_data, cnt);
}

int BNO055::write_reg(uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	uint8_t buffer[cnt + 1];
	buffer[0] = reg_addr;
	memcpy(buffer + 1, reg_data, cnt);
	return transfer(buffer, cnt + 1, nullptr, 0);
}

void BNO055::start()
{
	PX4_INFO("Polling BNO055 every %u us", POLLING_INTERVAL_US);
	ScheduleOnInterval(POLLING_INTERVAL_US);
}

int BNO055::get_sensor(BNO055::three_d *out, uint8_t addr, double divide_by)
{
	// x1 x2 y1 y2 z1 z2
	uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
	// x y z
	int16_t accel[3] = { 0, 0, 0 };

	if (set_page_id(0) != OK) {
		return -1;
	}

	// data is an array here, so already by-pointer
	if (read_reg(addr, data, 6) != OK) {
		return -1;
	}

	// append 2x8bit registers to 16bit values
	accel[0] = (int16_t(data[1]) << 8) | int16_t(data[0]);
	accel[1] = (int16_t(data[3]) << 8) | int16_t(data[2]);
	accel[2] = (int16_t(data[5]) << 8) | int16_t(data[4]);

	// convert to whatever format we want
	out->x = double(accel[0]) / divide_by;
	out->y = double(accel[1]) / divide_by;
	out->z = double(accel[2]) / divide_by;

	return OK;
}

int BNO055::get_accel(BNO055::three_d *out)
{
	return get_sensor(out, BNO055_ACCEL_DATA_X_LSB_ADDR, 100);
}

int BNO055::get_gyro(BNO055::three_d *out)
{
	return get_sensor(out, BNO055_GYRO_DATA_X_LSB_ADDR, 900);
}

int BNO055::get_mag(BNO055::three_d *out)
{
	return get_sensor(out, BNO055_MAG_DATA_X_LSB_ADDR, 16);
}

void BNO055::RunImpl()
{
	hrt_abstime current_time = hrt_absolute_time();

	if (get_accel(&accel_xyz) != OK) {
		PX4_DEBUG("Reading accel failed, skipping");
		_px4_accel.increase_error_count();

	} else {
		_px4_accel.update(current_time, accel_xyz.x, accel_xyz.y, accel_xyz.z);
	}

	if (get_gyro(&gyro_xyz) != OK) {
		PX4_DEBUG("Reading gyro failed, skipping");
		_px4_gyro.increase_error_count();

	} else {
		_px4_gyro.update(current_time, gyro_xyz.x, gyro_xyz.y, gyro_xyz.z);
	}

	// only read the magnetometer data when new data is available
	if ((current_time - mag_last_read) > MAG_INTERVAL_US) {

		if (get_mag(&mag_xyz) != OK) {
			PX4_DEBUG("Reading mag failed, skipping");
			_px4_mag.increase_error_count();

		} else {
			// for some reason, I need to do a /100 to make the values fit
			_px4_mag.update(current_time, mag_xyz.x / 100, mag_xyz.y / 100, mag_xyz.z / 100);
		}

		mag_last_read = current_time;
	}

}
