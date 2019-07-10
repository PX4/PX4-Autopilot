/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "BMI088_Accelerometer.hpp"

#include <ecl/geo/geo.h>

namespace Bosch_BMI088_Accelerometer
{

#define DIR_READ                0x80
#define DIR_WRITE               0x00

static constexpr uint32_t BMI088_BUS_SPEED = 10 * 1000 * 1000;

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
static constexpr Register _checked_registers[] {
	Register::ACC_CHIP_ID,
	Register::ACC_CONF
};

BMI088_Accelerometer::BMI088_Accelerometer(int bus, uint32_t device, enum Rotation rotation) :
	SPI("BMI088_ACCEL", nullptr, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088: accel read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi088: accel measure interval")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088: accel bad transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088: accel bad registers")),
	_duplicates(perf_alloc(PC_COUNT, "bmi088: accel duplicates"))
{
	_px4_accel.set_device_type(DRV_DEVTYPE_BMI088);
}

BMI088_Accelerometer::~BMI088_Accelerometer()
{
	// make sure we are truly inactive
	ScheduleClear();

	// delete the perf counter
	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_duplicates);
}

int
BMI088_Accelerometer::init()
{
	// do SPI init (and probe) first
	int ret = SPI::init();

	// if probe/setup failed, bail now
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	return reset();
}

uint8_t
BMI088_Accelerometer::registerRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
BMI088_Accelerometer::registerWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_WRITE;
	cmd[1] = value;
	transfer(cmd, cmd, sizeof(cmd));
}

void
BMI088_Accelerometer::registerSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (!(val & setbits)) {
		val |= setbits;
		registerWrite(reg, val);
	}
}

void
BMI088_Accelerometer::registerClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (val & clearbits) {
		val &= !clearbits;
		registerWrite(reg, val);
	}
}

int
BMI088_Accelerometer::reset()
{
	registerWrite(Register::ACC_SOFTRESET, 0xB6);	// Soft-reset
	px4_usleep(5000);

	registerWrite(Register::ACC_RANGE, 0x03);	// full 24G range

	registerWrite(Register::ACC_PWR_CONF, 0x0);	// low power disabled
	registerWrite(Register::ACC_PWR_CTRL, 0x04);

	// configure FIFO
	registerWrite(Register::FIFO_CONFIG_0, 0x00);
	registerWrite(Register::FIFO_CONFIG_0, 0x50);

	registerWrite(Register::INT1_IO_CONF, 0x8);		// Enable DRDY interrupt
	registerWrite(Register::INT1_INT2_MAP_DATA, 0x1);	// Map DRDY interrupt on pin INT1

	// TODO: flight test first
	//set_accel_range(BMI088_ACCEL_DEFAULT_RANGE_G);	// set accel range

	// Enable Accelerometer in normal mode
	//registerWrite(BMI088_ACC_PMU_LPW, BMI088_ACCEL_NORMAL);
	//px4_usleep(1000);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
			if (registerRead(_checked_registers[i]) != _checked_values[i]) {
				registerWrite(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	return OK;
}

int
BMI088_Accelerometer::probe()
{
	// look for device ID
	uint8_t whoami = registerRead(Register::ACC_CHIP_ID);

	// verify product revision
	switch (whoami) {
	case ID:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = whoami;
		_checked_bad[0] = whoami;
		return OK;
	}

	PX4_ERR("unexpected whoami 0x%02x", whoami);

	return -EIO;
}

void
BMI088_Accelerometer::registerWriteChecked(Register reg, uint8_t value)
{
	registerWrite(reg, value);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI088_Accelerometer::set_accel_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = 0; //BMI088_ACCEL_RANGE_3_G | BMI088_ACCEL_RANGE_24_G;
	float lsb_per_g;

	if (max_g == 0) {
		max_g = 24;
	}

	if (max_g <= 3) {
		//max_accel_g = 3;
		setbits |= 0; //BMI088_ACCEL_RANGE_3_G;
		lsb_per_g = 1024;

	} else if (max_g <= 6) {
		//max_accel_g = 6;
		setbits |= 0; //BMI088_ACCEL_RANGE_6_G;
		lsb_per_g = 512;

	} else if (max_g <= 12) {
		//max_accel_g = 12;
		setbits |= 0; //BMI088_ACCEL_RANGE_12_G;
		lsb_per_g = 256;

	} else if (max_g <= 24) {
		//max_accel_g = 24;
		setbits |= 0; //BMI088_ACCEL_RANGE_24_G;
		lsb_per_g = 128;

	} else {
		return -EINVAL;
	}

	_px4_accel.set_scale(CONSTANTS_ONE_G / lsb_per_g);

	registerSetBits(Register::ACC_RANGE, setbits);
	registerClearBits(Register::ACC_RANGE, clearbits);

	return OK;
}

void
BMI088_Accelerometer::check_registers()
{
	uint8_t v = 0;

	if ((v = registerRead(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {

		_checked_bad[_checked_next] = v;

		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			registerWrite(Register::ACC_SOFTRESET, 0xB6);
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			registerWrite(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % BMI088_ACCEL_NUM_CHECKED_REGISTERS;
}

void
BMI088_Accelerometer::Run()
{

}

void
BMI088_Accelerometer::measure()
{
	// start measuring
	perf_begin(_sample_perf);
	perf_count(_measure_interval);


	// empty FIFO

	uint8_t fifo_len_buf[3] {};
	fifo_len_buf[0] = static_cast<uint8_t>(Register::FIFO_LENGTH_0);

	if (OK != transfer(&fifo_len_buf[0], &fifo_len_buf[0], sizeof(fifo_len_buf))) {
		return;
	}

	uint16_t fifo_length = fifo_len_buf[1] + (fifo_len_buf[2] << 8);


	uint8_t fifo_buffer[fifo_length + 1];
	fifo_buffer[0] = static_cast<uint8_t>(Register::FIFO_DATA);

	if (OK != transfer(&fifo_buffer[0], &fifo_buffer[0], sizeof(fifo_buffer))) {
		return;
	}

	/*
	 * Fetch the full set of measurements from the BMI088 in one pass.
	 */
	//uint8_t index = 0;
	//uint8_t accel_data[8] {};
	//accel_data[index] = BMI088_ACC_X_L | DIR_READ;

	// const hrt_abstime timestamp_sample = hrt_absolute_time();

	// if (OK != transfer(accel_data, accel_data, sizeof(accel_data))) {
	// 	return;
	// }

	check_registers();


	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again, but don't return any data yet
		_register_wait--;
		return;
	}

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	const uint64_t error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_accel.set_error_count(error_count);


	// // The temperature sensor data is updated every 1.28 s.
	// uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
	// int16_t Temp_int11 = 0;

	// if (Temp_uint11 > 1023) {
	// 	Temp_int11 = Temp_uint11 - 2048;

	// } else {
	// 	Temp_int11 = Temp_uint11;
	// }

	// float Temperature = (Temp_int11 * 0.125f) + 23.0f;	// Temp_int11 * 0.125°C/LSB + 23°C

	/*
	 * Temperature is reported as Eight-bit 2’s complement sensor temperature value
	 * with 0.5 °C/LSB sensitivity and an offset of 23.0 °C
	 */
	//_px4_accel.set_temperature((report.temp * 0.5f) + 23.0f);

	// Accel_X_in_mg = Accel_X_int16 / 32768 * 1000 * 2^(<0x41> + 1) * 1.5
	//_px4_accel.update(timestamp_sample, report.accel_x, report.accel_y, report.accel_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI088_Accelerometer::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_duplicates);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI088_ACCEL_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = registerRead(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	_px4_accel.print_status();
}

void
BMI088_Accelerometer::print_registers()
{
	uint8_t index = 0;
	printf("BMI088 accel registers\n");

	Register reg = _checked_registers[index++];
	uint8_t v = registerRead(reg);
	printf("Accel Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Accel Bw: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Accel Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Accel Int-en-1: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Accel Int-Map-1: %02x:%02x ", (unsigned)reg, (unsigned)v);

	printf("\n");
}

} // namespace Bosch_BMI088_Accelerometer
