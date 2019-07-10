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

#include "BMI088_Gyroscope.hpp"

namespace Bosch_BMI088_Gyroscope
{

#define DIR_READ                0x80
#define DIR_WRITE               0x00

static constexpr uint32_t BMI088_BUS_SPEED = 10 * 1000 * 1000;

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
static constexpr Register _checked_registers[] = {
	Register::GYRO_CHIP_ID,
	Register::GYRO_BANDWIDTH
};

BMI088_Gyroscope::BMI088_Gyroscope(int bus, uint32_t device, enum Rotation rotation) :
	SPI("BMI088_GYRO", nullptr, bus, device, SPIDEV_MODE3, BMI088_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_px4_gyro(get_device_id(), ORB_PRIO_DEFAULT, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi088: gyro read")),
	_measure_interval(perf_alloc(PC_INTERVAL, "bmi088: gyro measure interval")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi088: gyro bad transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi088: gyro bad registers"))
{
	_px4_gyro.set_device_type(DRV_DEVTYPE_BMI088);
}

BMI088_Gyroscope::~BMI088_Gyroscope()
{
	// make sure we are truly inactive
	ScheduleClear();

	// delete the perf counter
	perf_free(_sample_perf);
	perf_free(_measure_interval);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
}

int
BMI088_Gyroscope::init()
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
BMI088_Gyroscope::registerRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
BMI088_Gyroscope::registerWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_WRITE;
	cmd[1] = value;
	transfer(cmd, cmd, sizeof(cmd));
}

void
BMI088_Gyroscope::registerSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (!(val & setbits)) {
		val |= setbits;
		registerWrite(reg, val);
	}
}

void
BMI088_Gyroscope::registerClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = registerRead(reg);

	// only write if necessary
	if (val & clearbits) {
		val &= !clearbits;
		registerWrite(reg, val);
	}
}

int
BMI088_Gyroscope::reset()
{
	registerWrite(Register::GYRO_SOFTRESET, 0xB6);	// Soft-reset
	px4_usleep(5000);

#define BMI088_GYR_DRDY_INT_EN      (1<<7)
#define BMI088_GYR_DRDY_INT1        (1<<0)

	registerWrite(Register::GYRO_INT_CTRL, BMI088_GYR_DRDY_INT_EN);		// Enable DRDY interrupt
	registerWrite(Register::INT3_INT4_IO_MAP, BMI088_GYR_DRDY_INT1);	// Map DRDY interrupt on pin INT1

	// TODO: flight test
	//set_gyro_range(BMI088_GYRO_DEFAULT_RANGE_DPS);	// set Gyro range
	// gyro_set_sample_rate(BMI088_GYRO_DEFAULT_RATE);	// set Gyro ODR & Filter Bandwidth

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI088_GYRO_NUM_CHECKED_REGISTERS; i++) {
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
BMI088_Gyroscope::probe()
{
	/* look for device ID */
	uint8_t whoami = registerRead(Register::GYRO_CHIP_ID);

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

int
BMI088_Gyroscope::gyro_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = 0; //BMI088_GYRO_BW_MASK;

	if (frequency <= 100) {
		setbits |= 0; //BMI088_GYRO_RATE_100; /* 32 Hz cutoff */
		//_gyro_sample_rate = 100;

	} else if (frequency <= 250) {
		setbits |= 0; //BMI088_GYRO_RATE_400; /* 47 Hz cutoff */
		//_gyro_sample_rate = 400;

	} else if (frequency <= 1000) {
		setbits |= 0; //BMI088_GYRO_RATE_1000; /* 116 Hz cutoff */
		//_gyro_sample_rate = 1000;

	} else if (frequency > 1000) {
		setbits |= 0; //BMI088_GYRO_RATE_2000; /* 230 Hz cutoff */
		//_gyro_sample_rate = 2000;

	} else {
		return -EINVAL;
	}

	registerSetBits(Register::GYRO_BANDWIDTH, setbits);
	registerClearBits(Register::GYRO_BANDWIDTH, clearbits);;

	return OK;
}

void
BMI088_Gyroscope::registerWriteChecked(Register reg, uint8_t value)
{
	registerWrite(reg, value);

	for (uint8_t i = 0; i < BMI088_GYRO_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI088_Gyroscope::set_gyro_range(unsigned max_dps)
{
	uint8_t setbits = 0;
	uint8_t clearbits = 0; // BMI088_GYRO_RANGE_125_DPS | BMI088_GYRO_RANGE_250_DPS;
	float lsb_per_dps;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 125) {
		//max_gyro_dps = 125;
		lsb_per_dps = 262.4;
		setbits |= 0; // BMI088_GYRO_RANGE_125_DPS;

	} else if (max_dps <= 250) {
		//max_gyro_dps = 250;
		lsb_per_dps = 131.2;
		setbits |= 0; // BMI088_GYRO_RANGE_250_DPS;

	} else if (max_dps <= 500) {
		//max_gyro_dps = 500;
		lsb_per_dps = 65.6;
		setbits |= 0; // BMI088_GYRO_RANGE_500_DPS;

	} else if (max_dps <= 1000) {
		//max_gyro_dps = 1000;
		lsb_per_dps = 32.8;
		setbits |= 0; // BMI088_GYRO_RANGE_1000_DPS;

	} else if (max_dps <= 2000) {
		//max_gyro_dps = 2000;
		lsb_per_dps = 16.4;
		setbits |= 0; // BMI088_GYRO_RANGE_2000_DPS;

	} else {
		return -EINVAL;
	}

	_px4_gyro.set_scale(M_PI_F / (180.0f * lsb_per_dps));

	registerClearBits(Register::GYRO_RANGE, clearbits);
	registerSetBits(Register::GYRO_RANGE, setbits);

	return OK;
}

void
BMI088_Gyroscope::check_registers()
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
			registerWrite(Register::GYRO_SOFTRESET, 0xB6);
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

	_checked_next = (_checked_next + 1) % BMI088_GYRO_NUM_CHECKED_REGISTERS;
}

void
BMI088_Gyroscope::Run()
{

}

void
BMI088_Gyroscope::measure()
{
	// start measuring
	perf_begin(_sample_perf);
	perf_count(_measure_interval);

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	/*
	 * Fetch the full set of measurements from the BMI088 gyro in one pass.
	 */
	//bmi_gyroreport.cmd = BMI088_GYR_X_L | DIR_READ;

	//const hrt_abstime timestamp_sample = hrt_absolute_time();

	// if (OK != transfer((uint8_t *)&bmi_gyroreport, ((uint8_t *)&bmi_gyroreport), sizeof(bmi_gyroreport))) {
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
	const uint64_t error_count  = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);
	_px4_gyro.set_error_count(error_count);

	/*
	 * Temperature is reported as Eight-bit 2’s complement sensor temperature value
	 * with 0.5 °C/LSB sensitivity and an offset of 23.0 °C
	 */
	//_px4_gyro.set_temperature((report.temp * 0.5f) + 23.0f);

	// Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
	//_px4_gyro.update(timestamp_sample, report.gyro_x, report.gyro_y, report.gyro_z);

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI088_Gyroscope::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_interval);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI088_GYRO_NUM_CHECKED_REGISTERS; i++) {
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

	_px4_gyro.print_status();
}

void
BMI088_Gyroscope::print_registers()
{
	uint8_t index = 0;
	printf("BMI088 gyro registers\n");

	Register reg = _checked_registers[index++];
	uint8_t v = registerRead(reg);
	printf("Gyro Chip Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Gyro Power: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Gyro Bw: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Gyro Range: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Gyro Int-en-0: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Gyro Int-en-1: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = _checked_registers[index++];
	v = registerRead(reg);
	printf("Gyro Int-Map-1: %02x:%02x ", (unsigned)reg, (unsigned)v);

	printf("\n");
}

} // namespace Bosch_BMI088_Gyroscope
