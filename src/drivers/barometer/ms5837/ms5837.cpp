/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file ms5837.cpp
 * Driver for the MS5837 barometric pressure sensor connected via I2C.
 */

#include "ms5837.hpp"

#include <cdev/CDev.hpp>

MS5837::MS5837(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_barometer(get_device_id()),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{

}

MS5837::~MS5837()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
}

int
MS5837::init()
{

	int ret = I2C::init();

	if (ret!= PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;

	while (true) {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		px4_usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_MS5837);

		ret = OK;

		break;

	}

	if (ret == 0) {
		start();
	}

	return ret;
}

int MS5837::_reset()
{
	unsigned	old_retrycount = _retries;
	uint8_t		cmd = MS5837_RESET;
	int		result;

	/* bump the retry count */
	_retries = 3;
	result = transfer(&cmd, 1, nullptr, 0);
	_retries = old_retrycount;

	return result;
}

int MS5837::probe()
{
	if ((PX4_OK == _probe_address(MS5837_ADDRESS))){

		return PX4_OK;
	}

	_retries = 1;

	return -EIO;
}

int
MS5837::_probe_address(uint8_t address)
{
	/* select the address we are going to try */
	set_device_address(address);

	/* send reset command */
	if (PX4_OK != _reset()) {
		return -EIO;
	}

	/* read PROM */
	if (PX4_OK != _read_prom()) {
		return -EIO;
	}

	return PX4_OK;
}


int
MS5837::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[3];

	/* read the most recent measurement */
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, &buf[0], 3);

	if (ret == PX4_OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[2];
		cvt->b[1] = buf[1];
		cvt->b[2] = buf[0];
		cvt->b[3] = 0;
	}

	return ret;
}

void
MS5837::RunImpl()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The ms5837 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//DEVICE_LOG("collection error %d", ret);
			}

			/* issue a reset command to the sensor */
			_reset();
			/* reset the collection state machine and try again - we need
			 * to wait 2.8 ms after issuing the sensor reset command
			 * according to the MS5837 datasheet
			 */
			ScheduleDelayed(2800);
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		/* issue a reset command to the sensor */
		_reset();
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(MS5837_CONVERSION_INTERVAL);
}

void
MS5837::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;

	/* schedule a cycle to start things */
	ScheduleDelayed(MS5837_CONVERSION_INTERVAL);
}

int
MS5837::measure()
{
	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	uint8_t cmd = addr;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	_px4_barometer.set_error_count(perf_event_count(_comms_errors));

	perf_end(_measure_perf);

	return ret;
}

int
MS5837::collect()
{
	uint32_t raw;

	perf_begin(_sample_perf);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.s.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.s.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */

		/* Perform MS5837 Caculation */
		_OFF  = ((int64_t)_prom.s.c2_pressure_offset << 16) + (((int64_t)_prom.s.c4_temp_coeff_pres_offset * dT) >> 7);
		_SENS = ((int64_t)_prom.s.c1_pressure_sens << 15) + (((int64_t)_prom.s.c3_temp_coeff_pres_sens * dT) >> 8);

		/* MS5837 temperature compensation */
		int64_t T2 =0;

		int64_t f = 0;
		int64_t OFF2 = 0;
		int64_t SENS2 = 0;
		if (TEMP < 2000) {

			T2 = 3*((int64_t)POW2(dT) >> 33);

			f = POW2((int64_t)TEMP - 2000);
			OFF2 = 3 * f >> 1;
			SENS2 = 5 * f >> 3;

			if (TEMP < -1500) {

				int64_t f2 = POW2(TEMP + 1500);
				OFF2 += 7 * f2;
				SENS2 += f2 << 2;
			}

		} else if (TEMP >= 2000) {
			T2 = 2*((int64_t)POW2(dT) >> 37);

			f = POW2((int64_t)TEMP - 2000);
			OFF2 = 1 * f >> 4;
			SENS2 = 0;
		}
		TEMP -= (int32_t)T2;
		_OFF  -= OFF2;
		_SENS -= SENS2;


		float temperature = TEMP / 100.0f;
		_px4_barometer.set_temperature(temperature);

	} else {
		/* pressure calculation, result in Pa */
		int32_t P = (((raw * _SENS) >> 21) - _OFF) >> 13;

		float pressure = P / 10.0f;		/* convert to millibar */

		_px4_barometer.update(timestamp_sample, pressure);
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, MS5837_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void MS5837::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("pressure:         %f\n", (double)_px4_barometer.get().pressure);
	printf("temperature:      %f\n", (double)_px4_barometer.get().temperature);
}


int
MS5837::_read_prom()
{
	uint8_t		prom_buf[2];
	union {
		uint8_t		b[2];
		uint16_t	w;
	} cvt;

	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	px4_usleep(3000);

	uint8_t last_val = 0;
	bool bits_stuck = true;

	/* read and convert PROM words */
	for (int i = 0; i < 7; i++) {
		uint8_t cmd = MS5837_PROM_READ + (i * 2);

		if (PX4_OK != transfer(&cmd, 1, &prom_buf[0], 2)) {
			break;
		}

		/* check if all bytes are zero */
		if (i == 0) {
			/* initialize to first byte read */
			last_val = prom_buf[0];
		}

		if ((prom_buf[0] != last_val) || (prom_buf[1] != last_val)) {
			bits_stuck = false;
		}

		/* assemble 16 bit value and convert from big endian (sensor) to little endian (MCU) */
		cvt.b[0] = prom_buf[1];
		cvt.b[1] = prom_buf[0];
		_prom.c[i] = cvt.w;
	}

	/* calculate CRC and return success/failure accordingly */
	return (_crc4(&_prom.c[0]) && !bits_stuck) ? PX4_OK : -EIO;
}

/**
 * MS5837 crc4 cribbed from the datasheet
 */
bool
MS5837::_crc4(uint16_t *n_prom)
{
	uint16_t n_rem = 0;
	uint16_t crcRead = n_prom[0] >> 12;
	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return (n_rem ^ 0x00) == crcRead;
}
