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

#include "MS5837.hpp"
#include "ms5837.h"

#include <cdev/CDev.hpp>

MS5837::MS5837(device::Device *interface, ms5837::prom_u &prom_buf, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_px4_barometer(interface->get_device_id()),
	_interface(interface),
	_prom(prom_buf.s),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
	switch (config.devid_driver_index) {
	case DRV_BARO_DEVTYPE_MS5837:
		_device_type = MS5837_DEVICE;
		break;
	default:
		_device_type = MS5837_DEVICE;
		break;
	}
}

MS5837::~MS5837()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
MS5837::init()
{
	int ret;

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

		/* state machine will have generated a report, copy it out */
		//const sensor_baro_s &brp = _px4_barometer.get();

		switch (_device_type) {
		default:

		/* fall through */
		case MS5837_DEVICE:
			_interface->set_device_type(DRV_BARO_DEVTYPE_MS5837);
			_px4_barometer.set_device_type(DRV_BARO_DEVTYPE_MS5837);
			break;
		}

		ret = OK;

		break;

	}

	if (ret == 0) {
		start();
	}

	return ret;
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

void
MS5837::RunImpl()
{
	int ret;
	unsigned dummy;

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
			_interface->ioctl(IOCTL_RESET, dummy);
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
		_interface->ioctl(IOCTL_RESET, dummy);
		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
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
	int ret = _interface->ioctl(IOCTL_MEASURE, addr);

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
	int ret = _interface->read(0, (void *)&raw, 0);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {

		/* temperature offset (in ADC units) */
		int32_t dT = (int32_t)raw - ((int32_t)_prom.c5_reference_temp << 8);

		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.c6_temp_coeff_temp) >> 23);

		/* base sensor scale/offset values */
		if (_device_type == MS5837_DEVICE) {
			/* Perform MS5837 Caculation */
			_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);
			_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);

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
		}

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
	printf("temperature:         %f\n", (double)_px4_barometer.get().temperature);
}

namespace ms5837
{

/**
 * MS5837 crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
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

} // namespace ms5837
