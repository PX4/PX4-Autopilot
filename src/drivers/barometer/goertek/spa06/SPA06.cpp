/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "SPA06.hpp"

SPA06::SPA06(const I2CSPIDriverConfig &config, spa06::ISPA06 *interface) :
	I2CSPIDriver(config),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

SPA06::~SPA06()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);

	delete _interface;
}

/*
float
SPA06::scale_factor(int oversampling_rate)
{
	float k;

	switch (oversampling_rate) {
	case 1:
		k = 524288.0f;
		break;

	case 2:
		k = 1572864.0f;
		break;

	case 4:
		k = 3670016.0f;
		break;

	case 8:
		k = 7864320.0f;
		break;

	case 16:
		k = 253952.0f;
		break;

	case 32:
		k = 516096.0f;
		break;

	case 64:
		k = 1040384.0f;
		break;

	case 128:
		k = 2088960.0f;
		break;

	default:
		k = 0;
		break;
	}

	return k;
}
*/

int
SPA06::calibrate()
{
	uint8_t buf[18];

	_interface->read(SPA06_ADDR_CAL, buf, sizeof(buf));

	_cal.c0 = (uint16_t)buf[0] << 4 | (uint16_t)buf[1] >> 4;
	_cal.c0 = (_cal.c0 & 1 << 11) ? (0xf000 | _cal.c0) : _cal.c0;

	_cal.c1 = (uint16_t)(buf[1] & 0x0f) << 8 | (uint16_t)buf[2];
	_cal.c1 = (_cal.c1 & 1 << 11) ? (0xf000 | _cal.c1) : _cal.c1;

	_cal.c00 = (uint32_t)buf[3] << 12 | (uint32_t)buf[4] << 4 | (uint16_t)buf[5] >> 4;
	_cal.c00 = (_cal.c00 & 1 << 19) ? (0xfff00000 | _cal.c00) : _cal.c00;

	_cal.c10 = (uint32_t)(buf[5] & 0x0f) << 16 | (uint32_t)buf[6] << 8 | (uint32_t)buf[7];
	_cal.c10 = (_cal.c10 & 1 << 19) ? (0xfff00000 | _cal.c10) : _cal.c10;

	_cal.c01 = (uint16_t)buf[8] << 8 | buf[9];
	_cal.c11 = (uint16_t)buf[10] << 8 | buf[11];
	_cal.c20 = (uint16_t)buf[12] << 8 | buf[13];
	_cal.c21 = (uint16_t)buf[14] << 8 | buf[15];
	_cal.c30 = (uint16_t)buf[16] << 8 | buf[17];

	// PX4_INFO("c0:%d \nc1:%d \nc00:%d \nc10:%d \nc01:%d \nc11:%d \nc20:%d \nc21:%d \nc30:%d\n",
	// _cal.c0,_cal.c1,
	// _cal.c00,_cal.c10,
	// _cal.c01,_cal.c11,_cal.c20,_cal.c21,_cal.c30
	// );
	//PX4_DEBUG("c0:%f",_cal.c0);
	return OK;
}
int
SPA06::init()
{
	int8_t tries = 5;
	// reset sensor
	_interface->set_reg(SPA06_VALUE_RESET, SPA06_ADDR_RESET);
	usleep(10000);

	// check id
	if (_interface->get_reg(SPA06_ADDR_ID) != SPA06_VALUE_ID) {
		PX4_DEBUG("id of your baro is not: 0x%02x", SPA06_VALUE_ID);
		return -EIO;
	}

	while (tries--) {
		uint8_t meas_cfg = _interface->get_reg(SPA06_ADDR_MEAS_CFG);

		if (meas_cfg & (1 << 7) && meas_cfg & (1 << 6)) {
			break;
		}

		usleep(10000);
	}

	if (tries < 0) {
		PX4_DEBUG("spa06 cal failed");
		return -EIO;
	}

	// get calibration and pre process them
	calibrate();

	// set config, recommended settings
	_interface->set_reg(_curr_prs_cfg, SPA06_ADDR_PRS_CFG);
	kp = 253952.0f; // refer to scale_factor()
	_interface->set_reg(_curr_tmp_cfg, SPA06_ADDR_TMP_CFG);
	kt = 524288.0f;


	_interface->set_reg(1 << 2, SPA06_ADDR_CFG_REG);
	_interface->set_reg(7, SPA06_ADDR_MEAS_CFG);

	Start();

	return OK;
}

void
SPA06::Start()
{
	// schedule a cycle to start things
	ScheduleNow();
}

void
SPA06::RunImpl()
{
	collect();

	ScheduleDelayed(_measure_interval);
}
int
SPA06::collect()
{
	perf_begin(_sample_perf);

	// this should be fairly close to the end of the conversion, so the best approximation of the time
	const hrt_abstime timestamp_sample = hrt_absolute_time();

	if (_interface->read(SPA06_ADDR_DATA, (uint8_t *)&_data, sizeof(_data)) != OK) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	int32_t temp_raw = (uint32_t)_data.t_msb << 16 | (uint32_t)_data.t_lsb << 8 | (uint32_t)_data.t_xlsb;
	temp_raw = (temp_raw & 1 << 23) ? (0xff000000 | temp_raw) : temp_raw;

	int32_t press_raw = (uint32_t)_data.p_msb << 16 | (uint32_t) _data.p_lsb << 8 | (uint32_t) _data.p_xlsb;
	press_raw = (press_raw & 1 << 23) ? (0xff000000 | press_raw) : press_raw;

	// calculate
	float ftsc = (float)temp_raw / kt;
	float fpsc = (float)press_raw / kp;
	float qua2 = (float)_cal.c10 + fpsc * ((float)_cal.c20 + fpsc * (float)_cal.c30);
	float qua3 = ftsc * fpsc * ((float)_cal.c11 + fpsc * (float)_cal.c21);

	float fp = (float)_cal.c00 + fpsc * qua2 + ftsc * (float)_cal.c01 + qua3;
	float temperature = (float)_cal.c0 * 0.5f + (float)_cal.c1 * ftsc;


	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = timestamp_sample;
	sensor_baro.device_id = _interface->get_device_id();
	sensor_baro.pressure = fp;
	sensor_baro.temperature = temperature;
	sensor_baro.error_count = perf_event_count(_comms_errors);
	sensor_baro.timestamp = hrt_absolute_time();
	_sensor_baro_pub.publish(sensor_baro);

	perf_end(_sample_perf);

	return OK;
}

void
SPA06::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_measure_perf);
	perf_print_counter(_comms_errors);
}
