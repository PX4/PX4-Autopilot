/****************************************************************************
 *
 *   Copyright (c) 2017-2024 PX4 Development Team. All rights reserved.
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

#include "AUAV.hpp"
#include "AUAV_Absolute.hpp"
#include "AUAV_Differential.hpp"

I2CSPIDriverBase *AUAV::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	AUAV *instance = nullptr;

	if (config.devid_driver_index == DRV_DIFF_PRESS_DEVTYPE_AUAV) {
		instance = new AUAV_Differential(config);

	} else if (config.devid_driver_index == DRV_BARO_DEVTYPE_AUAV) {
		instance = new AUAV_Absolute(config);
	}

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

AUAV::AUAV(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

AUAV::~AUAV()
{
	perf_free(_comms_errors);
}

void AUAV::RunImpl()
{
	switch (_state) {
	case STATE::READ_CALIBDATA:
		handle_state_read_calibdata();
		break;

	case STATE::REQUEST_MEASUREMENT:
		handle_state_request_measurement();
		break;

	case STATE::GATHER_MEASUREMENT:
		handle_state_gather_measurement();
		break;
	}
}

void AUAV::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
}

int AUAV::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	ScheduleClear();
	ScheduleNow();
	return OK;
}

void AUAV::handle_state_read_calibdata()
{
	int status = PX4_OK;
	calib_eeprom_addr_t calib_data_eeprom_addr = get_calib_eeprom_addr();
	calib_data_raw_t calib_data_raw {};

	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_a_hw, calib_data_raw.a_hw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_a_lw, calib_data_raw.a_lw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_b_hw, calib_data_raw.b_hw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_b_lw, calib_data_raw.b_lw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_c_hw, calib_data_raw.c_hw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_c_lw, calib_data_raw.c_lw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_d_hw, calib_data_raw.d_hw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_d_lw, calib_data_raw.d_lw);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_tc50, calib_data_raw.tc50);
	status = status || read_calibration_eeprom(calib_data_eeprom_addr.addr_es, calib_data_raw.es);

	if (status == PX4_OK) {
		convert_raw_calib_data(calib_data_raw);
		_state = STATE::REQUEST_MEASUREMENT;
		ScheduleNow();

	} else {
		perf_count(_comms_errors);
		ScheduleDelayed(1_ms);
	}
}

void AUAV::handle_state_request_measurement()
{
	uint8_t req_data = 0xAA;
	int status = transfer(&req_data, sizeof(req_data), nullptr, 0);

	if (status == PX4_OK) {
		_state = STATE::GATHER_MEASUREMENT;
		ScheduleDelayed(get_conversion_interval());

	} else {
		perf_count(_comms_errors);
		ScheduleDelayed(1_ms);
	}
}

void AUAV::handle_state_gather_measurement()
{
	uint8_t res_data[7];
	int status = transfer(nullptr, 0, res_data, sizeof(res_data));

	/* Continue processing if the transfer was a success and bit 5 of the status is set to 0 (indicating the sensor is finished) */
	if (status == PX4_OK && (res_data[0] & 0x20) == 0) {
		const hrt_abstime timestamp_sample = hrt_absolute_time();

		uint32_t pressure = (res_data[1] << 16) | (res_data[2] << 8) | (res_data[3]);
		uint32_t temperature = (res_data[4] << 16) | (res_data[5] << 8) | (res_data[6]);

		float corrected_pressure = correct_pressure(pressure, temperature);
		float absolute_pressure = 250.f + 1.25f * (corrected_pressure - (0.1f * (1 << 24))) / (1 << 24) * 1000.f;
		float absolute_pressure_p = absolute_pressure * 100;
		float corrected_temperature = ((temperature * 155.0f) / (1 << 24)) - 45.0f;

		publish_pressure(absolute_pressure_p, corrected_temperature, timestamp_sample);

		_state = STATE::REQUEST_MEASUREMENT;
		ScheduleNow();

	} else {
		perf_count(_comms_errors);
		_state = STATE::REQUEST_MEASUREMENT;
		ScheduleDelayed(1_ms);
	}
}

int AUAV::read_calibration_eeprom(uint8_t eeprom_address, uint16_t &data)
{
	uint8_t req_data[3] = {eeprom_address, 0x0, 0x0};
	int status = transfer(req_data, sizeof(req_data), nullptr, 0);

	/* Wait for the EEPROM read access. Worst case is 2000us */
	px4_usleep(2000);

	uint8_t res_data[3];
	status |= transfer(nullptr, 0, res_data, sizeof(res_data));

	/* If bit 5 is set to 1, the sensor is still busy. This read is considered invalid */
	if (res_data[0] & 0x20) {
		status = PX4_ERROR;
	}

	data = res_data[1] << 8 | res_data[2];
	return status;
}

void AUAV::convert_raw_calib_data(calib_data_raw_t calib_data_raw)
{
	/* Conversion of calib data as described in the datasheet */
	_calib_data.a = (float)((calib_data_raw.a_hw << 16) | calib_data_raw.a_lw) / 0x7FFFFFFF;
	_calib_data.b = (float)((calib_data_raw.b_hw << 16) | calib_data_raw.b_lw) / 0x7FFFFFFF;
	_calib_data.c = (float)((calib_data_raw.c_hw << 16) | calib_data_raw.c_lw) / 0x7FFFFFFF;
	_calib_data.d = (float)((calib_data_raw.d_hw << 16) | calib_data_raw.d_lw) / 0x7FFFFFFF;

	_calib_data.tc50h = (float)(calib_data_raw.tc50 >> 8) / 0x7F;
	_calib_data.tc50l = (float)(calib_data_raw.tc50 & 0xFF) / 0x7F;
	_calib_data.es = (float)(calib_data_raw.es & 0xFF) / 0x7F;
}

float AUAV::correct_pressure(uint32_t pressure, uint32_t temperature)
{
	/* Correct the pressure using the calib data as described in the datasheet */
	int32_t p_raw = pressure - 0x800000;
	float p_norm = (float) p_raw / 0x7FFFFF;

	float ap = _calib_data.a * p_norm * p_norm * p_norm;
	float bp = _calib_data.b * p_norm * p_norm;
	float cp = _calib_data.c * p_norm;

	float c_corr = ap + bp + cp + _calib_data.d;
	float p_corr = p_norm + c_corr;

	int32_t t_diff = temperature - 7576807;
	float p_nfso = (p_corr + 1.0f) / 2.0f;

	float tc50;
	float p_diff;

	if (t_diff > 0) {
		tc50 = _calib_data.tc50h;

	} else {
		tc50 = _calib_data.tc50l;
	}

	if (p_nfso > 0.5f) {
		p_diff = p_nfso - 0.5f;

	} else {
		p_diff = 0.5f - p_nfso;
	}

	float t_corr = ((1.0f - (_calib_data.es * 2.5f * p_diff)) * t_diff * tc50) / (100.0f * 100.0f * 167772.2f);
	float p_corrt = p_nfso - t_corr;
	float p_comp = (uint32_t)(p_corrt * (float)0xFFFFFF);
	return p_comp;
}
