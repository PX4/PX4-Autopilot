/****************************************************************************
 *
 *   Copyright (c) 2017-2026 PX4 Development Team. All rights reserved.
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

#include "DLVR.hpp"

I2CSPIDriverBase *DLVR::instantiate(const I2CSPIDriverConfig &config, const int runtime_instance)
{
	DLVR *instance = nullptr;

	instance = new DLVR(config);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

DLVR::DLVR(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	I2C::_retries = 5;
}

DLVR::~DLVR()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void DLVR::RunImpl()
{
	gather_measurement();
}

void DLVR::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int DLVR::init()
{
	const int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	int32_t hw_model = 0;

	param_get(param_find("SENS_EN_DLVR"), &hw_model);

	switch (hw_model) {
	case 1: /* DLVR F50D (+- 0.5 inH20) */
		_cal_range = 1.0f;
		_offset_out = 8129.f;
		break;

	case 2: /* DLVR L01D (+- 1 inH20) */
		_cal_range = 2.0f;
		_offset_out = 8129.f;
		break;

	case 3: /* DLVR L02D (+- 2 inH20) */
		_cal_range = 4.0f;
		_offset_out = 8129.f;
		break;

	case 4: /* DLVR L05D (+- 5 inH20) */
		_cal_range = 10.0f;
		_offset_out = 8129.f;
		break;

	case 5: /* DLVR L10D (+- 10 inH20) */
		_cal_range = 20.0f;
		_offset_out = 8129.f;
		break;

	case 6: /* DLVR L20D (+- 20 inH20) */
		_cal_range = 40.0f;
		_offset_out = 8129.f;
		break;

	case 7: /* DLVR L30D (+- 30 inH20) */
		_cal_range = 60.0f;
		_offset_out = 8129.f;
		break;

	case 8: /* DLVR L60D (+- 60 inH20) */
		_cal_range = 120.0f;
		_offset_out = 8129.f;
		break;

	case 9: /* DLVR L01G (0 to 1 inH20) */
		_cal_range = 1.0f;
		_offset_out = 1638.f;
		break;

	case 10: /* DLVR L02G (0 to 2 inH20) */
		_cal_range = 2.0f;
		_offset_out = 1638.f;
		break;

	case 11: /* DLVR L05G (0 to 5 inH20) */
		_cal_range = 5.0f;
		_offset_out = 1638.f;
		break;

	case 12: /* DLVR L10G (0 to 10 inH20) */
		_cal_range = 10.0f;
		_offset_out = 1638.f;
		break;

	case 13: /* DLVR L20G (0 to 20 inH20) */
		_cal_range = 20.0f;
		_offset_out = 1638.f;
		break;

	case 14: /* DLVR L30G (0 to 30 inH20) */
		_cal_range = 30.0f;
		_offset_out = 1638.f;
		break;

	case 15: /* DLVR L60G (0 to 60 inH20) */
		_cal_range = 60.0f;
		_offset_out = 1638.f;
		break;
	}

	ScheduleClear();
	ScheduleNow();

	return PX4_OK;
}

int DLVR::probe()
{
	for (unsigned i = 0; i < 10; i++) {
		// Start All (pressure and temperature)
		// if (transfer(nullptr, 0, &dummy, 0) == PX4_OK) {

		// uint8_t dummy;
		uint8_t res_data[2];
		int status = transfer(nullptr, 0, res_data, sizeof(res_data));

		/* Continue processing if the transfer was a success and bit 31/30 of the status is set to 0 (indicating the sensor is finished and no errors) */
		if (status == PX4_OK && (res_data[0] & 0xC0) == 0) {
			return PX4_OK;
		}

		px4_usleep(10'000);
	}

	return PX4_ERROR;
}

int64_t DLVR::get_conversion_interval() const
{
	return DIFF_CONVERSION_INTERVAL;
}

void DLVR::gather_measurement()
{
	perf_begin(_sample_perf);

	uint8_t res_data[4];
	int status = transfer(nullptr, 0, res_data, sizeof(res_data));

	/* Continue processing if the transfer was a success and bit 31/30 of the status is set to 0 (indicating the sensor is finished and no errors) */
	if (status == PX4_OK && (res_data[0] & 0xC0) == 0) {
		const hrt_abstime timestamp_sample = hrt_absolute_time();

		const uint32_t pressure_raw = ((res_data[0] & 0x3f) << 8) | (res_data[1]);
		const uint32_t temperature_raw = (res_data[2] << 3) | (res_data[3] >> 5);

		const float pressure_p = process_pressure_raw(pressure_raw);
		const float temperature_c = process_temperature_raw(temperature_raw);

		publish_pressure(pressure_p, temperature_c, timestamp_sample);

	} else {
		/* In ca of an error, ignore the results */
		perf_count(_comms_errors);
	}

	ScheduleDelayed(get_conversion_interval());

	perf_end(_sample_perf);
}

float DLVR::process_temperature_raw(const float temperature_raw) const
{
	return temperature_raw * 200.0f / ((1 << 11) - 1) - 50.0f;
}

float DLVR::process_pressure_raw(const float pressure_dig) const
{
	const float pressure_in_h = 1.25f * ((pressure_dig - _offset_out) / (1 << 14)) * _cal_range;
	return pressure_in_h * M_INH_TO_PA;
}

void DLVR::publish_pressure(const float pressure_p, const float temperature_c,
			    const hrt_abstime timestamp_sample)
{
	differential_pressure_s differential_pressure{};
	differential_pressure.timestamp = hrt_absolute_time();
	differential_pressure.timestamp_sample = timestamp_sample;
	differential_pressure.device_id = get_device_id();
	differential_pressure.differential_pressure_pa = pressure_p;
	int32_t differential_press_rev = 0;
	param_get(param_find("SENS_DPRES_REV"), &differential_press_rev);

	//If differential pressure reverse param set, swap positive and negative
	if (differential_press_rev == 1) {
		differential_pressure.differential_pressure_pa = -1.0f * pressure_p;
	}

	differential_pressure.temperature = temperature_c;
	differential_pressure.error_count = perf_event_count(_comms_errors);
	_differential_pressure_pub.publish(differential_pressure);
}
