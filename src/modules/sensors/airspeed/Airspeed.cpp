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

#include "Airspeed.hpp"

#include <px4_platform_common/log.h>
#include <drivers/drv_airspeed.h>
#include <lib/ecl/geo/geo.h>

#include <lib/airspeed/airspeed.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f

Airspeed::Airspeed() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_voter.set_timeout(SENSOR_TIMEOUT);
	_voter.set_equal_value_threshold(100);

	ParametersUpdate(true);
}

Airspeed::~Airspeed()
{
	Stop();
	perf_free(_cycle_perf);
}

bool Airspeed::Start()
{
	ScheduleNow();
	return true;
}

void Airspeed::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void Airspeed::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		/* update airspeed scale */
		int fd = px4_open(AIRSPEED0_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */
		if (fd >= 0) {
			struct airspeed_scale airscale = {
				_param_sens_dpres_off.get(),
				1.0f,
			};

			if (OK != px4_ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				PX4_ERR("failed to set offset for differential pressure sensor");
			}

			px4_close(fd);
		}
	}
}

void Airspeed::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	if (_vehicle_air_data_sub.updated()) {

		vehicle_air_data_s air_data;

		if (_vehicle_air_data_sub.copy(&air_data)) {
			if ((air_data.timestamp != 0) && PX4_ISFINITE(air_data.baro_temp_celcius)
			    && (air_data.baro_temp_celcius >= -40.f) && (air_data.baro_temp_celcius <= 125.f)) {

				// TODO: review PCB_TEMP_ESTIMATE_DEG, ignore for external baro
				_baro_air_temperature = air_data.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG;
			}
		}
	}

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {

		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if ((_last_data[uorb_index].timestamp == 0) || (hrt_elapsed_time(&_last_data[uorb_index].timestamp) > 1_s)) {
				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for %s %i", "DPRES", uorb_index);
						}
					}

					_advertised[uorb_index] = true;

					// advertise outputs in order if publishing all
					if (!_param_sens_dpres_mode.get()) {
						for (int instance = 0; instance < uorb_index; instance++) {
							_airspeed_multi_pub[instance].advertise();
						}
					}

					if (_selected_sensor_sub_index < 0) {
						_sensor_sub[uorb_index].registerCallback();
					}

				} else {
					_last_data[uorb_index].timestamp = hrt_absolute_time();
				}
			}
		}

		if (_advertised[uorb_index]) {
			differential_pressure_s diff_pres;

			while (_sensor_sub[uorb_index].update(&diff_pres)) {
				updated[uorb_index] = true;

				_device_id[uorb_index] = diff_pres.device_id;

				float vect[3] {diff_pres.differential_pressure_raw_pa, diff_pres.temperature, 0.f};
				_voter.put(uorb_index, diff_pres.timestamp, vect, diff_pres.error_count, _priority[uorb_index]);


				float air_temperature_celsius = NAN;

				// assume anything outside of a (generous) operating range of -40C to 125C is invalid
				if (PX4_ISFINITE(diff_pres.temperature) && (diff_pres.temperature >= -40.f) && (diff_pres.temperature <= 125.f)) {

					air_temperature_celsius = diff_pres.temperature;

				} else {
					// differential pressure temperature invalid, use barometer temperature if available
					if (PX4_ISFINITE(_baro_air_temperature)) {
						air_temperature_celsius = _baro_air_temperature;
					}
				}

				// average raw data for all instances
				_timestamp_sample_sum[uorb_index] += diff_pres.timestamp;
				_differential_pressure_sum[uorb_index] += diff_pres.differential_pressure_filtered_pa;
				_temperature_sum[uorb_index] += air_temperature_celsius;
				_sum_count[uorb_index]++;
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_param_sens_dpres_mode.get()) {
				if (_selected_sensor_sub_index >= 0) {
					PX4_INFO("%s switch from #%u -> #%d", "DPRES", _selected_sensor_sub_index, best_index);
				}
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	// Publish
	if (_param_sens_dpres_mode.get()) {
		// publish only best sensor
		if ((_selected_sensor_sub_index >= 0)
		    && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR)
		    && updated[_selected_sensor_sub_index]) {

			Publish(_selected_sensor_sub_index);
		}

	} else {
		// publish all
		for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
			// publish all sensors as separate instances
			if (updated[uorb_index] && (_device_id[uorb_index] != 0)) {
				Publish(uorb_index, true);
			}
		}
	}


	// check failover and report
	if (_param_sens_dpres_mode.get()) {
		if (_last_failover_count != _voter.failover_count()) {
			uint32_t flags = _voter.failover_state();
			int failover_index = _voter.failover_index();

			if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
				if (failover_index != -1) {
					const hrt_abstime now = hrt_absolute_time();

					if (now - _last_error_message > 3_s) {
						mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!",
								      "DPRES",
								      failover_index,
								      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
								      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
								      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
						_last_error_message = now;
					}

					// reduce priority of failed sensor to the minimum
					_priority[failover_index] = 1;
				}
			}

			_last_failover_count = _voter.failover_count();
		}
	}

	// reschedule timeout
	ScheduleDelayed(100_ms);

	perf_end(_cycle_perf);
}

void Airspeed::Publish(uint8_t instance, bool multi)
{
	if ((_param_sens_dpres_rate.get() > 0)
	    && hrt_elapsed_time(&_last_publication_timestamp[instance]) >= (1e6f / _param_sens_dpres_rate.get())) {

		const float differential_pressure = _differential_pressure_sum[instance] / _sum_count[instance];
		const float temperature = _temperature_sum[instance] / _sum_count[instance];
		const hrt_abstime timestamp_sample = _timestamp_sample_sum[instance] / _sum_count[instance];

		// reset
		_timestamp_sample_sum[instance] = 0;
		_differential_pressure_sum[instance] = 0;
		_temperature_sum[instance] = 0;
		_sum_count[instance] = 0;

		airspeed_s out{};
		out.timestamp_sample = timestamp_sample;
		out.air_temperature_celsius = temperature;
		out.confidence = 1.f; // TODO

		switch ((_device_id[instance] >> 16) & 0xFF) {
		case DRV_DIFF_PRESS_DEVTYPE_SDP31:

		// fallthrough
		case DRV_DIFF_PRESS_DEVTYPE_SDP32:

		// fallthrough
		case DRV_DIFF_PRESS_DEVTYPE_SDP33:
			out.indicated_airspeed_m_s = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL) _param_cal_air_cmodel.get(),
						     AIRSPEED_SENSOR_MODEL_SDP3X, _param_cal_air_tubelen.get(), _param_cal_air_tubed_mm.get(),
						     differential_pressure, _baro_pressure_pa, temperature);
			break;

		default:
			out.indicated_airspeed_m_s = calc_IAS(differential_pressure);
			break;
		}

		// assume that CAS = IAS as we don't have an CAS-scale here
		out.true_airspeed_m_s = calc_TAS_from_CAS(out.indicated_airspeed_m_s, _baro_pressure_pa, temperature);

		if (PX4_ISFINITE(out.indicated_airspeed_m_s) && PX4_ISFINITE(out.true_airspeed_m_s)) {
			out.timestamp = hrt_absolute_time();

			if (multi) {
				_airspeed_multi_pub[instance].publish(out);

			} else {
				// otherwise only ever publish the first instance
				_airspeed_multi_pub[0].publish(out);
			}

			_last_publication_timestamp[instance] = out.timestamp;
		}
	}
}

void Airspeed::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected differential pressure: %d (%d)", _device_id[_selected_sensor_sub_index], _selected_sensor_sub_index);
	}

	_voter.print();
}

}; // namespace sensors
