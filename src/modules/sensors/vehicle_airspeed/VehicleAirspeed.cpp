/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "VehicleAirspeed.hpp"

#include <drivers/drv_sensor.h>
#include <lib/airspeed/airspeed.h>
#include <px4_platform_common/log.h>

namespace sensors
{

VehicleAirspeed::VehicleAirspeed(uint8_t instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_instance(instance),
	_sensor_sub{this, ORB_ID(differential_pressure), instance}
{
	_airspeed_validator.set_timeout(300000);
	_airspeed_validator.set_equal_value_threshold(100);

	_airspeed_pub.advertise();
}

VehicleAirspeed::~VehicleAirspeed()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleAirspeed::Start()
{
	ParametersUpdate(true);

	return _sensor_sub.registerCallback();
}

void VehicleAirspeed::Stop()
{
	Deinit();

	_sensor_sub.unregisterCallback();
}

void VehicleAirspeed::ResetAccumulation()
{
	_timestamp_sample_sum = 0;
	_differential_pressure_sum = 0.f;
	_baro_pressure_sum = 0.f;
	_temperature_sum = 0.f;
	_sample_count = 0;
}

void VehicleAirspeed::ParametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();
	}
}

void VehicleAirspeed::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	differential_pressure_s diff_pres;

	while (_sensor_sub.update(&diff_pres)) {

		if (_calibration.device_id() != diff_pres.device_id) {
			_calibration.set_device_id(diff_pres.device_id);

			// take over a calibration migrated from the old global SENS_DPRES_OFF, if any
			if (_calibration.AdoptUnclaimedCalibration()) {
				_calibration.ParametersSave();
				param_notify_changes();
			}
		}

		if (!PX4_ISFINITE(diff_pres.differential_pressure_pa)) {
			// ignore invalid data and reset accumulated
			ResetAccumulation();
			continue;
		}

		vehicle_air_data_s air_data{};
		_vehicle_air_data_sub.copy(&air_data);

		// push raw data into validator
		float airspeed_input[3] { diff_pres.differential_pressure_pa, 0.0f, 0.0f };
		_airspeed_validator.put(diff_pres.timestamp_sample, airspeed_input, diff_pres.error_count, 100);

		// accumulate average for publication
		_timestamp_sample_sum += diff_pres.timestamp_sample;
		_differential_pressure_sum += diff_pres.differential_pressure_pa;
		_baro_pressure_sum += air_data.baro_pressure_pa;
		_temperature_sum += air_data.ambient_temperature;
		_sample_count++;
	}

	if ((_sample_count > 0) && (hrt_elapsed_time(&_last_publish) >= kPublishInterval)) {

		// average the accumulated data and remove this sensor's zero offset (CAL_DPRESx_OFF)
		const uint64_t timestamp_sample = _timestamp_sample_sum / _sample_count;
		const float differential_pressure_pa = _calibration.Correct(_differential_pressure_sum / _sample_count);
		const float baro_pressure_pa = _baro_pressure_sum / _sample_count;
		const float temperature = _temperature_sum / _sample_count;

		ResetAccumulation();

		enum AIRSPEED_SENSOR_MODEL smodel;

		switch ((_calibration.device_id() >> 16) & 0xFF) {
		case DRV_DIFF_PRESS_DEVTYPE_SDP31:

		// fallthrough
		case DRV_DIFF_PRESS_DEVTYPE_SDP32:

		// fallthrough
		case DRV_DIFF_PRESS_DEVTYPE_SDP33:
			smodel = AIRSPEED_SENSOR_MODEL_SDP3X;
			break;

		default:
			smodel = AIRSPEED_SENSOR_MODEL_MEMBRANE;
			break;
		}

		const float indicated_airspeed_m_s = calc_IAS_corrected((enum AIRSPEED_COMPENSATION_MODEL)_param_cal_air_cmodel.get(),
						     smodel, _param_cal_air_tubelen.get(), _param_cal_air_tubed_mm.get(),
						     differential_pressure_pa, baro_pressure_pa, temperature);

		// assume that CAS = IAS as we don't have an CAS-scale here
		const float true_airspeed_m_s = calc_TAS_from_CAS(indicated_airspeed_m_s, baro_pressure_pa, temperature);

		if (PX4_ISFINITE(indicated_airspeed_m_s) && PX4_ISFINITE(true_airspeed_m_s)) {
			airspeed_s airspeed{};
			airspeed.timestamp_sample = timestamp_sample;
			airspeed.device_id = _calibration.device_id();
			airspeed.indicated_airspeed_m_s = indicated_airspeed_m_s;
			airspeed.true_airspeed_m_s = true_airspeed_m_s;
			airspeed.confidence = _airspeed_validator.confidence(hrt_absolute_time());
			airspeed.timestamp = hrt_absolute_time();
			_airspeed_pub.publish(airspeed);

			_last_publish = airspeed.timestamp;
		}
	}

	perf_end(_cycle_perf);
}

void VehicleAirspeed::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_airspeed] instance %" PRIu8 ", airspeed instance %d\n", _instance,
		     _airspeed_pub.get_instance());

	_calibration.PrintStatus();
	_airspeed_validator.print();
	PX4_INFO_RAW("\n");

	perf_print_counter(_cycle_perf);
}

} // namespace sensors
