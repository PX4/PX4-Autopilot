/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_gps_position_pub.advertise();
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		for (int instance = 0; instance < GPS_MAX_RECEIVERS; instance++) {
			_configuration[instance].ParametersUpdate();

			_gps_blending.setAntennaOffset(_configuration[instance].position(), instance);
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());

		// TODO: select highest priority
		int primary_instance = -1;
		uint8_t highest_priority = 0;

		for (int instance = 0; instance < GPS_MAX_RECEIVERS; instance++) {
			if (_configuration[instance].enabled() && _configuration[instance].priority() > highest_priority) {
				primary_instance = instance;
				highest_priority = _configuration[instance].priority();
			}
		}

		if (primary_instance >= 0) {
			//_gps_blending.setPrimaryInstance(primary_instance);
		}
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// update attitude
	if (_vehicle_attitude_sub.updated()) {
		// Transform offset from NED to body frame
		vehicle_attitude_s attitude;

		if (_vehicle_attitude_sub.update(&attitude)) {
			// Quaternion rotation from the FRD body frame to the NED earth frame
			_gps_blending.setAttitude(matrix::Quatf{attitude.q});
		}
	}

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			if (_sensor_gps_sub[i].copy(&gps_data)) {
				_configuration[i].set_device_id(gps_data.device_id);

				_gps_blending.setGpsData(gps_data, i);

				if (!_sensor_gps_sub[i].registered()) {
					_sensor_gps_sub[i].registerCallback();
				}
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			sensor_gps_s gps_output{_gps_blending.getOutputGpsData()};

			const int selected_instance = _gps_blending.getSelectedGps();

			// clear device_id if blending
			if (selected_instance == GpsBlending::GPS_MAX_RECEIVERS_BLEND) {
				gps_output.device_id = 0;

				_gps_blending.blended_antenna_offset().copyTo(gps_output.position_offset);

			} else if (selected_instance >= 0 && selected_instance < GpsBlending::GPS_MAX_RECEIVERS_BLEND) {

				_configuration[selected_instance].position().copyTo(gps_output.position_offset);
			}

			_vehicle_gps_position_pub.publish(gps_output);

			// populate initial SENS_GNSSx configuration slots if necessary
			for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
				if ((_configuration[i].device_id() != 0) && (!_configuration[i].configured())) {
					_configuration[i].ParametersSave(i);
				}
			}
		}
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_blending.getSelectedGps());

	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_configuration[i].device_id() != 0) {
			_configuration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
