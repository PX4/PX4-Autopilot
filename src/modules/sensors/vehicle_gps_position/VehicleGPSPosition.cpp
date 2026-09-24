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

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_slot_binder.init("SENS_GPS%u_ID", GPS_MAX_RECEIVERS);
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

	for (auto &sub : _sensor_gps_sub) {
		sub.registerCallback();
	}

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

		_gps_param_slots[0] = {
			{_param_sens_gps0_offx.get(), _param_sens_gps0_offy.get(), _param_sens_gps0_offz.get()},
			static_cast<hrt_abstime>(_param_sens_gps0_delay.get()) * 1000
		};
		_gps_param_slots[1] = {
			{_param_sens_gps1_offx.get(), _param_sens_gps1_offy.get(), _param_sens_gps1_offz.get()},
			static_cast<hrt_abstime>(_param_sens_gps1_delay.get()) * 1000
		};
	}
}

int8_t VehicleGPSPosition::receiverSlot(uint8_t instance, uint32_t device_id)
{
	const int8_t slot = _slot_binder.slotForInstanceWithFallback(instance, device_id);

	if ((slot < 0) && !_no_slot_warned[instance]) {
		PX4_WARN("GPS %" PRIu8 " (device ID %" PRIu32 ") ignored, no free SENS_GPS slot", instance, device_id);
		_no_slot_warned[instance] = true;
	}

	_instance_slot[instance] = slot;
	return slot;
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	pps_capture_s pps_capture;

	if (_pps_capture_sub.update(&pps_capture)) {
		_pps_time_sync.process_pps(pps_capture);
	}

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		sensor_gps_s gps_data;

		if (!_sensor_gps_sub[i].update(&gps_data)) {
			continue;
		}

		const int8_t slot = receiverSlot(i, gps_data.device_id);

		if (slot < 0) {
			continue;
		}

		const GpsParamSlot &param_slot = _gps_param_slots[slot];

		// Apply delay to timestamp_sample if the driver didn't set one
		if (gps_data.timestamp_sample == 0 || gps_data.timestamp_sample == gps_data.timestamp) {
			if (param_slot.delay_us > 0 && gps_data.timestamp > param_slot.delay_us) {
				gps_data.timestamp_sample = gps_data.timestamp - param_slot.delay_us;
			}
		}

		const uint64_t pps_timestamp = _pps_time_sync.correct_gps_timestamp(gps_data.timestamp, gps_data.time_utc_usec);

		if (pps_timestamp != gps_data.timestamp) {
			// PPS provided a correction, use it instead of the per-receiver delay
			gps_data.timestamp_sample = pps_timestamp;
		}

		gps_data.antenna_offset_x = param_slot.offset(0);
		gps_data.antenna_offset_y = param_slot.offset(1);
		gps_data.antenna_offset_z = param_slot.offset(2);

		_vehicle_gps_position_pubs.publish(slot, gps_data);
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::PrintStatus()
{
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		PX4_INFO_RAW("[vehicle_gps_position] sensor_gps %" PRIu8 " -> slot %" PRIi8 "\n", i, _instance_slot[i]);
	}
}

}; // namespace sensors
