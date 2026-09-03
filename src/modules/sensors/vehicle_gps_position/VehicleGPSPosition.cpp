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
#include <lib/gnss/SensorGpsSelector.hpp>
#include <lib/mathlib/mathlib.h>

namespace sensors
{

inline gnssChecksSample gnssSampleFromSensorGpsMsg(const sensor_gps_s &gps)
{
	gnssChecksSample sample{};

	sample.time_us = gps.timestamp_sample;

	sample.lat = gps.latitude_deg;
	sample.lon = gps.longitude_deg;
	sample.alt = static_cast<float>(gps.altitude_msl_m);

	sample.vel = matrix::Vector3f(gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s);

	sample.hacc = gps.eph;
	sample.vacc = gps.epv;
	sample.sacc = gps.s_variance_m_s;

	sample.fix_type = gps.fix_type;
	sample.nsats    = gps.satellites_used;
	sample.pdop     = sqrtf(gps.hdop * gps.hdop + gps.vdop * gps.vdop);

	sample.spoofed = gps.spoofing_state == sensor_gps_s::SPOOFING_STATE_DETECTED;
	sample.jammed  = gps.jamming_state  == sensor_gps_s::JAMMING_STATE_DETECTED;

	return sample;
}

inline sensor_gps_checks_s sensorGpsChecksMsgFromGnssChecks(const GnssChecks &checks, uint64_t timestamp_sample, uint32_t device_id)
{
	sensor_gps_checks_s msg{};

	msg.timestamp = hrt_absolute_time();
	msg.timestamp_sample = timestamp_sample;

	msg.device_id = device_id;

	msg.position_drift_rate_horizontal_m_s = checks.horizontal_position_drift_rate_m_s();
	msg.position_drift_rate_vertical_m_s = checks.vertical_position_drift_rate_m_s();
	msg.filtered_horizontal_speed_m_s = checks.filtered_horizontal_velocity_m_s();

	msg.flags = checks.getFailStatus().value;

	msg.checks_passed = checks.passed();
	msg.initial_checks_passed = checks.initialChecksPassed();

	return msg;
}

VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_gps_position_pub.advertise();
	_vehicle_gps_position_checks_pub.advertise();

	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		_sensor_gps_checks_pub[i].advertise();
	}
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

		for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
			_gnss_checks[i].setParams(
				_param_gps_check.get(),
				_param_req_nsats.get(),
				_param_req_pdop.get(),
				_param_req_eph.get(),
				_param_req_epv.get(),
				_param_req_sacc.get(),
				_param_req_hdrift.get(),
				_param_req_vdrift.get(),
				_param_req_fix.get(),
				_param_ekf2_vel_lim.get(),
				_param_req_gps_h.get()
			);
		}

		_vehicle_gps_position_checks.setParams(
			_param_gps_check.get(),
			_param_req_nsats.get(),
			_param_req_pdop.get(),
			_param_req_eph.get(),
			_param_req_epv.get(),
			_param_req_sacc.get(),
			_param_req_hdrift.get(),
			_param_req_vdrift.get(),
			_param_req_fix.get(),
			_param_ekf2_vel_lim.get(),
			_param_req_gps_h.get()
		);

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());

		const int gps_prime = _param_sens_gps_prime.get();

		if (math::isInRange(gps_prime, -1, 1)) {
			_gps_blending.setPrimaryInstance(gps_prime);
		}

		_gps_param_slots[0] = {
			static_cast<uint32_t>(_param_sens_gps0_id.get()),
			{_param_sens_gps0_offx.get(), _param_sens_gps0_offy.get(), _param_sens_gps0_offz.get()},
			static_cast<hrt_abstime>(_param_sens_gps0_delay.get()) * 1000
		};
		_gps_param_slots[1] = {
			static_cast<uint32_t>(_param_sens_gps1_id.get()),
			{_param_sens_gps1_offx.get(), _param_sens_gps1_offy.get(), _param_sens_gps1_offz.get()},
			static_cast<hrt_abstime>(_param_sens_gps1_delay.get()) * 1000
		};
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	pps_capture_s pps_capture;

	if (_pps_capture_sub.update(&pps_capture)) {
		_pps_time_sync.process_pps(pps_capture);
	}

	_vehicle_land_detected_sub.update(&_vehicle_land_detected);

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;
	const int32_t gps_prime = _param_sens_gps_prime.get();

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps_data);

			// Match device_id to receiver slot
			matrix::Vector3f antenna_offset{};
			hrt_abstime delay_us = 110_ms; // matches SENS_GPS*_DELAY default
			bool matched = false;

			for (uint8_t slot = 0; slot < GPS_MAX_RECEIVERS; slot++) {
				if (_gps_param_slots[slot].device_id != 0
				    && _gps_param_slots[slot].device_id == gps_data.device_id) {
					antenna_offset = _gps_param_slots[slot].offset;
					delay_us = _gps_param_slots[slot].delay_us;
					matched = true;
					break;
				}
			}

			// Fallback: if no device IDs configured, match by instance index
			if (!matched && _gps_param_slots[0].device_id == 0 && _gps_param_slots[1].device_id == 0) {
				antenna_offset = _gps_param_slots[i].offset;
				delay_us = _gps_param_slots[i].delay_us;
			}

			// Apply delay to timestamp_sample if the driver didn't set one
			if (gps_data.timestamp_sample == 0 || gps_data.timestamp_sample == gps_data.timestamp) {
				if (delay_us > 0 && gps_data.timestamp > delay_us) {
					gps_data.timestamp_sample = gps_data.timestamp - delay_us;
				}
			}

			_gnss_checks[i].run(gnssSampleFromSensorGpsMsg(gps_data), !_vehicle_land_detected.landed, _vehicle_land_detected.at_rest);
			sensor_gps_checks_s checks_msg = sensorGpsChecksMsgFromGnssChecks(_gnss_checks[i], gps_data.timestamp_sample, gps_data.device_id);
			_sensor_gps_checks_pub[i].publish(checks_msg);

			_gps_blending.setAntennaOffset(antenna_offset, i);
			_gps_blending.setGpsData(gps_data, i);

			if (SensorGpsSelector::node_id_matches(gps_prime, gps_data.device_id)) {
				_gps_blending.setPrimaryInstance(i);
			}

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			sensor_gps_s gps_output{_gps_blending.getOutputGpsData()};

			// clear device_id if blending
			if (_gps_blending.getSelectedGps() == GpsBlending::GPS_MAX_RECEIVERS_BLEND) {
				gps_output.device_id = 0;
			}

			const matrix::Vector3f &out_offset = _gps_blending.getOutputAntennaOffset();
			gps_output.antenna_offset_x = out_offset(0);
			gps_output.antenna_offset_y = out_offset(1);
			gps_output.antenna_offset_z = out_offset(2);

			const uint64_t pps_timestamp = _pps_time_sync.correct_gps_timestamp(gps_output.timestamp, gps_output.time_utc_usec);

			if (pps_timestamp != gps_output.timestamp) {
				// PPS provided a correction — use it instead of the per-receiver delay
				gps_output.timestamp_sample = pps_timestamp;
			}

			_vehicle_gps_position_checks.run(gnssSampleFromSensorGpsMsg(gps_output), !_vehicle_land_detected.landed, _vehicle_land_detected.at_rest);
			sensor_gps_checks_s checks_msg = sensorGpsChecksMsgFromGnssChecks(_vehicle_gps_position_checks, gps_output.timestamp_sample,
							 gps_output.device_id);

			if ((_vehicle_gps_position_checks.passed() && !_vehicle_land_detected.landed) ||
			    (_vehicle_gps_position_checks.initialChecksPassed() && _vehicle_land_detected.landed)) {
				_vehicle_gps_position_pub.publish(gps_output);
			}

			_vehicle_gps_position_checks_pub.publish(checks_msg);
		}
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_blending.getSelectedGps());
}

}; // namespace sensors
