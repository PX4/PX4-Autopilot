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
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vehicle_gps_position_pub.advertise();
	_vehicle_gnss_heading_pub.advertise();
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

	for (auto &sub : _sensor_gnss_relative_sub) {
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

		for (auto &sub : _sensor_gnss_relative_sub) {
			sub.registerCallback();
		}

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

		updateBaselineRotation(_gps_param_slots[0], _param_sens_gps0_rot.get(), _param_sens_gps0_roll.get(),
				       _param_sens_gps0_pitch.get(), _param_sens_gps0_yaw.get());
		updateBaselineRotation(_gps_param_slots[1], _param_sens_gps1_rot.get(), _param_sens_gps1_roll.get(),
				       _param_sens_gps1_pitch.get(), _param_sens_gps1_yaw.get());
	}
}

void VehicleGPSPosition::updateBaselineRotation(GpsParamSlot &slot, int32_t rotation, float roll_deg, float pitch_deg,
		float yaw_deg)
{
	matrix::Dcmf R;

	const bool custom_set = (fabsf(roll_deg) > FLT_EPSILON) || (fabsf(pitch_deg) > FLT_EPSILON)
				|| (fabsf(yaw_deg) > FLT_EPSILON);

	if ((rotation == ROTATION_CUSTOM) || custom_set) {
		R = matrix::Dcmf(matrix::Eulerf(math::radians(roll_deg), math::radians(pitch_deg), math::radians(yaw_deg)));

	} else if ((rotation >= 0) && (rotation < ROTATION_MAX)) {
		R = get_rot_matrix(static_cast<Rotation>(rotation));
	}

	// baseline direction in the body frame; only its yaw enters the heading
	const matrix::Vector3f baseline = R * matrix::Vector3f(1.f, 0.f, 0.f);
	slot.heading_available = baseline.xy().norm() > 0.1f;
	slot.heading_offset = slot.heading_available ? atan2f(baseline(1), baseline(0)) : 0.f;
}

void VehicleGPSPosition::applyBaselineRotation(const GpsParamSlot *slot, float &heading, float &heading_offset)
{
	if (!PX4_ISFINITE(heading) || PX4_ISFINITE(heading_offset)) {
		return;
	}

	if (slot && !slot->heading_available) {
		heading = NAN;
		return;
	}

	heading_offset = slot ? slot->heading_offset : 0.f;
	heading = matrix::wrap_pi(heading - heading_offset);
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	pps_capture_s pps_capture;

	if (_pps_capture_sub.update(&pps_capture)) {
		_pps_time_sync.process_pps(pps_capture);
	}

	// Check all GPS instance
	bool any_gps_updated = false;
	sensor_gps_s gps_data[GPS_MAX_RECEIVERS] {};
	bool gps_updated[GPS_MAX_RECEIVERS] {};
	const int32_t gps_prime = _param_sens_gps_prime.get();

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated[i] = _sensor_gps_sub[i].update(&gps_data[i]);

		if (gps_updated[i]) {
			any_gps_updated = true;

			const GpsParamSlot *slot = findParamSlot(gps_data[i].device_id, i);
			const matrix::Vector3f antenna_offset = slot ? slot->offset : matrix::Vector3f{};
			const hrt_abstime delay_us = slot ? slot->delay_us : kDefaultDelay;

			gps_data[i].timestamp_sample = resolveSampleTimestamp(gps_data[i].timestamp_sample, gps_data[i].timestamp, delay_us);
			applyBaselineRotation(slot, gps_data[i].heading, gps_data[i].heading_offset);

			_gps_blending.setAntennaOffset(antenna_offset, i);
			_gps_blending.setGpsData(gps_data[i], i);

			if (SensorGpsSelector::node_id_matches(gps_prime, gps_data[i].device_id)) {
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

			_vehicle_gps_position_pub.publish(gps_output);
		}
	}

	UpdateGnssHeading(gps_data, gps_updated);

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::UpdateGnssHeading(const sensor_gps_s gps_data[GPS_MAX_RECEIVERS],
		const bool gps_updated[GPS_MAX_RECEIVERS])
{
	// A single source is published at a time: every source carries its own antenna offset, so alternating between
	// receivers would jump the heading and trip the EKF observation rate limit. The active source is kept until it
	// goes stale. sensor_gnss_relative is preferred over sensor_gps.heading and preempts it.
	//
	// TODO: with per-receiver rotation the selection can also follow the flight phase, e.g. a tailsitter with one
	// baseline aligned for hover and one for forward flight.
	const hrt_abstime now = hrt_absolute_time();
	const bool source_active = (_heading_source.last_publish != 0)
				   && (now - _heading_source.last_publish < kHeadingSourceTimeout);

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		sensor_gnss_relative_s gnss_rel;

		if (!_sensor_gnss_relative_sub[i].update(&gnss_rel)) {
			continue;
		}

		if (!gnss_rel.heading_valid || !PX4_ISFINITE(gnss_rel.heading)) {
			continue;
		}

		if (source_active && _heading_source.from_relative && (_heading_source.device_id != gnss_rel.device_id)) {
			continue;
		}

		// sensor_gnss_relative instances are numbered by advertise order, not by receiver, so the receiver's
		// sensor_gps instance is looked up by device_id for the parameter slot.
		const GpsParamSlot *slot = findParamSlot(gnss_rel.device_id, findGpsInstance(gnss_rel.device_id));
		const hrt_abstime delay_us = slot ? slot->delay_us : kDefaultDelay;

		// the relative position heading is always the raw baseline
		float heading = gnss_rel.heading;
		float heading_offset = NAN;
		applyBaselineRotation(slot, heading, heading_offset);

		if (!PX4_ISFINITE(heading)) {
			continue;
		}

		uint64_t timestamp_sample = resolveSampleTimestamp(gnss_rel.timestamp_sample, gnss_rel.timestamp, delay_us);
		const uint64_t pps_timestamp = _pps_time_sync.correct_gps_timestamp(gnss_rel.timestamp, gnss_rel.time_utc_usec);

		if (pps_timestamp != gnss_rel.timestamp) {
			timestamp_sample = pps_timestamp;
		}

		vehicle_gnss_heading_s heading_out{};
		heading_out.timestamp_sample = timestamp_sample;
		heading_out.device_id = gnss_rel.device_id;
		heading_out.heading = heading;
		heading_out.heading_accuracy = gnss_rel.heading_accuracy;
		heading_out.heading_offset = heading_offset;
		heading_out.timestamp = hrt_absolute_time();
		_vehicle_gnss_heading_pub.publish(heading_out);

		_heading_source = {gnss_rel.device_id, true, now};
		return;
	}

	if (source_active && _heading_source.from_relative) {
		return;
	}

	// Fallback for receivers that only report heading in sensor_gps (e.g. Septentrio). Uses the raw per-instance
	// data rather than the blended output so heading does not follow the position blending weights.
	const int32_t gps_prime = _param_sens_gps_prime.get();
	uint8_t primary = 0;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if ((gps_prime == i) || SensorGpsSelector::node_id_matches(gps_prime, gps_data[i].device_id)) {
			primary = i;
		}
	}

	for (uint8_t n = 0; n < GPS_MAX_RECEIVERS; n++) {
		const uint8_t i = (primary + n) % GPS_MAX_RECEIVERS;

		if (!gps_updated[i] || !PX4_ISFINITE(gps_data[i].heading)) {
			continue;
		}

		if (source_active && (_heading_source.device_id != gps_data[i].device_id)) {
			continue;
		}

		uint64_t timestamp_sample = gps_data[i].timestamp_sample;
		const uint64_t pps_timestamp = _pps_time_sync.correct_gps_timestamp(gps_data[i].timestamp, gps_data[i].time_utc_usec);

		if (pps_timestamp != gps_data[i].timestamp) {
			timestamp_sample = pps_timestamp;
		}

		vehicle_gnss_heading_s heading_out{};
		heading_out.timestamp_sample = timestamp_sample;
		heading_out.device_id = gps_data[i].device_id;
		heading_out.heading = gps_data[i].heading;
		heading_out.heading_accuracy = gps_data[i].heading_accuracy;
		heading_out.heading_offset = gps_data[i].heading_offset;
		heading_out.timestamp = hrt_absolute_time();
		_vehicle_gnss_heading_pub.publish(heading_out);

		_heading_source = {gps_data[i].device_id, false, now};
		return;
	}
}

const VehicleGPSPosition::GpsParamSlot *VehicleGPSPosition::findParamSlot(uint32_t device_id, int instance) const
{
	for (const GpsParamSlot &slot : _gps_param_slots) {
		if ((slot.device_id != 0) && (slot.device_id == device_id)) {
			return &slot;
		}
	}

	// No device IDs configured: match by sensor_gps instance
	if ((_gps_param_slots[0].device_id == 0) && (_gps_param_slots[1].device_id == 0)
	    && (instance >= 0) && (instance < GPS_MAX_RECEIVERS)) {
		return &_gps_param_slots[instance];
	}

	return nullptr;
}

int VehicleGPSPosition::findGpsInstance(uint32_t device_id)
{
	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		sensor_gps_s gps_data;

		if (_sensor_gps_sub[i].copy(&gps_data) && (gps_data.device_id == device_id)) {
			return i;
		}
	}

	return -1;
}

uint64_t VehicleGPSPosition::resolveSampleTimestamp(uint64_t driver_timestamp_sample, uint64_t driver_timestamp,
		hrt_abstime delay_us)
{
	// A sample timestamp within a few ms of the publish timestamp carries no latency information (u-blox stamps
	// sensor_gnss_relative with the parse time), so only a sample time meaningfully earlier than the publish time is
	// trusted. Otherwise the configured receiver delay is applied.
	if ((driver_timestamp_sample != 0) && (driver_timestamp_sample + kSampleTimestampTolerance < driver_timestamp)) {
		return driver_timestamp_sample;
	}

	return (delay_us > 0 && driver_timestamp > delay_us) ? driver_timestamp - delay_us : driver_timestamp;
}

void VehicleGPSPosition::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_blending.getSelectedGps());
}

}; // namespace sensors
