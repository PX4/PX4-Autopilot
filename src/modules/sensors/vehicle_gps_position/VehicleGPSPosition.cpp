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
#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
	_vehicle_gnss_heading_pub.advertise();
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
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

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)

	for (auto &sub : _sensor_gnss_relative_sub) {
		sub.unregisterCallback();
	}

#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
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

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)

		for (auto &sub : _sensor_gnss_relative_sub) {
			sub.registerCallback();
		}

#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

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

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
		const matrix::Vector3f baselines[GPS_MAX_RECEIVERS] {
			gnss_heading::configuredBaseline(_param_sens_gnss0_hdg.get(), _gps_param_slots[0].offset, _gps_param_slots[1].offset,
			{_param_sens_gnss0_auxx.get(), _param_sens_gnss0_auxy.get(), _param_sens_gnss0_auxz.get()}),
			gnss_heading::configuredBaseline(_param_sens_gnss1_hdg.get(), _gps_param_slots[1].offset, _gps_param_slots[0].offset,
			{_param_sens_gnss1_auxx.get(), _param_sens_gnss1_auxy.get(), _param_sens_gnss1_auxz.get()}),
		};

		for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
			_gps_param_slots[i].baseline_length = baselines[i].norm();
			_gps_param_slots[i].heading_offset = atan2f(baselines[i](1), baselines[i](0));
		}

#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING
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

	// Check all GPS instance
	bool any_gps_updated = false;
	const int32_t gps_prime = _param_sens_gps_prime.get();

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		sensor_gnss_s gps_data;

		if (_sensor_gps_sub[i].update(&gps_data)) {
			any_gps_updated = true;

			const GpsParamSlot *slot = findParamSlot(gps_data.device_id, i);
			const matrix::Vector3f antenna_offset = slot ? slot->offset : matrix::Vector3f{};
			const hrt_abstime delay_us = slot ? slot->delay_us : kDefaultDelay;

			gps_data.timestamp_sample = resolveSampleTimestamp(gps_data.timestamp_sample, gps_data.timestamp, delay_us);

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
			vehicle_gnss_s gps_output{};
			gps_output.receiver = _gps_blending.getOutputGpsData();

			const int selected_gps = _gps_blending.getSelectedGps();

			// clear device_id if blending
			if (selected_gps == GpsBlending::GPS_MAX_RECEIVERS_BLEND) {
				gps_output.receiver.device_id = 0;
				gps_output.selected_instance = UINT8_MAX;

			} else {
				gps_output.selected_instance = static_cast<uint8_t>(selected_gps);
			}

			const matrix::Vector3f &out_offset = _gps_blending.getOutputAntennaOffset();
			out_offset.copyTo(gps_output.antenna_offset);

			const uint64_t pps_timestamp = _pps_time_sync.correct_gps_timestamp(gps_output.receiver.timestamp,
						       gps_output.receiver.time_utc_usec);

			if (pps_timestamp != gps_output.receiver.timestamp) {
				// PPS provided a correction — use it instead of the per-receiver delay
				gps_output.receiver.timestamp_sample = pps_timestamp;
			}

			gps_output.timestamp_sample = gps_output.receiver.timestamp_sample;
			gps_output.timestamp = hrt_absolute_time();
			_vehicle_gps_position_pub.publish(gps_output);
		}
	}

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
	UpdateGnssHeading();
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
void VehicleGPSPosition::UpdateGnssHeading()
{
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		sensor_gnss_relative_s gnss_rel;

		if (!_sensor_gnss_relative_sub[i].update(&gnss_rel)) {
			continue;
		}

		// sensor_gnss_relative instances are numbered by advertise order, not by receiver, so the receiver's
		// sensor_gnss instance is looked up by device_id for the parameter slot and the receiver state.
		sensor_gnss_s receiver{};
		const GpsParamSlot *slot = findParamSlot(gnss_rel.device_id, findGpsInstance(gnss_rel.device_id, receiver));

		HeadingSample sample{};
		sample.timestamp_sample = resolveSampleTimestamp(gnss_rel.timestamp_sample, gnss_rel.timestamp,
					  slot ? slot->delay_us : kDefaultDelay);
		const uint64_t pps_timestamp = _pps_time_sync.correct_gps_timestamp(gnss_rel.timestamp, gnss_rel.time_utc_usec);

		if (pps_timestamp != gnss_rel.timestamp) {
			sample.timestamp_sample = pps_timestamp;
		}

		sample.device_id = gnss_rel.device_id;
		sample.heading = gnss_rel.heading_valid ? gnss_rel.heading : NAN;
		sample.heading_accuracy = gnss_rel.heading_accuracy;
		sample.baseline_length = gnss_rel.position_length;
		sample.baseline_down = gnss_rel.position[2];
		sample.jamming_state = receiver.jamming_state;
		sample.spoofing_state = receiver.spoofing_state;
		handleHeadingSample(sample, slot);
	}
}

void VehicleGPSPosition::handleHeadingSample(const HeadingSample &sample, const GpsParamSlot *slot)
{
	// A single source is published at a time: every source has its own baseline, so alternating between receivers
	// would jump the heading and trip the EKF observation rate limit. A source is kept until none of its samples has
	// passed the checks for kHeadingSourceTimeout.
	//
	// TODO: with per-receiver baselines the selection can also follow the flight phase, e.g. a tailsitter with one
	// baseline aligned for hover and one for forward flight.
	const hrt_abstime now = hrt_absolute_time();
	HeadingSource &source = _heading_source;
	const bool held = (source.last_pass != 0) && (now < source.last_pass + kHeadingSourceTimeout);
	const bool same_source = (sample.device_id == source.device_id);

	if (held && !same_source) {
		return;
	}

	const bool configured = slot && (slot->baseline_length >= gnss_heading::kMinAntennaSeparation);

	if (PX4_ISFINITE(sample.heading) && !configured && !_heading_unconfigured_reported) {
		PX4_WARN("GNSS heading from %" PRIu32 " not used: set SENS_GNSSn_HDG", sample.device_id);
		_heading_unconfigured_reported = true;
	}

	if (!PX4_ISFINITE(sample.heading) || !configured) {
		if (same_source) {
			source.settled_since = 0;
		}

		return;
	}

	// A sample whose baseline doesn't match is dropped; the settle restarts only when the receiver itself reports no
	// heading
	if (!gnss_heading::baselineConsistent(slot->baseline_length, sample.baseline_length, sample.baseline_down)) {
		return;
	}

	if (!held || !same_source) {
		source = {sample.device_id, 0, 0};
	}

	if (source.settled_since == 0) {
		source.settled_since = now;
	}

	source.last_pass = now;

	// Headings are published once the receiver has reported a matching one for kHeadingSettleTime, since the first
	// fixes after it (re)gains its heading are the likeliest to be wrong.
	if (now < source.settled_since + kHeadingSettleTime) {
		return;
	}

	vehicle_gnss_heading_s heading_out{};
	heading_out.timestamp_sample = sample.timestamp_sample;
	heading_out.device_id = sample.device_id;
	heading_out.heading = matrix::wrap_pi(sample.heading - slot->heading_offset);
	heading_out.heading_accuracy = sample.heading_accuracy;
	heading_out.heading_offset = slot->heading_offset;
	heading_out.baseline_length = sample.baseline_length;
	heading_out.jamming_state = sample.jamming_state;
	heading_out.spoofing_state = sample.spoofing_state;
	heading_out.timestamp = hrt_absolute_time();
	_vehicle_gnss_heading_pub.publish(heading_out);
}

#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

const VehicleGPSPosition::GpsParamSlot *VehicleGPSPosition::findParamSlot(uint32_t device_id, int instance) const
{
	for (const GpsParamSlot &slot : _gps_param_slots) {
		if ((slot.device_id != 0) && (slot.device_id == device_id)) {
			return &slot;
		}
	}

	// No device IDs configured: match by sensor_gnss instance
	if ((_gps_param_slots[0].device_id == 0) && (_gps_param_slots[1].device_id == 0)
	    && (instance >= 0) && (instance < GPS_MAX_RECEIVERS)) {
		return &_gps_param_slots[instance];
	}

	return nullptr;
}

#if defined(CONFIG_SENSORS_VEHICLE_GNSS_HEADING)
int VehicleGPSPosition::findGpsInstance(uint32_t device_id, sensor_gnss_s &gps_data)
{
	for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
		if (_sensor_gps_sub[i].copy(&gps_data) && (gps_data.device_id == device_id)) {
			return i;
		}
	}

	gps_data = {};
	return -1;
}
#endif // CONFIG_SENSORS_VEHICLE_GNSS_HEADING

uint64_t VehicleGPSPosition::resolveSampleTimestamp(uint64_t driver_timestamp_sample, uint64_t driver_timestamp,
		hrt_abstime delay_us)
{
	// A driver that doesn't know the receiver latency leaves timestamp_sample at 0 (or at the publish time); the
	// configured receiver delay applies then.
	if ((driver_timestamp_sample != 0) && (driver_timestamp_sample < driver_timestamp)) {
		return driver_timestamp_sample;
	}

	return (delay_us > 0 && driver_timestamp > delay_us) ? driver_timestamp - delay_us : driver_timestamp;
}

void VehicleGPSPosition::PrintStatus()
{
	PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_blending.getSelectedGps());
}

}; // namespace sensors
