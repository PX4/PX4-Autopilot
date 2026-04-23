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

#include "GnssRedundancyMonitor.hpp"

#include <lib/geo/geo.h>

using namespace time_literals;

namespace sensors
{

// eph is firmware-dependent and not a rigorous 1-sigma, so 3 is a heuristic consistency gate
// rather than a precise statistical claim. It relaxes the gate automatically when receivers degrade.
static constexpr float GNSS_DIVERGENCE_SIGMA = 3.0f;

// Matches the 1 s staleness threshold used by other sensor checks.
static constexpr uint64_t GNSS_ONLINE_TIMEOUT = 1_s;

// Multipath spikes typically resolve within 1 s; 2 s filters them while still
// detecting real receiver faults promptly.
static constexpr uint64_t GNSS_DIVERGENCE_SUSTAIN = 2_s;

GnssRedundancyMonitor::GnssRedundancyMonitor(ModuleParams *parent) :
	ModuleParams(parent)
{
}

void GnssRedundancyMonitor::update(const sensor_gps_s *gps_states,
				   const matrix::Vector3f *antenna_offsets,
				   uint8_t num_receivers,
				   bool is_armed)
{
	// Separate "online" (present + fresh data) from "fixed" (online + 3D fix).
	// Keeping them separate lets the consumer distinguish "receiver offline"
	// from "receiver lost fix" in its user-facing events.
	uint8_t online_count = 0;
	uint8_t fixed_count = 0;
	uint8_t fixed_idx[2] {};

	for (uint8_t i = 0; i < num_receivers && i < 2; i++) {
		const sensor_gps_s &gps = gps_states[i];

		if (gps.device_id != 0
		    && hrt_elapsed_time(&gps.timestamp) < GNSS_ONLINE_TIMEOUT) {
			online_count++;

			if (gps.fix_type >= 3) {
				fixed_idx[fixed_count++] = i;
			}
		}
	}

	// Position divergence check: flag if two fixed receivers disagree beyond their
	// combined uncertainty. Gate = lever-arm separation + sigma * RMS(eph).
	// Pre-arm: trigger immediately so the operator can decide before takeoff.
	// In-flight: require sustained divergence to suppress transient multipath.
	bool divergence_detected = false;
	float divergence_m = 0.f;
	float divergence_gate_m = 0.f;

	if (fixed_count >= 2) {
		const sensor_gps_s &gps0 = gps_states[fixed_idx[0]];
		const sensor_gps_s &gps1 = gps_states[fixed_idx[1]];

		float north, east;
		get_vector_to_next_waypoint(gps0.latitude_deg, gps0.longitude_deg,
					    gps1.latitude_deg, gps1.longitude_deg,
					    &north, &east);
		divergence_m = sqrtf(north * north + east * east);

		const float rms_eph = sqrtf(gps0.eph * gps0.eph + gps1.eph * gps1.eph);
		const matrix::Vector3f lever_arm = antenna_offsets[fixed_idx[0]] - antenna_offsets[fixed_idx[1]];
		const float expected_d = sqrtf(lever_arm(0) * lever_arm(0) + lever_arm(1) * lever_arm(1));
		divergence_gate_m = expected_d + rms_eph * GNSS_DIVERGENCE_SIGMA;

		const uint64_t sustain = is_armed ? GNSS_DIVERGENCE_SUSTAIN : 0_s;

		if (divergence_m > divergence_gate_m) {
			if (_divergence_since == 0) {
				_divergence_since = hrt_absolute_time();
			}

			if (hrt_elapsed_time(&_divergence_since) >= sustain) {
				divergence_detected = true;
			}

		} else {
			_divergence_since = 0;
		}

	} else {
		_divergence_since = 0;
	}

	// Track the highest fixed count seen — used by consumers to detect any
	// GPS loss regardless of SYS_HAS_NUM_GNSS.
	if (fixed_count > _peak_fixed_count) {
		_peak_fixed_count = fixed_count;
	}

	const uint8_t required = (uint8_t)_param_sys_has_num_gnss.get();

	gnss_redundancy_status_s status{};
	status.num_receivers_online = online_count;
	status.num_receivers_fixed = fixed_count;
	status.num_receivers_required = required;
	status.peak_receivers_fixed = _peak_fixed_count;
	status.below_required = (required > 0) && (fixed_count < required);
	status.dropped_below_peak = (_peak_fixed_count > 1) && (fixed_count < _peak_fixed_count);
	status.divergence_detected = divergence_detected;
	status.divergence_m = divergence_m;
	status.divergence_gate_m = divergence_gate_m;
	status.timestamp = hrt_absolute_time();

	_gnss_redundancy_status_pub.publish(status);
}

} // namespace sensors
