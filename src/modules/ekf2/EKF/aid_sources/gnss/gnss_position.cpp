/****************************************************************************
 *
 *   Copyright (c) 2021-2025 PX4 Development Team. All rights reserved.
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

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGnssPositionFusion(const imuSample &imu_sample, const gnssSample &gnss_sample, bool reset)
{
	static constexpr const char *AID_SRC_NAME = "GNSS position";

	auto &aid_src = _aid_src_gnss_pos;

	// correct position and height for offset relative to IMU
	const Vector3f pos_offset_body = _params.gps_pos_body - _params.imu_pos_body;
	const Vector3f pos_offset_earth = Vector3f(_R_to_earth * pos_offset_body);
	const LatLonAlt measurement(gnss_sample.lat, gnss_sample.lon, gnss_sample.alt);
	const LatLonAlt measurement_corrected = measurement + (-pos_offset_earth);
	const Vector2f innovation = (_gpos - measurement_corrected).xy();

	// relax the upper observation noise limit which prevents bad GPS perturbing the position estimate
	float pos_noise = math::max(gnss_sample.hacc, _params.gps_pos_noise);

	if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gnss_pos)) {
		// if we are not using another source of aiding, then we are reliant on the GNSS
		// observations to constrain attitude errors and must limit the observation noise value.
		if (pos_noise > _params.pos_noaid_noise) {
			pos_noise = _params.pos_noaid_noise;
		}
	}

	const float pos_var = math::max(sq(pos_noise), sq(0.01f));
	const Vector2f pos_obs_var(pos_var, pos_var);
	const matrix::Vector2d observation(measurement_corrected.latitude_deg(), measurement_corrected.longitude_deg());

	updateAidSourceStatus(aid_src,
			      gnss_sample.time_us,                                    // sample timestamp
			      observation,                                            // observation
			      pos_obs_var,                                            // observation variance
			      innovation,                                             // innovation
			      Vector2f(getStateVariance<State::pos>()) + pos_obs_var, // innovation variance
			      math::max(_params.gps_pos_innov_gate, 1.f));            // innovation gate

	const bool gnss_pos_enabled = (_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::HPOS));

	const bool continuing_conditions_passing = gnss_pos_enabled
			&& _control_status.flags.tilt_align
			&& _control_status.flags.yaw_align
			&& (gnss_sample.hacc < _params.req_hacc);

	const bool starting_conditions_passing = continuing_conditions_passing
					&& _gnss_common_checks_passed;

	const bool gpos_init_conditions_passing = gnss_pos_enabled && _gps_checks_passed;

	if (_control_status.flags.gnss_pos) {
		if (continuing_conditions_passing) {
			if (reset) {
				ECL_WARN("%s fusion timeout, resetting", AID_SRC_NAME);

				_information_events.flags.reset_pos_to_gps = true;
				resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
					      aid_src.observation_variance[0] + aid_src.observation_variance[1]);

				resetAidSourceStatusZeroInnovation(aid_src);

			} else {
				fuseHorizontalPosition(aid_src);
			}

		} else {
			stopGnssPosFusion();
		}

	} else {
		if (starting_conditions_passing) {
			ECL_INFO("%s starting fusion", AID_SRC_NAME);
			_information_events.flags.starting_gps_fusion = true;

			_information_events.flags.reset_pos_to_gps = true;
			resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
				      aid_src.observation_variance[0] +
				      aid_src.observation_variance[1]);

			resetAidSourceStatusZeroInnovation(aid_src);

			_control_status.flags.gnss_pos = true;

		} else if (gpos_init_conditions_passing && !_local_origin_lat_lon.isInitialized()) {

			_information_events.flags.reset_pos_to_gps = true;
			resetLatLonTo(aid_src.observation[0], aid_src.observation[1],
				      aid_src.observation_variance[0] +
				      aid_src.observation_variance[1]);

			resetAidSourceStatusZeroInnovation(aid_src);
		}
	}
}

void Ekf::stopGnssPosFusion()
{
	if (_control_status.flags.gnss_pos) {
		ECL_INFO("stopping GNSS position fusion");
		_control_status.flags.gnss_pos = false;

		//TODO: what if gnss yaw or height is used?
		if (!_control_status.flags.gnss_vel) {
			_last_gps_fail_us = 0;
			_last_gps_pass_us = 0;
		}
	}
}
