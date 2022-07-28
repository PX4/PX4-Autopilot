/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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

/**
 * @file height_control.cpp
 */

#include "ekf.h"

void Ekf::checkVerticalAccelerationHealth()
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.

	Likelihood inertial_nav_falling_likelihood = estimateInertialNavFallingLikelihood();

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1.f / _dt_ekf_avg;
	const bool is_clipping = _imu_sample_delayed.delta_vel_clipping[0] ||
				 _imu_sample_delayed.delta_vel_clipping[1] ||
				 _imu_sample_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

	} else if (_clip_counter > 0) {
		_clip_counter--;
	}

	_fault_status.flags.bad_acc_clipping = _clip_counter > clip_count_limit / 2;

	const bool is_clipping_frequently = _clip_counter > 0;

	// Do not require evidence of clipping if the likelihood of having the INS falling is high
	const bool bad_vert_accel = (is_clipping_frequently && (inertial_nav_falling_likelihood == Likelihood::MEDIUM))
				    || (inertial_nav_falling_likelihood == Likelihood::HIGH);

	if (bad_vert_accel) {
		_time_bad_vert_accel = _time_last_imu;

	} else {
		_time_good_vert_accel = _time_last_imu;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

Likelihood Ekf::estimateInertialNavFallingLikelihood() const
{
	bool likelihood_high = false;
	bool likelihood_medium = false;

	bool failed_min[4] = {false, false, false, false};
	bool failed_lim[4] = {false, false, false, false};

	if (isVerticalPositionAidingActive()) {
		if (_control_status.flags.baro_hgt) {
			const float innov_ratio = _aid_src_baro_hgt.innovation / sqrtf(_aid_src_baro_hgt.innovation_variance);
			failed_min[HeightSensorRef::BARO] = innov_ratio > _params.vert_innov_test_min;
			failed_lim[HeightSensorRef::BARO] = innov_ratio > _params.vert_innov_test_lim;
		}

		if (_control_status.flags.gps_hgt) {
			const float innov_ratio = _aid_src_gnss_pos.innovation[2] / sqrtf(_aid_src_gnss_pos.innovation_variance[2]);
			failed_min[HeightSensorRef::GPS] = innov_ratio > _params.vert_innov_test_min;
			failed_lim[HeightSensorRef::GPS] = innov_ratio > _params.vert_innov_test_lim;
		}

		if (_control_status.flags.rng_hgt) {
			const float innov_ratio = _aid_src_rng_hgt.innovation / sqrtf(_aid_src_rng_hgt.innovation_variance);
			failed_min[HeightSensorRef::RANGE] = innov_ratio > _params.vert_innov_test_min;
			failed_lim[HeightSensorRef::RANGE] = innov_ratio > _params.vert_innov_test_lim;
		}

		if (_control_status.flags.ev_hgt) {
			const float innov_ratio = _ev_pos_innov(2) / sqrtf(_ev_pos_innov_var(2));
			failed_min[HeightSensorRef::EV] = innov_ratio > _params.vert_innov_test_min;
			failed_lim[HeightSensorRef::EV] = innov_ratio > _params.vert_innov_test_lim;
		}

		for (unsigned i = 0; i < 4; i++) {

			if (failed_lim[i]) {
				// There is a chance that the interial nav is falling if one height source is failing the test
				likelihood_medium = true;
			}

			for (unsigned j = 0; j < 4; j++) {

				if ((i != j) && failed_lim[i] && failed_min[j]) {
					// There is a high chance that the interial nav is falling if two height sources are failing the test
					likelihood_high = true;
				}
			}
		}
	}

	if (isVerticalVelocityAidingActive()) {
		bool gps_vel_failed_min = false;
		bool gps_vel_failed_lim = false;

		if (_control_status.flags.gps) {
			const float innov_ratio = _aid_src_gnss_vel.innovation[2] / sqrtf(_aid_src_gnss_vel.innovation_variance[2]);
			gps_vel_failed_min = innov_ratio > _params.vert_innov_test_min;
			gps_vel_failed_lim = innov_ratio > _params.vert_innov_test_lim;
		}

		bool ev_vel_failed_min = false;
		bool ev_vel_failed_lim = false;

		if (_control_status.flags.ev_vel) {
			const float innov_ratio = _ev_vel_innov(2) / sqrtf(_ev_vel_innov_var(2));
			ev_vel_failed_min = innov_ratio > _params.vert_innov_test_min;
			ev_vel_failed_lim = innov_ratio > _params.vert_innov_test_lim;
		}

		// If vertical position and velocity come from independent sensors then we can
		// trust them more if they disagree with the IMU, but need to check that they agree
		likelihood_high |= gps_vel_failed_lim && (failed_min[HeightSensorRef::BARO]
							  || failed_min[HeightSensorRef::RANGE]
							  || failed_min[HeightSensorRef::EV]
							  || ev_vel_failed_min);
		likelihood_high |= gps_vel_failed_min && (failed_lim[HeightSensorRef::BARO]
							  || failed_lim[HeightSensorRef::RANGE]
							  || failed_lim[HeightSensorRef::EV]
							  || ev_vel_failed_lim);

		likelihood_high |= ev_vel_failed_lim && (failed_min[HeightSensorRef::BARO]
							 || failed_min[HeightSensorRef::RANGE]
							 || failed_min[HeightSensorRef::GPS]
							 || gps_vel_failed_min);
		likelihood_high |= ev_vel_failed_min && (failed_lim[HeightSensorRef::BARO]
							 || failed_lim[HeightSensorRef::RANGE]
							 || failed_lim[HeightSensorRef::GPS]
							 || gps_vel_failed_lim);

		likelihood_medium |= gps_vel_failed_lim;
		likelihood_medium |= ev_vel_failed_lim;
	}

	if (likelihood_high) {
		return Likelihood::HIGH;

	} else if (likelihood_medium) {
		return Likelihood::MEDIUM;
	}

	return Likelihood::LOW;
}
