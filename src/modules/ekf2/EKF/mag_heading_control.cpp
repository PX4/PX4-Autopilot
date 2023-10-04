/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file mag_heading_control.cpp
 * Control functions for ekf mag heading fusion
 */

#include "ekf.h"

void Ekf::controlMagHeadingFusion(const magSample &mag_sample, const bool common_starting_conditions_passing,
				  estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "mag heading";

	// use mag bias if variance good
	Vector3f mag_bias{0.f, 0.f, 0.f};
	const Vector3f mag_bias_var = getMagBiasVariance();

	if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
		mag_bias = _state.mag_B;
	}

	// calculate mag heading
	// Rotate the measurements into earth frame using the zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// the angle of the projection onto the horizontal gives the yaw angle
	// calculate the yaw innovation and wrap to the interval between +-pi
	const Vector3f mag_earth_pred = R_to_earth * (mag_sample.mag - mag_bias);
	const float declination = getMagDeclination();
	const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;

	resetEstimatorAidStatus(aid_src);
	aid_src.observation = measured_hdg;
	aid_src.observation_variance = math::max(sq(_params.mag_heading_noise), sq(0.01f));

	if (_control_status.flags.yaw_align) {
		// mag heading
		aid_src.innovation = wrap_pi(getEulerYaw(_R_to_earth) - aid_src.observation);
		_mag_heading_innov_lpf.update(aid_src.innovation);

	} else {
		// mag heading delta (logging only)
		aid_src.innovation = wrap_pi(wrap_pi(getEulerYaw(_R_to_earth) - _mag_heading_pred_prev)
					     - wrap_pi(measured_hdg - _mag_heading_prev));
		_mag_heading_innov_lpf.reset(0.f);
	}

	// determine if we should use mag heading aiding
	const bool mag_consistent_or_no_gnss = _control_status.flags.mag_heading_consistent || !_control_status.flags.gps;

	bool continuing_conditions_passing = ((_params.mag_fusion_type == MagFuseType::HEADING)
					      || (_params.mag_fusion_type == MagFuseType::AUTO && !_control_status.flags.mag_3D))
					     && _control_status.flags.tilt_align
					     && ((_control_status.flags.yaw_align && mag_consistent_or_no_gnss)
					         || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
					     && !_control_status.flags.mag_fault
					     && !_control_status.flags.mag_field_disturbed
					     && !_control_status.flags.ev_yaw
					     && !_control_status.flags.gps_yaw
					     && PX4_ISFINITE(aid_src.observation)
					     && PX4_ISFINITE(aid_src.observation_variance);

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& isTimedOut(aid_src.time_last_fuse, 3e6);

	if (_control_status.flags.mag_hdg) {
		aid_src.timestamp_sample = mag_sample.time_us;

		if (continuing_conditions_passing && _control_status.flags.yaw_align) {

			const bool declination_changed = _control_status_prev.flags.mag_hdg
							 && (fabsf(declination - _mag_heading_last_declination) > math::radians(3.f));
			bool skip_timeout_check = false;

			if (mag_sample.reset || declination_changed) {
				if (declination_changed) {
					ECL_INFO("reset to %s, declination changed %.5f->%.5f",
						 AID_SRC_NAME, (double)_mag_heading_last_declination, (double)declination);

				} else {
					ECL_INFO("reset to %s", AID_SRC_NAME);
				}

				resetMagHeading(_mag_lpf.getState());
				aid_src.time_last_fuse = _time_delayed_us;

			} else {
				VectorState H_YAW;
				computeYawInnovVarAndH(aid_src.observation_variance, aid_src.innovation_variance, H_YAW);

				if ((aid_src.innovation_variance - aid_src.observation_variance) > sq(_params.mag_heading_noise / 2.f)) {
					// Only fuse mag to constrain the yaw variance to a safe value
					fuseYaw(aid_src, H_YAW);

				} else {
					// Yaw variance is low enough, stay in mag heading mode but don't fuse the data and skip the fusion timeout check
					skip_timeout_check = true;
					aid_src.test_ratio = 0.f; // required for preflight checks to pass
				}
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max) && !skip_timeout_check;

			if (is_fusion_failing) {
				if (_nb_mag_heading_reset_available > 0) {
					// Data seems good, attempt a reset
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetMagHeading(_mag_lpf.getState());
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_mag_heading_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					_control_status.flags.mag_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopMagHdgFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopMagHdgFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopMagHdgFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion
			ECL_INFO("starting %s fusion", AID_SRC_NAME);

			if (!_control_status.flags.yaw_align) {
				// reset heading
				resetMagHeading(_mag_lpf.getState());
				_control_status.flags.yaw_align = true;
			}

			_control_status.flags.mag_hdg = true;
			aid_src.time_last_fuse = _time_delayed_us;

			_nb_mag_heading_reset_available = 1;
		}
	}

	// record corresponding mag heading and yaw state for future mag heading delta heading innovation (logging only)
	_mag_heading_prev = measured_hdg;
	_mag_heading_pred_prev = getEulerYaw(_state.quat_nominal);

	_mag_heading_last_declination = getMagDeclination();
}

void Ekf::stopMagHdgFusion()
{
	if (_control_status.flags.mag_hdg) {
		ECL_INFO("stopping mag heading fusion");
		resetEstimatorAidStatus(_aid_src_mag_heading);

		_control_status.flags.mag_hdg = false;

		_fault_status.flags.bad_hdg = false;
	}
}
