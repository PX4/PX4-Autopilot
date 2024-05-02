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
 * @file mag_3d_control.cpp
 * Control functions for ekf mag 3D fusion
 */

#include "ekf.h"

void Ekf::controlMag3DFusion(const magSample &mag_sample, const bool common_starting_conditions_passing,
			     estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "mag";

	resetEstimatorAidStatus(aid_src);

	const bool wmm_updated = (_wmm_gps_time_last_set >= aid_src.time_last_fuse); // WMM update can occur on the last epoch, just after mag fusion

	// determine if we should use mag fusion
	bool continuing_conditions_passing = (_params.mag_fusion_type != MagFuseType::NONE)
					     && _control_status.flags.tilt_align
					     && (_control_status.flags.yaw_align || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
					     && mag_sample.mag.longerThan(0.f)
					     && mag_sample.mag.isAllFinite();

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing;

	// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
	// before they are used to constrain heading drift
	_control_status.flags.mag_3D = (_params.mag_fusion_type == MagFuseType::AUTO)
				       && _control_status.flags.mag
				       && _control_status.flags.mag_aligned_in_flight
				       && (_control_status.flags.mag_heading_consistent || !_control_status.flags.gps)
				       && !_control_status.flags.mag_fault
				       && !_control_status.flags.ev_yaw
				       && !_control_status.flags.gps_yaw;

	const bool mag_consistent_or_no_gnss = _control_status.flags.mag_heading_consistent || !_control_status.flags.gps;

	_control_status.flags.mag_hdg = ((_params.mag_fusion_type == MagFuseType::HEADING)
					      || (_params.mag_fusion_type == MagFuseType::AUTO && !_control_status.flags.mag_3D))
					     && _control_status.flags.tilt_align
					     && ((_control_status.flags.yaw_align && mag_consistent_or_no_gnss)
					         || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
					     && !_control_status.flags.mag_fault
					     && !_control_status.flags.mag_field_disturbed
					     && !_control_status.flags.ev_yaw
					     && !_control_status.flags.gps_yaw;

	// TODO: allow clearing mag_fault if mag_3d is good?

	if (_control_status.flags.mag_3D && !_control_status_prev.flags.mag_3D) {
		ECL_INFO("starting mag 3D fusion");

	} else if (!_control_status.flags.mag_3D && _control_status_prev.flags.mag_3D) {
		ECL_INFO("stopping mag 3D fusion");
	}

	// if we are using 3-axis magnetometer fusion, but without external NE aiding,
	// then the declination must be fused as an observation to prevent long term heading drift
	// fusing declination when gps aiding is available is optional.
	const bool not_using_ne_aiding = !_control_status.flags.gps && !_control_status.flags.aux_gpos;
	_control_status.flags.mag_dec = (_control_status.flags.mag && (not_using_ne_aiding || !_control_status.flags.mag_aligned_in_flight));

	if (_control_status.flags.mag) {
		aid_src.timestamp_sample = mag_sample.time_us;

		if (continuing_conditions_passing && _control_status.flags.yaw_align) {

			if (mag_sample.reset || checkHaglYawResetReq() || wmm_updated) {
				ECL_INFO("reset to %s", AID_SRC_NAME);
				resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
				aid_src.time_last_fuse = _time_delayed_us;

			} else {
				if (!_mag_decl_cov_reset) {
					// After any magnetic field covariance reset event the earth field state
					// covariances need to be corrected to incorporate knowledge of the declination
					// before fusing magnetometer data to prevent rapid rotation of the earth field
					// states for the first few observations.
					fuseDeclination(0.02f);
					_mag_decl_cov_reset = true;
					fuseMag(mag_sample.mag, aid_src, false);

				} else {
					// The normal sequence is to fuse the magnetometer data first before fusing
					// declination angle at a higher uncertainty to allow some learning of
					// declination angle over time.
					const bool update_all_states = _control_status.flags.mag_3D || _control_status.flags.mag_hdg;
					const bool update_tilt = _control_status.flags.mag_3D;
					fuseMag(mag_sample.mag, aid_src, update_all_states, update_tilt);

					if (_control_status.flags.mag_dec) {
						fuseDeclination(0.5f);
					}
				}
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

			if (is_fusion_failing) {
				if (_nb_mag_3d_reset_available > 0) {
					// Data seems good, attempt a reset (mag states only unless mag_3D currently active)
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_mag_3d_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.mag_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopMagFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopMagFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_DEBUG("stopping %s fusion, continuing conditions no longer passing", AID_SRC_NAME);
			stopMagFusion();
		}

	} else {
		if (starting_conditions_passing) {

			_control_status.flags.mag = true;

			// activate fusion, reset mag states and initialize variance if first init or in flight reset
			if (!_control_status.flags.yaw_align
			    || wmm_updated
			    || !_mag_decl_cov_reset
			    || !_state.mag_I.longerThan(0.f)
			    || (getStateVariance<State::mag_I>().min() < kMagVarianceMin)
			    || (getStateVariance<State::mag_B>().min() < kMagVarianceMin)
			   ) {
				ECL_INFO("starting %s fusion, resetting states", AID_SRC_NAME);

				bool reset_heading = !_control_status.flags.yaw_align;

				resetMagStates(_mag_lpf.getState(), reset_heading);

				if (reset_heading) {
					_control_status.flags.yaw_align = true;
				}

			} else {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				fuseMag(mag_sample.mag, aid_src, false);
			}

			aid_src.time_last_fuse = _time_delayed_us;

			_nb_mag_3d_reset_available = 2;
		}
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag) {
		ECL_INFO("stopping mag fusion");

		_control_status.flags.mag = false;
		_control_status.flags.mag_dec = false;

		if (_control_status.flags.mag_3D) {
			ECL_INFO("stopping mag 3D fusion");
			_control_status.flags.mag_3D = false;
		}

		if (_control_status.flags.mag_hdg) {
			ECL_INFO("stopping mag heading fusion");
			_control_status.flags.mag_hdg = false;
			_fault_status.flags.bad_hdg = false;
		}

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;
	}
}
