/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file ev_yaw_control.cpp
 * Control functions for ekf external vision yaw fusion
 */

#include "ekf.h"

void Ekf::controlEvYawFusion(const imuSample &imu_sample, const extVisionSample &ev_sample,
			     const bool common_starting_conditions_passing, const bool ev_reset, const bool quality_sufficient,
			     estimator_aid_source1d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV yaw";

	float obs = getEulerYaw(ev_sample.quat);
	float obs_var = math::max(ev_sample.orientation_var(2), _params.ev_att_noise, sq(0.01f));

	float innov = wrap_pi(getEulerYaw(_R_to_earth) - obs);
	float innov_var = 0.f;

	VectorState H_YAW;
	computeYawInnovVarAndH(obs_var, innov_var, H_YAW);

	updateAidSourceStatus(aid_src,
			      ev_sample.time_us,                           // sample timestamp
			      obs,                                         // observation
			      obs_var,                                     // observation variance
			      innov,                                       // innovation
			      innov_var,                                   // innovation variance
			      math::max(_params.heading_innov_gate, 1.f)); // innovation gate

	if (ev_reset) {
		_control_status.flags.ev_yaw_fault = false;
	}

	// determine if we should use EV yaw aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::YAW))
					     && _control_status.flags.tilt_align
					     && !_control_status.flags.ev_yaw_fault
					     && PX4_ISFINITE(aid_src.observation)
					     && PX4_ISFINITE(aid_src.observation_variance);

	// if GPS enabled don't allow EV yaw if EV isn't NED
	if (_control_status.flags.gps && _control_status.flags.yaw_align
	    && (ev_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED)
	   ) {
		continuing_conditions_passing = false;
	}

	const bool starting_conditions_passing = common_starting_conditions_passing
			&& continuing_conditions_passing
			&& isTimedOut(aid_src.time_last_fuse, (uint32_t)1e6);

	if (_control_status.flags.ev_yaw) {
		if (continuing_conditions_passing) {

			if (ev_reset) {

				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvYawFusion();
					return;
				}

			} else if (quality_sufficient) {
				fuseYaw(aid_src, H_YAW);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max);

			if (is_fusion_failing) {
				if ((_nb_ev_yaw_reset_available > 0) && quality_sufficient) {
					// Data seems good, attempt a reset
					//_information_events.flags.reset_yaw_to_vision = true; // TODO
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					aid_src.time_last_fuse = _time_delayed_us;

					if (_control_status.flags.in_air) {
						_nb_ev_yaw_reset_available--;
					}

				} else if (starting_conditions_passing) {
					// Data seems good, but previous reset did not fix the issue
					// something else must be wrong, declare the sensor faulty and stop the fusion
					//_control_status.flags.ev_yaw_fault = true;
					ECL_WARN("stopping %s fusion, starting conditions failing", AID_SRC_NAME);
					stopEvYawFusion();

				} else {
					// A reset did not fix the issue but all the starting checks are not passing
					// This could be a temporary issue, stop the fusion without declaring the sensor faulty
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
					stopEvYawFusion();
				}
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvYawFusion();
		}

	} else {
		if (starting_conditions_passing) {
			// activate fusion
			if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

				if (_control_status.flags.yaw_align) {

					if (fuseYaw(aid_src, H_YAW)) {
						ECL_INFO("starting %s fusion", AID_SRC_NAME);
						_information_events.flags.starting_vision_yaw_fusion = true;

						_control_status.flags.ev_yaw = true;
					}

				} else {
					// reset yaw to EV and set yaw_align
					ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);
					_information_events.flags.starting_vision_yaw_fusion = true;

					resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
					_control_status.flags.yaw_align = true;
					_control_status.flags.ev_yaw = true;

					aid_src.time_last_fuse = _time_delayed_us;
				}

			} else if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
				// turn on fusion of external vision yaw measurements
				ECL_INFO("starting %s fusion, resetting state", AID_SRC_NAME);

				// reset yaw to EV
				resetQuatStateYaw(aid_src.observation, aid_src.observation_variance);
				aid_src.time_last_fuse = _time_delayed_us;

				_information_events.flags.starting_vision_yaw_fusion = true;
				_control_status.flags.yaw_align = false;
				_control_status.flags.ev_yaw = true;
			}

			if (_control_status.flags.ev_yaw) {
				_nb_ev_yaw_reset_available = 5;
			}
		}
	}
}

void Ekf::stopEvYawFusion()
{
#if defined(CONFIG_EKF2_EXTERNAL_VISION)

	if (_control_status.flags.ev_yaw) {

		_control_status.flags.ev_yaw = false;
	}

#endif // CONFIG_EKF2_EXTERNAL_VISION
}
