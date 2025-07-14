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
 * @file ev_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"
#include "aid_sources/external_vision/ev_vel.h"

void Ekf::controlExternalVisionFusion(const imuSample &imu_sample)
{
	_ev_pos_b_est.predict(_dt_ekf_avg);
	_ev_hgt_b_est.predict(_dt_ekf_avg);

	// Check for new external vision data
	extVisionSample ev_sample;

	if (_ext_vision_buffer && _ext_vision_buffer->pop_first_older_than(imu_sample.time_us, &ev_sample)) {

		bool ev_reset = (ev_sample.reset_counter != _ev_sample_prev.reset_counter);

		// determine if we should use the horizontal position observations
		bool quality_sufficient = (_params.ekf2_ev_qmin <= 0) || (ev_sample.quality >= _params.ekf2_ev_qmin);

		const bool starting_conditions_passing = quality_sufficient
				&& ((ev_sample.time_us - _ev_sample_prev.time_us) < EV_MAX_INTERVAL)
				&& ((_params.ekf2_ev_qmin <= 0)
				    || (_ev_sample_prev.quality >= _params.ekf2_ev_qmin)) // previous quality sufficient
				&& ((_params.ekf2_ev_qmin <= 0)
				    || (_ext_vision_buffer->get_newest().quality >= _params.ekf2_ev_qmin)) // newest quality sufficient
				&& isNewestSampleRecent(_time_last_ext_vision_buffer_push, EV_MAX_INTERVAL);

		updateEvAttitudeErrorFilter(ev_sample, ev_reset);

		controlEvYawFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_yaw);

		switch (ev_sample.vel_frame) {
		case VelocityFrame::BODY_FRAME_FRD: {
				EvVelBodyFrameFrd ev_vel_body(*this, ev_sample, _params.ekf2_evv_noise, imu_sample);
				controlEvVelFusion(ev_vel_body, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		case VelocityFrame::LOCAL_FRAME_NED: {
				EvVelLocalFrameNed ev_vel_ned(*this, ev_sample, _params.ekf2_evv_noise, imu_sample);
				controlEvVelFusion(ev_vel_ned, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		case VelocityFrame::LOCAL_FRAME_FRD: {
				EvVelLocalFrameFrd ev_vel_frd(*this, ev_sample, _params.ekf2_evv_noise, imu_sample);
				controlEvVelFusion(ev_vel_frd, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_vel);
				break;
			}

		default:
			return;
		}

		controlEvPosFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient, _aid_src_ev_pos);
		controlEvHeightFusion(imu_sample, ev_sample, starting_conditions_passing, ev_reset, quality_sufficient,
				      _aid_src_ev_hgt);

		if (quality_sufficient) {
			_ev_sample_prev = ev_sample;
		}

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel || _control_status.flags.ev_yaw
		    || _control_status.flags.ev_hgt)
		   && isTimedOut(_ev_sample_prev.time_us, 2 * EV_MAX_INTERVAL)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();
		stopEvHgtFusion();

		_ev_q_error_initialized = false;

		ECL_WARN("vision data stopped");
	}
}

void Ekf::updateEvAttitudeErrorFilter(extVisionSample &ev_sample, bool ev_reset)
{
	const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());

	if (!q_error.isAllFinite()) {
		return;
	}

	if (!_ev_q_error_initialized || ev_reset) {
		_ev_q_error_filt.reset(q_error);
		_ev_q_error_initialized = true;

	} else {
		_ev_q_error_filt.update(q_error);
	}
}
