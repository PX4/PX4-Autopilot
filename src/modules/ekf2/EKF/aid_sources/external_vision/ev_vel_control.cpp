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
 * @file ev_vel_control.cpp
 * Control functions for ekf external vision velocity fusion
 */

#include "ekf.h"
#include "aid_sources/external_vision/ev_vel.h"
#include "ekf_derivation/generated/compute_body_vel_innov_var_h.h"
#include "ekf_derivation/generated/compute_body_vel_y_innov_var.h"
#include "ekf_derivation/generated/compute_body_vel_z_innov_var.h"

void Ekf::controlEvVelFusion(ExternalVisionVel &ev, const bool common_starting_conditions_passing, const bool ev_reset,
			     const bool quality_sufficient, estimator_aid_source3d_s &aid_src)
{
	static constexpr const char *AID_SRC_NAME = "EV velocity";

	const bool yaw_alignment_changed = (!_control_status_prev.flags.ev_yaw && _control_status.flags.ev_yaw)
					   || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align);

	// determine if we should use EV velocity aiding
	bool continuing_conditions_passing = (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL))
					     && _control_status.flags.tilt_align
					     && ev._sample.vel.isAllFinite()
					     && !ev._sample.vel.longerThan(_params.velocity_limit);


	continuing_conditions_passing &= ev._measurement.isAllFinite() && ev._measurement_var.isAllFinite();

	float gate = math::max(_params.ev_vel_innov_gate, 1.f);

	if (_control_status.flags.ev_vel) {
		if (continuing_conditions_passing) {
			if ((ev_reset && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) || yaw_alignment_changed) {
				if (quality_sufficient) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					_information_events.flags.reset_vel_to_vision = true;
					ev.resetVelocity();
					resetAidSourceStatusZeroInnovation(aid_src);

				} else {
					// EV has reset, but quality isn't sufficient
					// we have no choice but to stop EV and try to resume once quality is acceptable
					stopEvVelFusion();
					return;
				}

			} else if (quality_sufficient) {
				ev.fuseVelocity(aid_src, gate);

			} else {
				aid_src.innovation_rejected = true;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.no_aid_timeout_max); // 1 second

			if (is_fusion_failing) {

				if ((_nb_ev_vel_reset_available > 0) && quality_sufficient) {
					_information_events.flags.reset_vel_to_vision = true;
					ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
					ev.resetVelocity();
					resetAidSourceStatusZeroInnovation(aid_src);

					if (_control_status.flags.in_air) {
						_nb_ev_vel_reset_available--;
					}

				} else {
					ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);


					stopEvVelFusion();
				}

			} else if (isHeightResetRequired()) {
				ev.resetVerticalVelocity();
			}

		} else {
			// Stop fusion but do not declare it faulty
			ECL_WARN("stopping %s fusion, continuing conditions failing", AID_SRC_NAME);
			stopEvVelFusion();
		}

	} else {

		if (common_starting_conditions_passing && continuing_conditions_passing) {
			// make starting condition more sensitive if horizontal aiding is active
			gate = isHorizontalAidingActive() ? (float)sqrt(10.f) * gate : gate;

			// activate fusion, only reset if necessary
			if (!isHorizontalAidingActive() || yaw_alignment_changed) {
				ECL_INFO("starting %s fusion, resetting velocity to (%.3f, %.3f, %.3f)", AID_SRC_NAME,
					 (double)ev._measurement(0), (double)ev._measurement(1),
					 (double)ev._measurement(2));
				_information_events.flags.reset_vel_to_vision = true;
				ev.resetVelocity();
				resetAidSourceStatusZeroInnovation(aid_src);

				_control_status.flags.ev_vel = true;

			} else if (ev.fuseVelocity(aid_src, gate)) {
				ECL_INFO("starting %s fusion", AID_SRC_NAME);
				_control_status.flags.ev_vel = true;
			}

			if (_control_status.flags.ev_vel) {
				_nb_ev_vel_reset_available = 5;
				_information_events.flags.starting_vision_vel_fusion = true;
			}
		}
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		_control_status.flags.ev_vel = false;
	}
}

void Ekf::fuseLocalFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp,
				 const Vector3f &measurement, const Vector3f &measurement_var, const float &innovation_gate)
{
	updateAidSourceStatus(aid_src,
			      timestamp,				// sample timestamp
			      measurement,				// observation
			      measurement_var,				// observation variance
			      _state.vel - measurement,			// innovation
			      getVelocityVariance() + measurement_var,	// innovation variance
			      innovation_gate);				// innovation gate
	fuseVelocity(aid_src);
}

void Ekf::fuseBodyFrameVelocity(estimator_aid_source3d_s &aid_src, const uint64_t &timestamp,
				const Vector3f &measurement, const Vector3f &measurement_var, const float &innovation_gate)
{
	VectorState H[3];
	Vector3f innov_var;
	Vector3f innov = _R_to_earth.transpose() * _state.vel - measurement;
	const auto state_vector = _state.vector();
	sym::ComputeBodyVelInnovVarH(state_vector, P, measurement_var, &innov_var, &H[0], &H[1], &H[2]);

	updateAidSourceStatus(aid_src,
			      timestamp,				// sample timestamp
			      measurement,				// observation
			      measurement_var,				// observation variance
			      innov,					// innovation
			      innov_var,				// innovation variance
			      innovation_gate);				// innovation gate

	if (!aid_src.innovation_rejected) {
		for (uint8_t index = 0; index <= 2; index++) {
			if (index == 1) {
				sym::ComputeBodyVelYInnovVar(state_vector, P, measurement_var(index), &aid_src.innovation_variance[index]);

			} else if (index == 2) {
				sym::ComputeBodyVelZInnovVar(state_vector, P, measurement_var(index), &aid_src.innovation_variance[index]);
			}

			aid_src.innovation[index] = Vector3f(_R_to_earth.transpose().row(index)) * _state.vel - measurement(index);

			VectorState Kfusion = P * H[index] / aid_src.innovation_variance[index];
			measurementUpdate(Kfusion, H[index], aid_src.observation_variance[index], aid_src.innovation[index]);
		}

		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;
	}
}
