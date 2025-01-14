/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file airspeed_fusion.cpp
 * airspeed fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_airspeed_h.h>
#include <ekf_derivation/generated/compute_airspeed_innov_and_innov_var.h>
#include <ekf_derivation/generated/compute_wind_init_and_cov_from_airspeed.h>

#include <mathlib/mathlib.h>

void Ekf::controlAirDataFusion(const imuSample &imu_delayed)
{
	// control activation and initialisation/reset of wind states required for airspeed fusion

	// If both airspeed and sideslip fusion have timed out and we are not using a drag observation model then we no longer have valid wind estimates
	const bool airspeed_timed_out = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);
	const bool sideslip_timed_out = isTimedOut(_aid_src_sideslip.time_last_fuse, (uint64_t)10e6);

	if (_control_status.flags.fake_pos || (airspeed_timed_out && sideslip_timed_out && (_params.drag_ctrl == 0))) {
		_control_status.flags.wind = false;
	}

	if (_control_status.flags.wind && _external_wind_init) {
		_external_wind_init = false;
	}

#if defined(CONFIG_EKF2_GNSS)

	// clear yaw estimator airspeed (updated later with true airspeed if airspeed fusion is active)
	if (_control_status.flags.fixed_wing) {
		if (_control_status.flags.in_air && !_control_status.flags.vehicle_at_rest) {
			if (!_control_status.flags.fuse_aspd) {
				_yawEstimator.setTrueAirspeed(_params.EKFGSF_tas_default);
			}

		} else {
			_yawEstimator.setTrueAirspeed(0.f);
		}
	}

#endif // CONFIG_EKF2_GNSS

	if (_params.arsp_thr <= 0.f) {
		stopAirspeedFusion();
		return;
	}

	if (_airspeed_buffer && _airspeed_buffer->pop_first_older_than(imu_delayed.time_us, &_airspeed_sample_delayed)) {

		const airspeedSample &airspeed_sample = _airspeed_sample_delayed;

		updateAirspeed(airspeed_sample, _aid_src_airspeed);

		_innov_check_fail_status.flags.reject_airspeed =
			_aid_src_airspeed.innovation_rejected; // TODO: remove this redundant flag

		const bool continuing_conditions_passing = _control_status.flags.in_air
				&& !_control_status.flags.fake_pos;

		const bool is_airspeed_significant = airspeed_sample.true_airspeed > _params.arsp_thr;
		const bool is_airspeed_consistent = (_aid_src_airspeed.test_ratio > 0.f && _aid_src_airspeed.test_ratio < 1.f);
		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_airspeed_significant
				&& (is_airspeed_consistent || !_control_status.flags.wind || _control_status.flags.inertial_dead_reckoning);

		if (_control_status.flags.fuse_aspd) {
			if (continuing_conditions_passing) {
				if (is_airspeed_significant) {
					fuseAirspeed(airspeed_sample, _aid_src_airspeed);
				}

#if defined(CONFIG_EKF2_GNSS)
				_yawEstimator.setTrueAirspeed(airspeed_sample.true_airspeed);
#endif // CONFIG_EKF2_GNSS

				const bool is_fusion_failing = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);

				if (is_fusion_failing) {
					stopAirspeedFusion();
				}

			} else {
				stopAirspeedFusion();
			}

		} else if (starting_conditions_passing) {
			ECL_INFO("starting airspeed fusion");

			if (_control_status.flags.inertial_dead_reckoning && !is_airspeed_consistent) {
				resetVelUsingAirspeed(airspeed_sample);

			} else if (!_external_wind_init
				   && (!_control_status.flags.wind
				       || getWindVelocityVariance().longerThan(sq(_params.initial_wind_uncertainty)))) {
				resetWindUsingAirspeed(airspeed_sample);
				_aid_src_airspeed.time_last_fuse = _time_delayed_us;
			}

			_control_status.flags.wind = true;
			_control_status.flags.fuse_aspd = true;
		}

	} else if (_control_status.flags.fuse_aspd && !isRecent(_airspeed_sample_delayed.time_us, (uint64_t)1e6)) {
		ECL_WARN("Airspeed data stopped");
		stopAirspeedFusion();
	}
}

void Ekf::updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src) const
{
	// Variance for true airspeed measurement - (m/sec)^2
	const float R = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			   math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	float innov = 0.f;
	float innov_var = 0.f;
	sym::ComputeAirspeedInnovAndInnovVar(_state.vector(), P, airspeed_sample.true_airspeed, R, FLT_EPSILON,
					     &innov, &innov_var);

	updateAidSourceStatus(aid_src,
			      airspeed_sample.time_us,                 // sample timestamp
			      airspeed_sample.true_airspeed,           // observation
			      R,                                       // observation variance
			      innov,                                   // innovation
			      innov_var,                               // innovation variance
			      math::max(_params.tas_innov_gate, 1.f)); // innovation gate
}

void Ekf::fuseAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source1d_s &aid_src)
{
	if (aid_src.innovation_rejected) {
		return;
	}

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	const float innov_var = aid_src.innovation_variance;

	if (innov_var < aid_src.observation_variance || innov_var < FLT_EPSILON) {
		// Reset the estimator covariance matrix
		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindUsingAirspeed(airspeed_sample);
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string);

		_fault_status.flags.bad_airspeed = true;

		return;
	}

	_fault_status.flags.bad_airspeed = false;

	const VectorState H = sym::ComputeAirspeedH(_state.vector(), FLT_EPSILON);
	VectorState K = P * H / aid_src.innovation_variance;

	if (update_wind_only) {
		const Vector2f K_wind = K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0);
		K.setZero();
		K.slice<State::wind_vel.dof, 1>(State::wind_vel.idx, 0) = K_wind;
	}

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;

	if (!update_wind_only) {
		_time_last_hor_vel_fuse = _time_delayed_us;
	}
}

void Ekf::stopAirspeedFusion()
{
	if (_control_status.flags.fuse_aspd) {
		ECL_INFO("stopping airspeed fusion");
		_control_status.flags.fuse_aspd = false;

#if defined(CONFIG_EKF2_GNSS)
		_yawEstimator.setTrueAirspeed(NAN);
#endif // CONFIG_EKF2_GNSS
	}
}

void Ekf::resetWindUsingAirspeed(const airspeedSample &airspeed_sample)
{
	constexpr float sideslip_var = sq(math::radians(15.0f));

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float airspeed_var = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f)
				      * math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	matrix::SquareMatrix<float, State::wind_vel.dof> P_wind;
	sym::ComputeWindInitAndCovFromAirspeed(_state.vel, euler_yaw, airspeed_sample.true_airspeed, getVelocityVariance(),
					       getYawVar(), sideslip_var, airspeed_var, &_state.wind_vel, &P_wind);

	resetStateCovariance<State::wind_vel>(P_wind);

	ECL_INFO("reset wind using airspeed to (%.3f, %.3f)", (double)_state.wind_vel(0), (double)_state.wind_vel(1));

	resetAidSourceStatusZeroInnovation(_aid_src_airspeed);
}

void Ekf::resetVelUsingAirspeed(const airspeedSample &airspeed_sample)
{
	const float euler_yaw = getEulerYaw(_R_to_earth);

	// Estimate velocity using zero sideslip assumption and airspeed measurement
	Vector2f horizontal_velocity;
	horizontal_velocity(0) = _state.wind_vel(0) + airspeed_sample.true_airspeed * cosf(euler_yaw);
	horizontal_velocity(1) = _state.wind_vel(1) + airspeed_sample.true_airspeed * sinf(euler_yaw);

	float vel_var = NAN; // Do not reset the velocity variance as wind variance estimate is most likely not correct
	resetHorizontalVelocityTo(horizontal_velocity, vel_var);

	_aid_src_airspeed.time_last_fuse = _time_delayed_us;
}
