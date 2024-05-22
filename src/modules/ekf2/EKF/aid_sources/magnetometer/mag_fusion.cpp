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
 * @file heading_fusion.cpp
 * Magnetometer fusion methods.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_mag_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_mag_z_innov_var_and_h.h>

#include <ekf_derivation/generated/compute_mag_declination_pred_innov_var_and_h.h>

#include <mathlib/mathlib.h>

bool Ekf::fuseMag(const Vector3f &mag, const float R_MAG, VectorState &H, estimator_aid_source3d_s &aid_src, bool update_all_states, bool update_tilt)
{
	// if any axis failed, abort the mag fusion
	if (aid_src.innovation_rejected) {
		return false;
	}

	const auto state_vector = _state.vector();

	bool fused[3] {false, false, false};

	// update the states and covariance using sequential fusion of the magnetometer components
	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagYInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			aid_src.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);

		} else if (index == 2) {
			// we do not fuse synthesized magnetomter measurements when doing 3D fusion
			if (_control_status.flags.synthetic_mag_z) {
				fused[2] = true;
				continue;
			}

			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeMagZInnovVarAndH(state_vector, P, R_MAG, FLT_EPSILON, &aid_src.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			aid_src.innovation[index] = _state.quat_nominal.rotateVectorInverse(_state.mag_I)(index) + _state.mag_B(index) - mag(index);
		}

		if (aid_src.innovation_variance[index] < R_MAG) {
			ECL_ERR("mag numerical error covariance reset");

			// we need to re-initialise covariances and abort this fusion step
			if (update_all_states) {
				resetQuatCov(_params.mag_heading_noise);
			}

			resetMagCov();

			return false;
		}

		VectorState Kfusion = P * H / aid_src.innovation_variance[index];

		if (update_all_states) {
			if (!update_tilt) {
				Kfusion(State::quat_nominal.idx + 0) = 0.f;
				Kfusion(State::quat_nominal.idx + 1) = 0.f;
			}

		} else {
			// zero non-mag Kalman gains if not updating all states

			// copy mag_I and mag_B Kalman gains
			const Vector3f K_mag_I = Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0);
			const Vector3f K_mag_B = Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0);

			// zero all Kalman gains, then restore mag
			Kfusion.setZero();
			Kfusion.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) = K_mag_I;
			Kfusion.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) = K_mag_B;
		}

		if (measurementUpdate(Kfusion, H, aid_src.observation_variance[index], aid_src.innovation[index])) {
			fused[index] = true;
		}
	}

	if (update_all_states) {
		_fault_status.flags.bad_mag_x = !fused[0];
		_fault_status.flags.bad_mag_y = !fused[1];
		_fault_status.flags.bad_mag_z = !fused[2];
	}

	if (fused[0] && fused[1] && fused[2]) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		if (update_all_states) {
			_time_last_heading_fuse = _time_delayed_us;
		}

		return true;
	}

	return false;
}

bool Ekf::fuseDeclination(float decl_sigma)
{
	float decl_measurement = NAN;

	if ((_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
	    && PX4_ISFINITE(_mag_declination_gps)
	   ) {
		decl_measurement = _mag_declination_gps;

	} else if ((_params.mag_declination_source & GeoDeclinationMask::SAVE_GEO_DECL)
		   && PX4_ISFINITE(_params.mag_declination_deg) && (fabsf(_params.mag_declination_deg) > 0.f)
		  ) {
		decl_measurement = math::radians(_params.mag_declination_deg);
	}

	if (PX4_ISFINITE(decl_measurement)) {

		// observation variance (rad**2)
		const float R_DECL = sq(decl_sigma);

		VectorState H;
		float decl_pred;
		float innovation_variance;

		sym::ComputeMagDeclinationPredInnovVarAndH(_state.vector(), P, R_DECL, FLT_EPSILON, &decl_pred, &innovation_variance, &H);

		const float innovation = wrap_pi(decl_pred - decl_measurement);

		if (innovation_variance < R_DECL) {
			// variance calculation is badly conditioned
			return false;
		}

		// Calculate the Kalman gains
		VectorState Kfusion = P * H / innovation_variance;

		const bool is_fused = measurementUpdate(Kfusion, H, R_DECL, innovation);

		_fault_status.flags.bad_mag_decl = !is_fused;

		return is_fused;
	}

	return false;
}

float Ekf::calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted)
{
	// theoretical magnitude of the magnetometer Z component value given X and Y sensor measurement and our knowledge
	// of the earth magnetic field vector at the current location
	const float mag_z_abs = sqrtf(math::max(sq(mag_earth_predicted.length()) - sq(mag_meas(0)) - sq(mag_meas(1)), 0.0f));

	// calculate sign of synthetic magnetomter Z component based on the sign of the predicted magnetometer Z component
	const float mag_z_body_pred = mag_earth_predicted.dot(_R_to_earth.col(2));

	return (mag_z_body_pred < 0) ? -mag_z_abs : mag_z_abs;
}
