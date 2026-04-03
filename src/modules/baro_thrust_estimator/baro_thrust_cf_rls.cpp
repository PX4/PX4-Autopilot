/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "baro_thrust_cf_rls.hpp"

void BaroThrustCfRls::reset()
{
	_rls.reset(RLS_P_INIT);

	_cf_alt = 0.f;
	_cf_vel = 0.f;
	_cf_initialized = false;

	_converged = false;
	_converged_locked = false;
	_converged_elapsed_s = 0.f;
	_k_stable_elapsed_s = 0.f;
	_excitation_elapsed_s = 0.f;
	_k_est_smoothed.reset(0.f);
	_thrust_mean.reset(0.f);
	_thrust_var.reset(0.f);
}

float BaroThrustCfRls::computeAccelUp(const matrix::Vector3f &accel_body,
				      const matrix::Quatf &attitude)
{
	// Rotate body-frame specific force to NED, extract Z component
	const float specific_force_ned_z = (attitude.rotateVector(accel_body))(2);

	// Convert to altitude-up coordinate acceleration
	// specific_force = a_inertial - g_vec, so a_inertial_z = specific_force_z + g
	// altitude = -NED_z, so accel_up = -(specific_force_z + g)
	return -(specific_force_ned_z + CONSTANTS_ONE_G);
}

float BaroThrustCfRls::updateCf(float baro_alt, float accel_up, float dt)
{
	if (!_cf_initialized) {
		_cf_alt = baro_alt;
		_cf_vel = 0.f;
		_cf_initialized = true;
		return 0.f;
	}

	// Predict altitude from accel integration
	const float alt_pred = _cf_alt + _cf_vel * dt + 0.5f * accel_up * dt * dt;
	const float vel_pred = _cf_vel + accel_up * dt;

	// CF residual: what the baro says minus what accel predicts
	const float residual = baro_alt - alt_pred;

	// Correct CF state with baro at low bandwidth to prevent accel drift
	// 2nd-order critically damped: K1 = 2*omega, K2 = omega^2
	const float omega = 2.f * static_cast<float>(M_PI) * _cf_bandwidth_hz;
	const float K1 = 2.f * omega;
	const float K2 = omega * omega;

	_cf_alt = alt_pred + K1 * dt * residual;
	_cf_vel = vel_pred + K2 * dt * residual;

	if (!std::isfinite(_cf_alt) || !std::isfinite(_cf_vel)) {
		_cf_alt = baro_alt;
		_cf_vel = 0.f;
		return 0.f;
	}

	return residual;
}

void BaroThrustCfRls::updateEstimator(float residual, float thrust, float dt)
{
	_rls.update(residual, thrust, dt, RLS_LAMBDA);

	// Track thrust excitation (compute deviation before updating mean
	// so the current sample doesn't bias the mean used for deviation)
	_thrust_mean.setParameters(dt, 2.f);
	const float thrust_dev = thrust - _thrust_mean.getState();
	_thrust_mean.update(thrust);
	_thrust_var.setParameters(dt, 2.f);
	_thrust_var.update(thrust_dev * thrust_dev);

	// Track K stability
	_k_est_smoothed.setParameters(dt, 5.f);
	_k_est_smoothed.update(_rls.theta[0]);
}

void BaroThrustCfRls::checkConvergence(float elapsed_since_start_s, float dt)
{
	if (_converged_locked) {
		return;
	}

	const bool variance_ok = _rls.P[0][0] < CONVERGENCE_VAR_THR;

	// Accumulate time with sufficient thrust excitation. RLS doesn't need
	// continuous excitation — intermittent bursts are enough to identify K.
	if (fmaxf(_thrust_var.getState(), 0.f) > (MIN_THRUST_EXCITATION * MIN_THRUST_EXCITATION)) {
		_excitation_elapsed_s += dt;
	}

	const bool excitation_ok = _excitation_elapsed_s > MIN_EXCITATION_TIME_S;

	// Dual-path error check: absolute threshold works for refinement flights
	// (PCOEF already set), relative threshold allows first calibration where
	// the uncompensated CF residual is noisy but the model explains most of it.
	const float K = _rls.theta[0];
	const float explained_var = K * K * fmaxf(_thrust_var.getState(), 0.f);
	const float total_var = explained_var + _rls.error_var;
	const bool error_ok = _rls.error_var < CONVERGENCE_ERR_THR
			      || (total_var > 1.f
				  && _rls.error_var / total_var < CONVERGENCE_ERR_REL_THR
				  && _rls.error_var < CONVERGENCE_ERR_MAX_THR);
	const bool time_ok = elapsed_since_start_s > MIN_ESTIMATION_TIME_S;

	const float k_current = _k_est_smoothed.getState();
	const float k_best = _rls.theta[0];

	if (fabsf(k_current - k_best) > K_STABILITY_DIFF_THR) {
		_k_stable_elapsed_s = 0.f;

	} else {
		_k_stable_elapsed_s += dt;
	}

	const bool stability_ok = _k_stable_elapsed_s > K_STABILITY_TIME_S;

	_converged = variance_ok && error_ok && excitation_ok && time_ok && stability_ok;

	// Once converged, keep accumulating hold time even if excitation
	// momentarily dips — the estimate quality checks (variance, error,
	// stability) are sufficient to guard the hold period.
	const bool hold_ok = variance_ok && error_ok && time_ok && stability_ok;

	if (_converged || (_converged_elapsed_s > 0.f && hold_ok)) {
		_converged_elapsed_s += dt;

		if (_converged_elapsed_s > CONVERGENCE_HOLD_TIME_S) {
			_converged_locked = true;
		}

	} else {
		_converged_elapsed_s = 0.f;
	}
}

// ---------------------------------------------------------------------------
// RlsEstimator
// ---------------------------------------------------------------------------

void BaroThrustCfRls::RlsEstimator::reset(float p_init)
{
	theta[0] = 0.f;
	theta[1] = 0.f;
	P[0][0] = p_init;
	P[0][1] = 0.f;
	P[1][0] = 0.f;
	P[1][1] = p_init;
	error_var = ERROR_VAR_INIT;
}

void BaroThrustCfRls::RlsEstimator::update(float residual, float thrust, float dt, float lambda)
{
	const float phi[2] = {thrust, 1.f};
	const float e = residual - (theta[0] * phi[0] + theta[1] * phi[1]);

	const float Pphi[2] = {
		P[0][0] *phi[0] + P[0][1] *phi[1],
		P[1][0] *phi[0] + P[1][1] *phi[1]
	};

	const float phiPphi = phi[0] * Pphi[0] + phi[1] * Pphi[1];
	const float denom = lambda + phiPphi;

	if (fabsf(denom) < 1e-10f) {
		return;
	}

	const float inv_denom = 1.f / denom;

	theta[0] += Pphi[0] * inv_denom * e;
	theta[1] += Pphi[1] * inv_denom * e;

	const float inv_lambda = 1.f / lambda;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			P[i][j] = (P[i][j] - Pphi[i] * Pphi[j] * inv_denom) * inv_lambda;
		}
	}

	constexpr float alpha_err = 0.01f;
	error_var = (1.f - alpha_err) * error_var + alpha_err * e * e;

	// Guard against numerical instability
	if (!std::isfinite(theta[0]) || !std::isfinite(theta[1])
	    || !std::isfinite(P[0][0]) || !std::isfinite(P[0][1])
	    || !std::isfinite(P[1][0]) || !std::isfinite(P[1][1])
	    || !std::isfinite(error_var)) {
		reset(RLS_P_INIT);
	}
}
