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

/**
 * @file BaroThrustEstimator.cpp
 *
 * Online estimator for barometer thrust compensation parameters.
 *
 * Uses a complementary filter (accel-integrated altitude vs baro) to isolate
 * thrust-induced baro pressure errors. The CF residual (baro - accel_prediction)
 * captures propwash effects while rejecting real altitude changes and accel drift.
 * A bank of parallel RLS estimators with different filter time constants identifies
 * the gain K and optimal tau. Converged parameters are saved on disarm.
 */

#include "BaroThrustEstimator.hpp"

#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using namespace time_literals;

constexpr float BaroThrustEstimator::TAU_CANDIDATES[];

ModuleBase::Descriptor BaroThrustEstimator::desc{task_spawn, custom_command, print_usage};

BaroThrustEstimator::BaroThrustEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	reset();
	updateParams();
}

BaroThrustEstimator::~BaroThrustEstimator()
{
	perf_free(_cycle_perf);
}

bool BaroThrustEstimator::init()
{
	if (!_vehicle_air_data_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void BaroThrustEstimator::updateParams()
{
	ModuleParams::updateParams();
}

void BaroThrustEstimator::reset()
{
	for (int i = 0; i < NUM_TAU_CANDIDATES; i++) {
		_bank[i].reset(TAU_CANDIDATES[i], RLS_P_INIT);
	}

	_cf_alt = 0.f;
	_cf_vel = 0.f;
	_cf_initialized = false;

	_estimation_start_time = 0;
	_last_update_time = 0;
	_last_publish_time = 0;
	_converged = false;
	_converged_locked = false;
	_converged_since = 0;
	_k_stable_since = 0;
	_best_bank_idx = 0;
	_k_est_smoothed.reset(0.f);
	_thrust_mean.reset(0.f);
	_thrust_var.reset(0.f);
}

void BaroThrustEstimator::Run()
{
	if (should_exit()) {
		_vehicle_air_data_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	// Handle parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	// Track arm state for disarm edge detection
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s status;

		if (_vehicle_status_sub.copy(&status)) {
			const bool was_armed = _armed;
			_armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (was_armed && !_armed) {
				saveParameters();
				reset();
			}

			if (!was_armed && _armed) {
				reset();
			}
		}
	}

	// Track landed state
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s ld;

		if (_vehicle_land_detected_sub.copy(&ld)) {
			_landed = ld.landed;
		}
	}

	// Get baro data (callback trigger)
	vehicle_air_data_s air_data{};

	if (!_vehicle_air_data_sub.copy(&air_data)) {
		perf_end(_cycle_perf);
		return;
	}

	// Hard guards: no estimation or CF when disarmed/landed
	if (!_armed || _landed) {
		perf_end(_cycle_perf);
		return;
	}

	// Compute dt
	const hrt_abstime now = air_data.timestamp_sample;

	if (_last_update_time == 0) {
		_last_update_time = now;
		_estimation_start_time = now;
		perf_end(_cycle_perf);
		return;
	}

	const float dt = math::constrain(static_cast<float>(now - _last_update_time) * 1e-6f, 0.001f, 0.5f);
	_last_update_time = now;

	// Always update CF to keep state fresh even when RLS guards are active
	const float residual = computeCfResidual(air_data.baro_alt_meter, dt);

	if (!PX4_ISFINITE(residual)) {
		perf_end(_cycle_perf);
		return;
	}

	// Soft guards: skip RLS update but keep CF current to avoid
	// re-initialization transient when guards clear
	bool estimation_active = true;

	vehicle_local_position_s local_pos{};

	if (_vehicle_local_position_sub.copy(&local_pos)) {
		if (local_pos.v_z_valid && fabsf(local_pos.vz) > MAX_VZ) {
			estimation_active = false;
		}

		if (local_pos.v_xy_valid
		    && (local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy) > MAX_VXY * MAX_VXY) {
			estimation_active = false;
		}
	}

	vehicle_thrust_setpoint_s thrust_sp{};
	float thrust = 0.f;

	if (!_vehicle_thrust_setpoint_sub.copy(&thrust_sp)
	    || hrt_elapsed_time(&thrust_sp.timestamp) > 500_ms) {
		estimation_active = false;

	} else {
		thrust = math::constrain(-thrust_sp.xyz[2], 0.f, 1.f);
	}

	if (estimation_active && !_converged_locked) {
		updateEstimators(residual, thrust, dt);
		checkConvergence(now);
	}

	if (now - _last_publish_time > 200_ms) {
		publishStatus(now, residual, estimation_active);
		_last_publish_time = now;
	}

	perf_end(_cycle_perf);
}

float BaroThrustEstimator::computeCfResidual(float baro_alt, float dt)
{
	// Reject NaN baro before mutating CF state
	if (!PX4_ISFINITE(baro_alt)) {
		return NAN;
	}

	// Get acceleration in body frame
	vehicle_acceleration_s accel{};

	if (!_vehicle_acceleration_sub.copy(&accel)
	    || hrt_elapsed_time(&accel.timestamp) > 100_ms
	    || !PX4_ISFINITE(accel.xyz[0]) || !PX4_ISFINITE(accel.xyz[1]) || !PX4_ISFINITE(accel.xyz[2])) {
		return NAN;
	}

	// Get attitude quaternion
	vehicle_attitude_s att{};

	if (!_vehicle_attitude_sub.copy(&att)
	    || hrt_elapsed_time(&att.timestamp) > 100_ms
	    || !PX4_ISFINITE(att.q[0])) {
		return NAN;
	}

	// Rotate body-frame specific force to NED, extract Z component
	const matrix::Quatf q{att.q};
	const matrix::Vector3f accel_body{accel.xyz};
	const float specific_force_ned_z = (q.rotateVector(accel_body))(2);

	// Convert to altitude-up coordinate acceleration
	// altitude = -NED_z, so d²(alt)/dt² = -(specific_force_ned_z + g)
	const float accel_up = -(specific_force_ned_z + CONSTANTS_ONE_G);

	// Initialize CF on first sample
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
	// Contains propwash error + slow drift (absorbed by CF correction below)
	const float residual = baro_alt - alt_pred;

	// Correct CF state with baro at low bandwidth to prevent accel drift
	// 2nd-order critically damped: K1 = 2*omega, K2 = omega²
	const float omega = 2.f * M_PI_F * CF_BANDWIDTH_HZ;
	const float K1 = 2.f * omega;  // ~0.63 for 0.05 Hz
	const float K2 = omega * omega; // ~0.099 for 0.05 Hz

	_cf_alt = alt_pred + K1 * dt * residual;
	_cf_vel = vel_pred + K2 * dt * residual;

	return residual;
}

void BaroThrustEstimator::updateEstimators(float residual, float thrust, float dt)
{
	for (int i = 0; i < NUM_TAU_CANDIDATES; i++) {
		_bank[i].update(residual, thrust, dt, RLS_LAMBDA);
	}

	// Track thrust excitation
	_thrust_mean.setParameters(dt, 2.f);
	_thrust_mean.update(thrust);
	const float thrust_dev = thrust - _thrust_mean.getState();
	_thrust_var.setParameters(dt, 2.f);
	_thrust_var.update(thrust_dev * thrust_dev);

	// Select best bank
	_best_bank_idx = selectBestBank();

	// Track K stability
	_k_est_smoothed.setParameters(dt, 5.f);
	_k_est_smoothed.update(_bank[_best_bank_idx].theta[0]);
}

int BaroThrustEstimator::selectBestBank() const
{
	int best = 0;
	float best_err = _bank[0].error_var;

	for (int i = 1; i < NUM_TAU_CANDIDATES; i++) {
		if (_bank[i].error_var < best_err) {
			best_err = _bank[i].error_var;
			best = i;
		}
	}

	// Hysteresis: keep current bank unless new best is significantly better
	if (best != _best_bank_idx) {
		const float current_err = _bank[_best_bank_idx].error_var;

		if (best_err > current_err * BANK_SWITCH_HYSTERESIS) {
			return _best_bank_idx;
		}
	}

	return best;
}

void BaroThrustEstimator::checkConvergence(hrt_abstime now)
{
	// Once locked, convergence is permanent until reset (arm/disarm)
	if (_converged_locked) {
		return;
	}

	const auto &best = _bank[_best_bank_idx];

	const bool variance_ok = best.P[0][0] < CONVERGENCE_VAR_THR;
	const bool error_ok = best.error_var < CONVERGENCE_ERR_THR;
	const bool excitation_ok = fmaxf(_thrust_var.getState(), 0.f) > (MIN_THRUST_EXCITATION * MIN_THRUST_EXCITATION);

	const float elapsed_s = static_cast<float>(now - _estimation_start_time) * 1e-6f;
	const bool time_ok = elapsed_s > MIN_ESTIMATION_TIME_S;

	const float k_current = _k_est_smoothed.getState();
	const float k_best = best.theta[0];

	if (fabsf(k_current - k_best) > K_STABILITY_DIFF_THR) {
		_k_stable_since = now;
	}

	if (_k_stable_since == 0) {
		_k_stable_since = now;
	}

	const bool stability_ok = static_cast<float>(now - _k_stable_since) * 1e-6f > K_STABILITY_TIME_S;

	_converged = variance_ok && error_ok && excitation_ok && time_ok && stability_ok;

	// Track how long we've been continuously converged
	if (_converged) {
		if (_converged_since == 0) {
			_converged_since = now;
		}

		const float converged_s = static_cast<float>(now - _converged_since) * 1e-6f;

		if (converged_s > CONVERGENCE_HOLD_TIME_S) {
			_converged_locked = true;
			PX4_INFO("convergence locked (K=%.2f, tau=%.3f)",
				 (double)best.theta[0], (double)TAU_CANDIDATES[_best_bank_idx]);
		}

	} else {
		_converged_since = 0;
	}
}

void BaroThrustEstimator::saveParameters()
{
	if (!_converged) {
		PX4_INFO("not converged, skipping parameter save");
		return;
	}

	const float K_est = _bank[_best_bank_idx].theta[0];
	const float best_tau = TAU_CANDIDATES[_best_bank_idx];

	const float pcoef_new = _param_sens_baro_pcoef.get() - K_est;
	const float ptau_new = best_tau;

	if (!PX4_ISFINITE(pcoef_new) || !PX4_ISFINITE(ptau_new)) {
		PX4_WARN("non-finite result, skipping save");
		return;
	}

	if (fabsf(pcoef_new) > PCOEF_MAX || ptau_new < 0.f || ptau_new > PTAU_MAX) {
		PX4_WARN("result out of range (pcoef=%.1f, ptau=%.2f)", (double)pcoef_new, (double)ptau_new);
		return;
	}

	PX4_INFO("saving SENS_BARO_PCOEF=%.1f SENS_BARO_PTAU=%.2f (K_est=%.2f, prev_pcoef=%.1f)",
		 (double)pcoef_new, (double)ptau_new, (double)K_est,
		 (double)_param_sens_baro_pcoef.get());

	_param_sens_baro_pcoef.set(pcoef_new);
	_param_sens_baro_ptau.set(ptau_new);
	_param_sens_baro_pcoef.commit_no_notification();
	_param_sens_baro_ptau.commit();
}

void BaroThrustEstimator::publishStatus(hrt_abstime now, float residual, bool estimation_active)
{
	const auto &best = _bank[_best_bank_idx];

	baro_thrust_estimate_s status{};
	status.timestamp_sample = now;
	status.residual = residual;
	status.k_estimate = best.theta[0];
	status.k_estimate_var = best.P[0][0];
	status.best_tau = TAU_CANDIDATES[_best_bank_idx];
	status.best_error_var = best.error_var;
	status.thrust_std = sqrtf(fmaxf(_thrust_var.getState(), 0.f));
	status.converged = _converged;
	status.estimation_active = estimation_active;
	status.best_bank_idx = static_cast<uint8_t>(_best_bank_idx);
	status.timestamp = hrt_absolute_time();

	_baro_thrust_estimate_pub.publish(status);
}

// ---------------------------------------------------------------------------
// RlsEstimator
// ---------------------------------------------------------------------------

void BaroThrustEstimator::RlsEstimator::reset(float tau_val, float p_init)
{
	tau = tau_val;
	theta[0] = 0.f;
	theta[1] = 0.f;
	P[0][0] = p_init;
	P[0][1] = 0.f;
	P[1][0] = 0.f;
	P[1][1] = p_init;
	error_var = ERROR_VAR_INIT;
	thrust_lpf.reset(0.f);
}

void BaroThrustEstimator::RlsEstimator::update(float residual, float thrust_raw, float dt, float lambda)
{
	float filtered_thrust;

	if (tau > FLT_EPSILON) {
		thrust_lpf.setParameters(dt, tau);
		thrust_lpf.update(thrust_raw);
		filtered_thrust = thrust_lpf.getState();

	} else {
		thrust_lpf.reset(thrust_raw);
		filtered_thrust = thrust_raw;
	}

	const float phi[2] = {filtered_thrust, 1.f};
	const float e = residual - (theta[0] * phi[0] + theta[1] * phi[1]);

	const float Pphi[2] = {
		P[0][0] * phi[0] + P[0][1] * phi[1],
		P[1][0] * phi[0] + P[1][1] * phi[1]
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

	// Guard against numerical instability — reset bank if any state is non-finite
	if (!PX4_ISFINITE(theta[0]) || !PX4_ISFINITE(theta[1])
	    || !PX4_ISFINITE(P[0][0]) || !PX4_ISFINITE(P[1][1])) {
		reset(tau, RLS_P_INIT);
	}
}

// ---------------------------------------------------------------------------
// Module boilerplate
// ---------------------------------------------------------------------------

int BaroThrustEstimator::task_spawn(int argc, char *argv[])
{
	BaroThrustEstimator *instance = new BaroThrustEstimator();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int BaroThrustEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BaroThrustEstimator::print_status()
{
	const auto &best = _bank[_best_bank_idx];

	PX4_INFO("converged: %s", _converged ? "yes" : "no");
	PX4_INFO("best bank: %d (tau=%.3f)", _best_bank_idx, (double)TAU_CANDIDATES[_best_bank_idx]);
	PX4_INFO("K_est: %.3f  bias: %.3f  P_K: %.4f",
		 (double)best.theta[0], (double)best.theta[1], (double)best.P[0][0]);
	PX4_INFO("thrust std: %.4f", (double)sqrtf(fmaxf(_thrust_var.getState(), 0.f)));
	PX4_INFO("current SENS_BARO_PCOEF: %.1f  SENS_BARO_PTAU: %.2f",
		 (double)_param_sens_baro_pcoef.get(), (double)_param_sens_baro_ptau.get());

	PX4_INFO("bank errors: [%.2f, %.2f, %.2f, %.2f, %.2f]",
		 (double)_bank[0].error_var, (double)_bank[1].error_var,
		 (double)_bank[2].error_var, (double)_bank[3].error_var,
		 (double)_bank[4].error_var);

	perf_print_counter(_cycle_perf);
	return 0;
}

int BaroThrustEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Online estimator for barometer thrust compensation parameters (SENS_BARO_PCOEF, SENS_BARO_PTAU).

Uses an accel-baro complementary filter to isolate thrust-induced baro pressure errors.
The CF residual captures propwash effects while rejecting real altitude changes and
accelerometer drift. A bank of parallel RLS estimators with different filter time
constants jointly identifies the gain and time constant. Converged parameters are
saved on disarm.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("baro_thrust_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int baro_thrust_estimator_main(int argc, char *argv[])
{
	return ModuleBase::main(BaroThrustEstimator::desc, argc, argv);
}
