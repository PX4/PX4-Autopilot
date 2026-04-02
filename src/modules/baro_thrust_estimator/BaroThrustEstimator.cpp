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
 * An RLS estimator identifies the gain K using the commanded collective thrust
 * magnitude from vehicle_thrust_setpoint (|xyz[2]|). Converged parameters are
 * saved on disarm.
 */

#include "BaroThrustEstimator.hpp"

#include <mathlib/mathlib.h>

using namespace time_literals;

ModuleBase::Descriptor BaroThrustEstimator::desc{task_spawn, custom_command, print_usage};

BaroThrustEstimator::BaroThrustEstimator() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_estimator.reset();
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
	_estimator.setCfBandwidth(_param_sens_bar_cf_bw.get());
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

			if (was_armed != _armed) {
				if (was_armed && !_armed) {
					saveParameters();
				}

				_estimator.reset();
				_estimation_start_time = 0;
				_last_update_time = 0;
				_last_publish_time = 0;
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

	// Validate baro
	if (!PX4_ISFINITE(air_data.baro_alt_meter)) {
		perf_end(_cycle_perf);
		return;
	}

	// Get acceleration in body frame
	vehicle_acceleration_s accel{};

	if (!_vehicle_acceleration_sub.copy(&accel)
	    || hrt_elapsed_time(&accel.timestamp) > 100_ms
	    || !PX4_ISFINITE(accel.xyz[0]) || !PX4_ISFINITE(accel.xyz[1]) || !PX4_ISFINITE(accel.xyz[2])) {
		perf_end(_cycle_perf);
		return;
	}

	// Get attitude quaternion
	vehicle_attitude_s att{};

	if (!_vehicle_attitude_sub.copy(&att)
	    || hrt_elapsed_time(&att.timestamp) > 100_ms
	    || !PX4_ISFINITE(att.q[0])) {
		perf_end(_cycle_perf);
		return;
	}

	// Compute upward linear acceleration and CF residual
	const float accel_up = BaroThrustCfRls::computeAccelUp(
				       matrix::Vector3f{accel.xyz}, matrix::Quatf{att.q});

	const float residual = _estimator.updateCf(air_data.baro_alt_meter, accel_up, dt);

	if (!PX4_ISFINITE(residual)) {
		perf_end(_cycle_perf);
		return;
	}

	// Soft guards: skip RLS update but keep CF current
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

	// Get vertical thrust setpoint — uses latest sample rather than time-matched
	// (unlike thrustCompensation). Fine for a statistical estimator; timing jitter is noise.
	float thrust = 0.f;

	vehicle_thrust_setpoint_s thrust_sp{};

	if (_vehicle_thrust_setpoint_sub.copy(&thrust_sp)
	    && hrt_elapsed_time(&thrust_sp.timestamp) < 500_ms
	    && PX4_ISFINITE(thrust_sp.xyz[2])) {
		thrust = fabsf(thrust_sp.xyz[2]);

	} else {
		estimation_active = false;
	}

	// Once converged, freeze RLS — the estimate is good and continued
	// updates during descent/landing would corrupt it with ground effect.
	// Note: when updateEstimator() stops being called, all internal
	// tracking state (thrust variance, K smoothed, RLS covariance) freezes,
	// so the convergence criteria checked by checkConvergence() remain
	// stable and converged() cannot flicker back to false.
	if (estimation_active && !_estimator.converged() && !_estimator.convergedLocked()) {
		_estimator.updateEstimator(residual, thrust, dt);
	}

	// Convergence checking runs independently of soft guards so the hold
	// timer keeps ticking during descent. With RLS frozen the checked
	// values (variance, error, stability) remain stable.
	if (!_estimator.convergedLocked()) {
		const float elapsed_s = static_cast<float>(now - _estimation_start_time) * 1e-6f;
		_estimator.checkConvergence(elapsed_s, dt);

		if (_estimator.convergedLocked()) {
			PX4_INFO("convergence locked (K=%.2f)", (double)_estimator.kEstimate());
		}
	}

	if (now - _last_publish_time > 200_ms) {
		publishStatus(now, residual, estimation_active);
		_last_publish_time = now;
	}

	perf_end(_cycle_perf);
}

void BaroThrustEstimator::saveParameters()
{
	if (!_estimator.convergedLocked()) {
		PX4_INFO("baro thrust estimator did not converge (K=%.2f, var=%.1f, err=%.2f, thr_std=%.3f)",
			 (double)_estimator.kEstimate(),
			 (double)_estimator.kEstimateVar(),
			 (double)_estimator.errorVar(),
			 (double)_estimator.thrustStd());
		return;
	}

	const float K_est = _estimator.kEstimate();

	// Only update if the remaining error is significant
	if (fabsf(K_est) < MIN_K_UPDATE_THRESHOLD) {
		PX4_INFO("K_est=%.2f below threshold, calibration adequate", (double)K_est);
		return;
	}

	// K_est is the residual gain the estimator sees *after* existing compensation,
	// so the corrected PCOEF shifts by -K_est to cancel it out.
	const float pcoef_new = _param_sens_baro_pcoef.get() - K_est;

	if (!PX4_ISFINITE(pcoef_new)) {
		PX4_WARN("non-finite result, skipping save");
		return;
	}

	if (fabsf(pcoef_new) > PCOEF_MAX) {
		PX4_WARN("result out of range (pcoef=%.1f)", (double)pcoef_new);
		return;
	}

	PX4_INFO("saving SENS_BARO_PCOEF=%.1f (K_est=%.2f, prev_pcoef=%.1f)",
		 (double)pcoef_new, (double)K_est,
		 (double)_param_sens_baro_pcoef.get());

	_param_sens_baro_pcoef.set(pcoef_new);
	_param_sens_baro_pcoef.commit_no_notification();
}

void BaroThrustEstimator::publishStatus(hrt_abstime now, float residual, bool estimation_active)
{
	baro_thrust_estimate_s status{};
	status.timestamp_sample = now;
	status.residual = residual;
	status.k_estimate = _estimator.kEstimate();
	status.k_estimate_var = _estimator.kEstimateVar();
	status.error_var = _estimator.errorVar();
	status.thrust_std = _estimator.thrustStd();
	status.converged = _estimator.converged();
	status.estimation_active = estimation_active && !_estimator.converged();
	status.timestamp = hrt_absolute_time();

	_baro_thrust_estimate_pub.publish(status);
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
	PX4_INFO("converged: %s", _estimator.converged() ? "yes" : "no");
	PX4_INFO("K_est: %.3f  K_var: %.4f  error_var: %.4f",
		 (double)_estimator.kEstimate(), (double)_estimator.kEstimateVar(),
		 (double)_estimator.errorVar());
	PX4_INFO("thrust std: %.4f", (double)_estimator.thrustStd());
	PX4_INFO("current SENS_BARO_PCOEF: %.1f", (double)_param_sens_baro_pcoef.get());

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

Online estimator for barometer thrust compensation (SENS_BARO_PCOEF).

Uses an accel-baro complementary filter to isolate thrust-induced baro pressure errors.
The CF residual captures propwash effects while rejecting real altitude changes and
accelerometer drift. An RLS estimator identifies the thrust-to-baro gain K using the
commanded collective thrust magnitude from vehicle_thrust_setpoint (|xyz[2]|).
Converged parameters are saved on disarm and refined over subsequent flights.

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
