/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file TargetEstimator.cpp
 *
 */

#include "TargetEstimator.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

using matrix::Vector2f;
using matrix::Vector3f;
using matrix::Vector3;

TargetEstimator::TargetEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

}

TargetEstimator::~TargetEstimator()
{
	Stop();
}

bool TargetEstimator::Start()
{
	// Initialize this filter
	reset();

	// force initialize parameters update
	parameters_update(true);

	// follow_target messages needed for estimation
	if (!_follow_target_sub.registerCallback()) {
		PX4_ERR("target_estimator callback registration failed");
	}

	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("target_estimator callback registration failed");
	}

	return true;
}

void TargetEstimator::Stop()
{
	// clear all registered callbacks
	_follow_target_sub.unregisterCallback();
	_vehicle_local_position_sub.unregisterCallback();

	Deinit();
}

void TargetEstimator::Run()
{
	ScheduleDelayed(10_ms);

	update();
}

void TargetEstimator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void TargetEstimator::update()
{
	const hrt_abstime now = hrt_absolute_time();

	// compute deltatime between update() calls
	if (_last_iteration_timestamp == 0) {
		_last_iteration_timestamp = now;
		return;
	}

	const float deltatime = math::constrain((now - _last_iteration_timestamp) * 1e-6f, 1e-3f, 2.0f);
	_last_iteration_timestamp = now;

	parameters_update();
	update_filter_gains(_filter_gains);


	// Get GPS reference location for NED frame, needed for projection
	_vehicle_local_position_sub.update(&_vehicle_local_position);

	// Perform sensor fusion update if there's a new GPS message from the follow-target
	prediction_update(deltatime);

	follow_target_s follow_target;

	if (_follow_target_sub.update(&follow_target)) {

		// Don't perform measurement update if two follow_target messages with identical timestamps are used
		// This can happen when using the MAVSDK and more than one outgoing follow_target message is queued.
		const bool duplicate_measurement_received = follow_target.timestamp == _last_follow_target_timestamp;

		// Skip measurements that lie in the past
		const bool measurement_in_the_past = _last_position_fusion_timestamp >= follow_target.timestamp;

		// Need at least one vehicle_local_position before estimator can work
		const bool vehicle_local_position_invalid = _vehicle_local_position.timestamp == 0;

		if (!duplicate_measurement_received && !measurement_in_the_past && !vehicle_local_position_invalid) {
			measurement_update(follow_target);
		}
	}

	// Keep position estimate as the last known position
	// but stop moving the estimate
	if (is_stale(GPS_MESSAGE_STALE_TIMEOUT_MS)) {
		_filter_states.vel_ned_est.setZero();
		_filter_states.acc_ned_est.setZero();
	}

	const bool states_are_finite = _filter_states.is_finite();

	if (!states_are_finite) {
		reset();
	}

	// Publish estimator message
	follow_target_estimator_s follow_target_estimator{};
	follow_target_estimator.timestamp = hrt_absolute_time();
	follow_target_estimator.valid = states_are_finite;
	follow_target_estimator.stale = is_stale(GPS_MESSAGE_STALE_TIMEOUT_MS);
	follow_target_estimator.last_filter_reset_timestamp = _last_filter_reset_timestamp;
	follow_target_estimator.lat_est = get_lat_lon_alt_est()(0);
	follow_target_estimator.lon_est = get_lat_lon_alt_est()(1);
	follow_target_estimator.alt_est = get_lat_lon_alt_est()(2);
	_filter_states.pos_ned_est.copyTo(follow_target_estimator.pos_est);
	_filter_states.vel_ned_est.copyTo(follow_target_estimator.vel_est);
	_filter_states.acc_ned_est.copyTo(follow_target_estimator.acc_est);
	follow_target_estimator.prediction_count = _prediction_count;
	follow_target_estimator.fusion_count = _fusion_count;
	_follow_target_estimator_pub.publish(follow_target_estimator);
}

void TargetEstimator::update_filter_gains(filter_gains_s &filter_gains) const
{
	const float responsiveness_param = math::constrain(_param_nav_ft_rs.get(), .1F, 1.0F);

	if (fabsf(filter_gains.responsiveness - responsiveness_param) < FLT_EPSILON) {
		// Parameter did not change since last execution. Skip calculations
		return;
	}

	filter_gains.responsiveness = responsiveness_param;

	// The "G" gain is equivalent to "(1-responsiveness)", but beta is required for H and K gains
	// From alpha-beta-gamma filter equations: G = 1-beta^3
	// Therefore: beta = (1-Gp)^(1/3) = (1-(1-responsiveness))^(1/3) = (r)^(1/3)
	const float beta_p = std::pow((filter_gains.responsiveness), 1.0f / 3.0f);
	const float beta_v = 0.9f * beta_p; // velocity fusion gain is slightly lower. TODO: individual parameter?

	// Estimator gains for horizontal position update
	filter_gains.G_p = 1.0f - beta_p * beta_p * beta_p;
	filter_gains.H_p = 1.5f * (1.0f - beta_p) * (1.0f - beta_p) * (1.0f + beta_p);
	filter_gains.K_p = 0.5f * (1.0f - beta_p) * (1.0f - beta_p) * (1.0f - beta_p);

	// Estimator gains for velocity update
	filter_gains.G_v = 1.0f - beta_v * beta_v ;
	filter_gains.H_v = (1.0f - beta_v) * (1.0f - beta_v);
}

bool TargetEstimator::measurement_can_be_fused(const Vector3f &current_measurement,
		const Vector3f &previous_measurement,
		uint64_t last_fusion_timestamp, float min_delta_t) const
{
	const bool measurement_valid = PX4_ISFINITE(current_measurement(0)) && PX4_ISFINITE(current_measurement(1))
				       && PX4_ISFINITE(current_measurement(2));

	const bool sensor_data_changed = Vector3f(current_measurement - previous_measurement).longerThan(2.0f * FLT_EPSILON)
					 || !PX4_ISFINITE(previous_measurement(0)) || !PX4_ISFINITE(previous_measurement(1))
					 || !PX4_ISFINITE(previous_measurement(2));

	// This is required as a throttle
	const bool fusion_old_enough = hrt_absolute_time() - last_fusion_timestamp >
				       min_delta_t * 1000;

	// TODO: Remove this workaround
	const bool fusion_too_old = hrt_absolute_time() - last_fusion_timestamp >
				    2 * min_delta_t * 1000;

	return measurement_valid && fusion_old_enough && (sensor_data_changed || fusion_too_old);
	// return measurement_valid;
}

void TargetEstimator::measurement_update(follow_target_s follow_target)
{
	_fusion_count++;
	// Decompose follow_target message into the individual measurements for position and velocity
	const Vector3f vel_measured{follow_target.vx, follow_target.vy, follow_target.vz};
	Vector3f pos_measured{NAN, NAN, -(follow_target.alt - _vehicle_local_position.ref_alt)};
	_reference_position.initReference(_vehicle_local_position.ref_lat, _vehicle_local_position.ref_lon);
	_reference_position.project(follow_target.lat, follow_target.lon, pos_measured(0), pos_measured(1));

	// Initialize filter if necessary
	if (_last_follow_target_timestamp == 0) {
		_filter_states.pos_ned_est = pos_measured;

		if (PX4_ISFINITE(vel_measured(0)) && PX4_ISFINITE(vel_measured(1)) && PX4_ISFINITE(vel_measured(2))) {
			_filter_states.vel_ned_est = vel_measured;

		} else {
			_filter_states.vel_ned_est.setAll(0.0f);
		}

		_pos_measurement_old = pos_measured;
		_vel_measurement_old = vel_measured;
		_filter_states.acc_ned_est.setZero();
	}

	_last_follow_target_timestamp = follow_target.timestamp;

	// Fuse position measurement
	//
	// Filter duplicate GPS POS and VEL messages
	// QGC sends the same GPS coordinates multiple times per second, even though the phone's GPS
	// typically only updates at 1 Hz

	if (measurement_can_be_fused(pos_measured, _pos_measurement_old, _last_position_fusion_timestamp,
				     MINIMUM_TIME_BETWEEN_POS_FUSIONS_MS)) {
		// Update with only position measurement

		const float dt_update_pos = math::constrain((follow_target.timestamp - _last_position_fusion_timestamp) * 1e-6f, 1e-3f,
					    20.0f);  // seconds
		_last_position_fusion_timestamp = follow_target.timestamp;

		const Vector3f pos_innovation = pos_measured - _filter_states.pos_ned_est;

		// Position, velocity and acceleration update
		_filter_states.pos_ned_est += _filter_gains.G_p * pos_innovation;
		_filter_states.vel_ned_est += _filter_gains.H_p / (dt_update_pos) * pos_innovation;
		_filter_states.acc_ned_est += 2.0f * _filter_gains.K_p /
					      (dt_update_pos * dt_update_pos) * pos_innovation;

		_pos_measurement_old = pos_measured;
	}

	// Fuse velocity measurement
	//
	// Use target's velocity data for update only if
	// - the last velocity fusion is a while ago to prevent repeated measurements to cause a quick convergence
	// - the target is considered to be moving. Otherwise it's enough to only update the position
	// - the GPS velocity measurement from the target is not stale
	// Additionally also wait with first velocity fusion until at least one position fusion has been done (states become finite)
	if (measurement_can_be_fused(vel_measured, _vel_measurement_old, _last_velocity_fusion_timestamp,
				     MINIMUM_TIME_BETWEEN_VEL_FUSIONS_MS)) {
		// Update with only velocity measurement

		const float dt_update_vel = math::constrain((follow_target.timestamp - _last_velocity_fusion_timestamp) * 1e-6f, 1e-3f,
					    20.0f); // seconds
		_last_velocity_fusion_timestamp = follow_target.timestamp;

		const Vector3f vel_innovation = vel_measured - _filter_states.vel_ned_est;

		// Velocity and acceleration update
		_filter_states.vel_ned_est += _filter_gains.G_v * vel_innovation;
		_filter_states.acc_ned_est += _filter_gains.H_v / (dt_update_vel) * vel_innovation;

		_vel_measurement_old = vel_measured;
	}

	_filter_states.saturate_acceleration(ACCELERATION_SATURATION);

}

void TargetEstimator::prediction_update(float deltatime)
{
	_prediction_count++;
	// Temporary copy to not mix old and new values during the update calculations
	const Vector3f vel_ned_est_prev = _filter_states.vel_ned_est;
	const Vector3f acc_ned_est_prev = _filter_states.acc_ned_est;

	if (PX4_ISFINITE(vel_ned_est_prev(0)) && PX4_ISFINITE(vel_ned_est_prev(1)) && PX4_ISFINITE(vel_ned_est_prev(2))) {
		_filter_states.pos_ned_est += deltatime * vel_ned_est_prev + 0.5f * acc_ned_est_prev * deltatime * deltatime;
	}

	if (PX4_ISFINITE(acc_ned_est_prev(0)) && PX4_ISFINITE(acc_ned_est_prev(1)) && PX4_ISFINITE(acc_ned_est_prev(2))) {
		_filter_states.vel_ned_est += deltatime * acc_ned_est_prev;
	}
}

Vector3<double> TargetEstimator::get_lat_lon_alt_est() const
{
	Vector3<double> lat_lon_alt{(double)NAN, (double)NAN, (double)NAN};

	if (PX4_ISFINITE(_filter_states.pos_ned_est(0)) && PX4_ISFINITE(_filter_states.pos_ned_est(0))) {
		_reference_position.reproject(_filter_states.pos_ned_est(0), _filter_states.pos_ned_est(1), lat_lon_alt(0),
					      lat_lon_alt(1));
		lat_lon_alt(2) = -(double)_filter_states.pos_ned_est(2) + (double)_vehicle_local_position.ref_alt;
	}

	return lat_lon_alt;
}

bool TargetEstimator::is_stale(const float timeout_duration_ms) const
{
	const bool measurements_stale = (hrt_absolute_time() - _last_follow_target_timestamp) / 1000.0f >=
					timeout_duration_ms;
	return measurements_stale;
}

void TargetEstimator::reset()
{
	_last_filter_reset_timestamp = hrt_absolute_time();  // debug only
	_last_position_fusion_timestamp = _last_velocity_fusion_timestamp = 0;
	_last_follow_target_timestamp = 0;
	_filter_states.pos_ned_est.setAll(NAN);
	_filter_states.vel_ned_est.setAll(NAN);
	_filter_states.acc_ned_est.setAll(NAN);
	_pos_measurement_old.setAll(NAN);
	_vel_measurement_old.setAll(NAN);
}
