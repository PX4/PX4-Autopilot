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

#include "RoverControl.hpp"
using namespace matrix;
namespace RoverControl
{
float throttleControl(SlewRate<float> &motor_setpoint, const float throttle_setpoint,
		      const float current_motor_setpoint, const float max_accel, const float max_decel, const float max_thr_spd,
		      const float dt)
{
	// Sanitize inputs
	if (!PX4_ISFINITE(throttle_setpoint) || !PX4_ISFINITE(current_motor_setpoint) || !PX4_ISFINITE(dt)) {
		return NAN;
	}

	const bool accelerating = fabsf(throttle_setpoint) > fabsf(current_motor_setpoint);

	if (accelerating && max_accel > FLT_EPSILON && max_thr_spd > FLT_EPSILON) { // Acceleration slew rate
		motor_setpoint.setSlewRate(max_accel / max_thr_spd);
		motor_setpoint.update(throttle_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(motor_setpoint.getState() - current_motor_setpoint) > fabsf(throttle_setpoint -
				current_motor_setpoint)) {
			motor_setpoint.setForcedValue(throttle_setpoint);
		}

	} else if (!accelerating && max_decel > FLT_EPSILON && max_thr_spd > FLT_EPSILON) { // Deceleration slew rate
		motor_setpoint.setSlewRate(max_decel / max_thr_spd);
		motor_setpoint.update(throttle_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(motor_setpoint.getState() - current_motor_setpoint) > fabsf(throttle_setpoint -
				current_motor_setpoint)) {
			motor_setpoint.setForcedValue(throttle_setpoint);
		}

	} else { // Fallthrough if slew rate parameters are not configured
		motor_setpoint.setForcedValue(throttle_setpoint);
	}

	return motor_setpoint.getState();
}

float attitudeControl(SlewRateYaw<float> &adjusted_yaw_setpoint, PID &pid_yaw,
		      const float yaw_slew_rate, float vehicle_yaw, float yaw_setpoint, const float dt)
{
	// Sanitize inputs
	if (!PX4_ISFINITE(yaw_setpoint) || !PX4_ISFINITE(vehicle_yaw) || !PX4_ISFINITE(dt)) {
		return NAN;
	}

	yaw_setpoint = wrap_pi(yaw_setpoint);
	vehicle_yaw = wrap_pi(vehicle_yaw);

	if (yaw_slew_rate > FLT_EPSILON) { // Apply slew rate if configured
		adjusted_yaw_setpoint.setSlewRate(yaw_slew_rate);
		adjusted_yaw_setpoint.update(yaw_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(wrap_pi(adjusted_yaw_setpoint.getState() - vehicle_yaw)) > fabsf(wrap_pi(yaw_setpoint - vehicle_yaw))) {
			adjusted_yaw_setpoint.setForcedValue(yaw_setpoint);
		}

	} else {
		adjusted_yaw_setpoint.setForcedValue(yaw_setpoint);
	}

	// Feedback control
	pid_yaw.setSetpoint(
		wrap_pi(adjusted_yaw_setpoint.getState() - vehicle_yaw));  // Error as setpoint to take care of wrapping
	return pid_yaw.update(0.f, dt);

}

float speedControl(SlewRate<float> &speed_with_rate_limit, PID &pid_speed, const float speed_setpoint,
		   const float vehicle_speed, const float max_accel, const float max_decel, const float max_thr_speed, const float dt)
{
	// Sanitize inputs
	if (!PX4_ISFINITE(speed_setpoint) || !PX4_ISFINITE(vehicle_speed) || !PX4_ISFINITE(dt)) {
		return NAN;
	}

	// Apply acceleration and deceleration limit
	if (fabsf(speed_setpoint) >= fabsf(vehicle_speed) && max_accel > FLT_EPSILON) {
		speed_with_rate_limit.setSlewRate(max_accel);
		speed_with_rate_limit.update(speed_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(speed_with_rate_limit.getState() - vehicle_speed) > fabsf(
			    speed_setpoint - vehicle_speed)) {
			speed_with_rate_limit.setForcedValue(speed_setpoint);
		}

	} else if (fabsf(speed_setpoint) < fabsf(vehicle_speed) && max_decel > FLT_EPSILON) {
		speed_with_rate_limit.setSlewRate(max_decel);
		speed_with_rate_limit.update(speed_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(speed_with_rate_limit.getState() - vehicle_speed) > fabsf(
			    speed_setpoint - vehicle_speed)) {
			speed_with_rate_limit.setForcedValue(speed_setpoint);
		}

	} else { // Fallthrough if slew rate is not configured
		speed_with_rate_limit.setForcedValue(speed_setpoint);
	}

	// Calculate normalized forward speed setpoint
	float forward_speed_normalized{0.f};

	// Feedforward
	if (max_thr_speed > FLT_EPSILON) {
		forward_speed_normalized = math::interpolate<float>(speed_with_rate_limit.getState(),
					   -max_thr_speed, max_thr_speed,
					   -1.f, 1.f);
	}

	// Feedback control
	if (fabsf(speed_with_rate_limit.getState()) > FLT_EPSILON) {
		pid_speed.setSetpoint(speed_with_rate_limit.getState());
		forward_speed_normalized += pid_speed.update(vehicle_speed, dt);

	} else {
		pid_speed.resetIntegral();
	}


	return math::constrain(forward_speed_normalized, -1.f, 1.f);
}

float rateControl(SlewRate<float> &adjusted_yaw_rate_setpoint, PID &pid_yaw_rate, const float yaw_rate_setpoint,
		  const float vehicle_yaw_rate, const float max_thr_yaw_r, const float max_yaw_accel, const float max_yaw_decel,
		  const float wheel_track, const float dt)
{
	// Apply acceleration and deceleration limit
	if (fabsf(yaw_rate_setpoint) >= fabsf(vehicle_yaw_rate) && max_yaw_accel > FLT_EPSILON) {
		adjusted_yaw_rate_setpoint.setSlewRate(max_yaw_accel);
		adjusted_yaw_rate_setpoint.update(yaw_rate_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(adjusted_yaw_rate_setpoint.getState() - vehicle_yaw_rate) > fabsf(
			    yaw_rate_setpoint - vehicle_yaw_rate)) {
			adjusted_yaw_rate_setpoint.setForcedValue(yaw_rate_setpoint);
		}


	} else if (fabsf(yaw_rate_setpoint) < fabsf(vehicle_yaw_rate) && max_yaw_decel > FLT_EPSILON) {
		adjusted_yaw_rate_setpoint.setSlewRate(max_yaw_decel);
		adjusted_yaw_rate_setpoint.update(yaw_rate_setpoint, dt);

		// Reinitialize slew rate if current value is closer to setpoint than post slew rate value
		if (fabsf(adjusted_yaw_rate_setpoint.getState() - vehicle_yaw_rate) > fabsf(
			    yaw_rate_setpoint - vehicle_yaw_rate)) {
			adjusted_yaw_rate_setpoint.setForcedValue(yaw_rate_setpoint);
		}


	} else { // Fallthrough if slew rate is not configured
		adjusted_yaw_rate_setpoint.setForcedValue(yaw_rate_setpoint);
	}

	// Transform yaw rate into speed difference
	float speed_diff_normalized{0.f};

	if (wheel_track > FLT_EPSILON && max_thr_yaw_r > FLT_EPSILON) { // Feedforward
		const float speed_diff = adjusted_yaw_rate_setpoint.getState() * wheel_track /
					 2.f;
		speed_diff_normalized = math::interpolate<float>(speed_diff, -max_thr_yaw_r,
					max_thr_yaw_r, -1.f, 1.f);
	}

	// Feedback control
	if (fabsf(adjusted_yaw_rate_setpoint.getState()) > FLT_EPSILON) {
		pid_yaw_rate.setSetpoint(adjusted_yaw_rate_setpoint.getState());
		speed_diff_normalized += pid_yaw_rate.update(vehicle_yaw_rate, dt);

	} else {
		pid_yaw_rate.resetIntegral();
	}


	return math::constrain(speed_diff_normalized, -1.f, 1.f);
}

void globalToLocalSetpointTriplet(Vector2f &curr_wp_ned, Vector2f &prev_wp_ned, Vector2f &next_wp_ned,
				  position_setpoint_triplet_s position_setpoint_triplet, Vector2f &curr_pos_ned, MapProjection &global_ned_proj_ref)
{
	if (position_setpoint_triplet.current.valid && PX4_ISFINITE(position_setpoint_triplet.current.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.current.lon)) {
		curr_wp_ned = global_ned_proj_ref.project(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	} else {
		curr_wp_ned = curr_pos_ned.isAllFinite() ? curr_pos_ned : Vector2f(NAN, NAN); // Fallback if current waypoint is invalid
	}

	if (position_setpoint_triplet.previous.valid && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.previous.lon)) {
		prev_wp_ned = global_ned_proj_ref.project(position_setpoint_triplet.previous.lat,
				position_setpoint_triplet.previous.lon);

	} else {
		prev_wp_ned = curr_pos_ned.isAllFinite() ? curr_pos_ned : Vector2f(NAN,
				NAN); // Fallback if previous waypoint is invalid
	}

	if (position_setpoint_triplet.next.valid && PX4_ISFINITE(position_setpoint_triplet.next.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.next.lon)) {
		next_wp_ned = global_ned_proj_ref.project(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	} else {
		next_wp_ned = Vector2f(NAN, NAN);
	}
}

float calcWaypointTransitionAngle(Vector2f &prev_wp_ned, Vector2f &curr_wp_ned, Vector2f &next_wp_ned)
{
	// Sanitize inputs
	if (!prev_wp_ned.isAllFinite() || !curr_wp_ned.isAllFinite() || !next_wp_ned.isAllFinite()) {
		return NAN;
	}

	const Vector2f curr_to_next_wp_ned = next_wp_ned - curr_wp_ned;
	const Vector2f curr_to_prev_wp_ned = prev_wp_ned - curr_wp_ned;

	// Waypoint overlap
	if (curr_to_next_wp_ned.norm() < FLT_EPSILON || curr_to_prev_wp_ned.norm() < FLT_EPSILON) {
		return NAN;
	}

	float cosin = curr_to_prev_wp_ned.unit_or_zero() * curr_to_next_wp_ned.unit_or_zero();
	cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
	return acosf(cosin);
}

} // RoverControl
