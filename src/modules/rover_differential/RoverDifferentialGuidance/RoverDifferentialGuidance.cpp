/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverDifferentialGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

RoverDifferentialGuidance::RoverDifferentialGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_max_forward_speed = _param_rd_miss_spd_def.get();
	_rover_differential_guidance_status_pub.advertise();
}

void RoverDifferentialGuidance::updateParams()
{
	ModuleParams::updateParams();
}

void RoverDifferentialGuidance::computeGuidance(const float vehicle_yaw, const float forward_speed, const int nav_state)
{
	updateSubscriptions();

	// Catch return to launch
	const float distance_to_curr_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
					  _curr_wp(0), _curr_wp(1));

	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		_mission_finished =  distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	// State machine
	float desired_yaw = _pure_pursuit.calcDesiredHeading(_curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
			    math::max(forward_speed, 0.f));
	const float heading_error = matrix::wrap_pi(desired_yaw - vehicle_yaw);

	if (!_mission_finished && distance_to_curr_wp > _param_nav_acc_rad.get()) {
		if (_currentState == GuidanceState::STOPPED) {
			_currentState = GuidanceState::DRIVING;
		}

		if (_currentState == GuidanceState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
			_currentState = GuidanceState::SPOT_TURNING;

		} else if (_currentState == GuidanceState::SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
			_currentState = GuidanceState::DRIVING;
		}

	} else { // Mission finished or delay command
		_currentState = GuidanceState::STOPPED;
	}

	// Guidance logic
	float desired_forward_speed{0.f};

	switch (_currentState) {
	case GuidanceState::DRIVING: {
			// Calculate desired forward speed
			desired_forward_speed = _max_forward_speed;

			if (_waypoint_transition_angle < M_PI_F - _param_rd_trans_drv_trn.get()) {
				if (_param_rd_max_jerk.get() > FLT_EPSILON && _param_rd_max_decel.get() > FLT_EPSILON) {
					desired_forward_speed = math::trajectory::computeMaxSpeedFromDistance(_param_rd_max_jerk.get(),
								_param_rd_max_decel.get(), distance_to_curr_wp, 0.0f);
					desired_forward_speed = math::constrain(desired_forward_speed, -_max_forward_speed, _max_forward_speed);
				}
			}

		} break;

	case GuidanceState::SPOT_TURNING:
		if (forward_speed > TURN_MAX_VELOCITY) {
			desired_yaw = vehicle_yaw; // Wait for the rover to stop

		}

		break;

	case GuidanceState::STOPPED:
	default:
		desired_yaw = vehicle_yaw;
		break;

	}

	// Publish differential guidance status (logging)
	hrt_abstime timestamp = hrt_absolute_time();
	rover_differential_guidance_status_s rover_differential_guidance_status{};
	rover_differential_guidance_status.timestamp = timestamp;
	rover_differential_guidance_status.lookahead_distance = _pure_pursuit.getLookaheadDistance();
	rover_differential_guidance_status.heading_error_deg = M_RAD_TO_DEG_F * heading_error;
	rover_differential_guidance_status.state_machine = (uint8_t) _currentState;
	_rover_differential_guidance_status_pub.publish(rover_differential_guidance_status);

	// Publish setpoints
	rover_differential_setpoint_s rover_differential_setpoint{};
	rover_differential_setpoint.timestamp = timestamp;
	rover_differential_setpoint.forward_speed_setpoint = desired_forward_speed;
	rover_differential_setpoint.forward_speed_setpoint_normalized = NAN;
	rover_differential_setpoint.yaw_rate_setpoint = NAN;
	rover_differential_setpoint.speed_diff_setpoint_normalized = NAN;
	rover_differential_setpoint.yaw_setpoint = desired_yaw;
	_rover_differential_setpoint_pub.publish(rover_differential_setpoint);
}

void RoverDifferentialGuidance::updateSubscriptions()
{
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(local_position.ref_lat, local_position.ref_lon, local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(local_position.x, local_position.y);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypoints();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}
}

void RoverDifferentialGuidance::updateWaypoints()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	if (position_setpoint_triplet.current.valid && PX4_ISFINITE(position_setpoint_triplet.current.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.current.lon)) {
		_curr_wp = Vector2d(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	} else {
		_curr_wp = Vector2d(0, 0);
	}

	if (position_setpoint_triplet.previous.valid && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.previous.lon)) {
		_prev_wp = Vector2d(position_setpoint_triplet.previous.lat, position_setpoint_triplet.previous.lon);

	} else {
		_prev_wp = _curr_pos;
	}

	if (position_setpoint_triplet.next.valid && PX4_ISFINITE(position_setpoint_triplet.next.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.next.lon)) {
		_next_wp = Vector2d(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	} else {
		_next_wp = _home_position;
	}

	// NED waypoint coordinates
	_curr_wp_ned = _global_ned_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_ned = _global_ned_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_ned = _global_ned_proj_ref.project(_next_wp(0), _next_wp(1));

	// Distances
	const Vector2f curr_to_next_wp_ned = _next_wp_ned - _curr_wp_ned;
	const Vector2f curr_to_prev_wp_ned = _prev_wp_ned - _curr_wp_ned;
	float cosin = curr_to_prev_wp_ned.unit_or_zero() * curr_to_next_wp_ned.unit_or_zero();
	cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
	_waypoint_transition_angle = acosf(cosin);

	// Waypoint cruising speed
	if (position_setpoint_triplet.current.cruising_speed > 0.f) {
		_max_forward_speed = math::constrain(position_setpoint_triplet.current.cruising_speed, 0.f, _param_rd_max_speed.get());

	} else {
		_max_forward_speed = _param_rd_miss_spd_def.get();
	}
}
