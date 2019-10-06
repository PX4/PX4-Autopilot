/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ObstacleAvoidance.cpp
 */

#include "ObstacleAvoidance.hpp"

using namespace matrix;
using namespace time_literals;

/** Timeout in us for trajectory data to get considered invalid */
static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
/** If Flighttask fails, keep 0.5 seconds the current setpoint before going into failsafe land */
static constexpr uint64_t TIME_BEFORE_FAILSAFE = 500_ms;
static constexpr uint64_t Z_PROGRESS_TIMEOUT_US = 2_s;

ObstacleAvoidance::ObstacleAvoidance(ModuleParams *parent) :
	ModuleParams(parent)
{
	_desired_waypoint = empty_trajectory_waypoint;
	_failsafe_position.setNaN();
	_avoidance_point_not_valid_hysteresis.set_hysteresis_time_from(false, TIME_BEFORE_FAILSAFE);
	_no_progress_z_hysteresis.set_hysteresis_time_from(false, Z_PROGRESS_TIMEOUT_US);

}

void ObstacleAvoidance::injectAvoidanceSetpoints(Vector3f &pos_sp, Vector3f &vel_sp, float &yaw_sp,
		float &yaw_speed_sp)
{
	_sub_vehicle_status.update();
	_sub_vehicle_trajectory_waypoint.update();

	if (_sub_vehicle_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
		// if in failsafe nav_state LOITER, don't inject setpoints from avoidance system
		return;
	}

	const bool avoidance_data_timeout = hrt_elapsed_time((hrt_abstime *)&_sub_vehicle_trajectory_waypoint.get().timestamp)
					    > TRAJECTORY_STREAM_TIMEOUT_US;
	const bool avoidance_point_valid =
		_sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid == true;

	_avoidance_point_not_valid_hysteresis.set_state_and_update(!avoidance_point_valid, hrt_absolute_time());

	if (avoidance_data_timeout || _avoidance_point_not_valid_hysteresis.get_state()) {
		PX4_WARN("Obstacle Avoidance system failed, loitering");
		_publishVehicleCmdDoLoiter();

		if (!PX4_ISFINITE(_failsafe_position(0)) || !PX4_ISFINITE(_failsafe_position(1))
		    || !PX4_ISFINITE(_failsafe_position(2))) {
			// save vehicle position when entering failsafe
			_failsafe_position = _position;
		}

		pos_sp = _failsafe_position;
		vel_sp.setNaN();
		yaw_sp = NAN;
		yaw_speed_sp = NAN;
		return;

	} else {
		_failsafe_position.setNaN();
	}

	if (avoidance_point_valid) {
		pos_sp = Vector3f(_sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].position);
		vel_sp = Vector3f(_sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity);

		if (!_ext_yaw_active) {
			// inject yaw setpoints only if weathervane isn't active
			yaw_sp =  _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw;
			yaw_speed_sp = _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed;
		}
	}

}

void ObstacleAvoidance::updateAvoidanceDesiredWaypoints(const Vector3f &curr_wp, const float curr_yaw,
		const float curr_yawspeed, const Vector3f &next_wp, const float next_yaw, const float next_yawspeed,
		const bool ext_yaw_active, const int wp_type)
{
	_desired_waypoint.timestamp = hrt_absolute_time();
	_desired_waypoint.type = vehicle_trajectory_waypoint_s::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	_curr_wp = curr_wp;
	_ext_yaw_active = ext_yaw_active;

	curr_wp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].position);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].velocity);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].acceleration);

	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw = curr_yaw;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw_speed = curr_yawspeed;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].point_valid = true;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].type = wp_type;

	next_wp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].position);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].velocity);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].acceleration);

	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw = next_yaw;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw_speed = next_yawspeed;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].point_valid = true;
}

void ObstacleAvoidance::updateAvoidanceDesiredSetpoints(const Vector3f &pos_sp, const Vector3f &vel_sp, const int type)
{
	pos_sp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position);
	vel_sp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity);
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].type = type;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid = true;

	_pub_traj_wp_avoidance_desired.publish(_desired_waypoint);

	_desired_waypoint = empty_trajectory_waypoint;
}

void ObstacleAvoidance::checkAvoidanceProgress(const Vector3f &pos, const Vector3f &prev_wp,
		float target_acceptance_radius, const Vector2f &closest_pt)
{
	_position = pos;
	position_controller_status_s pos_control_status = {};
	pos_control_status.timestamp = hrt_absolute_time();

	// vector from previous triplet to current target
	Vector2f prev_to_target = Vector2f(_curr_wp - prev_wp);
	// vector from previous triplet to the vehicle projected position on the line previous-target triplet
	Vector2f prev_to_closest_pt = closest_pt - Vector2f(prev_wp);
	// fraction of the previous-tagerget line that has been flown
	const float prev_curr_travelled = prev_to_closest_pt.length() / prev_to_target.length();

	Vector2f pos_to_target = Vector2f(_curr_wp - _position);

	if (prev_curr_travelled > 1.0f) {
		// if the vehicle projected position on the line previous-target is past the target waypoint,
		// increase the target acceptance radius such that navigator will update the triplets
		pos_control_status.acceptance_radius = pos_to_target.length() + 0.5f;
	}

	const float pos_to_target_z = fabsf(_curr_wp(2) - _position(2));

	bool no_progress_z = (pos_to_target_z > _prev_pos_to_target_z);
	_no_progress_z_hysteresis.set_state_and_update(no_progress_z, hrt_absolute_time());

	if (pos_to_target.length() < target_acceptance_radius && pos_to_target_z > _param_nav_mc_alt_rad.get()
	    && _no_progress_z_hysteresis.get_state()) {
		// vehicle above or below the target waypoint
		pos_control_status.altitude_acceptance = pos_to_target_z + 0.5f;
	}

	_prev_pos_to_target_z = pos_to_target_z;

	// do not check for waypoints yaw acceptance in navigator
	pos_control_status.yaw_acceptance = NAN;

	_pub_pos_control_status.publish(pos_control_status);
}

void ObstacleAvoidance::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

	// publish the vehicle command
	_pub_vehicle_command.publish(command);
}
