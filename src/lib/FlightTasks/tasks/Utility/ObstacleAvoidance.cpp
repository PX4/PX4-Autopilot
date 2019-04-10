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

const vehicle_trajectory_waypoint_s empty_trajectory_waypoint = {0, 0, {0, 0, 0, 0, 0, 0, 0},
	{	{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, {0, 0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, {0, 0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, {0, 0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, {0, 0, 0}},
		{0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN, false, {0, 0, 0}}
	}
};

ObstacleAvoidance::ObstacleAvoidance(ModuleParams *parent) :
	ModuleParams(parent)
{
	_desired_waypoint = empty_trajectory_waypoint;
}

ObstacleAvoidance::~ObstacleAvoidance()
{
	//unadvertise publishers
	if (_pub_traj_wp_avoidance_desired != nullptr) {
		orb_unadvertise(_pub_traj_wp_avoidance_desired);
	}

	if (_pub_pos_control_status != nullptr) {
		orb_unadvertise(_pub_pos_control_status);
	}

}

bool ObstacleAvoidance::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!subscription_array.get(ORB_ID(vehicle_trajectory_waypoint), _sub_vehicle_trajectory_waypoint)) {
		return false;
	}

	if (!subscription_array.get(ORB_ID(vehicle_status), _sub_vehicle_status)) {
		return false;
	}

	return true;
}

void ObstacleAvoidance::injectAvoidanceSetpoints(Vector3f &pos_sp, Vector3f &vel_sp, float &yaw_sp,
		float &yaw_speed_sp)
{

	if (!COM_OBS_AVOID.get() || _sub_vehicle_status->get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) {
		// if avoidance disabled or in failsafe nav_state LOITER, don't inject setpoints from avoidance system
		return;
	}

	const bool avoidance_data_timeout = hrt_elapsed_time((hrt_abstime *)&_sub_vehicle_trajectory_waypoint->get().timestamp)
					    > TRAJECTORY_STREAM_TIMEOUT_US;
	const bool avoidance_point_valid =
		_sub_vehicle_trajectory_waypoint->get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid == true;

	if (avoidance_data_timeout || !avoidance_point_valid) {
		PX4_WARN("Obstacle Avoidance system failed, loitering");
		_publishVehicleCmdDoLoiter();
		return;
	}

	pos_sp = _sub_vehicle_trajectory_waypoint->get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].position;
	vel_sp = _sub_vehicle_trajectory_waypoint->get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity;
	yaw_sp =  _sub_vehicle_trajectory_waypoint->get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw;
	yaw_speed_sp = _sub_vehicle_trajectory_waypoint->get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed;
}

void ObstacleAvoidance::updateAvoidanceDesiredWaypoints(const Vector3f &curr_wp, const float curr_yaw,
		const float curr_yawspeed,
		const Vector3f &next_wp, const float next_yaw, const float next_yawspeed)
{
	_desired_waypoint.timestamp = hrt_absolute_time();
	_desired_waypoint.type = vehicle_trajectory_waypoint_s::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	_curr_wp = curr_wp;

	curr_wp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].position);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].velocity);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].acceleration);

	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw = curr_yaw;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].yaw_speed = curr_yawspeed;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_1].point_valid = true;

	next_wp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].position);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].velocity);
	Vector3f(NAN, NAN, NAN).copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].acceleration);

	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw = next_yaw;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].yaw_speed = next_yawspeed;
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_2].point_valid = true;
}

void ObstacleAvoidance::updateAvoidanceDesiredSetpoints(const Vector3f &pos_sp, const Vector3f &vel_sp)
{
	pos_sp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position);
	vel_sp.copyTo(_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity);
	_desired_waypoint.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid = true;

	_publishAvoidanceDesiredWaypoint();

	_desired_waypoint = empty_trajectory_waypoint;
}

void ObstacleAvoidance::checkAvoidanceProgress(const Vector3f &pos, const Vector3f &prev_wp,
		float target_acceptance_radius, const Vector2f &closest_pt)
{
	position_controller_status_s pos_control_status = {};
	pos_control_status.timestamp = hrt_absolute_time();

	// vector from previous triplet to current target
	Vector2f prev_to_target = Vector2f(_curr_wp - prev_wp);
	// vector from previous triplet to the vehicle projected position on the line previous-target triplet
	Vector2f prev_to_closest_pt = closest_pt - Vector2f(prev_wp);
	// fraction of the previous-tagerget line that has been flown
	const float prev_curr_travelled = prev_to_closest_pt.length() / prev_to_target.length();

	Vector2f pos_to_target = Vector2f(_curr_wp - pos);

	if (prev_curr_travelled > 1.0f) {
		// if the vehicle projected position on the line previous-target is past the target waypoint,
		// increase the target acceptance radius such that navigator will update the triplets
		pos_control_status.acceptance_radius = pos_to_target.length() + 0.5f;
	}

	const float pos_to_target_z = fabsf(_curr_wp(2) - pos(2));

	if (pos_to_target.length() < target_acceptance_radius && pos_to_target_z > NAV_MC_ALT_RAD.get()) {
		// vehicle above or below the target waypoint
		pos_control_status.altitude_acceptance = pos_to_target_z + 0.5f;
	}

	// do not check for waypoints yaw acceptance in navigator
	pos_control_status.yaw_acceptance = NAN;

	if (_pub_pos_control_status == nullptr) {
		_pub_pos_control_status = orb_advertise(ORB_ID(position_controller_status), &pos_control_status);

	} else {
		orb_publish(ORB_ID(position_controller_status), _pub_pos_control_status, &pos_control_status);

	}

}

void
ObstacleAvoidance::_publishAvoidanceDesiredWaypoint()
{
	// publish desired waypoint
	if (_pub_traj_wp_avoidance_desired != nullptr) {
		orb_publish(ORB_ID(vehicle_trajectory_waypoint_desired), _pub_traj_wp_avoidance_desired, &_desired_waypoint);

	} else {
		_pub_traj_wp_avoidance_desired = orb_advertise(ORB_ID(vehicle_trajectory_waypoint_desired),
						 &_desired_waypoint);
	}
}

void ObstacleAvoidance::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
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
	if (_pub_vehicle_command == nullptr) {
		_pub_vehicle_command = orb_advertise_queue(ORB_ID(vehicle_command), &command,
				       vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), _pub_vehicle_command, &command);
	}
}
