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
 * @file ObstacleAvoidance.hpp
 * This class is used to inject the setpoints of an obstacle avoidance system
 * into the FlightTasks
 *
 * @author Martina Rivizzigno
 */

#pragma once

#include <px4_defines.h>
#include <px4_module_params.h>
#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

#include <matrix/matrix/math.hpp>

#include <SubscriptionArray.hpp>

class ObstacleAvoidance : public ModuleParams
{
public:
	ObstacleAvoidance(ModuleParams *parent);
	~ObstacleAvoidance();

	bool initializeSubscriptions(SubscriptionArray &subscription_array);

	/**
	 * Inject setpoints from obstacle avoidance system into FlightTasks.
	 * @param pos_sp, position setpoint
	 * @param vel_sp, velocity setpoint
	 * @param yaw_sp, yaw setpoint
	 * @param yaw_speed_sp, yaw speed setpoint
	 */
	void injectAvoidanceSetpoints(matrix::Vector3f &pos_sp, matrix::Vector3f &vel_sp, float &yaw_sp, float &yaw_speed_sp);

	/**
	 * Updates the desired waypoints to send to the obstacle avoidance system. These messages don't have any direct impact on the flight.
	 * @param curr_wp, current position triplet
	 * @param curr_yaw, current yaw triplet
	 * @param curr_yawspeed, current yaw speed triplet
	 * @param next_wp, next position triplet
	 * @param next_yaw, next yaw triplet
	 * @param next_yawspeed, next yaw speed triplet
	 */
	void updateAvoidanceDesiredWaypoints(const matrix::Vector3f &curr_wp, const float curr_yaw, const float curr_yawspeed,
					     const matrix::Vector3f &next_wp, const float next_yaw, const float next_yawspeed);
	/**
	 * Updates the desired setpoints to send to the obstacle avoidance system.
	 * @param pos_sp, desired position setpoint computed by the active FlightTask
	 * @param vel_sp, desired velocity setpoint computed by the active FlightTask
	 */
	void updateAvoidanceDesiredSetpoints(const matrix::Vector3f &pos_sp, const matrix::Vector3f &vel_sp);

	/**
	 * Checks the vehicle progress between previous and current position waypoint of the triplet.
	 * @param pos, vehicle position
	 * @param prev_wp, previous position triplet
	 * @param target_acceptance_radius, current position triplet xy acceptance radius
	 * @param closest_pt, closest point to the vehicle on the line previous-current position triplet
	 */
	void checkAvoidanceProgress(const matrix::Vector3f &pos, const matrix::Vector3f &prev_wp,
				    float target_acceptance_radius, const matrix::Vector2f &closest_pt);

private:

	uORB::Subscription<vehicle_trajectory_waypoint_s> *_sub_vehicle_trajectory_waypoint{nullptr}; /**< vehicle trajectory waypoint subscription */
	uORB::Subscription<vehicle_status_s> *_sub_vehicle_status{nullptr}; /**< vehicle status subscription */

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::COM_OBS_AVOID>) COM_OBS_AVOID,             /**< obstacle avoidance enabled */
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>) NAV_MC_ALT_RAD          /**< Acceptance radius for multicopter altitude */
	);

	vehicle_trajectory_waypoint_s _desired_waypoint = {};  /**< desired vehicle trajectory waypoint to be sent to OA */
	orb_advert_t _pub_traj_wp_avoidance_desired{nullptr}; /**< trajectory waypoint desired publication */
	orb_advert_t _pub_pos_control_status{nullptr}; /**< position controller status publication */
	orb_advert_t _pub_vehicle_command{nullptr}; /**< vehicle command do publication */

	matrix::Vector3f _curr_wp = {}; /**< current position triplet */

	/**
	 * Publishes vehicle trajectory waypoint desired.
	 */
	void _publishAvoidanceDesiredWaypoint();

	/**
	 * Publishes vehicle command.
	 */
	void _publishVehicleCmdDoLoiter();

};
