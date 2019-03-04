/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_module_params.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/FlightTasks/tasks/FlightTask/SubscriptionArray.hpp>
#include <commander/px4_custom_mode.h>
#include <matrix/matrix/math.hpp>
#include <px4_defines.h>

class ObstacleAvoidance : public ModuleParams
{
public:
	ObstacleAvoidance(ModuleParams *parent);
	~ObstacleAvoidance();

	bool initializeSubscriptions(SubscriptionArray &subscription_array);

	void prepareAvoidanceSetpoints(matrix::Vector3f &pos_sp, matrix::Vector3f &vel_sp, float &yaw_sp, float &yaw_speed_sp);
	void updateAvoidanceWaypoints(const matrix::Vector3f &curr_wp, const float curr_yaw, const float curr_yawspeed,
				      const matrix::Vector3f &next_wp, const float next_yaw, const float next_yawspeed);
	void updateAvoidanceSetpoints(const matrix::Vector3f &pos_sp, const matrix::Vector3f &vel_sp);
	void checkAvoidanceProgress(const matrix::Vector3f &pos, const matrix::Vector3f &prev_wp,
				    float target_acceptance_radius,
				    const matrix::Vector2f &closest_pt);

private:

	uORB::Subscription<vehicle_trajectory_waypoint_s> *_sub_vehicle_trajectory_waypoint{nullptr};
	uORB::Subscription<vehicle_status_s> *_sub_vehicle_status{nullptr};

  DEFINE_PARAMETERS(
    (ParamInt<px4::params::COM_OBS_AVOID>) COM_OBS_AVOID,             /**< obstacle avoidance enabled */
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>) NAV_MC_ALT_RAD
  );

	vehicle_trajectory_waypoint_s _desired_waypoint = {};
	orb_advert_t _traj_wp_avoidance_desired_pub{nullptr}; /**< trajectory waypoint desired publication */
	orb_advert_t _pub_pos_control_status{nullptr};
	orb_advert_t _pub_vehicle_command{nullptr};

	matrix::Vector3f _curr_wp = {};

	void _publish_avoidance_desired_waypoint();
	void _publish_vehicle_cmd_do_loiter();

};
