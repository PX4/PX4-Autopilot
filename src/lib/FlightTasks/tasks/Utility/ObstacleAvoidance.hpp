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
#include <lib/FlightTasks/tasks/FlightTask/SubscriptionArray.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_defines.h>

class ObstacleAvoidance : public ModuleParams
{
public:
	ObstacleAvoidance(ModuleParams *parent);
	~ObstacleAvoidance() = default;

  void prepareAvoidanceSetpoints(matrix::Vector3f &pos_sp, matrix::Vector3f &vel_sp, float &yaw_sp, float &yaw_speed_sp);

private:

  uORB::Subscription<vehicle_trajectory_waypoint_s> _sub_vehicle_trajectory_waypoint{ORB_ID(vehicle_trajectory_waypoint)};

  DEFINE_PARAMETERS(
    (ParamInt<px4::params::COM_OBS_AVOID>) COM_OBS_AVOID             /**< obstacle avoidance enabled */
  );

};
