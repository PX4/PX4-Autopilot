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
 * @file ObstacleAvoidance.cpp
 */

#include "ObstacleAvoidance.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>


using namespace matrix;
using namespace time_literals;

/** Timeout in us for trajectory data to get considered invalid */
static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

ObstacleAvoidance::ObstacleAvoidance(ModuleParams *parent) :
	ModuleParams(parent)
{

}

void ObstacleAvoidance::prepareAvoidanceSetpoints(Vector3f &pos_sp, Vector3f &vel_sp, float &yaw_sp, float &yaw_speed_sp) {

	if (!COM_OBS_AVOID.get()) {
		return;
	}

	_sub_vehicle_trajectory_waypoint.update();
	const bool avoidance_data_timeout = hrt_elapsed_time((hrt_abstime *)&_sub_vehicle_trajectory_waypoint.get().timestamp) > TRAJECTORY_STREAM_TIMEOUT_US;
	const bool avoidance_point_valid = _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid == true;

	if (!avoidance_data_timeout && avoidance_point_valid) {
		pos_sp = _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].position;
		vel_sp = _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity;
		yaw_sp =  _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw;
		yaw_speed_sp = _sub_vehicle_trajectory_waypoint.get().waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed;
	}
}
