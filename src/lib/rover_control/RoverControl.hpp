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
 * @file RoverControl.hpp
 *
 * Functions that are shared among the different rover modules.
 * Also includes the parameters that are shared among the rover modules.
 */

#pragma once
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/pid/PID.hpp>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/position_setpoint_triplet.h>
#include <lib/geo/geo.h>

using namespace matrix;
namespace RoverControl
{
/**
 * Applies acceleration/deceleration slew rate to a throttle setpoint.
 * @param motor_setpoint Throttle setpoint with applied slew rate [-1, 1] (Updated by this function)
 * @param throttle_setpoint Throttle setpoint pre slew rate [-1, 1]
 * @param current_motor_setpoint Currently applied motor input [-1, 1]
 * @param max_accel Maximum allowed acceleration [m/s^2]
 * @param max_decel Maximum allowed deceleration [m/s^2]
 * @param max_thr_spd Speed the rover drives at maximum throttle [m/s]
 * @param dt Time since last update [s]
 * @return Motor Setpoint [-1, 1]
 */
float throttleControl(SlewRate<float> &motor_setpoint, float throttle_setpoint, float current_motor_setpoint,
		      float max_accel, float max_decel, float max_thr_spd, float dt);

/**
 * Applies yaw rate slew rate to a yaw setpoint and calculates the necessary yaw rate setpoint
 * using a PID controller.
 * @param adjusted_yaw_setpoint Yaw setpoint with applied slew rate [-1, 1] (Updated by this function)
 * @param pid_yaw Yaw PID (Updated by this function)
 * @param yaw_slew_rate Yaw slew rate [rad/s]
 * @param vehicle_yaw Measured vehicle yaw [rad]
 * @param yaw_setpoint Yaw setpoint [rad]
 * @param dt Time since last update [s]
 * @return Yaw rate setpoint [rad/s]
 */
float attitudeControl(SlewRateYaw<float> &adjusted_yaw_setpoint, PID &pid_yaw, float yaw_slew_rate,
		      float vehicle_yaw, float yaw_setpoint, float dt);
/**
 * Applies acceleration/deceleration slew rate to a speed setpoint and calculates the necessary throttle setpoint
 * using a feed forward term and PID controller.
 * @param speed_with_rate_limit Speed setpoint with applied slew rate [-1, 1] (Updated by this function)
 * @param pid_speed Speed PID (Updated by this function)
 * @param speed_setpoint Speed setpoint [m/s]
 * @param vehicle_speed Measured vehicle speed [m/s]
 * @param max_accel Maximum allowed acceleration [m/s^2]
 * @param max_decel Maximum allowed deceleration [m/s^2]
 * @param max_thr_speed Speed at maximum throttle [m/s]
 * @param dt Time since last update [s]
 * @return Throttle setpoint [-1, 1]
 */
float speedControl(SlewRate<float> &speed_with_rate_limit, PID &pid_speed, float speed_setpoint,
		   float vehicle_speed, float max_accel, float max_decel, float max_thr_speed, float dt);

/**
 * Applies yaw acceleration slew rate to a yaw rate setpoint and calculates the necessary speed diff setpoint
 * using a feedforward term and/or a PID controller.
 * Note: This function is only for rovers that control the rate through a speed difference between the left/right wheels.
 * @param adjusted_yaw_rate_setpoint Yaw rate setpoint with applied slew rate [-1, 1] (Updated by this function).
 * @param pid_yaw_rate Yaw rate PID (Updated by this function).
 * @param yaw_rate_setpoint Yaw rate setpoint [rad/s].
 * @param vehicle_yaw_rate Measured vehicle yaw rate [rad/s].
 * @param max_thr_yaw_r Yaw rate turning left/right wheels at max speed in opposite directions [m/s].
 * @param max_yaw_accel Maximum allowed yaw acceleration [rad/s^2].
 * @param max_yaw_decel Maximum allowed yaw deceleration [rad/s^2].
 * @param wheel_track Distance from the center of the right wheel to the center of the left wheel [m].
 * @param dt Time since last update [s].
 * @return Normalized speed difference setpoint [-1, 1].
 */
float rateControl(SlewRate<float> &adjusted_yaw_rate_setpoint, PID &pid_yaw_rate,
		  float yaw_rate_setpoint, float vehicle_yaw_rate, float max_thr_yaw_r, float max_yaw_accel, float max_yaw_decel,
		  float wheel_track, float dt);

/**
 * Projects positionSetpointTriplet waypoints from global to ned frame.
 * @param curr_wp_ned Current waypoint in NED frame (Updated by this function)
 * @param prev_wp_ned Previous waypoint in NED frame (Updated by this function)
 * @param next_wp_ned Next waypoint in NED frame (Updated by this function)
 * @param position_setpoint_triplet Position Setpoint Triplet
 * @param curr_pos Current position of the rover in global frame
 * @param global_ned_proj_ref Global to ned projection
 */
void globalToLocalSetpointTriplet(Vector2f &curr_wp_ned, Vector2f &prev_wp_ned, Vector2f &next_wp_ned,
				  position_setpoint_triplet_s position_setpoint_triplet, Vector2f &curr_pos_ned,
				  MapProjection &global_ned_proj_ref);

/**
 * Calculate and return the angle between the prevWP-currWP and currWP-nextWP line segments [rad]
 * @param prev_wp_ned Previous waypoint in NED frame
 * @param curr_wp_ned Current waypoint in NED frame
 * @param next_wp_ned Next waypoint in NED frame
 * @return Waypoint transition angle [rad]
 */
float calcWaypointTransitionAngle(Vector2f &prev_wp_ned, Vector2f &curr_wp_ned, Vector2f &next_wp_ned);

}
