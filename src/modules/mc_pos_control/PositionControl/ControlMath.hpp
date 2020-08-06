/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.hpp
 *
 * Simple functions for vector manipulation that do not fit into matrix lib.
 * These functions are specific for controls.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/omni_attitude_status.h>

namespace ControlMath
{
/**
 * Converts thrust vector and yaw set-point to a desired attitude.
 * @param thr_sp desired 3D thrust vector
 * @param yaw_sp the desired yaw
 * @param att current attitude of the robot
 * @param omni_att_mode attitude mode for omnidirectional vehicles
 * @param omni_dfc_max_thrust maximum direct-force (horizontal) scaled thrust for omnidirectional vehicles
 * @param omni_att_tilt_angle the desired tilt for the vehicle in mode=3, is output for mode > 5 (in radians)
 * @param omni_att_tilt_dir the direction of the desired tilt with respec to North in mode=3, is output for mode > 5 (in radians)
 * @param omni_att_roll the desired roll for the vehicle in mode=4, is output for mode > 5 (in radians)
 * @param omni_att_pitch the desired pitch for the vehicle in mode=4, is output for mode=6 (in radians)
 * @param omni_att_rate the attitude change rate for mode=6
 * @param omni_proj_axes the axes used for thrust projection (0=calculated, 1=current)
 * @param att_sp attitude setpoint to fill
 */
void thrustToAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
		      const int omni_att_mode, const float omni_dfc_max_thrust, float &omni_att_tilt_angle, float &omni_att_tilt_dir,
		      float &omni_att_roll, float &omni_att_pitch, const float omni_att_rate, const int omni_proj_axes,
		      vehicle_attitude_setpoint_s &att_sp, omni_attitude_status_s &omni_status);

/**
 * Converts a body z vector and yaw set-point to a desired attitude.
 * @param body_z a world frame 3D vector in direction of the desired body z axis
 * @param yaw_sp the desired yaw setpoint
 * @param att_sp attitude setpoint to fill
 */
void bodyzToAttitude(matrix::Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp);

/**
 * Converts inertial thrust vector and yaw set-point to a zero-tilt attitude and body thrust vector for an omni-directional multirotor.
 * @param thr_sp a 3D vector
 * @param yaw_sp the desired yaw
 * @param att current attitude of the robot
 * @param omni_proj_axes the axes used for thrust projection (0=calculated, 1=current)
 * @param att_sp attitude setpoint to fill
 */
void thrustToZeroTiltAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			      int omni_proj_axes, vehicle_attitude_setpoint_s &att_sp);

/**
 * Converts inertial thrust vector and yaw set-point to a minimum-tilt attitude and body thrust vector for an omni-directional multirotor.
 * @param thr_sp a 3D vector
 * @param yaw_sp the desired yaw
 * @param omni_dfc_max_thrust maximum direct-force (horizontal) scaled thrust for omnidirectional vehicles
 * @param att current attitude of the robot
 * @param omni_proj_axes the axes used for thrust projection (0=calculated, 1=current)
 * @param att_sp attitude setpoint to fill
 */
void thrustToMinTiltAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, const float omni_dfc_max_thrust,
			     const matrix::Quatf &att, int omni_proj_axes, vehicle_attitude_setpoint_s &att_sp);
/**
 * Converts inertial thrust vector and yaw set-point to a desired-tilt attitude and body thrust vector for an omni-directional multirotor.
 * @param thr_sp a 3D vector
 * @param yaw_sp the desired yaw
 * @param att current attitude of the robot
 * @param tilt_angle the desired tilt angle
 * @param tilt_dir the desired tilt direction
 * @param omni_proj_axes the axes used for thrust projection (0=calculated, 1=current)
 * @param att_sp attitude setpoint to fill
 */
void thrustToFixedTiltAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			       const float tilt_angle, const float tilt_dir, int omni_proj_axes, vehicle_attitude_setpoint_s &att_sp);

/**
 * Converts inertial thrust vector and yaw set-point to a desired given attitude and body thrust vector for an omni-directional multirotor.
 * @param thr_sp a 3D vector
 * @param yaw_sp the desired yaw
 * @param att current attitude of the robot
 * @param roll_angle the desired roll angle
 * @param pitch_angle the desired pitch angle
 * @param omni_proj_axes the axes used for thrust projection (0=calculated, 1=current)
 * @param att_sp attitude setpoint to fill
 */
void thrustToFixedRollPitch(const matrix::Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			    const float roll_angle, const float pitch_angle, int omni_proj_axes, vehicle_attitude_setpoint_s &att_sp);

/**
 * Converts inertial thrust vector and yaw set-point to a slow-changing attitude and body thrust vector for an omni-directional multirotor.
 * @param thr_sp a 3D vector
 * @param yaw_sp the desired yaw
 * @param att current attitude of the robot
 * @param tilt_rate rate for the tilt change (non-negative in radians per loop)
 * @param omni_proj_axes the axes used for thrust projection (0=calculated, 1=current)
 * @param att_sp attitude setpoint to fill
 */
void thrustToSlowAttitude(const matrix::Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			  const float tilt_angle_rate, int omni_proj_axes, vehicle_attitude_setpoint_s &att_sp);

/**
 * Outputs the sum of two vectors but respecting the limits and priority.
 * The sum of two vectors are constraint such that v0 has priority over v1.
 * This means that if the length of (v0+v1) exceeds max, then it is constraint such
 * that v0 has priority.
 *
 * @param v0 a 2D vector that has priority given the maximum available magnitude.
 * @param v1 a 2D vector that less priority given the maximum available magnitude.
 * @return 2D vector
 */
matrix::Vector2f constrainXY(const matrix::Vector2f &v0, const matrix::Vector2f &v1, const float &max);

/**
 * This method was used for smoothing the corners along two lines.
 *
 * @param sphere_c
 * @param sphere_r
 * @param line_a
 * @param line_b
 * @param res
 * return boolean
 *
 * Note: this method is not used anywhere and first requires review before usage.
 */
bool cross_sphere_line(const matrix::Vector3f &sphere_c, const float sphere_r, const matrix::Vector3f &line_a,
		       const matrix::Vector3f &line_b, matrix::Vector3f &res);
}
