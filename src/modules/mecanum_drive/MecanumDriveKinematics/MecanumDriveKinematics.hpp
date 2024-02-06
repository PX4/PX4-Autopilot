/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/mecanum_drive_setpoint.h>

/**
 * @brief Mecanum Drive Kinematics class for computing the kinematics of a mecanum drive robot.
 *
 * This class provides functions to set the wheel base and radius, and to compute the inverse kinematics
 * given linear velocity and yaw rate.
 */
class MecanumDriveKinematics : public ModuleParams
{
public:
	MecanumDriveKinematics(ModuleParams *parent);
	~MecanumDriveKinematics() = default;

	/**
	 * @brief Sets the wheel base of the robot.
	 *
	 * @param wheel_base The distance between the centers of the wheels.
	 */
	void setWheelBase(const float lx, const float ly) { _lx = lx; _ly = ly; };

	/**
	 * @brief Sets the maximum speed of the robot.
	 *
	 * @param max_speed The maximum speed of the robot.
	 */
	void setMaxSpeed(const float vx_max, const float vy_max) { _vx_max = vx_max; _vy_max = vy_max; };

	/**
	 * @brief Sets the maximum angular speed of the robot.
	 *
	 * @param max_angular_speed The maximum angular speed of the robot.
	 */
	void setMaxAngularVelocity(const float max_angular_velocity) { _max_angular_velocity = max_angular_velocity; };

	void setWheelRadius(const float wheel_radius) {_r = wheel_radius;};

	void setArmed(const bool armed) { _armed = armed; };

	void allocate();

	/**
	 * @brief Computes the inverse kinematics for mecanum drive.
	 *
	 * @param linear_velocity_x Linear velocity along the x-axis.
	 * @param yaw_rate Yaw rate of the robot.
	 * @return matrix::Vector2f Motor velocities for the right and left motors.
	 */
	matrix::Vector4f computeInverseKinematics(float linear_velocity_x, float linear_velocity_y, float yaw_rate) const;

private:
	uORB::Subscription _mecanum_drive_control_output_sub{ORB_ID(mecanum_drive_control_output)};
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	mecanum_drive_setpoint_s _mecanum_drive_control_output{};
	bool _armed = false;

	float _lx{0.f};
	float _ly{0.f};
	float _vx_max{0.f};
	float _vy_max{0.f};
	float _r{1.f};
	float _max_angular_velocity{0.f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
