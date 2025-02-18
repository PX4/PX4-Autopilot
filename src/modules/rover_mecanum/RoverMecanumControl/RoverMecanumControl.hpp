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

#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_mecanum_setpoint.h>
#include <uORB/topics/rover_mecanum_status.h>
#include <uORB/topics/actuator_motors.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/slew_rate/SlewRateYaw.hpp>

// Standard libraries
#include <lib/pid/PID.hpp>
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <math.h>

using namespace matrix;

/**
 * @brief Class for mecanum rover guidance.
 */
class RoverMecanumControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for RoverMecanumControl.
	 * @param parent The parent ModuleParams object.
	 */
	RoverMecanumControl(ModuleParams *parent);
	~RoverMecanumControl() = default;

	/**
	 * @brief Compute motor commands based on setpoints
	 * @param vehicle_yaw Yaw of the vehicle [rad]
	 * @param vehicle_yaw_rate Yaw rate of the vehicle [rad/s]
	 * @param vehicle_forward_speed Forward speed of the vehicle [m/s]
	 * @param vehicle_lateral_speed Lateral speed of the vehicle [m/s]
	 */
	void computeMotorCommands(float vehicle_yaw, float vehicle_yaw_rate, float vehicle_forward_speed,
				  float vehicle_lateral_speed);

	/**
	 * @brief Reset PID controllers
	 */
	void resetControllers();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Compute normalized speed diff setpoint between the left and right wheels and apply slew rates.
	 * @param yaw_rate_setpoint Yaw rate setpoint [rad/s or normalized [-1, 1]].
	 * @param vehicle_yaw_rate Measured yaw rate [rad/s].
	 * @param max_thr_yaw_r Yaw rate turning left/right wheels at max speed in opposite directions [m/s].
	 * @param max_yaw_accel Maximum allowed yaw acceleration for the rover [rad/s^2].
	 * @param wheel_track Wheel track [m].
	 * @param dt Time since last update [s].
	 * @param yaw_rate_with_accel_limit Yaw rate slew rate.
	 * @param pid_yaw_rate Yaw rate PID
	 * @param normalized Indicates if the forward speed setpoint is already normalized.
	 * @return Normalized speed differece setpoint with applied slew rates [-1, 1].
	 */
	float calcNormalizedSpeedDiff(float yaw_rate_setpoint, float vehicle_yaw_rate, float max_thr_yaw_r, float max_yaw_accel,
				      float wheel_track, float dt, SlewRate<float> &yaw_rate_with_accel_limit, PID &pid_yaw_rate, bool normalized);
	/**
	 * @brief Compute normalized speed setpoint and apply slew rates.
	 * to the speed setpoint and doing closed loop speed control if enabled.
	 * @param speed_setpoint Speed setpoint [m/s].
	 * @param vehicle_speed Actual speed [m/s].
	 * @param max_thr_spd Speed the rover drives at maximum throttle [m/s].
	 * @param speed_setpoint_with_accel_limit Speed slew rate.
	 * @param pid_throttle Throttle PID
	 * @param max_accel Maximum acceleration [m/s^2]
	 * @param max_decel Maximum deceleration [m/s^2]
	 * @param dt Time since last update [s].
	 * @param normalized Indicates if the speed setpoint is already normalized.
	 * @return Normalized speed setpoint with applied slew rates [-1, 1].
	 */
	float calcNormalizedSpeedSetpoint(float speed_setpoint, float vehicle_forward_speed, float max_thr_spd,
					  SlewRate<float> &speed_setpoint_with_accel_limit, PID &pid_throttle, float max_accel, float max_decel,
					  float dt, bool normalized);

	/**
	 * @brief Turn normalized speed setpoints into normalized motor commands.
	 *
	 * @param forward_speed Normalized linear speed in body forward direction [-1, 1].
	 * @param lateral_speed Normalized linear speed in body lateral direction [-1, 1].
	 * @param speed_diff Normalized speed difference between left and right wheels [-1, 1].
	 * @return matrix::Vector4f: Normalized motor inputs [-1, 1]
	 */
	matrix::Vector4f computeInverseKinematics(float forward_speed, float lateral_speed, float speed_diff);

	// uORB subscriptions
	uORB::Subscription _rover_mecanum_setpoint_sub{ORB_ID(rover_mecanum_setpoint)};

	// uORB publications
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<rover_mecanum_status_s> _rover_mecanum_status_pub{ORB_ID(rover_mecanum_status)};
	rover_mecanum_status_s _rover_mecanum_status{};

	// Variables
	rover_mecanum_setpoint_s _rover_mecanum_setpoint{};
	hrt_abstime _timestamp{0};
	float _max_yaw_rate{0.f};
	float _max_yaw_accel{0.f};

	// Controllers
	PID _pid_forward_throttle; // PID for the closed loop forward speed control
	PID _pid_lateral_throttle; // PID for the closed loop lateral speed control
	PID _pid_yaw; // PID for the closed loop yaw control
	PID _pid_yaw_rate; // PID for the closed loop yaw rate control
	SlewRate<float> _forward_speed_setpoint_with_accel_limit{0.f};
	SlewRate<float> _lateral_speed_setpoint_with_accel_limit{0.f};
	SlewRate<float> _yaw_rate_with_accel_limit{0.f};
	SlewRateYaw<float> _yaw_setpoint_with_yaw_rate_limit;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RM_WHEEL_TRACK>)   _param_rm_wheel_track,
		(ParamFloat<px4::params::RM_MAX_THR_SPD>)   _param_rm_max_thr_spd,
		(ParamFloat<px4::params::RM_MAX_ACCEL>)     _param_rm_max_accel,
		(ParamFloat<px4::params::RM_MAX_DECEL>)     _param_rm_max_decel,
		(ParamFloat<px4::params::RM_MAX_THR_YAW_R>) _param_rm_max_thr_yaw_r,
		(ParamFloat<px4::params::RM_MAX_YAW_RATE>)  _param_rm_max_yaw_rate,
		(ParamFloat<px4::params::RM_MAX_YAW_ACCEL>) _param_rm_max_yaw_accel,
		(ParamFloat<px4::params::RM_YAW_RATE_P>)    _param_rm_yaw_rate_p,
		(ParamFloat<px4::params::RM_YAW_RATE_I>)    _param_rm_yaw_rate_i,
		(ParamFloat<px4::params::RM_SPEED_P>)       _param_rm_p_gain_speed,
		(ParamFloat<px4::params::RM_SPEED_I>)       _param_rm_i_gain_speed,
		(ParamFloat<px4::params::RM_YAW_P>)         _param_rm_p_gain_yaw,
		(ParamFloat<px4::params::RM_YAW_I>)         _param_rm_i_gain_yaw,
		(ParamInt<px4::params::CA_R_REV>)           _param_r_rev
	)
};
