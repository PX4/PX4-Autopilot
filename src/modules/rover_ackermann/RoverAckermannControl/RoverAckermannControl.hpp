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
#include <uORB/topics/rover_ackermann_setpoint.h>
#include <uORB/topics/rover_ackermann_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>

// Standard libraries
#include <lib/pid/PID.hpp>
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <math.h>
#include <lib/slew_rate/SlewRate.hpp>

using namespace matrix;
using namespace time_literals;

/**
 * @brief Class for ackermann rover guidance.
 */
class RoverAckermannControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for RoverAckermannControl.
	 * @param parent The parent ModuleParams object.
	 */
	RoverAckermannControl(ModuleParams *parent);
	~RoverAckermannControl() = default;

	/**
	 * @brief Compute motor commands based on setpoints
	 * @param vehicle_forward_speed Measured speed in body x direction. Positiv: forwards, Negativ: backwards [m/s]
	 * @param vehicle_yaw Measured yaw [rad]
	 * @param vehicle_lateral_acceleration Measured acceleration in body y direction. Positiv: right, Negativ: left [m/s^s]
	 */
	void computeMotorCommands(float vehicle_forward_speed, float vehicle_yaw, float vehicle_lateral_acceleration);

	/**
	 * @brief Reset PID controllers and slew rates
	 */
	void resetControllers();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Compute normalized forward speed setpoint by applying slew rates
	 * to the forward speed setpoint and doing closed loop speed control if enabled.
	 * @param forward_speed_setpoint Forward speed setpoint [m/s].
	 * @param vehicle_forward_speed Actual forward speed [m/s].
	 * @param dt Time since last update [s].
	 * @param normalized Indicates if the forward speed setpoint is already normalized.
	 * @return Normalized forward speed setpoint with applied slew rates [-1, 1].
	 */
	float calcNormalizedSpeedSetpoint(float forward_speed_setpoint, float vehicle_forward_speed, float dt, bool normalized);

	// uORB subscriptions
	uORB::Subscription _rover_ackermann_setpoint_sub{ORB_ID(rover_ackermann_setpoint)};
	rover_ackermann_setpoint_s _rover_ackermann_setpoint{};

	// uORB publications
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<rover_ackermann_status_s> _rover_ackermann_status_pub{ORB_ID(rover_ackermann_status)};
	rover_ackermann_status_s _rover_ackermann_status{};

	// Variables
	hrt_abstime _timestamp{0};

	// Controllers
	PID _pid_throttle; // The PID controller for the closed loop speed control
	PID _pid_lat_accel; // The PID controller for the closed loop speed control
	SlewRate<float> _steering_with_rate_limit{0.f};
	SlewRate<float> _forward_speed_setpoint_with_accel_limit{0.f};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RA_SPEED_P>) _param_ra_speed_p,
		(ParamFloat<px4::params::RA_SPEED_I>) _param_ra_speed_i,
		(ParamFloat<px4::params::RA_LAT_ACCEL_P>) _param_ra_lat_accel_p,
		(ParamFloat<px4::params::RA_LAT_ACCEL_I>) _param_ra_lat_accel_i,
		(ParamFloat<px4::params::RA_MAX_ACCEL>) _param_ra_max_accel,
		(ParamFloat<px4::params::RA_MAX_DECEL>) _param_ra_max_decel,
		(ParamFloat<px4::params::RA_MAX_SPEED>) _param_ra_max_speed,
		(ParamFloat<px4::params::RA_MAX_THR_SPEED>) _param_ra_max_thr_speed,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>) _param_ra_max_steer_angle,
		(ParamFloat<px4::params::RA_MAX_STR_RATE>) _param_ra_max_steering_rate,
		(ParamFloat<px4::params::RA_WHEEL_BASE>) _param_ra_wheel_base,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
