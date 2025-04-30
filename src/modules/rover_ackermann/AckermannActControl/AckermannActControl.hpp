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

#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>

// Libraries
#include <lib/rover_control/RoverControl.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <math.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>

/**
 * @brief Class for ackermann actuator control.
 */
class AckermannActControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for AckermannActControl.
	 * @param parent The parent ModuleParams object.
	 */
	AckermannActControl(ModuleParams *parent);
	~AckermannActControl() = default;

	/**
	 * @brief Generate and publish actuatorMotors/actuatorServos setpoints from roverThrottleSetpoint/roverSteeringSetpoint.
	 */
	void updateActControl();

	/**
	 * @brief Manual mode
	 */
	void manualMode();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:

	// uORB subscriptions
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _rover_steering_setpoint_sub{ORB_ID(rover_steering_setpoint)};
	uORB::Subscription _rover_throttle_setpoint_sub{ORB_ID(rover_throttle_setpoint)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// uORB publications
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};

	// Variables
	hrt_abstime _timestamp{0};
	float _dt{0.f};

	// Controllers
	SlewRate<float> _servo_setpoint{0.f};
	SlewRate<float> _motor_setpoint{0.f};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev,
		(ParamFloat<px4::params::RA_STR_RATE_LIM>) _param_ra_str_rate_limit,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>) _param_ra_max_str_ang,
		(ParamFloat<px4::params::RO_ACCEL_LIM>) _param_ro_accel_limit,
		(ParamFloat<px4::params::RO_DECEL_LIM>) _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed
	)
};
