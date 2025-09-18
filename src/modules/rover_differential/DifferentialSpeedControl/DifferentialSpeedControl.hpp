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
#include <px4_platform_common/events.h>

// Libraries
#include <lib/rover_control/RoverControl.hpp>
#include <lib/pid/PID.hpp>
#include <matrix/matrix/math.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_speed_status.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

/**
 * @brief Class for differential speed control.
 */
class DifferentialSpeedControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for DifferentialSpeedControl.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialSpeedControl(ModuleParams *parent);
	~DifferentialSpeedControl() = default;

	/**
	 * @brief Generate and publish RoverThrottleSetpoint from roverSpeedSetpoint.
	 */
	void updateSpeedControl();

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

	/**
	 * @brief Reset speed controller.
	 */
	void reset() {_pid_speed.resetIntegral(); _speed_setpoint = NAN; _adjusted_speed_setpoint.setForcedValue(0.f);};

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Update uORB subscriptions used in speed controller.
	 */
	void updateSubscriptions();

	/**
	 * @brief Calculate the speed setpoint based on the current state.
	 * @return Speed setpoint.
	 */
	float calcSpeedSetpoint();

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rover_speed_setpoint_sub{ORB_ID(rover_speed_setpoint)};
	uORB::Subscription _rover_steering_setpoint_sub{ORB_ID(rover_steering_setpoint)};

	// uORB publications
	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_speed_status_s> _rover_speed_status_pub{ORB_ID(rover_speed_status)};

	// Variables
	hrt_abstime _timestamp{0};
	Quatf _vehicle_attitude_quaternion{};
	float _vehicle_speed{0.f}; // [m/s] Positiv: Forwards, Negativ: Backwards
	float _speed_setpoint{NAN};
	float _normalized_speed_diff{NAN};

	// Controllers
	PID _pid_speed;
	SlewRate<float> _adjusted_speed_setpoint{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed,
		(ParamFloat<px4::params::RO_SPEED_P>) 	    _param_ro_speed_p,
		(ParamFloat<px4::params::RO_SPEED_I>)       _param_ro_speed_i,
		(ParamFloat<px4::params::RO_ACCEL_LIM>)     _param_ro_accel_limit,
		(ParamFloat<px4::params::RO_DECEL_LIM>)     _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_JERK_LIM>)      _param_ro_jerk_limit,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::RO_SPEED_TH>)      _param_ro_speed_th
	)
};
