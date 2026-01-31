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
#include <lib/slew_rate/SlewRate.hpp>
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_rate_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>

/**
 * @brief Class for differential rate control.
 */
class DifferentialRateControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for DifferentialRateControl.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialRateControl(ModuleParams *parent);
	~DifferentialRateControl() = default;

	/**
	 * @brief Generate and publish roverSteeringSetpoint from roverRateSetpoint.
	 */
	void updateRateControl();

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

	/**
	 * @brief Reset rate controller.
	 */
	void reset() {_pid_yaw_rate.resetIntegral(); _yaw_rate_setpoint = NAN;};

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:

	// uORB subscriptions
	uORB::Subscription _rover_rate_setpoint_sub{ORB_ID(rover_rate_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	// uORB publications
	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_rate_status_s> _rover_rate_status_pub{ORB_ID(rover_rate_status)};

	// Variables
	float _vehicle_yaw_rate{0.f};
	float _yaw_rate_setpoint{NAN};
	hrt_abstime _timestamp{0};

	// Controllers
	PID _pid_yaw_rate;
	SlewRate<float> _adjusted_yaw_rate_setpoint{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RD_WHEEL_TRACK>)   _param_rd_wheel_track,
		(ParamFloat<px4::params::RO_YAW_RATE_TH>)   _param_ro_yaw_rate_th,
		(ParamFloat<px4::params::RO_YAW_RATE_P>)    _param_ro_yaw_rate_p,
		(ParamFloat<px4::params::RO_YAW_RATE_I>)    _param_ro_yaw_rate_i,
		(ParamFloat<px4::params::RO_YAW_ACCEL_LIM>) _param_ro_yaw_accel_limit,
		(ParamFloat<px4::params::RO_YAW_DECEL_LIM>) _param_ro_yaw_decel_limit,
		(ParamFloat<px4::params::RO_YAW_RATE_CORR>) _param_ro_yaw_rate_corr,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed
	)
};
