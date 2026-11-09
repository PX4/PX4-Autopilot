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
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

// Libraries
#include <lib/rover_control/RoverControl.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <math.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/surface_vehicle_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>

/**
 * @brief Class for ackermann actuator control.
 */
class AckermannActControl : public ModuleParams, public px4::WorkItem
{
public:
	/**
	 * @brief Constructor for AckermannActControl.
	 * @param parent The parent ModuleParams object.
	 */
	AckermannActControl(ModuleParams *parent);
	~AckermannActControl() = default;

	/**
	 * @brief Register the callback on the topic that triggers the controller.
	 */
	bool init();

	/**
	 * @brief Run the controller on its trigger topic only while enabled.
	 *
	 * Only an enabled controller is woken by its trigger topic. On enabling it also runs once right away,
	 * so it does not miss the trigger that was published before it registered.
	 * It only counts as enabled once the trigger callback is registered, otherwise the next call retries.
	 *
	 * @return true if the operation succeeded
	 */
	bool setEnabled(bool enabled)
	{
		if (enabled && !_enabled) {
			_timestamp = hrt_absolute_time(); // The first dt after (re-)enabling must not span the time spent disabled
			_enabled = _vehicle_angular_velocity_sub.registerCallback();

			if (_enabled) {
				ScheduleNow();
			}

		} else if (!enabled && _enabled) {
			_vehicle_angular_velocity_sub.unregisterCallback();
			_enabled = false;
		}

		return _enabled == enabled;
	}

	bool enabled() const { return _enabled; }

	/**
	 * @brief Move the trigger callback to the end of the callback list. Callbacks sharing a topic run in registration order.
	 */
	bool reregisterTrigger()
	{
		_vehicle_angular_velocity_sub.unregisterCallback();
		_enabled = _vehicle_angular_velocity_sub.registerCallback(); // On failure the next setEnabled() retries
		return _enabled;
	}

	/**
	 * @brief Generate and publish actuatorMotors/actuatorServos setpoints from roverThrottleSetpoint/surfaceVehicleSteeringSetpoint.
	 */
	void updateActControl();

	/**
	 * @brief Stop the vehicle by sending 0 commands to motors and servos.
	 */
	void stopVehicle();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	void Run() override;


	// uORB subscriptions
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _surface_vehicle_steering_setpoint_sub{ORB_ID(surface_vehicle_steering_setpoint)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)}; // Triggers the controller after the rate controller
	uORB::Subscription _rover_throttle_setpoint_sub{ORB_ID(rover_throttle_setpoint)};

	// uORB publications
	uORB::Publication<actuator_motors_s> 	     _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> 	     _actuator_servos_pub{ORB_ID(actuator_servos)};

	// Variables
	bool _enabled{false};
	hrt_abstime _timestamp{0};
	float _throttle_setpoint{NAN};
	float _steering_setpoint{NAN};

	// Controllers
	SlewRate<float> _servo_setpoint{0.f};
	SlewRate<float> _motor_setpoint{0.f};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) 	    _param_r_rev,
		(ParamFloat<px4::params::RA_STR_RATE_LIM>)  _param_ra_str_rate_limit,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>)   _param_ra_max_str_ang,
		(ParamFloat<px4::params::RO_ACCEL_LIM>)     _param_ro_accel_limit,
		(ParamFloat<px4::params::RO_DECEL_LIM>)     _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed
	)
};
