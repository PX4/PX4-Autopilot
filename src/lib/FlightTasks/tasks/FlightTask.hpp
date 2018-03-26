/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTask.hpp
 *
 * Abstract base class for different advanced flight tasks like orbit, follow me, ...
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <px4_module_params.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_command.h>

#include "../SubscriptionArray.hpp"


class FlightTask : public ModuleParams
{
public:
	FlightTask() :
		ModuleParams(nullptr)
	{ _resetSetpoints(); }

	virtual ~FlightTask() = default;

	/**
	 * Initialize the uORB subscriptions using an array
	 * @return true on success, false on error
	 */
	virtual bool initializeSubscriptions(SubscriptionArray &subscription_array);

	/**
	 * Call once on the event where you switch to the task
	 * @return true on success, false on error
	 */
	virtual bool activate();

	/**
	 * To be called to adopt parameters from an arrived vehicle command
	 * @return true if accepted, false if declined
	 */
	virtual bool applyCommandParameters(const vehicle_command_s &command) { return true; };

	/**
	 * Call before activate() or update()
	 * to initialize time and input data
	 * @return true on success, false on error
	 */
	virtual bool updateInitialize();

	/**
	 * To be called regularly in the control loop cycle to execute the task
	 * @return true on success, false on error
	 */
	virtual bool update() = 0;

	/**
	 * Get the output data
	 */
	const vehicle_local_position_setpoint_s getPositionSetpoint();

	static const vehicle_local_position_setpoint_s empty_setpoint;

	/**
	 * Call this whenever a parameter update notification is received (parameter_update uORB message)
	 */
	void handleParameterUpdate()
	{
		updateParams();
	}

protected:
	/* Time abstraction */
	static constexpr uint64_t _timeout = 500000; /**< maximal time in us before a loop or data times out */
	float _time = 0; /**< passed time in seconds since the task was activated */
	float _deltatime = 0; /**< passed time in seconds since the task was last updated */
	hrt_abstime _time_stamp_activate = 0; /**< time stamp when task was activated */
	hrt_abstime _time_stamp_current = 0; /**< time stamp at the beginning of the current task update */
	hrt_abstime _time_stamp_last = 0; /**< time stamp when task was last updated */

	/* Current vehicle state */
	matrix::Vector3f _position; /**< current vehicle position */
	matrix::Vector3f _velocity; /**< current vehicle velocity */
	float _yaw = 0.f;

	/* Setpoints the position controller needs to execute
	 * NAN values mean the state does not get controlled */
	matrix::Vector3f _position_setpoint;
	matrix::Vector3f _velocity_setpoint;
	matrix::Vector3f _acceleration_setpoint;
	matrix::Vector3f _thrust_setpoint;
	float _yaw_setpoint;
	float _yawspeed_setpoint;

	/**
	 * Get the output data
	 */
	void _resetSetpoints();

private:
	uORB::Subscription<vehicle_local_position_s> *_sub_vehicle_local_position{nullptr};

	bool _evaluateVehiclePosition();
};
