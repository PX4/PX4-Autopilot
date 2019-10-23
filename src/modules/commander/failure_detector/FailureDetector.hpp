
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
* @file FailureDetector.hpp
* Base class for failure detection logic based on vehicle states
* for failsafe triggering.
*
* @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
*
*/

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_module_params.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>

typedef enum {
	FAILURE_NONE = vehicle_status_s::FAILURE_NONE,
	FAILURE_ROLL = vehicle_status_s::FAILURE_ROLL,
	FAILURE_PITCH = vehicle_status_s::FAILURE_PITCH,
	FAILURE_ALT = vehicle_status_s::FAILURE_ALT,
} failure_detector_bitmak;

using uORB::Subscription;

class FailureDetector : public ModuleParams
{
public:
	FailureDetector(ModuleParams *parent);

	bool update();

	uint8_t get_status() const {return _status;}

private:

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FD_FAIL_P>) _param_fd_fail_p,
		(ParamInt<px4::params::FD_FAIL_R>) _param_fd_fail_r
	)

	// Subscriptions
	Subscription<vehicle_attitude_s> _sub_vehicle_attitude_setpoint;
	Subscription<vehicle_attitude_s> _sub_vehicule_attitude;

	uint8_t _status{FAILURE_NONE};

	bool update_attitude_status();
};
