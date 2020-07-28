/****************************************************************************
 *
 *   Copyright (c) 2020 ThunderFly s.r.o. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>

#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/input_rc.h>


extern "C" __EXPORT int assisted_release_main(int argc, char *argv[]);


class AssistedRelease : public ModuleBase<AssistedRelease>, public ModuleParams
{
public:
	AssistedRelease(int example_param, bool example_flag);

	virtual ~AssistedRelease() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static AssistedRelease *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	int _rpm_value = 0;
	int _airspeed_value = 0;
	float32 _rc_channel = 0;
	float32 _throttle = 0;

	int _rpm_sub{-1};
	int _airspeed_sub{-1};
	int _vehicle_status_sub{-1};
    int _actuator_controls_0_sub{-1};
    int _input_rc_sub{-1};

	rpm_s _rpm{};
	vehicle_status_s _vehicle_status{};
	airspeed_s _airspeed{};
    actuator_controls_s _actuator_controls{};
    input_rc_s _input_rc{};

	void rpm_poll();
	void airspeed_poll();
	void vehicle_status_poll();
	void actuator_controls_poll();
	void input_rc_poll();


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ASREL_LATCH_TIME>) _param_latch_time,
		(ParamInt<px4::params::ASREL_MIN_ASPD>) _param_min_aspd,
		(ParamInt<px4::params::ASREL_MIN_RPM>) _param_min_rpm,
		(ParamInt<px4::params::ASREL_RC_CHAN>) _param_rc_chan,
		(ParamInt<px4::params::ASREL_OUT_CHAN>) _param_out_chan
	)

	// Subscriptions
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

};
