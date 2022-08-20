/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester_failure.h"

#include "math_helpers.h"
#include <iostream>
#include <future>
#include <thread>
#include <unistd.h>


void AutopilotTesterFailure::connect(const std::string uri)
{
	AutopilotTester::connect(uri);

	auto system = get_system();
	_failure.reset(new Failure(system));
}

void AutopilotTesterFailure::set_param_sys_failure_en(bool value)
{
	CHECK(getParams()->set_param_int("SYS_FAILURE_EN", value) == Param::Result::Success);
}

void AutopilotTesterFailure::set_param_fd_act_en(bool value)
{
	CHECK(getParams()->set_param_int("FD_ACT_EN", value) == Param::Result::Success);
}

void AutopilotTesterFailure::set_param_mc_airmode(int value)
{
	CHECK(getParams()->set_param_int("MC_AIRMODE", value) == Param::Result::Success);
}

void AutopilotTesterFailure::set_param_ca_failure_mode(int value)
{
	CHECK(getParams()->set_param_int("CA_FAILURE_MODE", value) == Param::Result::Success);
}

void AutopilotTesterFailure::set_param_com_act_fail_act(int value)
{
	CHECK(getParams()->set_param_int("COM_ACT_FAIL_ACT", value) == Param::Result::Success);
}

void AutopilotTesterFailure::inject_failure(mavsdk::Failure::FailureUnit failure_unit,
		mavsdk::Failure::FailureType failure_type, int instance, mavsdk::Failure::Result expected_result)
{
	CHECK(_failure->inject(failure_unit, failure_type, instance) == expected_result);
}

void AutopilotTesterFailure::enable_actuator_output_status()
{
	CHECK(getTelemetry()->set_rate_actuator_output_status(20.f) == Telemetry::Result::Success);
}

void AutopilotTesterFailure::ensure_motor_stopped(unsigned index, unsigned num_motors)
{
	const Telemetry::ActuatorOutputStatus &status = getTelemetry()->actuator_output_status();
	CHECK(status.active >= num_motors);

	for (unsigned i = 0; i < num_motors; ++i) {
		if (i == index) {
			CHECK(status.actuator[i] <= 901.f);

		} else { // ensure all others are still running
			CHECK(status.actuator[i] >= 999.f);
		}
	}
}
