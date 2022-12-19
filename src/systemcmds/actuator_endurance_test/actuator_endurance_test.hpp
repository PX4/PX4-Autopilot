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

#pragma once

#include <math.h>

#include <drivers/drv_hrt.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/vehicle_status_flags.h>

using namespace time_literals;

class ActuatorEnduranceTest : public ModuleBase<ActuatorEnduranceTest>
{
public:
	static constexpr uint8_t _num_channels{8};
	static constexpr uint32_t _control_frequency_hz{100};
	static constexpr uint32_t _control_period_us{1_s / _control_frequency_hz};
	static constexpr uint8_t _max_act_frequency_hz{1};
	static constexpr hrt_abstime _module_timeout{60_s};

	enum class Profile {
		Sine = 0,
		Triangle = 1,
		Square = 2,
	};

	struct TestParams {
		int8_t driver_instance = -1;
		uint8_t cmd_interval_ms = 0;
		uint32_t test_duration_s = 0;
		Profile profile[_num_channels] {};
		float frequency[_num_channels] {};
		float min[_num_channels] {};
		float max[_num_channels] {};
	};

	ActuatorEnduranceTest() = default;

	~ActuatorEnduranceTest() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ActuatorEnduranceTest *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int print_params();

	const TestParams &get_params();

	bool set_params(const TestParams &test_params);

	void execute();

private:
	hrt_abstime _time_finished_abs{0};
	hrt_abstime _last_command_time[_num_channels] {};
	TestParams _test_params;
	px4::atomic_bool _execute{false};

	bool params_feasible(const TestParams &test_params);

	void run_test();

	void test_motor_pub(unsigned channel, float value, uint8_t driver_instance, int act_timeout_ms);

	void stop_motors();

	uORB::Publication<test_motor_s> _test_motor_pub{ORB_ID(test_motor)};
};
