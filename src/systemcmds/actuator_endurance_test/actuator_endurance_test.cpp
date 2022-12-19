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

#include "actuator_endurance_test.hpp"

using namespace time_literals;

int ActuatorEnduranceTest::print_status()
{
	int time_remaining = (int)((_time_finished_abs - hrt_absolute_time()) / 1_s);

	if (_execute.load()) {
		PX4_INFO("running: %i seconds remaining", time_remaining);

	} else {
		PX4_INFO("running: timeout in %i seconds", time_remaining);
	}

	print_params();

	return 0;
}

int ActuatorEnduranceTest::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd(
			   "actuator_endurance_test",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   4096,
			   (px4_main_t)&run_trampoline,
			   (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ActuatorEnduranceTest *ActuatorEnduranceTest::instantiate(int argc, char *argv[])
{
	ActuatorEnduranceTest *instance = new ActuatorEnduranceTest();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int ActuatorEnduranceTest::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	ActuatorEnduranceTest *instance = get_instance();

	// bump timeout
	instance->_time_finished_abs = hrt_absolute_time() + _module_timeout;

	// default values unset
	int motor = -1;
	int profile = -1;
	float frequency = -1.0f;
	float min = -1.0f;
	float max = -1.0f;
	int duration = -1;
	int interval = -1;
	int driver = -1;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:t:i:m:p:f:l:h:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			driver = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 't':
			duration = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 'i':
			interval = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 'm':
			motor = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 'p':
			profile = (int)strtol(myoptarg, nullptr, 0);
			break;

		case 'f':
			frequency = strtof(myoptarg, nullptr);
			break;

		case 'l':
			min = strtof(myoptarg, nullptr);
			break;

		case 'h':
			max = strtof(myoptarg, nullptr);
			break;

		default:
			print_usage(nullptr);
			return 1;
		}
	}

	if (argc == 1 && !strcmp(argv[0], "execute")) {
		instance->execute();
		return PX4_OK;

	} else if (!strcmp(argv[0], "set")) {

		TestParams new_params = instance->get_params();

		// check global settings
		if (driver >= 0) {
			new_params.driver_instance = (uint8_t)driver;
		}

		if (interval >= 0) {
			new_params.cmd_interval_ms = (uint8_t)interval;
		}

		if (duration >= 0) {
			new_params.test_duration_s = (uint32_t)duration;
		}

		// check motor settings
		if (motor >= 0 && motor <= instance->_num_channels) {
			// motor valid
			if (profile >= 0 && profile <= 2) {
				new_params.profile[motor] = (Profile)profile;
			}

			if (frequency >= 0.0f) {
				new_params.frequency[motor] = frequency;
			}

			if (min >= 0.0f) {
				new_params.min[motor] = min;
			}

			if (max >= 0.0f) {
				new_params.max[motor] = max;
			}
		}

		if (instance->set_params(new_params)) {
			PX4_INFO("params accepted");
			return PX4_OK;

		} else {
			print_usage("params rejected");
			return PX4_ERROR;
		}

	}

	return print_usage("unknown command");
}

int ActuatorEnduranceTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
Utility to test actuators.
WARNING: remove all props before using this command.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("actuator_endurance_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "start the module (no actuator outputs yet)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "define the test parameters using the flags below");
	PRINT_MODULE_USAGE_PARAM_COMMENT("global settings, define these first");
	PRINT_MODULE_USAGE_PARAM_INT('d', -1, 0, 4, "driver instance", false);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, UINT32_MAX, "test duration [s]", false);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, UINT8_MAX, "min interval [ms] between commands per channel", false);
	PRINT_MODULE_USAGE_PARAM_COMMENT("motor settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', 0, 0, _num_channels, "driver instance", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 3, "profile: 0 = sine, 1 = triangle, 2 = square", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('f', 0.0, 0.0, 1.0, "frequency [Hz]", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('l', 0.0, 0.0, 1.0, "normalized min value", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('h', 1.0, 0.0, 1.0, "normalized max value", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "prints the time remaining and the current settings");
	PRINT_MODULE_USAGE_COMMAND_DESCR("execute", "execute the actuator test with the current settings");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "send actuator disarm value and stop the module");
	return 0;
}

void ActuatorEnduranceTest::run()
{
	PX4_INFO("started");
	_time_finished_abs = hrt_absolute_time() + _module_timeout; // timeout if test not started

	while (true) {
		if (should_exit()) {
			PX4_INFO("stopped");
			break;

		} else if (hrt_absolute_time() > _time_finished_abs) {
			PX4_INFO("timed out");
			break;

		} else if (_execute.load()) {
			PX4_INFO("execute test");
			run_test();
			PX4_INFO("done");
			break;
		}

		px4_usleep(100_ms);
	}

	exit_and_cleanup();
}

int ActuatorEnduranceTest::print_params()
{
	PX4_INFO("*** Test Parameters ***");
	PX4_INFO("driver instance: %i ", _test_params.driver_instance);
	PX4_INFO("act cmd interval: %i milliseconds ", (int)_test_params.cmd_interval_ms);
	PX4_INFO("test duration: %i seconds", (int)_test_params.test_duration_s);

	for (unsigned ch = 0; ch < _num_channels; ch++) {
		PX4_INFO("channel %i: profile: %i, frequency: %.2f, min: %.3f, max: %.3f",
			 ch,
			 (int)_test_params.profile[ch],
			 (double)_test_params.frequency[ch],
			 (double)_test_params.min[ch],
			 (double)_test_params.max[ch]);
	}

	return 0;
}

const ActuatorEnduranceTest::TestParams& ActuatorEnduranceTest::get_params()
{
	return _test_params;
}

bool ActuatorEnduranceTest::set_params(const TestParams &test_params)
{
	if (!_execute.load() && params_feasible(test_params)) {
		_test_params = test_params;
		return true;

	}

	return false;
}

void ActuatorEnduranceTest::execute()
{
	if (params_feasible(_test_params)) {
		_execute.store(true);

	} else {
		PX4_INFO("execute rejected: parameters infeasible");
	}
}

bool ActuatorEnduranceTest::params_feasible(const TestParams &test_params)
{
	for (unsigned ch = 0; ch < _num_channels; ch++) {
		if (test_params.driver_instance < 0 ||
		    test_params.driver_instance > 4 ||
		    test_params.frequency[ch] < 0.0f ||
		    test_params.frequency[ch] > (float)_max_act_frequency_hz ||
		    test_params.min[ch] < 0.0f ||
		    test_params.min[ch] > 1.0f ||
		    test_params.max[ch] < 0.0f ||
		    test_params.max[ch] > 1.0f ||
		    test_params.min[ch] > test_params.max[ch]) {
			return false;
		}
	}

	return true;
}

void ActuatorEnduranceTest::run_test()
{
	hrt_abstime start_time = hrt_absolute_time();
	_time_finished_abs = start_time + _test_params.test_duration_s * 1_s;

	while (hrt_absolute_time() < _time_finished_abs) {

		if (should_exit()) {
			stop_motors();
			exit_and_cleanup();
			return;
		}

		for (size_t ch = 0; ch < _num_channels; ch++) {

			if (_test_params.frequency[ch] > 0.0f) {

				float val = -1.0f;
				float tf = (float)hrt_elapsed_time(&start_time) / 1e6f * _test_params.frequency[ch];

				if (_test_params.profile[ch] == Profile::Sine) {
					val = (0.5f * sinf(2 * M_PI_F * tf) + 0.5f);

				} else if (_test_params.profile[ch] == Profile::Triangle) {
					val = 2.0f * fabsf(tf - floorf(tf + 0.5f));

				} else if (_test_params.profile[ch] == Profile::Square) {
					val = (2.0f * floorf(tf) - floorf(2.0f * tf)) + 1.0f;
				}

				val = _test_params.min[ch] + (_test_params.max[ch] - _test_params.min[ch]) * val;

				if (hrt_elapsed_time(&_last_command_time[ch]) > _test_params.cmd_interval_ms * 1_ms) {
					test_motor_pub(ch, val, _test_params.driver_instance, 0);
					_last_command_time[ch] = hrt_absolute_time();
				}
			}
		}

		px4_usleep(_control_period_us);
	}

	stop_motors();
}

void ActuatorEnduranceTest::test_motor_pub(unsigned channel, float value, uint8_t driver_instance, int timeout_ms)
{
	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;
	test_motor.action = value >= 0.f ? test_motor_s::ACTION_RUN : test_motor_s::ACTION_STOP;
	test_motor.driver_instance = driver_instance;
	test_motor.timeout_ms = timeout_ms;

	_test_motor_pub.publish(test_motor);

	if (test_motor.action == test_motor_s::ACTION_STOP) {
		PX4_INFO("motors stop command sent");
	}
}

void ActuatorEnduranceTest::stop_motors()
{
	test_motor_pub(-1, -1.0f, _test_params.driver_instance, 0);
}

extern "C" __EXPORT int actuator_endurance_test_main(int argc, char *argv[])
{
	return ActuatorEnduranceTest::main(argc, argv);
}
