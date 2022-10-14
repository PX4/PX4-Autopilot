/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * @file io_tester.cpp
 * Example for Linux
 *
 */

#include "io_tester.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>
#include <px4_arch/io_timer.h>

#include <drivers/drv_pwm_output.h>
#include <px4_platform_common/log.h>
#include <lib/perf/perf_counter.h>

using namespace time_literals;

IOTester *IOTester::_instance;


int IOTester::start()
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	_instance = new IOTester(MODULE_NAME, px4::wq_configurations::hp_default);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(ScheduleIntervalMs * 1000);

	return PX4_OK;
}

void IOTester::print_info()
{
	PX4_INFO("Rate %li", _rate);

	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		PX4_INFO("Channel %i %i", i, up_pwm_servo_get(i));
	}
}

void IOTester::setRate(uint32_t rate)
{
	_rate = rate;

	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		up_pwm_servo_set_rate_group_update(timer, _rate);
	}
}

IOTester::IOTester(const char *name, const px4::wq_config_t &config) :
	px4::ScheduledWorkItem(name, config)
{
	up_pwm_servo_init(_pwm_mask);
	up_pwm_servo_arm(1, _pwm_mask);
}

void IOTester::Run()
{

	actuator_test_s actuator_test;

	while (_actuator_test_sub.update(&actuator_test)) {

		if (actuator_test.timestamp == 0 ||
		    hrt_elapsed_time(&actuator_test.timestamp) > 100_ms) {
			continue;
		}

		if (actuator_test.action == actuator_test_s::ACTION_DO_CONTROL) {
			if (actuator_test.function < (actuator_test_s::FUNCTION_MOTOR1 + DIRECT_PWM_OUTPUT_CHANNELS)) {
				/* Convert value to duty */
				uint32_t setpoint = ((actuator_test.value + 1) * (1.0f / (float)(_rate * 2)) * 1000000);

				up_pwm_servo_set((actuator_test.function - actuator_test_s::FUNCTION_MOTOR1), setpoint);
				up_pwm_update((actuator_test.function - actuator_test_s::FUNCTION_MOTOR1));
			}
		}

	}


}
