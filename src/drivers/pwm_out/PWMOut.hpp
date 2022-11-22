/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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

#include <float.h>
#include <math.h>

#include <board_config.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class PWMOut final : public ModuleBase<PWMOut>, public OutputModuleInterface
{
public:
	PWMOut();
	~PWMOut() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;

	void update_params();
	bool update_pwm_out_state(bool on);

	MixingOutput _mixing_output{PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	int _timer_rates[MAX_IO_TIMERS] {};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	unsigned	_num_outputs{DIRECT_PWM_OUTPUT_CHANNELS};

	bool		_pwm_on{false};
	uint32_t	_pwm_mask{0};
	bool		_pwm_initialized{false};
	bool		_first_update_cycle{true};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
