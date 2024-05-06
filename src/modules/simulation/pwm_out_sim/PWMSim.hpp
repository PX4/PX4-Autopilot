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

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#define PARAM_PREFIX "PWM_MAIN"
#else
#define PARAM_PREFIX "HIL_ACT"
#endif

using namespace time_literals;

class PWMSim : public ModuleBase<PWMSim>, public OutputModuleInterface
{
public:
	PWMSim(bool hil_mode_enabled);
	~PWMSim() override;

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

	uint16_t PWM_SIM_DISARMED_MAGIC;
	uint16_t PWM_SIM_FAILSAFE_MAGIC;
	uint16_t PWM_SIM_PWM_MIN_MAGIC;
	uint16_t PWM_SIM_PWM_MAX_MAGIC;

	MixingOutput _mixing_output{PARAM_PREFIX, MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<actuator_outputs_s> _actuator_outputs_sim_pub{ORB_ID(actuator_outputs_sim)};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PWM_SIM_PWM_MAX>) _param_pwm_sim_max_magic,
		(ParamInt<px4::params::PWM_SIM_PWM_MIN>) _param_pwm_sim_min_magic,
		(ParamInt<px4::params::PWM_SIM_FAILSAFE>) _param_pwm_sim_failsafe_magic,
		(ParamInt<px4::params::PWM_SIM_DISARM>) _param_pwm_sim_disarmed_magic
	)
};
