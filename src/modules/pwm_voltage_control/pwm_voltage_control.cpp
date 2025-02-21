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

#include "pwm_voltage_control.hpp"

#  ifndef GPIO_PWM_VOLT_SEL
#  error "To use the pwm voltage control driver, the board_config.h must define and initialize GPIO_PWM_VOLT_SEL"
#  endif

PWMVoltageControl::PWMVoltageControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	px4_arch_configgpio(GPIO_PWM_VOLT_SEL);
	_last_pwm_volt = 0;
}

PWMVoltageControl::~PWMVoltageControl()
{
	px4_arch_unconfiggpio(GPIO_PWM_VOLT_SEL);
}

void PWMVoltageControl::Run(void)
{
	_p_pwm_volt_set = param_find("PWM_VOLT_SET");
	param_get(_p_pwm_volt_set, &_pwm_volt_set);

	if (_pwm_volt_set != _last_pwm_volt) {
		if (_pwm_volt_set != 0) {
			PWM_5V_VOLT_SEL(true);

		} else {
			PWM_5V_VOLT_SEL(false);
		}
	}

	_last_pwm_volt = _pwm_volt_set;

	ScheduleDelayed(_interval);
}

int PWMVoltageControl::task_spawn(int argc, char *argv[])
{
	PWMVoltageControl *instance = new PWMVoltageControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleNow();
		return PX4_OK;

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PWMVoltageControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PWMVoltageControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Set PWM_VOL_SET to control PWM output voltage.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_voltage_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_voltage_control_main(int argc, char *argv[])
{
	return PWMVoltageControl::main(argc, argv);
}
