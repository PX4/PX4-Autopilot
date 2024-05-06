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

#include "PWMSim.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

#include <px4_platform_common/sem.hpp>

PWMSim::PWMSim(bool hil_mode_enabled) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// initialize parameters - dynamic instead of static
	PWM_SIM_PWM_MAX_MAGIC = _param_pwm_sim_max_magic.get();
	PWM_SIM_PWM_MIN_MAGIC = _param_pwm_sim_min_magic.get();
	PWM_SIM_FAILSAFE_MAGIC = _param_pwm_sim_failsafe_magic.get();
	PWM_SIM_DISARMED_MAGIC = _param_pwm_sim_disarmed_magic.get();

	_mixing_output.setAllDisarmedValues(PWM_SIM_DISARMED_MAGIC);
	_mixing_output.setAllFailsafeValues(PWM_SIM_FAILSAFE_MAGIC);
	_mixing_output.setAllMinValues(PWM_SIM_PWM_MIN_MAGIC);
	_mixing_output.setAllMaxValues(PWM_SIM_PWM_MAX_MAGIC);
	_mixing_output.setIgnoreLockdown(hil_mode_enabled);
}

PWMSim::~PWMSim()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool PWMSim::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated)
{
	// Only publish once we receive actuator_controls (important for lock-step to work correctly)
	if (num_control_groups_updated > 0) {
		actuator_outputs_s actuator_outputs{};
		actuator_outputs.noutputs = num_outputs;

		const uint32_t reversible_outputs = _mixing_output.reversibleOutputs();

		for (int i = 0; i < (int)num_outputs; i++) {
			if (outputs[i] != PWM_SIM_DISARMED_MAGIC) {

				OutputFunction function = _mixing_output.outputFunction(i);
				bool is_reversible = reversible_outputs & (1u << i);
				float output = outputs[i];

				if (((int)function >= (int)OutputFunction::Motor1 && (int)function <= (int)OutputFunction::MotorMax)
				    && !is_reversible) {
					// Scale non-reversible motors to [0, 1]
					actuator_outputs.output[i] = (output - PWM_SIM_PWM_MIN_MAGIC) / (PWM_SIM_PWM_MAX_MAGIC - PWM_SIM_PWM_MIN_MAGIC);

				} else {
					// Scale everything else to [-1, 1]
					const float pwm_center = (PWM_SIM_PWM_MAX_MAGIC + PWM_SIM_PWM_MIN_MAGIC) / 2.f;
					const float pwm_delta = (PWM_SIM_PWM_MAX_MAGIC - PWM_SIM_PWM_MIN_MAGIC) / 2.f;
					actuator_outputs.output[i] = (output - pwm_center) / pwm_delta;
				}
			}
		}

		actuator_outputs.timestamp = hrt_absolute_time();
		_actuator_outputs_sim_pub.publish(actuator_outputs);
		return true;
	}

	return false;
}

void PWMSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);
}

int PWMSim::task_spawn(int argc, char *argv[])
{
	bool hil_mode = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			hil_mode = strcmp(myoptarg, "hil") == 0;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	PWMSim *instance = new PWMSim(hil_mode);

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int PWMSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PWMSim::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	return 0;
}

int PWMSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for simulated PWM outputs.

Its only function is to take `actuator_control` uORB messages,
mix them with any loaded mixer and output the result to the
`actuator_output` uORB topic.

It is used in SITL and HITL.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pwm_out_sim", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the module");
	PRINT_MODULE_USAGE_PARAM_STRING('m', "sim", "hil|sim", "Mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pwm_out_sim_main(int argc, char *argv[])
{
	return PWMSim::main(argc, argv);
}
