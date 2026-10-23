/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "GpioOut.hpp"

#include <drivers/drv_io_timer_gpio.h>

ModuleBase::Descriptor GpioOut::desc{task_spawn, custom_command, print_usage};

GpioOut::GpioOut() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setMaxTopicUpdateRate(50_ms);
}

GpioOut::~GpioOut()
{
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		if (_gpio_pin[i]) {
			px4_arch_gpiowrite(_gpio_pin[i], _mixing_output.isAboveMidpoint(i, _mixing_output.disarmedValue(i)));
			io_timer_unallocate_channel(i);
		}
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

void GpioOut::claim_gpio_channels()
{
	char param_name[17];
	snprintf(param_name, sizeof(param_name), "%s_GPIO_EN", _mixing_output.paramPrefix());
	param_t handle = param_find(param_name);
	int32_t gpio_en = 0;

	param_get(handle, &gpio_en);

	if (gpio_en == 0) {
		// nothing configured
		request_stop();
		return;
	}

	_gpio_mask = (uint32_t)gpio_en;

	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; i++) {
		if (!(_gpio_mask & (1 << i))) {
			continue;
		}

		const bool initial_high = _mixing_output.isAboveMidpoint(i, _mixing_output.disarmedValue(i));
		uint32_t gpio = io_timer_channel_claim_as_gpio_output(i, IOTimerChanMode_GPIO, initial_high);

		if (gpio != 0) {
			_gpio_pin[i] = gpio;

		} else {
			PX4_WARN("%s: ch%d busy", _mixing_output.paramPrefix(), i + 1);
			_gpio_mask &= ~(1u << i);
		}
	}

	if (_gpio_mask == 0) {
		PX4_WARN("no channel could be claimed, stopping");
		request_stop();
		return;
	}

	_channels_claimed = true;

	// disable unused functions
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		if (((1 << i) & _gpio_mask) == 0) {
			_mixing_output.disableFunction(i);
		}
	}
}

bool GpioOut::updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_channels_claimed) {
		return false;
	}

	for (size_t i = 0; i < num_outputs; i++) {
		if (_gpio_pin[i]) {
			px4_arch_gpiowrite(_gpio_pin[i], _mixing_output.isAboveMidpoint(i, outputs[i]));
		}
	}

	return true;
}

void GpioOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

int GpioOut::task_spawn(int argc, char *argv[])
{
	GpioOut *instance = new GpioOut();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	// claim at task_spawn to be as early as possible
	instance->claim_gpio_channels();

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int GpioOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GpioOut::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();

	if (_channels_claimed) {
		PX4_INFO_RAW("GPIO channels:");

		for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
			if (_gpio_mask & (1 << i)) {
				PX4_INFO_RAW(" %u", i);
			}
		}

		PX4_INFO_RAW("\n");
	}

	return 0;
}

int GpioOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Claims the FMU channels selected via PWM_MAIN_GPIO_EN / PWM_AUX_GPIO_EN and drives
them as plain digital (on/off) outputs.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gpio_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gpio_out_main(int argc, char *argv[])
{
	return ModuleBase::main(GpioOut::desc, argc, argv);
}
