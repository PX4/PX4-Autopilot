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

#include "linux_pwm_out.hpp"

#include <board_pwm_out.h>
#include <drivers/drv_hrt.h>

using namespace pwm_out;

LinuxPWMOut::LinuxPWMOut() :
	CDev("/dev/pwm_out"),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
}

LinuxPWMOut::~LinuxPWMOut()
{
	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	delete _pwm_out;
}

int LinuxPWMOut::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH);

	_mixing_output.setDriverInstance(_class_instance);

	_pwm_out = new BOARD_PWM_OUT_IMPL(MAX_ACTUATORS);

	ret = _pwm_out->init();

	if (ret != 0) {
		PX4_ERR("PWM output init failed");
		delete _pwm_out;
		_pwm_out = nullptr;
		return ret;
	}

	update_params();

	ScheduleNow();

	return ret;
}

int LinuxPWMOut::task_spawn(int argc, char *argv[])
{
	LinuxPWMOut *instance = new LinuxPWMOut();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool LinuxPWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				unsigned num_outputs, unsigned num_control_groups_updated)
{
	_pwm_out->send_output_pwm(outputs, num_outputs);
	return true;
}

void LinuxPWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
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
		update_params();
	}

	_mixing_output.updateSubscriptions(false);

	perf_end(_cycle_perf);
}

void LinuxPWMOut::update_params()
{
	updateParams();

	// skip update when armed or dynamic mixing enabled
	if (_mixing_output.armed().armed || _mixing_output.useDynamicMixing()) {
		return;
	}

	if (_mixing_output.mixers()) {
		int16_t values[MAX_ACTUATORS] {};

		for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
			sprintf(str, "%s_TRIM%u", prefix, i + 1);

			float pval = 0.0f;
			param_get(param_find(str), &pval);
			values[i] = roundf(10000 * pval);
		}

		// copy the trim values to the mixer offsets
		_mixing_output.mixers()->set_trims(values, MAX_ACTUATORS);
	}
}

int LinuxPWMOut::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			update_params();
			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

int LinuxPWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LinuxPWMOut::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();
	return 0;
}

int LinuxPWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Linux PWM output driver with board-specific backend implementation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("linux_pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int linux_pwm_out_main(int argc, char *argv[])
{
	return LinuxPWMOut::main(argc, argv);
}
