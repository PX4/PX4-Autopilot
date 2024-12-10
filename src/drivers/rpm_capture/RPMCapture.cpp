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


#include "RPMCapture.hpp"
#include <px4_arch/io_timer.h>
#include <board_config.h>
#include <parameters/param.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>

RPMCapture::RPMCapture() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
	_pwm_input_pub.advertise();
	ScheduleNow();
}

RPMCapture::~RPMCapture()
{
	if (_channel >= 0) {
		io_timer_unallocate_channel(_channel);
		px4_arch_gpiosetevent(_rpm_capture_gpio, false, false, false, nullptr, nullptr);
	}
}

bool RPMCapture::init()
{
	bool success = false;

	_min_pulse_period_us = static_cast<uint32_t>(60.f * 1e6f / static_cast<float>(RPM_MAX_VALUE *
			       _param_rpm_puls_per_rev.get()));

	for (unsigned i = 0; i < 16; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
		param_t function_handle = param_find(param_name);
		int32_t function;

		if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
			if (function == 2070) { // RPM_Input
				_channel = i;
			}
		}
	}

	if (_channel == -1) {
		PX4_WARN("No RPM channel configured");
		return false;
	}

	int ret = io_timer_allocate_channel(_channel, IOTimerChanMode_Other); // TODO: add IOTimerChanMode_RPM

	if (ret != PX4_OK) {
		PX4_ERR("gpio alloc failed (%i) for RPM at channel (%d)", ret, _channel);
		return false;
	}

	_rpm_capture_gpio = PX4_MAKE_GPIO_EXTI(io_timer_channel_get_as_pwm_input(_channel));
	int ret_val = px4_arch_gpiosetevent(_rpm_capture_gpio, true, false, true, &RPMCapture::gpio_interrupt_callback, this);

	if (ret_val == PX4_OK) {
		success = true;
	}

	success = success && _rpm_pub.advertise();
	return success;
}

void RPMCapture::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	hrt_abstime now = hrt_absolute_time();

	if (_interrupt_happened.load()) {
		// There was an interrupt
		_period = _hrt_timestamp - _hrt_timestamp_prev;
		_hrt_timestamp_prev = _hrt_timestamp;
		_interrupt_happened.store(false);

		pwm_input_s pwm_input{};
		pwm_input.timestamp = now;
		pwm_input.period = _period;
		pwm_input.error_count = _error_count;
		_pwm_input_pub.publish(pwm_input);

		ScheduleClear(); // Do not run on previously scheduled timeout

	} else {
		// Timeout for no interrupts
		_period = UINT32_MAX;
	}

	ScheduleDelayed(RPM_PULSE_TIMEOUT); // Schule a new timeout

	if (_period > _min_pulse_period_us) {
		// Only update if the period is above the min pulse period threshold
		float rpm_raw{0.f};

		if (_period < RPM_PULSE_TIMEOUT) {
			// 1'000'000 / [us] -> pulses per second * 60 -> pulses per minute
			rpm_raw = 60.f * 1e6f / static_cast<float>(_param_rpm_puls_per_rev.get() * _period);
		}

		const float dt = math::constrain((now - _timestamp_last_update) * 1e-6f, 0.01f, 1.f);
		_timestamp_last_update = now;
		_rpm_filter.setParameters(dt, 0.5f);
		_rpm_filter.update(_rpm_median_filter.apply(rpm_raw));

		rpm_s rpm{};
		rpm.timestamp = now;
		rpm.rpm_raw = rpm_raw;
		rpm.rpm_estimate = _rpm_filter.getState();
		_rpm_pub.publish(rpm);
	}

}

int RPMCapture::gpio_interrupt_callback(int irq, void *context, void *arg)
{
	RPMCapture *instance = static_cast<RPMCapture *>(arg);

	if (instance->_interrupt_happened.load()) {
		++instance->_error_count;
	}

	instance->_hrt_timestamp = hrt_absolute_time();
	instance->_interrupt_happened.store(true);
	instance->ScheduleNow();

	return PX4_OK;
}

int RPMCapture::task_spawn(int argc, char *argv[])
{
	RPMCapture *instance = new RPMCapture();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int RPMCapture::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RPMCapture::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("rpm_capture", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void RPMCapture::stop()
{
	exit_and_cleanup();
}

extern "C" __EXPORT int rpm_capture_main(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[1], "stop") && RPMCapture::is_running()) {
		RPMCapture::stop();
	}

	return RPMCapture::main(argc, argv);
}
