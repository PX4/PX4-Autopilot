/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
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

#include "PpmRc.hpp"

#include <string.h>

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

static constexpr hrt_abstime kLostTimeout = 1000000;

ModuleBase::Descriptor PpmRc::desc{task_spawn, custom_command, print_usage};

PpmRc::PpmRc() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
	updateParams();
}

PpmRc::~PpmRc()
{
#if defined(HRT_PPM_CHANNEL)
	px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif
	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int PpmRc::task_spawn(int argc, char *argv[])
{
#if !defined(HRT_PPM_CHANNEL)
	PX4_ERR("PPM not supported on this board");
	return PX4_ERROR;
#else

#ifdef RC_SERIAL_PORT_SHARED_PPM_PIN_GPIO_RX
	int32_t rc_prot = 0;

	if (param_get(param_find("SER_RC_PROTO"), &rc_prot) == PX4_OK && rc_prot != 0) {
		PX4_ERR("PPM shares the RC UART pin; set SER_RC_PROTO to Disabled");
		return PX4_ERROR;
	}

	px4_arch_unconfiggpio(RC_SERIAL_PORT_SHARED_PPM_PIN_GPIO_RX);
#endif

	PpmRc *instance = new PpmRc();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	px4_arch_configgpio(GPIO_PPM_IN);

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(_current_update_interval);
	return PX4_OK;
#endif
}

void PpmRc::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	_analog_rssi.update();

#if defined(HRT_PPM_CHANNEL)

	uint16_t values[PPM_MAX_CHANNELS];
	unsigned decoded;
	uint16_t frame_length;
	hrt_abstime last_decode;

	irqstate_t flags = px4_enter_critical_section();
	decoded = ppm_decoded_channels;
	last_decode = ppm_last_valid_decode;
	frame_length = ppm_frame_length;
	memcpy(values, ppm_buffer, sizeof(values));
	px4_leave_critical_section(flags);

	if ((last_decode != _timestamp_last_signal) && decoded > 3) {
		input_rc_s input_rc{};
		input_rc.timestamp_last_signal = last_decode;
		input_rc.channel_count = math::min(decoded,
						   math::min((unsigned)PPM_MAX_CHANNELS,
								   (unsigned)input_rc_s::RC_INPUT_MAX_CHANNELS));
		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
		input_rc.rssi = -1;
		input_rc.link_quality = -1;
		input_rc.rssi_dbm = NAN;
		input_rc.rc_ppm_frame_length = frame_length;

		unsigned valid_chans = 0;

		for (unsigned i = 0; i < input_rc.channel_count; i++) {
			input_rc.values[i] = values[i];

			if (values[i] != 0) {
				valid_chans++;
			}
		}

		if ((_param_rc_rssi_pwm_chan.get() > 0) && (_param_rc_rssi_pwm_chan.get() < input_rc.channel_count)) {
			const int32_t rssi_pwm_chan = _param_rc_rssi_pwm_chan.get();
			const int32_t rssi_pwm_min = _param_rc_rssi_pwm_min.get();
			const int32_t rssi_pwm_max = _param_rc_rssi_pwm_max.get();
			int rc_rssi = ((input_rc.values[rssi_pwm_chan - 1] - rssi_pwm_min) * 100) /
				      (rssi_pwm_max - rssi_pwm_min);
			input_rc.rssi = math::constrain(rc_rssi, 0, 100);
		}

		_analog_rssi.fill_missing(input_rc.rssi);

		if (valid_chans == 0) {
			input_rc.rssi = 0;
		}

		input_rc.rc_lost = (valid_chans == 0);
		input_rc.timestamp = hrt_absolute_time();
		_input_rc_pub.publish(input_rc);
		perf_count(_publish_interval_perf);
		_timestamp_last_signal = last_decode;

		if (valid_chans > 0 && !_locked) {
			_locked = true;
			PX4_INFO("RC input locked");
		}

	} else if (_locked && hrt_elapsed_time(&_timestamp_last_signal) > kLostTimeout) {
		_locked = false;
	}

#endif

	perf_end(_cycle_perf);
}

int PpmRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PpmRc::print_status()
{
#if defined(HRT_PPM_CHANNEL)
	PX4_INFO("RC state: %s", _locked ? "found" : "searching for signal");
#else
	PX4_INFO("PPM not supported on this board");
#endif
	_analog_rssi.print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);
	return 0;
}

int PpmRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PPM RC input from the FMU capture pin.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ppm_rc", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("radio_control");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ppm_rc_main(int argc, char *argv[])
{
	return ModuleBase::main(PpmRc::desc, argc, argv);
}
