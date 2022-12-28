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

#include "RCInput.hpp"

using namespace time_literals;

RcPpm::RcPpm() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// initialize raw_rc values and count
	for (unsigned i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; i++) {
		_raw_rc_values[i] = UINT16_MAX;
	}
}

RcPpm::~RcPpm()
{
	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int RcPpm::init()
{
#ifdef GPIO_PPM_IN
	// disable CPPM input by mapping it away from the timer capture input
	px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif // GPIO_PPM_IN

	return 0;
}

int RcPpm::task_spawn(int argc, char *argv[])
{
	RcPpm *instance = new RcPpm();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->ScheduleOnInterval(_current_update_interval);

	return PX4_OK;
}

void RcPpm::fill_rc_in(uint16_t raw_rc_count_local,
		       uint16_t raw_rc_values_local[input_rc_s::RC_INPUT_MAX_CHANNELS],
		       hrt_abstime now, bool frame_drop, bool failsafe,
		       unsigned frame_drops, int rssi = -1)
{
	// fill rc_in struct for publishing
	_rc_in.channel_count = raw_rc_count_local;

	if (_rc_in.channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_rc_in.channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	unsigned valid_chans = 0;

	for (unsigned i = 0; i < _rc_in.channel_count; i++) {
		_rc_in.values[i] = raw_rc_values_local[i];

		if (raw_rc_values_local[i] != UINT16_MAX) {
			valid_chans++;
		}

		// once filled, reset values back to default
		_raw_rc_values[i] = UINT16_MAX;
	}

	_rc_in.timestamp = now;
	_rc_in.timestamp_last_signal = _rc_in.timestamp;
	_rc_in.rc_ppm_frame_length = 0;

	/* fake rssi if no value was provided */
	if (rssi == -1) {
		if ((_param_rc_rssi_pwm_chan.get() > 0) && (_param_rc_rssi_pwm_chan.get() < _rc_in.channel_count)) {
			const int32_t rssi_pwm_chan = _param_rc_rssi_pwm_chan.get();
			const int32_t rssi_pwm_min = _param_rc_rssi_pwm_min.get();
			const int32_t rssi_pwm_max = _param_rc_rssi_pwm_max.get();

			// get RSSI from input channel
			int rc_rssi = ((_rc_in.values[rssi_pwm_chan - 1] - rssi_pwm_min) * 100) / (rssi_pwm_max - rssi_pwm_min);
			_rc_in.rssi = math::constrain(rc_rssi, 0, 100);

		} else if (_analog_rc_rssi_stable) {
			// set RSSI if analog RSSI input is present
			float rssi_analog = ((_analog_rc_rssi_volt - 0.2f) / 3.0f) * 100.0f;

			if (rssi_analog > 100.0f) {
				rssi_analog = 100.0f;
			}

			if (rssi_analog < 0.0f) {
				rssi_analog = 0.0f;
			}

			_rc_in.rssi = rssi_analog;

		} else {
			_rc_in.rssi = 255;
		}

	} else {
		_rc_in.rssi = rssi;
	}

	if (valid_chans == 0) {
		_rc_in.rssi = 0;
	}

	_rc_in.rc_failsafe = failsafe;
	_rc_in.rc_lost = (valid_chans == 0);
	_rc_in.rc_lost_frame_count = frame_drops;
	_rc_in.rc_total_frame_count = 0;
}

void RcPpm::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {

		perf_begin(_cycle_perf);

		const hrt_abstime cycle_timestamp = hrt_absolute_time();

#if defined(ADC_RC_RSSI_CHANNEL)

		// update ADC sampling
		if (_adc_report_sub.updated()) {
			adc_report_s adc;

			if (_adc_report_sub.copy(&adc)) {
				for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
					if (adc.channel_id[i] == ADC_RC_RSSI_CHANNEL) {
						float adc_volt = adc.raw_data[i] *
								 adc.v_ref /
								 adc.resolution;

						if (_analog_rc_rssi_volt < 0.0f) {
							_analog_rc_rssi_volt = adc_volt;
						}

						_analog_rc_rssi_volt = _analog_rc_rssi_volt * 0.995f + adc_volt * 0.005f;

						/* only allow this to be used if we see a high RSSI once */
						if (_analog_rc_rssi_volt > 2.5f) {
							_analog_rc_rssi_stable = true;
						}
					}
				}
			}
		}

#endif // ADC_RC_RSSI_CHANNEL

		bool rc_updated = false;

		// This block scans for a supported serial RC input and locks onto the first one found
		// Scan for 500 msec, then switch protocol
		constexpr hrt_abstime rc_scan_max = 500_ms;

		const bool rc_scan_locked = _rc_scan_locked;

		if (_rc_scan_begin == 0) {
			_rc_scan_begin = cycle_timestamp;
			// Configure timer input pin for CPPM
			px4_arch_configgpio(GPIO_PPM_IN);

		} else if (_rc_scan_locked || cycle_timestamp - _rc_scan_begin < rc_scan_max) {

			// see if we have new PPM input data
			if ((ppm_last_valid_decode != _rc_in.timestamp_last_signal) && ppm_decoded_channels > 3) {
				// we have a new PPM frame. Publish it.
				rc_updated = true;
				_rc_in.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
				fill_rc_in(ppm_decoded_channels, ppm_buffer, cycle_timestamp, false, false, 0);
				_rc_scan_locked = true;
				_rc_in.rc_ppm_frame_length = ppm_frame_length;
				_rc_in.timestamp_last_signal = ppm_last_valid_decode;
			}

		} else {
			// disable CPPM input by mapping it away from the timer capture input
			px4_arch_unconfiggpio(GPIO_PPM_IN);
		}

		perf_end(_cycle_perf);

		if (rc_updated) {
			perf_count(_publish_interval_perf);

			_to_input_rc.publish(_rc_in);

		} else if (!rc_updated && (hrt_elapsed_time(&_rc_in.timestamp_last_signal) > 1_s)) {
			_rc_scan_locked = false;
		}

		if (!rc_scan_locked && _rc_scan_locked) {
			PX4_INFO("RC input locked");
		}
	}
}

int RcPpm::custom_command(int argc, char *argv[])
{
	/* start the FMU if not running */
	if (!is_running()) {
		int ret = RcPpm::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int RcPpm::print_status()
{
	PX4_INFO("Max update rate: %u Hz", 1000000 / _current_update_interval);

	PX4_INFO("RC state: %s", _rc_scan_locked ? "found" : "searching for signal");

#if ADC_RC_RSSI_CHANNEL

	if (_analog_rc_rssi_stable) {
		PX4_INFO("vrssi: %dmV", (int)(_analog_rc_rssi_volt * 1000.0f));
	}

#endif

	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);

	if (hrt_elapsed_time(&_rc_in.timestamp) < 1_s) {
		print_message(ORB_ID(input_rc), _rc_in);
	}

	return 0;
}

int
RcPpm::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module does the RC input parsing and auto-selecting the method. Supported methods are:
- PPM

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ppm_in", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ppm_in_main(int argc, char *argv[])
{
	return RcPpm::main(argc, argv);
}
