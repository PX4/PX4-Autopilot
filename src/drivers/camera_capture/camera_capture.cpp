/****************************************************************************
 *
 *   Copyright (c) 2018, 2021 PX4 Development Team. All rights reserved.
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
 * @file camera_capture.cpp
 *
 * Online and offline geotagging from camera feedback
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 */

#include "camera_capture.hpp"

#include <board_config.h>

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

namespace camera_capture
{
CameraCapture *g_camera_capture{nullptr};
}

struct work_s CameraCapture::_work_publisher;

CameraCapture::CameraCapture() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	memset(&_work_publisher, 0, sizeof(_work_publisher));

	// Capture Parameters
	_p_strobe_delay = param_find("CAM_CAP_DELAY");
	param_get(_p_strobe_delay, &_strobe_delay);

	_p_camera_capture_mode = param_find("CAM_CAP_MODE");
	param_get(_p_camera_capture_mode, &_camera_capture_mode);

	_p_camera_capture_edge = param_find("CAM_CAP_EDGE");
	param_get(_p_camera_capture_edge, &_camera_capture_edge);


	// get the capture channel from function configuration params
	param_t p_ctrl_alloc = param_find("SYS_CTRL_ALLOC");
	bool ctrl_alloc = false;

	if (p_ctrl_alloc != PARAM_INVALID) {
		param_get(p_ctrl_alloc, &ctrl_alloc);
	}

	if (ctrl_alloc) {
		_capture_channel = -1;

		for (unsigned i = 0; i < 16 && _capture_channel == -1; ++i) {
			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
			param_t function_handle = param_find(param_name);
			int32_t function;

			if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0) {
				if (function == 2032) { // Camera_Capture
					_capture_channel = i;
				}
			}
		}
	}

	_trigger_pub.advertise();
}

CameraCapture::~CameraCapture()
{
	camera_capture::g_camera_capture = nullptr;
}

void
CameraCapture::capture_callback(uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	_trigger.chan_index = chan_index;
	_trigger.hrt_edge_time = edge_time;
	_trigger.edge_state = edge_state;
	_trigger.overflow = overflow;

	work_queue(HPWORK, &_work_publisher, (worker_t)&CameraCapture::publish_trigger_trampoline, this, 0);
}

int
CameraCapture::gpio_interrupt_routine(int irq, void *context, void *arg)
{
	CameraCapture *dev = static_cast<CameraCapture *>(arg);

	dev->_trigger.chan_index = 0;
	dev->_trigger.hrt_edge_time = hrt_absolute_time();
	dev->_trigger.edge_state = 0;
	dev->_trigger.overflow = 0;

	work_queue(HPWORK, &_work_publisher, (worker_t)&CameraCapture::publish_trigger_trampoline, dev, 0);

	return PX4_OK;
}

void
CameraCapture::publish_trigger_trampoline(void *arg)
{
	CameraCapture *dev = static_cast<CameraCapture *>(arg);

	dev->publish_trigger();
}

void
CameraCapture::publish_trigger()
{
	bool publish = false;

	camera_trigger_s trigger{};

	// MODES 1 and 2 are not fully tested
	if (_camera_capture_mode == 0 || _gpio_capture) {
		trigger.timestamp = _trigger.hrt_edge_time - uint64_t(1000 * _strobe_delay);
		trigger.seq = _capture_seq++;
		_last_trig_time = trigger.timestamp;

		publish = true;

	} else if (_camera_capture_mode == 1) { // Get timestamp of mid-exposure (active high)
		if (_trigger.edge_state == 1) {
			_last_trig_begin_time = _trigger.hrt_edge_time - uint64_t(1000 * _strobe_delay);

		} else if (_trigger.edge_state == 0 && _last_trig_begin_time > 0) {
			trigger.timestamp = _trigger.hrt_edge_time - ((_trigger.hrt_edge_time - _last_trig_begin_time) / 2);
			trigger.seq = _capture_seq++;
			_last_exposure_time = _trigger.hrt_edge_time - _last_trig_begin_time;
			_last_trig_time = trigger.timestamp;
			publish = true;
			_capture_seq++;
		}

	} else { // Get timestamp of mid-exposure (active low)
		if (_trigger.edge_state == 0) {
			_last_trig_begin_time = _trigger.hrt_edge_time - uint64_t(1000 * _strobe_delay);

		} else if (_trigger.edge_state == 1 && _last_trig_begin_time > 0) {
			trigger.timestamp = _trigger.hrt_edge_time - ((_trigger.hrt_edge_time - _last_trig_begin_time) / 2);
			trigger.seq = _capture_seq++;
			_last_exposure_time = _trigger.hrt_edge_time - _last_trig_begin_time;
			_last_trig_time = trigger.timestamp;
			publish = true;
		}

	}

	trigger.feedback = true;
	_capture_overflows = _trigger.overflow;

	if (!publish) {
		return;
	}

	pps_capture_s pps_capture;

	if (_pps_capture_sub.update(&pps_capture)) {
		_pps_hrt_timestamp = pps_capture.timestamp;
		_pps_rtc_timestamp = pps_capture.rtc_timestamp;
	}


	if (_pps_hrt_timestamp > 0) {
		// Last PPS RTC time + elapsed time to the camera capture interrupt
		trigger.timestamp_utc = _pps_rtc_timestamp + (trigger.timestamp - _pps_hrt_timestamp);

	} else {
		// No PPS capture received, use RTC clock as fallback
		timespec tv{};
		px4_clock_gettime(CLOCK_REALTIME, &tv);
		trigger.timestamp_utc = ts_to_abstime(&tv) - hrt_elapsed_time(&trigger.timestamp);
	}

	_trigger_pub.publish(trigger);
}

void
CameraCapture::capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state,
				  uint32_t overflow)
{
	camera_capture::g_camera_capture->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
CameraCapture::Run()
{
	// Command handling
	vehicle_command_s cmd{};

	if (_command_sub.update(&cmd)) {

		// TODO : this should eventuallly be a capture control command
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {

			// Enable/disable signal capture
			if (commandParamToInt(cmd.param1) == 1) {
				set_capture_control(true);

			} else if (commandParamToInt(cmd.param1) == 0) {
				set_capture_control(false);

			}

			// Reset capture sequence
			if (commandParamToInt(cmd.param2) == 1) {
				reset_statistics(true);

			}

			// Acknowledge the command
			vehicle_command_ack_s command_ack{};

			command_ack.timestamp = hrt_absolute_time();
			command_ack.command = cmd.command;
			command_ack.result = (uint8_t)vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;
			command_ack.target_system = cmd.source_system;
			command_ack.target_component = cmd.source_component;

			_command_ack_pub.publish(command_ack);
		}
	}
}

void
CameraCapture::set_capture_control(bool enabled)
{
// a board can define BOARD_CAPTURE_GPIO to use a separate capture pin. It's used if no channel is configured
#if defined(BOARD_CAPTURE_GPIO)

	if (_capture_channel == -1) {
		px4_arch_gpiosetevent(BOARD_CAPTURE_GPIO, true, false, true, &CameraCapture::gpio_interrupt_routine, this);
		_capture_enabled = enabled;
		_gpio_capture = true;
		reset_statistics(false);
	}

#endif

	if (!_gpio_capture) {
		if (_capture_channel == -1) {
			PX4_WARN("No capture channel configured");
			_capture_enabled = false;

		} else {
			capture_callback_t callback = nullptr;
			void *context = nullptr;

			if (enabled) {
				callback = &CameraCapture::capture_trampoline;
				context = this;
			}

			int ret = up_input_capture_set_callback(_capture_channel, callback, context);

			if (ret == 0) {
				_capture_enabled = enabled;
				_gpio_capture = false;

			} else {
				PX4_ERR("Unable to set capture callback for chan %" PRIu8 " (%i)", _capture_channel, ret);
				_capture_enabled = false;
			}

			reset_statistics(false);
		}
	}

}

void
CameraCapture::reset_statistics(bool reset_seq)
{
	if (reset_seq) {
		_capture_seq = 0;
	}

	_last_trig_begin_time = 0;
	_last_exposure_time = 0;
	_last_trig_time = 0;
	_capture_overflows = 0;
}

int
CameraCapture::start()
{
	if (!_gpio_capture && _capture_channel != -1) {
		input_capture_edge edge = Both;

		if (_camera_capture_mode == 0) {
			edge = _camera_capture_edge ? Rising : Falling;
		}

		int ret = up_input_capture_set(_capture_channel, edge, 0, nullptr, nullptr);

		if (ret != 0) {
			PX4_ERR("up_input_capture_set failed (%i)", ret);
			return ret;
		}
	}

	// run every 100 ms (10 Hz)
	ScheduleOnInterval(100000, 10000);

	return PX4_OK;
}

void
CameraCapture::stop()
{
	ScheduleClear();

	work_cancel(HPWORK, &_work_publisher);

	if (camera_capture::g_camera_capture != nullptr) {
		delete (camera_capture::g_camera_capture);
	}
}

void
CameraCapture::status()
{
	PX4_INFO("Capture enabled : %s", _capture_enabled ? "YES" : "NO");
	PX4_INFO("Frame sequence : %" PRIu32, _capture_seq);

	if (_last_trig_time != 0) {
		PX4_INFO("Last trigger timestamp : %" PRIu64 " (%i ms ago)", _last_trig_time,
			 (int)(hrt_elapsed_time(&_last_trig_time) / 1000));

	} else {
		PX4_INFO("No trigger yet");
	}

	if (_camera_capture_mode != 0) {
		PX4_INFO("Last exposure time : %0.2f ms", double(_last_exposure_time) / 1000.0);
	}

	PX4_INFO("Number of overflows : %" PRIu32, _capture_overflows);

	if (_gpio_capture) {
		PX4_INFO("Using board GPIO pin");

	} else if (_capture_channel == -1) {
		PX4_INFO("No Capture channel configured");

	} else {
		input_capture_stats_t stats;
		int ret =  up_input_capture_get_stats(_capture_channel, &stats, false);

		if (ret != 0) {
			PX4_ERR("Unable to get stats for chan %" PRIu8 " (%i)", _capture_channel, ret);

		} else {
			PX4_INFO("Status chan: %" PRIu8 " edges: %" PRIu32 " last time: %" PRIu64 " last state: %" PRIu32
				 " overflows: %" PRIu32 " latency: %" PRIu16,
				 _capture_channel,
				 stats.edges,
				 stats.last_time,
				 stats.last_edge,
				 stats.overflows,
				 stats.latency);
		}
	}
}

static int usage()
{
	PX4_INFO("usage: camera_capture {start|stop|on|off|reset|status}\n");
	return 1;
}

extern "C" __EXPORT int camera_capture_main(int argc, char *argv[]);

int camera_capture_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_capture::g_camera_capture != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_capture::g_camera_capture = new CameraCapture();

		if (camera_capture::g_camera_capture == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (!camera_capture::g_camera_capture->start()) {
			return 0;

		} else {
			return 1;
		}

	}

	if (camera_capture::g_camera_capture == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_capture::g_camera_capture->stop();

	} else if (!strcmp(argv[1], "status")) {
		camera_capture::g_camera_capture->status();

	} else if (!strcmp(argv[1], "on")) {
		camera_capture::g_camera_capture->set_capture_control(true);

	} else if (!strcmp(argv[1], "off")) {
		camera_capture::g_camera_capture->set_capture_control(false);

	} else if (!strcmp(argv[1], "reset")) {
		camera_capture::g_camera_capture->set_capture_control(false);
		camera_capture::g_camera_capture->reset_statistics(true);

	} else {
		return usage();
	}

	return 0;
}
