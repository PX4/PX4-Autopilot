/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

namespace camera_capture
{
CameraCapture	*g_camera_capture;
}

CameraCapture::CameraCapture() :
	_capture_enabled(false),
	_trigger_pub(nullptr),
	_command_ack_pub(nullptr),
	_command_sub(-1),
	_trig_buffer(nullptr),
	_camera_capture_mode(0),
	_camera_capture_edge(0),
	_capture_seq(0),
	_last_fall_time(0),
	_last_exposure_time(0),
	_capture_overflows(0)
{

	memset(&_work, 0, sizeof(_work));

	// Parameters
	_p_strobe_delay = param_find("CAM_CAP_DELAY");
	param_get(_p_strobe_delay, &_strobe_delay);

	_p_camera_capture_mode = param_find("CAM_CAP_MODE");
	param_get(_p_camera_capture_mode, &_camera_capture_mode);

	_p_camera_capture_edge = param_find("CAM_CAP_EDGE");
	param_get(_p_camera_capture_edge, &_camera_capture_edge);

}

CameraCapture::~CameraCapture()
{
	/* free any existing reports */
	if (_trig_buffer != nullptr) {
		delete _trig_buffer;
	}

	camera_capture::g_camera_capture = nullptr;
}

void
CameraCapture::capture_callback(uint32_t chan_index,
				hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{

	struct _trig_s trigger;

	trigger.chan_index = chan_index;
	trigger.edge_time = edge_time;
	trigger.edge_state = edge_state;
	trigger.overflow = overflow;

	/* post message to the ring */
	_trig_buffer->put(&trigger);

	work_queue(LPWORK, &_work, (worker_t)&CameraCapture::publish_trigger_trampoline, this, 0);
}

void
CameraCapture::publish_trigger_trampoline(void *arg)
{
	CameraCapture *dev = reinterpret_cast<CameraCapture *>(arg);

	dev->publish_trigger();
}

void
CameraCapture::publish_trigger()
{
	struct _trig_s trig;

	_trig_buffer->get(&trig);

	if (_last_fall_time > 0) {

		struct camera_trigger_s	trigger {};

		if (_camera_capture_mode == 0) {
			trigger.timestamp = trig.edge_time;

		} else {
			trigger.timestamp = trig.edge_time - ((trig.edge_time - _last_fall_time) / 2);	// Get timestamp of mid-exposure
		}

		trigger.seq = _capture_seq++;
		trigger.feedback = true;

		if (_trigger_pub == nullptr) {

			_trigger_pub = orb_advertise(ORB_ID(camera_trigger_feedback), &trigger);

		} else {

			orb_publish(ORB_ID(camera_trigger_feedback), _trigger_pub, &trigger);
		}

		_last_exposure_time = trig.edge_time - _last_fall_time;
	}

	// Timestamp and compensate for strobe delay
	_last_fall_time = trig.edge_time - uint64_t(1000 * _strobe_delay);

	_capture_overflows = trig.overflow;
}

void
CameraCapture::capture_trampoline(void *context, uint32_t chan_index,
				  hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	camera_capture::g_camera_capture->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void
CameraCapture::cycle_trampoline(void *arg)
{
	CameraCapture *dev = reinterpret_cast<CameraCapture *>(arg);

	dev->cycle();
}

void
CameraCapture::cycle()
{

	if (_command_sub < 0) {
		_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	bool updated = false;
	orb_check(_command_sub, &updated);

	// Command handling
	if (updated) {

		vehicle_command_s cmd;
		orb_copy(ORB_ID(vehicle_command), _command_sub, &cmd);

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
			vehicle_command_ack_s command_ack = {
				.timestamp = 0,
				.result_param2 = 0,
				.command = cmd.command,
				.result = (uint8_t)vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED,
				.from_external = false,
				.result_param1 = 0,
				.target_system = cmd.source_system,
				.target_component = cmd.source_component
			};

			if (_command_ack_pub == nullptr) {
				_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
								       vehicle_command_ack_s::ORB_QUEUE_LENGTH);

			} else {
				orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);

			}
		}

	}

	work_queue(LPWORK, &_work, (worker_t)&CameraCapture::cycle_trampoline, camera_capture::g_camera_capture,
		   USEC2TICK(100000)); // 100ms
}

void
CameraCapture::set_capture_control(bool enabled)
{
	int fd = -1;
	fd = ::open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd < 0) {
		PX4_ERR("open fail");
		return;
	}

	input_capture_config_t conf;
	conf.channel = 5; // FMU chan 6
	conf.filter = 0;
	conf.edge = _camera_capture_edge ? Rising : Falling;
	conf.callback = NULL;
	conf.context = NULL;

	if (enabled) {

		conf.callback = &CameraCapture::capture_trampoline;
		conf.context = this;

		unsigned int capture_count = 0;

		if (::ioctl(fd, INPUT_CAP_GET_COUNT, (unsigned long)&capture_count) != 0) {
			PX4_INFO("Not in a capture mode");
			unsigned long mode = PWM_SERVO_MODE_5PWM1CAP;

			if (::ioctl(fd, PWM_SERVO_SET_MODE, mode) == 0) {
				PX4_INFO("Mode changed to 5PWM1CAP");

			} else {
				PX4_ERR("Mode NOT changed to 4PWM2CAP");
				goto err_out;
			}
		}
	}

	if (::ioctl(fd, INPUT_CAP_SET_CALLBACK, (unsigned long)&conf) == 0) {
		_capture_enabled = enabled;

	} else {
		PX4_ERR("Unable to set capture callback for chan %u\n", conf.channel);
		_capture_enabled = false;
		goto err_out;
	}

	reset_statistics(false);
err_out:
	::close(fd);
	return;
}

void
CameraCapture::reset_statistics(bool reset_seq)
{
	if (reset_seq) { _capture_seq = 0; }

	_last_fall_time = 0;
	_last_exposure_time = 0;
	_capture_overflows = 0;
}

int
CameraCapture::start()
{
	/* allocate basic report buffers */
	_trig_buffer = new ringbuffer::RingBuffer(2, sizeof(_trig_s));

	if (_trig_buffer == nullptr) {
		return PX4_ERROR;
	}

	// start to monitor at low rates for capture control commands
	work_queue(LPWORK, &_work, (worker_t)&CameraCapture::cycle_trampoline, this,
		   USEC2TICK(1)); // TODO : is this low rate??!

	return PX4_OK;
}

void
CameraCapture::stop()
{

	work_cancel(LPWORK, &_work);

	if (camera_capture::g_camera_capture != nullptr) {
		delete (camera_capture::g_camera_capture);
	}
}

void
CameraCapture::status()
{
	PX4_INFO("Capture enabled : %s", _capture_enabled ? "YES" : "NO");
	PX4_INFO("Frame sequence : %u", _capture_seq);
	PX4_INFO("Last fall timestamp : %llu", _last_fall_time);
	PX4_INFO("Last exposure time : %0.2f ms", double(_last_exposure_time) / 1000.0);
	PX4_INFO("Number of overflows : %u", _capture_overflows);
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
