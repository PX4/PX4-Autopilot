/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file camera_trigger.cpp
 *
 * External camera-IMU synchronisation and triggering, and support for
 * camera manipulation using PWM signals over FMU auxillary pins.
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 * @author Kelly Steich <kelly.steich@wingtra.com>
 * @author Andreas Bircher <andreas@wingtra.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <px4_workqueue.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

#include <drivers/drv_hrt.h>

#include "interfaces/src/camera_interface.h"
#include "interfaces/src/gpio.h"
#include "interfaces/src/pwm.h"
#include "interfaces/src/seagull_map2.h"

extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[]);

typedef enum : uint8_t {
	CAMERA_INTERFACE_MODE_NONE = 0,
	CAMERA_INTERFACE_MODE_GPIO,
	CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM,
	CAMERA_INTERFACE_MODE_MAVLINK,
	CAMERA_INTERFACE_MODE_GENERIC_PWM
} camera_interface_mode_t;

typedef enum : uint8_t {
	TRIGGER_MODE_NONE = 0,
	TRIGGER_MODE_INTERVAL_ON_CMD,
	TRIGGER_MODE_INTERVAL_ALWAYS_ON,
	TRIGGER_MODE_DISTANCE_ALWAYS_ON,
	TRIGGER_MODE_DISTANCE_ON_CMD
} trigger_mode_t;

#define commandParamToInt(n) static_cast<int>(n >= 0 ? n + 0.5f : n - 0.5f)

class CameraTrigger
{
public:
	/**
	 * Constructor
	 */
	CameraTrigger();

	/**
	 * Destructor, also kills task.
	 */
	~CameraTrigger();

	/**
	 * Run intervalometer update
	 */
	void		update_intervalometer();

	/**
	 * Run distance-based trigger update
	 */
	void		update_distance();

	/**
	 * Trigger the camera just once
	 */
	void		shoot_once();

	/**
	 * Toggle keep camera alive functionality
	 */
	void		enable_keep_alive(bool on);

	/**
	 * Toggle camera power (on/off)
	 */
	void        toggle_power();

	/**
	 * Start the task.
	 */
	void		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * Display status.
	 */
	void		status();

	/**
	 * Trigger one image
	 */
	void		test();

private:

	struct hrt_call		_engagecall;
	struct hrt_call		_disengagecall;
	struct hrt_call     _engage_turn_on_off_call;
	struct hrt_call     _disengage_turn_on_off_call;
	struct hrt_call		_keepalivecall_up;
	struct hrt_call		_keepalivecall_down;

	static struct work_s	_work;

	float			_activation_time;
	float			_interval;
	float 			_distance;
	uint32_t 		_trigger_seq;
	bool			_trigger_enabled;
	bool			_trigger_paused;
	bool			_one_shot;
	bool			_test_shot;
	bool 			_turning_on;
	math::Vector<2>		_last_shoot_position;
	bool			_valid_position;

	int			_command_sub;
	int			_lpos_sub;

	orb_advert_t		_trigger_pub;
	orb_advert_t		_cmd_ack_pub;

	param_t			_p_mode;
	param_t			_p_activation_time;
	param_t			_p_interval;
	param_t			_p_distance;
	param_t			_p_interface;

	trigger_mode_t		_trigger_mode;

	camera_interface_mode_t	_camera_interface_mode;
	CameraInterface		*_camera_interface;  ///< instance of camera interface

	/**
	 * Vehicle command handler
	 */
	static void	cycle_trampoline(void *arg);
	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);
	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);
	/**
	 * Fires on/off
	 */
	static void engange_turn_on_off(void *arg);
	/**
	 * Resets  on/off
	 */
	static void disengage_turn_on_off(void *arg);
	/**
	 * Enables keep alive signal
	 */
	static void	keep_alive_up(void *arg);
	/**
	 * Disables keep alive signal
	 */
	static void	keep_alive_down(void *arg);

};

struct work_s CameraTrigger::_work;

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	_engagecall {},
	_disengagecall {},
	_engage_turn_on_off_call {},
	_disengage_turn_on_off_call {},
	_keepalivecall_up {},
	_keepalivecall_down {},
	_activation_time(0.5f /* ms */),
	_interval(100.0f /* ms */),
	_distance(25.0f /* m */),
	_trigger_seq(0),
	_trigger_enabled(false),
	_trigger_paused(false),
	_one_shot(false),
	_test_shot(false),
	_turning_on(false),
	_last_shoot_position(0.0f, 0.0f),
	_valid_position(false),
	_command_sub(-1),
	_lpos_sub(-1),
	_trigger_pub(nullptr),
	_cmd_ack_pub(nullptr),
	_trigger_mode(TRIGGER_MODE_NONE),
	_camera_interface_mode(CAMERA_INTERFACE_MODE_GPIO),
	_camera_interface(nullptr)
{
	// Initiate camera interface based on camera_interface_mode
	if (_camera_interface != nullptr) {
		delete (_camera_interface);
		// set to zero to ensure parser is not used while not instantiated
		_camera_interface = nullptr;
	}

	memset(&_work, 0, sizeof(_work));

	// Parameters
	_p_interval = param_find("TRIG_INTERVAL");
	_p_distance = param_find("TRIG_DISTANCE");
	_p_activation_time = param_find("TRIG_ACT_TIME");
	_p_mode = param_find("TRIG_MODE");
	_p_interface = param_find("TRIG_INTERFACE");

	param_get(_p_activation_time, &_activation_time);
	param_get(_p_interval, &_interval);
	param_get(_p_distance, &_distance);
	param_get(_p_mode, &_trigger_mode);
	param_get(_p_interface, &_camera_interface_mode);

	switch (_camera_interface_mode) {
#ifdef __PX4_NUTTX

	case CAMERA_INTERFACE_MODE_GPIO:
		_camera_interface = new CameraInterfaceGPIO();
		break;

	case CAMERA_INTERFACE_MODE_GENERIC_PWM:
		_camera_interface = new CameraInterfacePWM();
		break;

	case CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM:
		_camera_interface = new CameraInterfaceSeagull();
		break;

#endif

	case CAMERA_INTERFACE_MODE_MAVLINK:
		// start an interface that does nothing. Instead mavlink will listen to the camera_trigger uORB message
		_camera_interface = new CameraInterface();
		break;

	default:
		PX4_ERR("unknown camera interface mode: %i", (int)_camera_interface_mode);
		break;
	}

	// Enforce a lower bound on the activation interval in PWM modes to not miss
	// engage calls in-between 50Hz PWM pulses. (see PX4 PR #6973)
	if ((_activation_time < 40.0f) &&
	    (_camera_interface_mode == CAMERA_INTERFACE_MODE_GENERIC_PWM ||
	     _camera_interface_mode == CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM)) {
		_activation_time = 40.0f;
		PX4_WARN("Trigger interval too low for PWM interface, setting to 40 ms");
		param_set(_p_activation_time, &(_activation_time));
	}

	// Advertise critical publishers here, because we cannot advertise in interrupt context
	struct camera_trigger_s trigger = {};
	_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &trigger);

}

CameraTrigger::~CameraTrigger()
{
	delete (_camera_interface);

	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::update_intervalometer()
{

	// the actual intervalometer runs in interrupt context, so we only need to call
	// control_intervalometer once on enabling/disabling trigger to schedule the calls.

	if (_trigger_enabled && !_trigger_paused) {
		// schedule trigger on and off calls
		hrt_call_every(&_engagecall, 0, (_interval * 1000),
			       (hrt_callout)&CameraTrigger::engage, this);

		// schedule trigger on and off calls
		hrt_call_every(&_disengagecall, 0 + (_activation_time * 1000), (_interval * 1000),
			       (hrt_callout)&CameraTrigger::disengage, this);

	}

}

void
CameraTrigger::update_distance()
{

	if (_lpos_sub < 0) {
		_lpos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	}

	if (_turning_on) {
		return;
	}

	if (_trigger_enabled) {

		struct vehicle_local_position_s local = {};
		orb_copy(ORB_ID(vehicle_local_position), _lpos_sub, &local);

		if (local.xy_valid) {

			// Initialize position if not done yet
			math::Vector<2> current_position(local.x, local.y);

			if (!_valid_position) {
				// First time valid position, take first shot
				_last_shoot_position = current_position;
				_valid_position = local.xy_valid;
				shoot_once();
			}

			// Check that distance threshold is exceeded
			if ((_last_shoot_position - current_position).length() >= _distance) {
				shoot_once();
				_last_shoot_position = current_position;

			}
		}
	}
}

void
CameraTrigger::enable_keep_alive(bool on)
{
	if (on) {
		// schedule keep-alive up and down calls
		hrt_call_every(&_keepalivecall_up, 0, (60000 * 1000),
			       (hrt_callout)&CameraTrigger::keep_alive_up, this);

		hrt_call_every(&_keepalivecall_down, 0 + (30000 * 1000), (60000 * 1000),
			       (hrt_callout)&CameraTrigger::keep_alive_down, this);

	} else {
		// cancel all calls
		hrt_cancel(&_keepalivecall_up);
		hrt_cancel(&_keepalivecall_down);
	}

}

void
CameraTrigger::toggle_power()
{

	// schedule power toggle calls
	hrt_call_after(&_engage_turn_on_off_call, 0,
		       (hrt_callout)&CameraTrigger::engange_turn_on_off, this);

	hrt_call_after(&_disengage_turn_on_off_call, 0 + (200 * 1000),
		       (hrt_callout)&CameraTrigger::disengage_turn_on_off, this);
}

void
CameraTrigger::shoot_once()
{
	if (!_trigger_paused) {

		// schedule trigger on and off calls
		hrt_call_after(&_engagecall, 0,
			       (hrt_callout)&CameraTrigger::engage, this);

		hrt_call_after(&_disengagecall, 0 + (_activation_time * 1000),
			       (hrt_callout)&CameraTrigger::disengage, this);
	}

}

void
CameraTrigger::start()
{

	if ((_trigger_mode == TRIGGER_MODE_INTERVAL_ALWAYS_ON ||
	     _trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) &&
	    _camera_interface->has_power_control() &&
	    !_camera_interface->is_powered_on()) {

		// If in always-on mode and the interface supports it, enable power to the camera
		toggle_power();
		enable_keep_alive(true);

	} else {
		enable_keep_alive(false);
	}

	// enable immediately if configured that way
	if (_trigger_mode == TRIGGER_MODE_INTERVAL_ALWAYS_ON) {
		// enable and start triggering
		_trigger_enabled = true;
		update_intervalometer();

	} else if (_trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) {
		// just enable, but do not fire. actual trigger is based on distance covered
		_trigger_enabled = true;
	}

	// start to monitor at high rate for trigger enable command
	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline, this, USEC2TICK(1));

}

void
CameraTrigger::stop()
{
	work_cancel(LPWORK, &_work);
	hrt_cancel(&_engagecall);
	hrt_cancel(&_disengagecall);
	hrt_cancel(&_engage_turn_on_off_call);
	hrt_cancel(&_disengage_turn_on_off_call);
	hrt_cancel(&_keepalivecall_up);
	hrt_cancel(&_keepalivecall_down);

	if (camera_trigger::g_camera_trigger != nullptr) {
		delete (camera_trigger::g_camera_trigger);
	}
}

void
CameraTrigger::test()
{
	struct vehicle_command_s cmd = {};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
	cmd.param5 = 1.0f;

	orb_advert_t pub;
	pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	(void)orb_unadvertise(pub);
}

void
CameraTrigger::cycle_trampoline(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	// default loop polling interval
	int poll_interval_usec = 5000;

	if (trig->_command_sub < 0) {
		trig->_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	bool updated = false;
	orb_check(trig->_command_sub, &updated);

	struct vehicle_command_s cmd = {};
	unsigned cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
	bool need_ack = false;

	// this flag is set when the polling loop is slowed down to allow the camera to power on
	trig->_turning_on = false;

	// these flags are used to detect state changes in the command loop
	bool main_state = trig->_trigger_enabled;
	bool pause_state = trig->_trigger_paused;

	// Command handling
	if (updated) {

		orb_copy(ORB_ID(vehicle_command), trig->_command_sub, &cmd);

		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL) {

			need_ack = true;

			if (commandParamToInt(cmd.param7) == 1) {
				// test shots are not logged or forwarded to GCS for geotagging
				trig->_test_shot = true;

			}

			if (commandParamToInt((float)cmd.param5) == 1) {
				// Schedule shot
				trig->_one_shot = true;

			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {

			need_ack = true;

			if (commandParamToInt(cmd.param3) == 1) {
				// pause triggger
				trig->_trigger_paused = true;

			} else if (commandParamToInt(cmd.param3) == 0) {
				trig->_trigger_paused = false;

			}

			if (commandParamToInt(cmd.param2) == 1) {
				// reset trigger sequence
				trig->_trigger_seq = 0;

			}

			if (commandParamToInt(cmd.param1) == 1) {
				trig->_trigger_enabled = true;

			} else if (commandParamToInt(cmd.param1) == 0) {
				trig->_trigger_enabled = false;

			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST) {

			need_ack = true;

			/*
			 * TRANSITIONAL SUPPORT ADDED AS OF 11th MAY 2017 (v1.6 RELEASE)
			*/

			if (cmd.param1 > 0.0f) {
				trig->_distance = cmd.param1;
				param_set(trig->_p_distance, &(trig->_distance));

				trig->_trigger_enabled = true;
				trig->_trigger_paused = false;

			} else if (commandParamToInt(cmd.param1) == 0) {
				trig->_trigger_paused = true;

			} else if (commandParamToInt(cmd.param1) == -1) {
				trig->_trigger_enabled = false;
			}

			// We can only control the shutter integration time of the camera in GPIO mode (for now)
			if (cmd.param2 > 0.0f) {
				if (trig->_camera_interface_mode == CAMERA_INTERFACE_MODE_GPIO) {
					trig->_activation_time = cmd.param2;
					param_set(trig->_p_activation_time, &(trig->_activation_time));
				}
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL) {

			need_ack = true;

			if (cmd.param1 > 0.0f) {
				trig->_interval = cmd.param1;
				param_set(trig->_p_interval, &(trig->_interval));
			}

			// We can only control the shutter integration time of the camera in GPIO mode
			if (cmd.param2 > 0.0f) {
				if (trig->_camera_interface_mode == CAMERA_INTERFACE_MODE_GPIO) {
					trig->_activation_time = cmd.param2;
					param_set(trig->_p_activation_time, &(trig->_activation_time));
				}
			}

			cmd_result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

		}

	}

	// State change handling
	if ((main_state != trig->_trigger_enabled) ||
	    (pause_state != trig->_trigger_paused) ||
	    trig->_one_shot) {

		if (trig->_trigger_enabled || trig->_one_shot) { // Just got enabled via a command

			// If camera isn't already powered on, we enable power to it
			if (!trig->_camera_interface->is_powered_on() &&
			    trig->_camera_interface->has_power_control()) {

				trig->toggle_power();
				trig->enable_keep_alive(true);

				// Give the camera time to turn on before starting to send trigger signals
				poll_interval_usec = 3000000;
				trig->_turning_on = true;
			}

		} else if (!trig->_trigger_enabled || trig->_trigger_paused) { // Just got disabled/paused via a command

			// Power off the camera if we are disabled
			if (trig->_camera_interface->is_powered_on() &&
			    trig->_camera_interface->has_power_control() &&
			    !trig->_trigger_enabled) {

				trig->enable_keep_alive(false);
				trig->toggle_power();
			}

			// cancel all calls for both disabled and paused
			hrt_cancel(&trig->_engagecall);
			hrt_cancel(&trig->_disengagecall);

			// ensure that the pin is off
			hrt_call_after(&trig->_disengagecall, 0,
				       (hrt_callout)&CameraTrigger::disengage, trig);

			// reset distance counter if needed
			if (trig->_trigger_mode == TRIGGER_MODE_DISTANCE_ON_CMD ||
			    trig->_trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) {

				// this will force distance counter reinit on getting enabled/unpaused
				trig->_valid_position = false;

			}

		}

		// only run on state changes, not every loop iteration
		if (trig->_trigger_mode == TRIGGER_MODE_INTERVAL_ON_CMD) {

			// update intervalometer state and reset calls
			trig->update_intervalometer();

		}

	}

	// run every loop iteration and trigger if needed
	if (trig->_trigger_mode == TRIGGER_MODE_DISTANCE_ON_CMD ||
	    trig->_trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON) {

		// update distance counter and trigger
		trig->update_distance();

	}

	// One shot command-based capture handling
	if (trig->_one_shot && !trig->_turning_on) {

		// One-shot trigger
		trig->shoot_once();
		trig->_one_shot = false;

		if (trig->_test_shot) {
			trig->_test_shot = false;
		}

	}

	// Command ACK handling
	if (updated && need_ack) {
		vehicle_command_ack_s command_ack = {};

		command_ack.command = cmd.command;
		command_ack.result = cmd_result;

		if (trig->_cmd_ack_pub == nullptr) {
			trig->_cmd_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
					     vehicle_command_ack_s::ORB_QUEUE_LENGTH);

		} else {
			orb_publish(ORB_ID(vehicle_command_ack), trig->_cmd_ack_pub, &command_ack);

		}
	}

	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline,
		   camera_trigger::g_camera_trigger, USEC2TICK(poll_interval_usec));
}

void
CameraTrigger::engage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	// Trigger the camera
	trig->_camera_interface->trigger(true);

	if (trig->_test_shot) {
		// do not send messages or increment frame count for test shots
		return;
	}

	// Send camera trigger message. This messages indicates that we sent
	// the camera trigger request. Does not guarantee capture.

	struct camera_trigger_s	trigger = {};

	// Set timestamp the instant after the trigger goes off
	trigger.timestamp = hrt_absolute_time();

	timespec tv = {};
	px4_clock_gettime(CLOCK_REALTIME, &tv);
	trigger.timestamp_utc = (uint64_t) tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

	trigger.seq = trig->_trigger_seq;

	orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &trigger);

	// increment frame count
	trig->_trigger_seq++;

}

void
CameraTrigger::disengage(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->trigger(false);
}

void
CameraTrigger::engange_turn_on_off(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_toggle_power(true);
}

void
CameraTrigger::disengage_turn_on_off(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_toggle_power(false);
}

void
CameraTrigger::keep_alive_up(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_keep_alive(true);
}

void
CameraTrigger::keep_alive_down(void *arg)
{
	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	trig->_camera_interface->send_keep_alive(false);
}

void
CameraTrigger::status()
{
	PX4_INFO("main state : %s", _trigger_enabled ? "enabled" : "disabled");
	PX4_INFO("pause state : %s", _trigger_paused ? "paused" : "active");
	PX4_INFO("mode : %i", _trigger_mode);

	if (_trigger_mode == TRIGGER_MODE_INTERVAL_ALWAYS_ON ||
	    _trigger_mode == TRIGGER_MODE_INTERVAL_ON_CMD) {
		PX4_INFO("interval : %.2f [ms]", (double)_interval);

	} else if (_trigger_mode == TRIGGER_MODE_DISTANCE_ALWAYS_ON ||
		   _trigger_mode == TRIGGER_MODE_DISTANCE_ON_CMD) {
		PX4_INFO("distance : %.2f [m]", (double)_distance);
	}

	if (_camera_interface->has_power_control())	{
		PX4_INFO("camera power : %s", _camera_interface->is_powered_on() ? "ON" : "OFF");
	}

	PX4_INFO("activation time : %.2f [ms]", (double)_activation_time);
	_camera_interface->info();
}

static int usage()
{
	PX4_INFO("usage: camera_trigger {start|stop|status|test|test_power}\n");
	return 1;
}

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		return usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		camera_trigger::g_camera_trigger = new CameraTrigger();

		if (camera_trigger::g_camera_trigger == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		camera_trigger::g_camera_trigger->start();
		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		PX4_WARN("not running");
		return 1;

	} else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop();

	} else if (!strcmp(argv[1], "status")) {
		camera_trigger::g_camera_trigger->status();

	} else if (!strcmp(argv[1], "test")) {
		camera_trigger::g_camera_trigger->test();

	} else if (!strcmp(argv[1], "test_power")) {
		camera_trigger::g_camera_trigger->toggle_power();

	} else {
		return usage();
	}

	return 0;
}

