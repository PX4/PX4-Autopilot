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
#include <mathlib/mathlib.h>
#include <px4_workqueue.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>

#include "interfaces/src/camera_interface.h"
#include "interfaces/src/gpio.h"
#include "interfaces/src/pwm.h"
#include "interfaces/src/seagull_map2.h"

#define TRIGGER_PIN_DEFAULT 1

extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[]);

typedef enum : int32_t {
	CAMERA_INTERFACE_MODE_NONE = 0,
	CAMERA_INTERFACE_MODE_GPIO,
	CAMERA_INTERFACE_MODE_SEAGULL_MAP2_PWM,
	CAMERA_INTERFACE_MODE_MAVLINK,
	CAMERA_INTERFACE_MODE_GENERIC_PWM
} camera_interface_mode_t;

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
	 * Set the trigger on / off
	 */
	void		control(bool on);

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

	int			_mode;
	float			_activation_time;
	float			_interval;
	float 			_distance;
	uint32_t 		_trigger_seq;
	bool			_trigger_enabled;
	math::Vector<2>		_last_shoot_position;
	bool			_valid_position;

	int			_vcommand_sub;
	int			_vlposition_sub;

	orb_advert_t		_trigger_pub;

	param_t			_p_mode;
	param_t			_p_activation_time;
	param_t			_p_interval;
	param_t			_p_distance;
	param_t			_p_interface;

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
	 * Fires trigger
	 */
	static void	keep_alive_up(void *arg);
	/**
	 * Resets trigger
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
	_mode(0),
	_activation_time(0.5f /* ms */),
	_interval(100.0f /* ms */),
	_distance(25.0f /* m */),
	_trigger_seq(0),
	_trigger_enabled(false),
	_last_shoot_position(0.0f, 0.0f),
	_valid_position(false),
	_vcommand_sub(-1),
	_vlposition_sub(-1),
	_trigger_pub(nullptr),
	_camera_interface_mode(CAMERA_INTERFACE_MODE_GPIO),
	_camera_interface(nullptr)
{
	//Initiate Camera interface basedon camera_interface_mode
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
	param_get(_p_mode, &_mode);
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

	struct camera_trigger_s	report = {};

	_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &report);
}

CameraTrigger::~CameraTrigger()
{
	delete (_camera_interface);

	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::control(bool on)
{
	// always execute, even if already on
	// to reset timings if necessary

	if (on) {
		// schedule trigger on and off calls
		hrt_call_every(&_engagecall, 0, (_interval * 1000),
			       (hrt_callout)&CameraTrigger::engage, this);

		// schedule trigger on and off calls
		hrt_call_every(&_disengagecall, 0 + (_activation_time * 1000), (_interval * 1000),
			       (hrt_callout)&CameraTrigger::disengage, this);

	} else {
		// cancel all calls
		hrt_cancel(&_engagecall);
		hrt_cancel(&_disengagecall);
		// ensure that the pin is off
		hrt_call_after(&_disengagecall, 0,
			       (hrt_callout)&CameraTrigger::disengage, this);
	}

	_trigger_enabled = on;
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
	// schedule trigger on and off calls
	hrt_call_after(&_engagecall, 0,
		       (hrt_callout)&CameraTrigger::engage, this);

	hrt_call_after(&_disengagecall, 0 + (_activation_time * 1000),
		       (hrt_callout)&CameraTrigger::disengage, this);
}

void
CameraTrigger::start()
{
	// enable immediate if configured that way
	if (_mode == 2) {
		control(true);
	}

	// Prevent camera from sleeping, if triggering is enabled and the interface supports it
	if (_mode > 0 && _mode < 4 && _camera_interface->has_power_control()) {
		toggle_power();
		enable_keep_alive(true);

	} else {
		enable_keep_alive(false);
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

	if (trig->_vcommand_sub < 0) {
		trig->_vcommand_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	bool updated;
	orb_check(trig->_vcommand_sub, &updated);

	// while the trigger is inactive it has to be ready
	// to become active instantaneously
	int poll_interval_usec = 5000;

	if (trig->_mode < 3) {

		if (updated) {

			struct vehicle_command_s cmd;

			orb_copy(ORB_ID(vehicle_command), trig->_vcommand_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL) {
				// Set trigger rate from command
				if (cmd.param2 > 0) {
					trig->_interval = cmd.param2;
					param_set(trig->_p_interval, &(trig->_interval));
				}

				if (cmd.param1 < 1.0f) {
					trig->control(false);

				} else if (cmd.param1 >= 1.0f) {
					trig->control(true);
					// while the trigger is active there is no
					// need to poll at a very high rate
					poll_interval_usec = 100000;
				}

			} else if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL) {
				if (cmd.param5 > 0) {
					// One-shot trigger
					trig->shoot_once();
				}
			}
		}

	} else {

		// Set trigger based on covered distance
		if (trig->_vlposition_sub < 0) {
			trig->_vlposition_sub = orb_subscribe(ORB_ID(vehicle_local_position));
		}

		struct vehicle_local_position_s pos;

		orb_copy(ORB_ID(vehicle_local_position), trig->_vlposition_sub, &pos);

		if (pos.xy_valid) {

			bool turning_on = false;

			if (updated && trig->_mode == 4) {

				// Check update from command
				struct vehicle_command_s cmd;
				orb_copy(ORB_ID(vehicle_command), trig->_vcommand_sub, &cmd);

				if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST) {

					// Set trigger to disabled if the set distance is not positive
					if (cmd.param1 > 0.0f && !trig->_trigger_enabled) {

						if (trig->_camera_interface->has_power_control()) {
							trig->toggle_power();
							trig->enable_keep_alive(true);
							// Give the camera time to turn on, before starting to send trigger signals
							poll_interval_usec = 5000000;
							turning_on = true;
						}

					} else if (cmd.param1 <= 0.0f && trig->_trigger_enabled) {
						hrt_cancel(&(trig->_engagecall));
						hrt_cancel(&(trig->_disengagecall));

						if (trig->_camera_interface->has_power_control()) {
							trig->enable_keep_alive(false);
							trig->toggle_power();
						}
					}

					trig->_trigger_enabled = cmd.param1 > 0.0f;
					trig->_distance = cmd.param1;
				}
			}

			if ((trig->_trigger_enabled || trig->_mode < 4) && !turning_on) {

				// Initialize position if not done yet
				math::Vector<2> current_position(pos.x, pos.y);

				if (!trig->_valid_position) {
					// First time valid position, take first shot
					trig->_last_shoot_position = current_position;
					trig->_valid_position = pos.xy_valid;
					trig->shoot_once();
				}

				// Check that distance threshold is exceeded and the time between last shot is large enough
				if ((trig->_last_shoot_position - current_position).length() >= trig->_distance) {
					trig->shoot_once();
					trig->_last_shoot_position = current_position;
				}
			}

		} else {
			poll_interval_usec = 100000;
		}
	}

	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline,
		   camera_trigger::g_camera_trigger, USEC2TICK(poll_interval_usec));
}

void
CameraTrigger::engage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	struct camera_trigger_s	report = {};

	/* set timestamp the instant before the trigger goes off */
	report.timestamp = hrt_absolute_time();

	trig->_camera_interface->trigger(true);

	report.seq = trig->_trigger_seq++;

	orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &report);
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
	PX4_INFO("state : %s", _trigger_enabled ? "enabled" : "disabled");
	PX4_INFO("mode : %i", _mode);
	PX4_INFO("interval : %.2f [ms]", (double)_interval);
	PX4_INFO("distance : %.2f [m]", (double)_distance);
	PX4_INFO("activation time : %.2f [ms]", (double)_activation_time);
	_camera_interface->info();
}

static int usage()
{
	PX4_INFO("usage: camera_trigger {start|stop|enable|disable|status|test|test_power}\n");
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

	} else if (!strcmp(argv[1], "enable")) {
		camera_trigger::g_camera_trigger->control(true);

	} else if (!strcmp(argv[1], "disable")) {
		camera_trigger::g_camera_trigger->control(false);

	} else if (!strcmp(argv[1], "test")) {
		camera_trigger::g_camera_trigger->test();

	} else if (!strcmp(argv[1], "test_power")) {
		camera_trigger::g_camera_trigger->toggle_power();

	} else {
		return usage();
	}

	return 0;
}

