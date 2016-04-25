/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * External camera-IMU synchronisation and triggering via FMU auxiliary pins.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <mathlib/mathlib.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
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

#define TRIGGER_PIN_DEFAULT 1

extern "C" __EXPORT int camera_trigger_main(int argc, char *argv[]);

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
	 * Trigger just once
	 */
	void		shootOnce();

	/**
	 * Start the task.
	 */
	void		start();

	/**
	 * Stop the task.
	 */
	void		stop();

	/**
	 * Display info.
	 */
	void		info();

	int			_pins[6];

private:

	struct hrt_call		_engagecall;
	struct hrt_call		_disengagecall;
	static struct work_s	_work;

	int 			_gpio_fd;

	int 			_polarity;
	int			_mode;
	float 			_activation_time;
	float  			_interval;
	float  			_distance;
	uint32_t 		_trigger_seq;
	bool	 		_trigger_enabled;
	math::Vector<2>	_last_shoot_position;
	bool			_valid_position;

	int			_vcommand_sub;
	int			_vlposition_sub;

	orb_advert_t		_trigger_pub;

	param_t _p_polarity;
	param_t _p_mode;
	param_t _p_activation_time;
	param_t _p_interval;
	param_t _p_distance;
	param_t _p_pin;

	static constexpr uint32_t _gpios[6] = {
		GPIO_GPIO0_OUTPUT,
		GPIO_GPIO1_OUTPUT,
		GPIO_GPIO2_OUTPUT,
		GPIO_GPIO3_OUTPUT,
		GPIO_GPIO4_OUTPUT,
		GPIO_GPIO5_OUTPUT
	};

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

	static void trigger(CameraTrigger *trig, bool trigger);

};

struct work_s CameraTrigger::_work;
constexpr uint32_t CameraTrigger::_gpios[6];

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	_pins{},
	_engagecall {},
	_disengagecall {},
	_gpio_fd(-1),
	_polarity(0),
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
	_trigger_pub(nullptr)
{
	memset(&_work, 0, sizeof(_work));

	// Parameters
	_p_polarity = param_find("TRIG_POLARITY");
	_p_interval = param_find("TRIG_INTERVAL");
	_p_distance = param_find("TRIG_DISTANCE");
	_p_activation_time = param_find("TRIG_ACT_TIME");
	_p_mode = param_find("TRIG_MODE");
	_p_pin = param_find("TRIG_PINS");

	param_get(_p_polarity, &_polarity);
	param_get(_p_activation_time, &_activation_time);
	param_get(_p_interval, &_interval);
	param_get(_p_distance, &_distance);
	param_get(_p_mode, &_mode);
	int pin_list;
	param_get(_p_pin, &pin_list);

	// Set all pins as invalid
	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
		_pins[i] = -1;
	}

	// Convert number to individual channels
	unsigned i = 0;
	int single_pin;

	while ((single_pin = pin_list % 10)) {

		_pins[i] = single_pin - 1;

		if (_pins[i] < 0 || _pins[i] >= static_cast<int>(sizeof(_gpios) / sizeof(_gpios[0]))) {
			_pins[i] = -1;
		}

		pin_list /= 10;
		i++;
	}

	struct camera_trigger_s	trigger = {};

	_trigger_pub = orb_advertise(ORB_ID(camera_trigger), &trigger);
}

CameraTrigger::~CameraTrigger()
{
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
CameraTrigger::shootOnce()
{
	// schedule trigger on and off calls
	hrt_call_after(&_engagecall, 0,
		       (hrt_callout)&CameraTrigger::engage, this);

	// schedule trigger on and off calls
	hrt_call_after(&_disengagecall, 0 + (_activation_time * 1000),
		       (hrt_callout)&CameraTrigger::disengage, this);
}

void
CameraTrigger::start()
{

	for (unsigned i = 0; i < sizeof(_pins) / sizeof(_pins[0]); i++) {
		stm32_configgpio(_gpios[_pins[i]]);
		stm32_gpiowrite(_gpios[_pins[i]], !_polarity);
	}

	// enable immediate if configured that way
	if (_mode == 2) {
		control(true);
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

	if (camera_trigger::g_camera_trigger != nullptr) {
		delete(camera_trigger::g_camera_trigger);
	}
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
					// One-shot trigger, default 1 ms interval
					trig->_interval = 1000;
					trig->control(true);
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

			if (updated && trig->_mode == 4) {

				// Check update from command
				struct vehicle_command_s cmd;
				orb_copy(ORB_ID(vehicle_command), trig->_vcommand_sub, &cmd);

				if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST) {

					// Set trigger to false if the set distance is not positive
					trig->_trigger_enabled = cmd.param1 > 0.0f;
					trig->_distance = cmd.param1;
				}
			}

			if (trig->_trigger_enabled || trig->_mode < 4) {

				// Initialize position if not done yet
				math::Vector<2> current_position(pos.x, pos.y);

				if (!trig->_valid_position) {
					// First time valid position, take first shot
					trig->_last_shoot_position = current_position;
					trig->_valid_position = pos.xy_valid;
					trig->shootOnce();
				}

				// Check that distance threshold is exceeded and the time between last shot is large enough
				if ((trig->_last_shoot_position - current_position).length() >= trig->_distance) {
					trig->shootOnce();
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

	CameraTrigger::trigger(trig, trig->_polarity);

	report.seq = trig->_trigger_seq++;

	orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &report);
}

void
CameraTrigger::disengage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	CameraTrigger::trigger(trig, !(trig->_polarity));
}

void
CameraTrigger::trigger(CameraTrigger *trig, bool trigger)
{
	for (unsigned i = 0; i < sizeof(trig->_pins) / sizeof(trig->_pins[0]); i++) {
		if (trig->_pins[i] >= 0) {
			// ACTIVE_LOW == 1
			stm32_gpiowrite(trig->_gpios[trig->_pins[i]], trigger);
		}
	}
}

void
CameraTrigger::info()
{
	warnx("state : %s", _trigger_enabled ? "enabled" : "disabled");
	warnx("pins 1-3 : %d,%d,%d polarity : %s", _pins[0], _pins[1], _pins[2],
	      _polarity ? "ACTIVE_HIGH" : "ACTIVE_LOW");
	warnx("mode : %i", _mode);
	warnx("interval : %.2f", (double)_interval);
	warnx("distance : %.2f", (double)_distance);
}

static void usage()
{
	errx(1, "usage: camera_trigger {start|stop|info} [-p <n>]\n");
}

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			errx(0, "already running");
		}

		camera_trigger::g_camera_trigger = new CameraTrigger;

		if (camera_trigger::g_camera_trigger == nullptr) {
			errx(1, "alloc failed");
		}

		camera_trigger::g_camera_trigger->start();

		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		errx(1, "not running");

	} else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop();

	} else if (!strcmp(argv[1], "info")) {
		camera_trigger::g_camera_trigger->info();

	} else if (!strcmp(argv[1], "enable")) {
		camera_trigger::g_camera_trigger->control(true);

	} else if (!strcmp(argv[1], "disable")) {
		camera_trigger::g_camera_trigger->control(false);

	} else {
		usage();
	}

	return 0;
}

