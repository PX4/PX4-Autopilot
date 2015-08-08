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
 * External camera-IMU synchronisation and triggering via FMU auxillary pins.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <mavlink/mavlink_log.h>
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

	int			_pin;

private:

	struct hrt_call		_engagecall;
	struct hrt_call		_disengagecall;
	static struct work_s	_work;

	int 			_gpio_fd;

	int 			_polarity;
	int			_mode;
	float 			_activation_time;
	float  			_interval;
	uint32_t 		_trigger_seq;
	bool	 		_trigger_enabled;

	int			_vcommand_sub;

	orb_advert_t		_trigger_pub;

	param_t _p_polarity;
	param_t _p_mode;
	param_t _p_activation_time;
	param_t _p_interval;

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

};

struct work_s CameraTrigger::_work;
constexpr uint32_t CameraTrigger::_gpios[6];

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	_pin(TRIGGER_PIN_DEFAULT),
	_engagecall {},
	_disengagecall {},
	_gpio_fd(-1),
	_polarity(0),
	_mode(0),
	_activation_time(0.5f /* ms */),
	_interval(100.0f /* ms */),
	_trigger_seq(0),
	_trigger_enabled(false),
	_vcommand_sub(-1),
	_trigger_pub(nullptr)
{
	memset(&_work, 0, sizeof(_work));

	// Parameters
	_p_polarity = param_find("TRIG_POLARITY");
	_p_interval = param_find("TRIG_INTERVAL");
	_p_activation_time = param_find("TRIG_ACT_TIME");
	_p_mode = param_find("TRIG_ACT_TIME");

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

	// schedule trigger on and off calls
	hrt_call_every(&_engagecall, 500, (_interval * 1000),
			       (hrt_callout)&CameraTrigger::engage, this);

	// schedule trigger on and off calls
	hrt_call_every(&_disengagecall, 500 + (_activation_time * 1000), (_interval * 1000),
			       (hrt_callout)&CameraTrigger::disengage, this);

	_trigger_enabled = on;
}

void
CameraTrigger::start()
{

	param_get(_p_polarity, &_polarity);
	param_get(_p_activation_time, &_activation_time);
	param_get(_p_interval, &_interval);
	param_get(_p_mode, &_mode);

	stm32_configgpio(_gpios[_pin - 1]);

	if (_polarity == 0) {
		stm32_gpiowrite(_gpios[_pin - 1], 1);	 	// GPIO pin pull high

	} else if (_polarity == 1) {
		stm32_gpiowrite(_gpios[_pin - 1], 0);	 	// GPIO pin pull low

	} else {
		warnx(" invalid trigger polarity setting. stopping.");
		stop();
	}

	// enable immediate if configured that way
	if (_mode > 1) {
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
		}
	}

	work_queue(LPWORK, &_work, (worker_t)&CameraTrigger::cycle_trampoline,
		camera_trigger::g_camera_trigger, USEC2TICK(poll_interval_usec));
}

void
CameraTrigger::engage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	stm32_configgpio(trig->_gpios[trig->_pin - 1]);

	struct camera_trigger_s	trigger;

	/* set timestamp the instant before the trigger goes off */
	trigger.timestamp = hrt_absolute_time();

	if (trig->_polarity == 0) {	// ACTIVE_LOW
		stm32_gpiowrite(trig->_gpios[trig->_pin - 1], 0);

	} else if (trig->_polarity == 1) {	// ACTIVE_HIGH
		stm32_gpiowrite(trig->_gpios[trig->_pin - 1], 1);
	}

	trigger.seq = trig->_trigger_seq++;

	orb_publish(ORB_ID(camera_trigger), trig->_trigger_pub, &trigger);
}

void
CameraTrigger::disengage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);

	stm32_configgpio(trig->_gpios[trig->_pin - 1]);

	if (trig->_polarity == 0) {		// ACTIVE_LOW
		stm32_gpiowrite(trig->_gpios[trig->_pin - 1], 1);

	} else if (trig->_polarity == 1) {	// ACTIVE_HIGH
		stm32_gpiowrite(trig->_gpios[trig->_pin - 1], 0);
	}

}

void
CameraTrigger::info()
{
	warnx("state : %s", _trigger_enabled ? "enabled" : "disabled");
	warnx("pin : %i, polarity : %s", _pin, _polarity ? "ACTIVE_HIGH" : "ACTIVE_LOW");
	warnx("interval : %.2f", (double)_interval);
}

static void usage()
{
	errx(1, "usage: camera_trigger {start|stop|info} [-p <n>]\n"
	     "\t-p <n>\tUse specified AUX OUT pin number (default: %d)", TRIGGER_PIN_DEFAULT
	    );
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

		if (argc > 3) {

			camera_trigger::g_camera_trigger->_pin = (int)argv[3];

			if (atoi(argv[3]) > 0 && atoi(argv[3]) < 6) {
				warnx("starting trigger on pin : %li ", atoi(argv[3]));
				camera_trigger::g_camera_trigger->_pin = atoi(argv[3]);

			} else {
				usage();
			}
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

