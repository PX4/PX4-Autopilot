/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *   Author: Mohammed Kabir <mhkabir98@gmail.com>
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
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <poll.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>



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
	
private:
	
	struct hrt_call		_call;
	
	int 			_gpio_fd;
	int 			_pin;
	int 			_trigger_polarity;
	int 			_trigger_activation_time;

	int			_camera_trigger_sub;
	struct camera_trigger_s	_trigger;
	
	/**
	 * Trigger main loop.
	 */
	static void	cycle(void *arg);

};

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	_gpio_fd(-1),
	_pin(0),
	_trigger_polarity(0),
	_trigger_activation_time(0),
	_camera_trigger_sub(-1),
	_trigger{}
{
}

CameraTrigger::~CameraTrigger()
{
	stop();

	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::start()
{
	/* Pull parameters */
	param_t trigger_polarity = param_find("TRIG_POLARITY");
	param_t trigger_activation_time = param_find("TRIG_ACT_TIME");
	
	param_get(trigger_polarity, &_trigger_polarity); 
	param_get(trigger_activation_time, &_trigger_activation_time); 

	_gpio_fd = open(PX4FMU_DEVICE_PATH, 0);

	if (_gpio_fd < 0) {
		
		errx(1, "camera_trigger: GPIO device open fail");
	}
	else
	{
		warnx("camera_trigger: GPIO device opened");
	}

	_camera_trigger_sub = orb_subscribe(ORB_ID(camera_trigger));

	ioctl(_gpio_fd, GPIO_SET_OUTPUT, _pin);
	
	if(_trigger_polarity == 0)
	{
		ioctl(_gpio_fd, GPIO_SET, _pin); 		/* GPIO pin pull high */
	}
	else if(_trigger_polarity == 1)
	{
		ioctl(_gpio_fd, GPIO_CLEAR, _pin); 	/* GPIO pin pull low */
	}	
	else
	{
		errx(1, "camera_trigger: invalid trigger polarity setting. stopping.");
	}

	hrt_call_every(&_call, 0, 1000, (hrt_callout)&CameraTrigger::cycle, this); 
}

void
CameraTrigger::stop()
{
	hrt_cancel(&_call);
}

void
CameraTrigger::cycle(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
	
	/* check for trigger updates*/
	bool updated;
	orb_check(trig->_camera_trigger_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(camera_trigger), trig->_camera_trigger_sub, &trig->_trigger);
	}
	
	if(trig->_trigger.trigger_on == true){

		if(trig->_trigger_polarity == 0)  	/* ACTIVE_LOW */
		{
			ioctl(trig->_gpio_fd, GPIO_CLEAR, trig->_pin);
			up_udelay(trig->_trigger_activation_time*1000);
			ioctl(trig->_gpio_fd, GPIO_SET, trig->_pin);
		}
		else if(trig->_trigger_polarity == 1)	/* ACTIVE_HIGH */
		{
			ioctl(trig->_gpio_fd, GPIO_SET, trig->_pin);
			up_udelay(trig->_trigger_activation_time*1000);
			ioctl(trig->_gpio_fd, GPIO_CLEAR, trig->_pin);
		}
	}
}

void
CameraTrigger::status()
{
	// XXX 
}

static void usage()
{
	errx(1, "usage: camera_trigger {start|stop|status} [-p <n>]\n"
		     "\t-p <n>\tUse specified AUX OUT pin number (default: 1)"
		    );
}

int camera_trigger_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (camera_trigger::g_camera_trigger != nullptr) {
			errx(1, "already running");
		}

		camera_trigger::g_camera_trigger = new CameraTrigger;

		if (camera_trigger::g_camera_trigger == nullptr) {
			errx(1, "alloc failed");
		}

		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete camera_trigger::g_camera_trigger;
		camera_trigger::g_camera_trigger = nullptr;

	} else {
		usage();
	}

	return 0;
}

