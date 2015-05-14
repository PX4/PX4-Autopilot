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
	 * Display info.
	 */
	void		info();
		
	int 		pin;
	
private:
	
	struct hrt_call		_pollcall;
	struct hrt_call		_firecall;
	
	int 			_gpio_fd;
	int 			_trigger_polarity;
	float 			_trigger_activation_time;
	float  			_trigger_integration_time;
	float  			_trigger_transfer_time;

	int			_camera_trigger_sub;

	uint32_t 		_trigger_seq;		/* Trigger sequence */
	uint32_t 		_trigger_enabled;	/* Trigger enabled flag */

	struct camera_trigger_s	_trigger;
	
	/**
	 * Topic poller to check for fire info.
	 */
	static void	poll(void *arg);
	/**
	 * Fires trigger
	 */
	static void	engage(void *arg);
	/**
	 * Resets trigger
	 */
	static void	disengage(void *arg);

};

namespace camera_trigger
{

CameraTrigger	*g_camera_trigger;
}

CameraTrigger::CameraTrigger() :
	pin(1),
	_gpio_fd(-1),
	_trigger_polarity(0),
	_trigger_activation_time(0.0f),
	_trigger_integration_time(0.0f),
	_trigger_transfer_time(0.0f),
	_camera_trigger_sub(-1),
	_trigger_seq(0),
	_trigger_enabled(false),
	_trigger{}
{
}

CameraTrigger::~CameraTrigger()
{
	camera_trigger::g_camera_trigger = nullptr;
}

void
CameraTrigger::start()
{

	/* Pull parameters */
	param_t trigger_polarity = param_find("TRIG_POLARITY");
	param_t trigger_activation_time = param_find("TRIG_ACT_TIME");	
	param_t trigger_integration_time = param_find("TRIG_INT_TIME");
	param_t trigger_transfer_time = param_find("TRIG_TRANS_TIME");
	
	param_get(trigger_polarity, &_trigger_polarity); 
	param_get(trigger_activation_time, &_trigger_activation_time);	
	param_get(trigger_integration_time, &_trigger_integration_time); 	
	param_get(trigger_transfer_time, &_trigger_transfer_time); 

	_gpio_fd = open(PX4FMU_DEVICE_PATH, 0);

	if (_gpio_fd < 0) {
		
		warnx("GPIO device open fail");
		stop();
	}
	else
	{
		warnx("GPIO device opened");
	}

	_camera_trigger_sub = orb_subscribe(ORB_ID(camera_trigger));

	ioctl(_gpio_fd, GPIO_SET_OUTPUT, pin);
	
	if(_trigger_polarity == 0)
	{
		ioctl(_gpio_fd, GPIO_SET, pin); 	/* GPIO pin pull high */
	}
	else if(_trigger_polarity == 1)
	{
		ioctl(_gpio_fd, GPIO_CLEAR, pin); 	/* GPIO pin pull low */
	}	
	else
	{
		warnx("camera_trigger: invalid trigger polarity setting. stopping.");
		stop();
	}

	hrt_call_every(&_pollcall, 0, 1000, (hrt_callout)&CameraTrigger::poll, this); 
}

void
CameraTrigger::stop()
{
	hrt_cancel(&_pollcall);
	hrt_cancel(&_firecall);

	delete camera_trigger::g_camera_trigger;
}

void
CameraTrigger::poll(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
	
	/* check for trigger updates*/
	bool updated;
	orb_check(trig->_camera_trigger_sub, &updated);

	if (updated) {

		orb_copy(ORB_ID(camera_trigger), trig->_camera_trigger_sub, &trig->_trigger);
		
		trig->_trigger_enabled = trig->_trigger.trigger_enabled;	// update flag
		
		if(trig->_trigger.seq > trig->_trigger_seq)
		{
			trig->_trigger_seq = trig->_trigger.seq;

			engage(trig);
			hrt_call_after(&trig->_firecall, trig->_trigger_activation_time*1000, (hrt_callout)&CameraTrigger::disengage, trig);
		}
	}

	hrt_call_after(&trig->_pollcall, 1000, (hrt_callout)&CameraTrigger::poll, trig);

}

void
CameraTrigger::engage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
	
	if(trig->_trigger_polarity == 0)  	/* ACTIVE_LOW */
	{
		ioctl(trig->_gpio_fd, GPIO_CLEAR, trig->pin);	
	}
	else if(trig->_trigger_polarity == 1)	/* ACTIVE_HIGH */
	{
		ioctl(trig->_gpio_fd, GPIO_SET, trig->pin);		
	}
	
}

void
CameraTrigger::disengage(void *arg)
{

	CameraTrigger *trig = reinterpret_cast<CameraTrigger *>(arg);
	
	if(trig->_trigger_polarity == 0)  	/* ACTIVE_LOW */
	{
		ioctl(trig->_gpio_fd, GPIO_SET, trig->pin);	
	}
	else if(trig->_trigger_polarity == 1)	/* ACTIVE_HIGH */
	{
		ioctl(trig->_gpio_fd, GPIO_CLEAR, trig->pin);		
	}
	
}

void
CameraTrigger::info()
{
	warnx("Trigger state : %s", _trigger_enabled ? "enabled" : "disabled");
	warnx("Trigger pin : %i", pin);
	warnx("Trigger polarity : %s", _trigger_polarity ? "ACTIVE_HIGH" : "ACTIVE_LOW");
	warnx("Shutter integration time : %f", (double)_trigger_integration_time);
}

static void usage()
{
	errx(1, "usage: camera_trigger {start|stop|info} [-p <n>]\n"
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
			errx(0, "already running");
		}
			
		camera_trigger::g_camera_trigger = new CameraTrigger;

		if (camera_trigger::g_camera_trigger == nullptr) {
			errx(1, "alloc failed");
		}
		
		if (argc > 3) {
	
			camera_trigger::g_camera_trigger->pin = (int)argv[3];
			if (atoi(argv[3]) > 0 && atoi(argv[3]) < 6) {	
				warnx("starting trigger on pin : %li ", atoi(argv[3]));	
				camera_trigger::g_camera_trigger->pin = atoi(argv[3]);
			}
			else
			{
				usage(); 
			}
		}
		camera_trigger::g_camera_trigger->start();

		return 0;
	}

	if (camera_trigger::g_camera_trigger == nullptr) {
		errx(1, "not running");
	}

	else if (!strcmp(argv[1], "stop")) {
		camera_trigger::g_camera_trigger->stop(); 

	}
	else if (!strcmp(argv[1], "info")) {
		camera_trigger::g_camera_trigger->info(); 

	} else {
		usage();
	}

	return 0;
}

