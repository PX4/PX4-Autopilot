/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file rgbled_pwm.cpp
 *
 * Driver for the onboard RGB LED controller by PWM.
 * this driver is based the PX4 led driver
 *
 */

#include <nuttx/config.h>

//#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <nuttx/wqueue.h>
#include <drivers/drv_hrt.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_led.h>
#include <lib/led/led.h>
#include <drivers/device/device.h>
#include <systemlib/err.h>

#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120

class RGBLED_PWM : public device::CDev
{
public:
	RGBLED_PWM();
	virtual ~RGBLED_PWM();


	virtual int		init();
	virtual int		probe();
	int		status();

private:
	work_s			_work;

	uint8_t			_r;
	uint8_t			_g;
	uint8_t			_b;
	float			_brightness;

	volatile bool		_running;
	volatile bool		_should_run;

	LedController		_led_controller;

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
};

extern "C" __EXPORT int rgbled_pwm_main(int argc, char *argv[]);
extern int led_pwm_servo_set(unsigned channel, uint8_t  value);
extern unsigned led_pwm_servo_get(unsigned channel);
extern int led_pwm_servo_init(void);
extern void led_pwm_servo_deinit(void);


/* for now, we only support one RGBLED */
namespace
{
RGBLED_PWM *g_rgbled = nullptr;
}

RGBLED_PWM::RGBLED_PWM() :
	CDev("rgbled_pwm", RGBLED_PWM0_DEVICE_PATH),
	_work{},
	_r(0),
	_g(0),
	_b(0),
	_brightness(1.0f),
	_running(false),
	_should_run(true)
{
}

RGBLED_PWM::~RGBLED_PWM()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		usleep(100000);
	}
}

int
RGBLED_PWM::init()
{
	/* switch off LED on start */
	CDev::init();
	led_pwm_servo_init();
	send_led_rgb();

	_running = true;
	// kick off work queue
	work_queue(LPWORK, &_work, (worker_t)&RGBLED_PWM::led_trampoline, this, 0);

	return OK;
}

int
RGBLED_PWM::status()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	ret = get(on, powersave, r, g, b);

	if (ret == OK) {
		/* we don't care about power-save mode */
		DEVICE_LOG("state: %s", on ? "ON" : "OFF");
		DEVICE_LOG("red: %u, green: %u, blue: %u", (unsigned)r, (unsigned)g, (unsigned)b);

	} else {
		PX4_WARN("failed to read led");
	}

	return ret;
}
int
RGBLED_PWM::probe()
{
	return (OK);
}

void
RGBLED_PWM::led_trampoline(void *arg)
{
	RGBLED_PWM *rgbl = reinterpret_cast<RGBLED_PWM *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
RGBLED_PWM::led()
{
	if (!_should_run) {
		int led_control_sub = _led_controller.led_control_subscription();

		if (led_control_sub >= 0) {
			orb_unsubscribe(led_control_sub);
		}

		_running = false;
		return;
	}

	if (!_led_controller.is_init()) {
		int led_control_sub = orb_subscribe(ORB_ID(led_control));
		_led_controller.init(led_control_sub);
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = 255; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = 255; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = 255;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = 255; _g = 255; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = 255; _g = 0; _b = 255;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = 255; _b = 255;
			break;

		case led_control_s::COLOR_WHITE:
			_r = 255; _g = 255; _b = 255;
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;

		send_led_rgb();
	}

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&RGBLED_PWM::led_trampoline, this,
		   USEC2TICK(_led_controller.maximum_update_interval()));
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_PWM::send_led_rgb()
{
#if defined(BOARD_HAS_LED_PWM)
	led_pwm_servo_set(0, _r);
	led_pwm_servo_set(1, _g);
	led_pwm_servo_set(2, _b);
#endif

#if defined(BOARD_HAS_UI_LED_PWM)
	led_pwm_servo_set(3, _r);
	led_pwm_servo_set(4, _g);
	led_pwm_servo_set(5, _b);
#endif

	return (OK);
}

int
RGBLED_PWM::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	powersave = OK;
	on = _r > 0 || _g > 0 || _b > 0;
	r = _r;
	g = _g;
	b = _b;
	return OK;
}

static void
rgbled_usage()
{
	PX4_INFO("missing command: try 'start', 'status', 'stop'");
}

int
rgbled_pwm_main(int argc, char *argv[])
{
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			break;

		case 'b':
			break;

		default:
			rgbled_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		rgbled_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (g_rgbled != nullptr) {
			errx(1, "already started");
		}

		if (g_rgbled == nullptr) {
			g_rgbled = new RGBLED_PWM();

			if (g_rgbled == nullptr) {
				errx(1, "new failed");
			}

			if (OK != g_rgbled->init()) {
				delete g_rgbled;
				g_rgbled = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_rgbled == nullptr) {
		PX4_WARN("not started");
		rgbled_usage();
		exit(1);
	}

	if (!strcmp(verb, "status")) {
		g_rgbled->status();
		exit(0);
	}

	if (!strcmp(verb, "stop")) {
		delete g_rgbled;
		g_rgbled = nullptr;
		exit(0);
	}

	rgbled_usage();
	exit(0);
}
