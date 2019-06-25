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
 * @file rgbled_ncp5623c.cpp
 *
 * Driver for the onboard RGB LED controller (NCP5623C) connected via I2C.
 *
 * @author CUAVcaijie <caijie@cuav.net>
 */

#include <px4_config.h>
#include <px4_getopt.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <px4_work_queue/ScheduledWorkItem.hpp>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <board_config.h>

#include <drivers/drv_led.h>
#include <lib/led/led.h>

#include "uORB/topics/parameter_update.h"

#define ADDR			0x39	/**< I2C adress of NCP5623C */

#define NCP5623_LED_CURRENT	0x20	/**< Current register */
#define NCP5623_LED_PWM0	0x40	/**< pwm0 register */
#define NCP5623_LED_PWM1	0x60	/**< pwm1 register */
#define NCP5623_LED_PWM2	0x80	/**< pwm2 register */

#define NCP5623_LED_BRIGHT	0x1f	/**< full brightness */
#define NCP5623_LED_OFF		0x00	/**< off */


class RGBLED_NPC5623C : public device::I2C, public px4::ScheduledWorkItem
{
public:
	RGBLED_NPC5623C(int bus, int rgbled);
	virtual ~RGBLED_NPC5623C();


	virtual int		init();
	virtual int		probe();
private:

	float			_brightness;
	float			_max_brightness;

	uint8_t			_r;
	uint8_t			_g;
	uint8_t			_b;
	volatile bool		_running;
	volatile bool		_should_run;
	bool			_leds_enabled;
	int			_param_sub;

	LedController		_led_controller;

	void			Run() override;

	int			send_led_rgb();
	void			update_params();

	int			write(uint8_t reg, uint8_t data);
};

/* for now, we only support one RGBLED */
namespace
{
RGBLED_NPC5623C *g_rgbled = nullptr;
}

void rgbled_ncp5623c_usage();

extern "C" __EXPORT int rgbled_ncp5623c_main(int argc, char *argv[]);

RGBLED_NPC5623C::RGBLED_NPC5623C(int bus, int rgbled) :
	I2C("rgbled1", RGBLED1_DEVICE_PATH, bus, rgbled, 100000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_brightness(1.0f),
	_max_brightness(1.0f),
	_r(0),
	_g(0),
	_b(0),
	_running(false),
	_should_run(true),
	_leds_enabled(true),
	_param_sub(-1)
{
}

RGBLED_NPC5623C::~RGBLED_NPC5623C()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		usleep(100000);
	}
}

int
RGBLED_NPC5623C::write(uint8_t reg, uint8_t data)
{
	uint8_t msg[1] = { 0x00 };
	msg[0] = ((reg & 0xe0) | (data & 0x1f));

	int ret = transfer(&msg[0], 1, nullptr, 0);

	return ret;
}

int
RGBLED_NPC5623C::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	update_params();

	_running = true;

	ScheduleNow();

	return OK;
}

int
RGBLED_NPC5623C::probe()
{
	_retries = 4;

	return write(NCP5623_LED_CURRENT, 0x00);
}

/**
 * Main loop function
 */
void
RGBLED_NPC5623C::Run()
{
	if (!_should_run) {
		if (_param_sub >= 0) {
			orb_unsubscribe(_param_sub);
		}

		int led_control_sub = _led_controller.led_control_subscription();

		if (led_control_sub >= 0) {
			orb_unsubscribe(led_control_sub);
		}

		_running = false;
		return;
	}

	if (_param_sub < 0) {
		_param_sub = orb_subscribe(ORB_ID(parameter_update));
	}

	if (!_led_controller.is_init()) {
		int led_control_sub = orb_subscribe(ORB_ID(led_control));
		_led_controller.init(led_control_sub);
	}

	if (_param_sub >= 0) {
		bool updated = false;
		orb_check(_param_sub, &updated);

		if (updated) {
			parameter_update_s pupdate;
			orb_copy(ORB_ID(parameter_update), _param_sub, &pupdate);
			update_params();
			// Immediately update to change brightness
			send_led_rgb();
		}
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = NCP5623_LED_BRIGHT; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = NCP5623_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = NCP5623_LED_BRIGHT; _g = NCP5623_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = NCP5623_LED_BRIGHT; _g = 0; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = NCP5623_LED_BRIGHT; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_WHITE:
			_r = NCP5623_LED_BRIGHT; _g = NCP5623_LED_BRIGHT; _b = NCP5623_LED_BRIGHT;
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;
		send_led_rgb();

	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_NPC5623C::send_led_rgb()
{

	uint8_t msg[7] = {0x20, 0x70, 0x40, 0x70, 0x60, 0x70, 0x80};
	uint8_t brightness = 0x1f * _max_brightness;

	msg[0] = NCP5623_LED_CURRENT | (brightness & 0x1f);
	msg[2] = NCP5623_LED_PWM0 | (uint8_t(_r * _brightness) & 0x1f);
	msg[4] = NCP5623_LED_PWM1 | (uint8_t(_g * _brightness) & 0x1f);
	msg[6] = NCP5623_LED_PWM2 | (uint8_t(_b * _brightness) & 0x1f);

	return transfer(&msg[0], 7, nullptr, 0);
}

void
RGBLED_NPC5623C::update_params()
{
	int32_t maxbrt = 31;
	param_get(param_find("LED_RGB1_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 31 ? 31 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	if (maxbrt == 0) {
		maxbrt = 1;
	}

	_max_brightness = maxbrt / 31.0f;
}

void
rgbled_ncp5623c_usage()
{
	PX4_INFO("missing command: try 'start', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	PX4_INFO("    -a addr (0x%x)", ADDR);
}

int
rgbled_ncp5623c_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int rgbledadr = ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			rgbledadr = strtol(myoptarg, nullptr, 0);
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, nullptr, 0);
			break;

		default:
			rgbled_ncp5623c_usage();
			return 1;
		}
	}

	if (myoptind >= argc) {
		rgbled_ncp5623c_usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		if (g_rgbled != nullptr) {
			PX4_WARN("already started");
			return 1;
		}

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_rgbled = new RGBLED_NPC5623C(PX4_I2C_BUS_EXPANSION, rgbledadr);

			if (g_rgbled != nullptr && OK != g_rgbled->init()) {
				delete g_rgbled;
				g_rgbled = nullptr;
			}

			if (g_rgbled == nullptr) {
				// fall back to default bus
				if (PX4_I2C_BUS_LED == PX4_I2C_BUS_EXPANSION) {
					PX4_WARN("no RGB led on bus #%d", i2cdevice);
					return 1;
				}

				i2cdevice = PX4_I2C_BUS_LED;
			}
		}

		if (g_rgbled == nullptr) {
			g_rgbled = new RGBLED_NPC5623C(i2cdevice, rgbledadr);

			if (g_rgbled == nullptr) {
				PX4_WARN("alloc failed");
				return 1;
			}

			if (OK != g_rgbled->init()) {
				delete g_rgbled;
				g_rgbled = nullptr;
				PX4_WARN("no RGB led on bus #%d", i2cdevice);
				return 1;
			}
		}

		return 0;
	}

	/* need the driver past this point */
	if (g_rgbled == nullptr) {
		PX4_WARN("not started");
		rgbled_ncp5623c_usage();
		return 1;
	}

	if (!strcmp(verb, "stop")) {
		delete g_rgbled;
		g_rgbled = nullptr;
		return 0;
	}

	rgbled_ncp5623c_usage();
	return 1;
}
