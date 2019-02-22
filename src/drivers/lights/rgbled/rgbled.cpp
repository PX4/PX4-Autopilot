/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file rgbled.cpp
 *
 * Driver for the onboard RGB LED controller (TCA62724FMG) connected via I2C.
 *
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
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

#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120

#define ADDR			0x55	/**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */


class RGBLED : public device::I2C, public px4::ScheduledWorkItem
{
public:
	RGBLED(int bus, int rgbled);
	virtual ~RGBLED();


	virtual int		init();
	virtual int		probe();
	int		status();

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

	int			send_led_enable(bool enable);
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
	void		update_params();
};

/* for now, we only support one RGBLED */
namespace
{
RGBLED *g_rgbled = nullptr;
}

void rgbled_usage();

extern "C" __EXPORT int rgbled_main(int argc, char *argv[]);

RGBLED::RGBLED(int bus, int rgbled) :
	I2C("rgbled", RGBLED0_DEVICE_PATH, bus, rgbled, 100000),
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

RGBLED::~RGBLED()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		px4_usleep(100000);
	}
}

int
RGBLED::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);
	send_led_rgb();

	update_params();

	_running = true;

	// kick off work queue
	ScheduleNow();

	return OK;
}

int
RGBLED::probe()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	/**
	   this may look strange, but is needed. There is a serial
	   EEPROM (Microchip-24aa01) that responds to a bunch of I2C
	   addresses, including the 0x55 used by this LED device. So
	   we need to do enough operations to be sure we are talking
	   to the right device. These 3 operations seem to be enough,
	   as the 3rd one consistently fails if no RGBLED is on the bus.
	 */

	unsigned prevretries = _retries;
	_retries = 4;

	if ((ret = get(on, powersave, r, g, b)) != OK ||
	    (ret = send_led_enable(false) != OK) ||
	    (ret = send_led_enable(false) != OK)) {
		return ret;
	}

	_retries = prevretries;

	return ret;
}

int
RGBLED::status()
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

/**
 * Main loop function
 */
void
RGBLED::Run()
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
			_r = 255; _g = 0; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = 255; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = 255; _g = 255; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_PURPLE:
			_r = 255; _g = 0; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = 255; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_WHITE:
			_r = 255; _g = 255; _b = 255;
			send_led_enable(true);
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			send_led_enable(false);
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;

		send_led_rgb();
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED::send_led_enable(bool enable)
{
	if (_leds_enabled && enable) {
		// already enabled
		return 0;
	}

	_leds_enabled = enable;
	uint8_t settings_byte = 0;

	if (enable) {
		settings_byte |= SETTING_ENABLE;
	}

	settings_byte |= SETTING_NOT_POWERSAVE;

	const uint8_t msg[2] = { SUB_ADDR_SETTINGS, settings_byte};

	return transfer(msg, sizeof(msg), nullptr, 0);
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED::send_led_rgb()
{
	/* To scale from 0..255 -> 0..15 shift right by 4 bits */
	const uint8_t msg[6] = {
		SUB_ADDR_PWM0, static_cast<uint8_t>((_b >> 4) * _brightness * _max_brightness + 0.5f),
		SUB_ADDR_PWM1, static_cast<uint8_t>((_g >> 4) * _brightness * _max_brightness + 0.5f),
		SUB_ADDR_PWM2, static_cast<uint8_t>((_r >> 4) * _brightness * _max_brightness + 0.5f)
	};
	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
RGBLED::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	uint8_t result[2] = {0, 0};
	int ret;

	ret = transfer(nullptr, 0, &result[0], 2);

	if (ret == OK) {
		on = ((result[0] >> 4) & SETTING_ENABLE);
		powersave = !((result[0] >> 4) & SETTING_NOT_POWERSAVE);
		/* XXX check, looks wrong */
		r = (result[0] & 0x0f) << 4;
		g = (result[1] & 0xf0);
		b = (result[1] & 0x0f) << 4;
	}

	return ret;
}

void
RGBLED::update_params()
{
	int32_t maxbrt = 15;
	param_get(param_find("LED_RGB_MAXBRT"), &maxbrt);
	maxbrt = maxbrt > 15 ? 15 : maxbrt;
	maxbrt = maxbrt <  0 ?  0 : maxbrt;

	// A minimum of 2 "on" steps is required for breathe effect
	if (maxbrt == 1) {
		maxbrt = 2;
	}

	_max_brightness = maxbrt / 15.0f;
}

void
rgbled_usage()
{
	PX4_INFO("missing command: try 'start', 'status', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	PX4_INFO("    -a addr (0x%x)", ADDR);
}

int
rgbled_main(int argc, char *argv[])
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
			rgbled_usage();
			return 1;
		}
	}

	if (myoptind >= argc) {
		rgbled_usage();
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
			g_rgbled = new RGBLED(PX4_I2C_BUS_EXPANSION, rgbledadr);

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
			g_rgbled = new RGBLED(i2cdevice, rgbledadr);

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
		rgbled_usage();
		return 1;
	}

	if (!strcmp(verb, "status")) {
		g_rgbled->status();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_rgbled;
		g_rgbled = nullptr;
		return 0;
	}

	rgbled_usage();
	return 1;
}
