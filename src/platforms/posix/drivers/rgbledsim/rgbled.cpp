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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>

#include <px4_workqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>

#include "VirtDevObj.hpp"

using namespace DriverFramework;

#define DEVICE_LOG PX4_INFO
#define RGBLED_ONTIME 120
#define RGBLED_OFFTIME 120

#define ADDR			PX4_I2C_OBDEV_LED	/**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */


class RGBLEDSIM : public VirtDevObj
{
public:
	RGBLEDSIM(int bus, int rgbled);
	virtual ~RGBLEDSIM();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		devIOCTL(unsigned long cmd, unsigned long arg);

	// Don't use the default periodic callback
	virtual void 		_measure() {}

private:
	work_s			_work;

	rgbled_mode_t		_mode;
	rgbled_pattern_t	_pattern;

	uint8_t			_r;
	uint8_t			_g;
	uint8_t			_b;
	float			_brightness;
	float			_max_brightness;

	bool			_running;
	int			_led_interval;
	bool			_should_run;
	int			_counter;
	int			_param_sub;

	void 			set_color(rgbled_color_t ledcolor);
	void			set_mode(rgbled_mode_t mode);
	void			set_pattern(rgbled_pattern_t *pattern);

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable);
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
	void			update_params();

	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	int		transfer(const uint8_t *send, unsigned send_len,
				 uint8_t *recv, unsigned recv_len);

};

/* for now, we only support one RGBLEDSIM */
namespace
{
RGBLEDSIM *g_rgbled = nullptr;
}

void rgbled_usage();

extern "C" __EXPORT int rgbledsim_main(int argc, char *argv[]);

RGBLEDSIM::RGBLEDSIM(int bus, int rgbled) :
	VirtDevObj("rgbled", "/dev/rgbledsim", RGBLED_BASE_DEVICE_PATH, 0),
	_mode(RGBLED_MODE_OFF),
	_r(0),
	_g(0),
	_b(0),
	_brightness(1.0f),
	_max_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0),
	_param_sub(-1)
{
	memset(&_work, 0, sizeof(_work));
	memset(&_pattern, 0, sizeof(_pattern));
}

RGBLEDSIM::~RGBLEDSIM()
{
}

int
RGBLEDSIM::init()
{
	int ret;
	ret = VirtDevObj::init();

	if (ret != OK) {
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);
	send_led_rgb();

	return OK;
}

int
RGBLEDSIM::probe()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	/**
	   this may look strange, but is needed. There is a serial
	   EEPROM (Microchip-24aa01) on the PX4FMU-v1 that responds to
	   a bunch of I2C addresses, including the 0x55 used by this
	   LED device. So we need to do enough operations to be sure
	   we are talking to the right device. These 3 operations seem
	   to be enough, as the 3rd one consistently fails if no
	   RGBLEDSIM is on the bus.
	 */

	if ((ret = get(on, powersave, r, g, b)) != OK ||
	    (ret = send_led_enable(false) != OK) ||
	    (ret = send_led_enable(false) != OK)) {
		return ret;
	}

	return ret;
}

int
RGBLEDSIM::info()
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
RGBLEDSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case RGBLED_SET_RGB:
		/* set the specified color */
		_r = ((rgbled_rgbset_t *) arg)->red;
		_g = ((rgbled_rgbset_t *) arg)->green;
		_b = ((rgbled_rgbset_t *) arg)->blue;
		send_led_rgb();
		return OK;

	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg);
		send_led_rgb();
		return OK;

	case RGBLED_SET_MODE:
		/* set the specified mode */
		set_mode((rgbled_mode_t)arg);
		return OK;

	case RGBLED_SET_PATTERN:
		/* set a special pattern */
		set_pattern((rgbled_pattern_t *)arg);
		return OK;

	default:
		/* see if the parent class can make any use of it */
		ret = VirtDevObj::devIOCTL(cmd, arg);
		break;
	}

	return ret;
}


void
RGBLEDSIM::led_trampoline(void *arg)
{
	RGBLEDSIM *rgbl = reinterpret_cast<RGBLEDSIM *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
RGBLEDSIM::led()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	if (_param_sub < 0) {
		_param_sub = orb_subscribe(ORB_ID(parameter_update));
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

	switch (_mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
		if (_counter >= 2) {
			_counter = 0;
		}

		send_led_enable(_counter == 0);

		break;

	case RGBLED_MODE_BREATHE:

		if (_counter >= 62) {
			_counter = 0;
		}

		int n;

		if (_counter < 32) {
			n = _counter;

		} else {
			n = 62 - _counter;
		}

		_brightness = n * n / (31.0f * 31.0f);
		send_led_rgb();
		break;

	case RGBLED_MODE_PATTERN:

		/* don't run out of the pattern array and stop if the next frame is 0 */
		if (_counter >= RGBLED_PATTERN_LENGTH || _pattern.duration[_counter] <= 0) {
			_counter = 0;
		}

		set_color(_pattern.color[_counter]);
		send_led_rgb();
		_led_interval = _pattern.duration[_counter];
		break;

	default:
		break;
	}

	_counter++;
}

/**
 * Parse color constant and set _r _g _b values
 */
void
RGBLEDSIM::set_color(rgbled_color_t color)
{
	switch (color) {
	case RGBLED_COLOR_OFF:
		_r = 0;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_RED:
		_r = 255;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_YELLOW:
		_r = 255;
		_g = 200;
		_b = 0;
		break;

	case RGBLED_COLOR_PURPLE:
		_r = 255;
		_g = 0;
		_b = 255;
		break;

	case RGBLED_COLOR_GREEN:
		_r = 0;
		_g = 255;
		_b = 0;
		break;

	case RGBLED_COLOR_BLUE:
		_r = 0;
		_g = 0;
		_b = 255;
		break;

	case RGBLED_COLOR_WHITE:
		_r = 255;
		_g = 255;
		_b = 255;
		break;

	case RGBLED_COLOR_AMBER:
		_r = 255;
		_g = 80;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_RED:
		_r = 90;
		_g = 0;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_YELLOW:
		_r = 80;
		_g = 30;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_PURPLE:
		_r = 45;
		_g = 0;
		_b = 45;
		break;

	case RGBLED_COLOR_DIM_GREEN:
		_r = 0;
		_g = 90;
		_b = 0;
		break;

	case RGBLED_COLOR_DIM_BLUE:
		_r = 0;
		_g = 0;
		_b = 90;
		break;

	case RGBLED_COLOR_DIM_WHITE:
		_r = 30;
		_g = 30;
		_b = 30;
		break;

	case RGBLED_COLOR_DIM_AMBER:
		_r = 80;
		_g = 20;
		_b = 0;
		break;

	default:
		PX4_WARN("color unknown");
		break;
	}
}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
RGBLEDSIM::set_mode(rgbled_mode_t mode)
{
	if (mode != _mode) {
		_mode = mode;

		switch (mode) {
		case RGBLED_MODE_OFF:
			_should_run = false;
			send_led_enable(false);
			break;

		case RGBLED_MODE_ON:
			_brightness = 1.0f;
			send_led_rgb();
			send_led_enable(true);
			break;

		case RGBLED_MODE_BLINK_SLOW:
			_should_run = true;
			_counter = 0;
			_led_interval = 2000;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			_should_run = true;
			_counter = 0;
			_led_interval = 500;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BLINK_FAST:
			_should_run = true;
			_counter = 0;
			_led_interval = 100;
			_brightness = 1.0f;
			send_led_rgb();
			break;

		case RGBLED_MODE_BREATHE:
			_should_run = true;
			_counter = 0;
			_led_interval = 25;
			send_led_enable(true);
			break;

		case RGBLED_MODE_PATTERN:
			_should_run = true;
			_counter = 0;
			_brightness = 1.0f;
			send_led_enable(true);
			break;

		default:
			PX4_WARN("mode unknown");
			break;
		}

		/* if it should run now, start the workq */
		if (_should_run && !_running) {
			_running = true;
			work_queue(LPWORK, &_work, (worker_t)&RGBLEDSIM::led_trampoline, this, 1);
		}

	}
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
RGBLEDSIM::set_pattern(rgbled_pattern_t *pattern)
{
	memcpy(&_pattern, pattern, sizeof(rgbled_pattern_t));
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLEDSIM::send_led_enable(bool enable)
{
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
RGBLEDSIM::send_led_rgb()
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
RGBLEDSIM::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	uint8_t result[2] = {0, 0};
	int ret;

	ret = transfer(nullptr, 0, &result[0], 2);

	if (ret == OK) {
		on = result[0] & SETTING_ENABLE;
		powersave = !(result[0] & SETTING_NOT_POWERSAVE);
		/* XXX check, looks wrong */
		r = (result[0] & 0x0f) << 4;
		g = (result[1] & 0xf0);
		b = (result[1] & 0x0f) << 4;
	}

	return ret;
}

void
RGBLEDSIM::update_params()
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
	PX4_WARN("missing command: try 'start', 'test', 'info', 'off', 'stop', 'rgb 30 40 50'");
	PX4_WARN("options:");
	PX4_WARN("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	PX4_WARN("    -a addr (0x%x)", ADDR);
}

int
rgbledsim_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int rgbledadr = ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			rgbledadr = strtol(myoptarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(myoptarg, NULL, 0);
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

	int ret;

	if (!strcmp(verb, "start")) {
		if (g_rgbled != nullptr) {
			PX4_WARN("already started");
			return 1;
		}

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_rgbled = new RGBLEDSIM(PX4_I2C_BUS_EXPANSION, rgbledadr);

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
			g_rgbled = new RGBLEDSIM(i2cdevice, rgbledadr);

			if (g_rgbled == nullptr) {
				PX4_WARN("new failed");
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

	if (!strcmp(verb, "test")) {
		DevHandle h;
		DevMgr::getHandle(RGBLED0_DEVICE_PATH, h);

		if (!h.isValid()) {
			PX4_WARN("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		rgbled_pattern_t pattern = { {RGBLED_COLOR_RED, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_WHITE, RGBLED_COLOR_OFF, RGBLED_COLOR_OFF},
			{500, 500, 500, 500, 1000, 0 }	// "0" indicates end of pattern
		};

		ret = h.ioctl(RGBLED_SET_PATTERN, (unsigned long)&pattern);
		ret = h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_PATTERN);

		DevMgr::releaseHandle(h);
		return ret;
	}

	if (!strcmp(verb, "info")) {
		g_rgbled->info();
		return 0;
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		DevHandle h;
		DevMgr::getHandle(RGBLED0_DEVICE_PATH, h);

		if (!h.isValid()) {
			PX4_WARN("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		ret = h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		DevMgr::releaseHandle(h);

		/* delete the rgbled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {
			delete g_rgbled;
			g_rgbled = nullptr;
			return 0;
		}

		return ret;
	}

	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			PX4_WARN("Usage: rgbled rgb <red> <green> <blue>");
			return 1;
		}

		DevHandle h;
		DevMgr::getHandle(RGBLED0_DEVICE_PATH, h);

		if (!h.isValid()) {
			PX4_WARN("Unable to open " RGBLED0_DEVICE_PATH);
			return 1;
		}

		rgbled_rgbset_t v;
		v.red   = strtol(argv[2], NULL, 0);
		v.green = strtol(argv[3], NULL, 0);
		v.blue  = strtol(argv[4], NULL, 0);
		ret = h.ioctl(RGBLED_SET_RGB, (unsigned long)&v);
		ret = h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
		DevMgr::releaseHandle(h);
		return ret;
	}

	rgbled_usage();
	return 1;
}

int RGBLEDSIM::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	return 0;
}
