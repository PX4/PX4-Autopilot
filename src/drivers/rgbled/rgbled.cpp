/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Julian Oes <joes@student.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 */

#include <nuttx/config.h>

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

#include <nuttx/wqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_rgbled.h>

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


class RGBLED : public device::I2C
{
public:
	RGBLED(int bus, int rgbled);
	virtual ~RGBLED();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	work_s			_work;

	rgbled_mode_t		_mode;
	rgbled_pattern_t	_pattern;

	uint8_t			_r;
	uint8_t			_g;
	uint8_t			_b;
	float			_brightness;

	bool			_running;
	int			_led_interval;
	bool			_should_run;
	int			_counter;

	void 			set_color(rgbled_color_t ledcolor);
	void			set_mode(rgbled_mode_t mode);
	void			set_pattern(rgbled_pattern_t *pattern);

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable);
	int			send_led_rgb();
	int			get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b);
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
	_mode(RGBLED_MODE_OFF),
	_r(0),
	_g(0),
	_b(0),
	_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0)
{
	memset(&_work, 0, sizeof(_work));
	memset(&_pattern, 0, sizeof(_pattern));
}

RGBLED::~RGBLED()
{
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
	   EEPROM (Microchip-24aa01) on the PX4FMU-v1 that responds to
	   a bunch of I2C addresses, including the 0x55 used by this
	   LED device. So we need to do enough operations to be sure
	   we are talking to the right device. These 3 operations seem
	   to be enough, as the 3rd one consistently fails if no
	   RGBLED is on the bus.
	 */
	if ((ret=get(on, powersave, r, g, b)) != OK ||
	    (ret=send_led_enable(false) != OK) ||
	    (ret=send_led_enable(false) != OK)) {
		return ret;
	}

	return ret;
}

int
RGBLED::info()
{
	int ret;
	bool on, powersave;
	uint8_t r, g, b;

	ret = get(on, powersave, r, g, b);

	if (ret == OK) {
		/* we don't care about power-save mode */
		log("state: %s", on ? "ON" : "OFF");
		log("red: %u, green: %u, blue: %u", (unsigned)r, (unsigned)g, (unsigned)b);

	} else {
		warnx("failed to read led");
	}

	return ret;
}

int
RGBLED::ioctl(struct file *filp, int cmd, unsigned long arg)
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
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}


void
RGBLED::led_trampoline(void *arg)
{
	RGBLED *rgbl = reinterpret_cast<RGBLED *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
RGBLED::led()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	switch (_mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
		if (_counter >= 2)
			_counter = 0;

		send_led_enable(_counter == 0);

		break;

	case RGBLED_MODE_BREATHE:

		if (_counter >= 62)
			_counter = 0;

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
		if (_counter >= RGBLED_PATTERN_LENGTH || _pattern.duration[_counter] <= 0)
			_counter = 0;

		set_color(_pattern.color[_counter]);
		send_led_rgb();
		_led_interval = _pattern.duration[_counter];
		break;

	default:
		break;
	}

	_counter++;

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, _led_interval);
}

/**
 * Parse color constant and set _r _g _b values
 */
void
RGBLED::set_color(rgbled_color_t color)
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
		warnx("color unknown");
		break;
	}
}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
RGBLED::set_mode(rgbled_mode_t mode)
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
			warnx("mode unknown");
			break;
		}

		/* if it should run now, start the workq */
		if (_should_run && !_running) {
			_running = true;
			work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, 1);
		}

	}
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
RGBLED::set_pattern(rgbled_pattern_t *pattern)
{
	memcpy(&_pattern, pattern, sizeof(rgbled_pattern_t));
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED::send_led_enable(bool enable)
{
	uint8_t settings_byte = 0;

	if (enable)
		settings_byte |= SETTING_ENABLE;

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
		SUB_ADDR_PWM0, (uint8_t)((int)(_b * _brightness) >> 4),
		SUB_ADDR_PWM1, (uint8_t)((int)(_g * _brightness) >> 4),
		SUB_ADDR_PWM2, (uint8_t)((int)(_r * _brightness) >> 4)
	};
	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
RGBLED::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	uint8_t result[2];
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
rgbled_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'rgb 30 40 50'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", ADDR);
}

int
rgbled_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int rgbledadr = ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			rgbledadr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
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

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_rgbled != nullptr)
			errx(1, "already started");

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
					errx(1, "init failed");
				}
				i2cdevice = PX4_I2C_BUS_LED;
			}
		}

		if (g_rgbled == nullptr) {
			g_rgbled = new RGBLED(i2cdevice, rgbledadr);

			if (g_rgbled == nullptr)
				errx(1, "new failed");

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
		warnx("not started");
		rgbled_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " RGBLED0_DEVICE_PATH);
		}

		rgbled_pattern_t pattern = { {RGBLED_COLOR_RED, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_WHITE, RGBLED_COLOR_OFF, RGBLED_COLOR_OFF},
			{500, 500, 500, 500, 1000, 0 }	// "0" indicates end of pattern
		};

		ret = ioctl(fd, RGBLED_SET_PATTERN, (unsigned long)&pattern);
		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_PATTERN);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_rgbled->info();
		exit(0);
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		fd = open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " RGBLED0_DEVICE_PATH);
		}

		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		close(fd);
		/* delete the rgbled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {
			delete g_rgbled;
			g_rgbled = nullptr;
			exit(0);
		}
		exit(ret);
	}

	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			errx(1, "Usage: rgbled rgb <red> <green> <blue>");
		}

		fd = open(RGBLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " RGBLED0_DEVICE_PATH);
		}

		rgbled_rgbset_t v;
		v.red   = strtol(argv[2], NULL, 0);
		v.green = strtol(argv[3], NULL, 0);
		v.blue  = strtol(argv[4], NULL, 0);
		ret = ioctl(fd, RGBLED_SET_RGB, (unsigned long)&v);
		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_ON);
		close(fd);
		exit(ret);
	}

	rgbled_usage();
	exit(0);
}
