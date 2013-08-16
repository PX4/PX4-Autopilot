/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

	rgbled_color_t		_colors[8];
	rgbled_mode_t		_mode;

	bool			_should_run;
	bool			_running;
	int			_led_interval;
	int			_counter;

	void 			set_color(rgbled_color_t ledcolor);
	void			set_mode(rgbled_mode_t mode);

	static void		led_trampoline(void *arg);
	void			led();

	int			set(bool on, uint8_t r, uint8_t g, uint8_t b);
	int			set_on(bool on);
	int			set_rgb(uint8_t r, uint8_t g, uint8_t b);
	int			get(bool &on, bool &not_powersave, uint8_t &r, uint8_t &g, uint8_t &b);
};

/* for now, we only support one RGBLED */
namespace
{
	RGBLED *g_rgbled;
}


extern "C" __EXPORT int rgbled_main(int argc, char *argv[]);

RGBLED::RGBLED(int bus, int rgbled) :
	I2C("rgbled", RGBLED_DEVICE_PATH, bus, rgbled, 100000),
	_colors({RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF}),
	_mode(RGBLED_MODE_OFF),
	_running(false),
	_led_interval(0),
	_counter(0)
{
	memset(&_work, 0, sizeof(_work));
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

	/* start off */
	set(false, 0, 0, 0);

	return OK;
}

int
RGBLED::probe()
{
	int ret;
	bool on, not_powersave;
	uint8_t r, g, b;

	ret = get(on, not_powersave, r, g, b);

	return ret;
}

int
RGBLED::info()
{
	int ret;
	bool on, not_powersave;
	uint8_t r, g, b;

	ret = get(on, not_powersave, r, g, b);

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
		/* set the specified RGB values */
		rgbled_rgbset_t rgbset;
		memcpy(&rgbset, (rgbled_rgbset_t*)arg, sizeof(rgbset));
		set_rgb(rgbset.red, rgbset.green, rgbset.blue);
		set_mode(RGBLED_MODE_ON);
		return OK;

	case RGBLED_SET_COLOR:
		/* set the specified color name */
		set_color((rgbled_color_t)arg);
		return OK;

	case RGBLED_SET_MODE:
		/* set the specified blink pattern/speed */
		set_mode((rgbled_mode_t)arg);
		return OK;


	default:
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



void
RGBLED::led()
{
	switch (_mode) {
		case RGBLED_MODE_BLINK_SLOW:
		case RGBLED_MODE_BLINK_NORMAL:
		case RGBLED_MODE_BLINK_FAST:
			if(_counter % 2 == 0)
				set_on(true);
			else
				set_on(false);
			break;
		default:
			break;
	}

	_counter++;

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, _led_interval);
}

void
RGBLED::set_color(rgbled_color_t color) {
	switch (color) {
		case RGBLED_COLOR_OFF:		// off
			set_rgb(0,0,0);
			break;
		case RGBLED_COLOR_RED:		// red
			set_rgb(255,0,0);
			break;
		case RGBLED_COLOR_YELLOW:	// yellow
			set_rgb(255,70,0);
			break;
		case RGBLED_COLOR_PURPLE:	// purple
			set_rgb(255,0,255);
			break;
		case RGBLED_COLOR_GREEN:	// green
			set_rgb(0,255,0);
			break;
		case RGBLED_COLOR_BLUE:		// blue
			set_rgb(0,0,255);
			break;
		case RGBLED_COLOR_WHITE:	// white
			set_rgb(255,255,255);
			break;
		case RGBLED_COLOR_AMBER:	// amber
			set_rgb(255,20,0);
			break;
		default:
			warnx("color unknown");
			break;
	}
}

void
RGBLED::set_mode(rgbled_mode_t mode)
{
	_mode = mode;

	switch (mode) {
		case RGBLED_MODE_OFF:
			_should_run = false;
			set_on(false);
			break;
		case RGBLED_MODE_ON:
			_should_run = false;
			set_on(true);
			break;
		case RGBLED_MODE_BLINK_SLOW:
			_should_run = true;
			_led_interval = 2000;
			break;
		case RGBLED_MODE_BLINK_NORMAL:
			_should_run = true;
			_led_interval = 1000;
			break;
		case RGBLED_MODE_BLINK_FAST:
			_should_run = true;
			_led_interval = 500;
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
	/* if it should stop, then cancel the workq */
	if (!_should_run && _running) {
		_running = false;
		work_cancel(LPWORK, &_work);
	}
}

int
RGBLED::set(bool on, uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t settings_byte = 0;

	if (on)
		settings_byte |= SETTING_ENABLE;
/* powersave not used */
//	if (not_powersave)
		settings_byte |= SETTING_NOT_POWERSAVE;

	const uint8_t msg[5] = { SUB_ADDR_START, (uint8_t)(b*15/255), (uint8_t)(g*15/255), (uint8_t)(r*15/255), settings_byte};

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
RGBLED::set_on(bool on)
{
	uint8_t settings_byte = 0;

	if (on)
		settings_byte |= SETTING_ENABLE;

/* powersave not used */
//	if (not_powersave)
		settings_byte |= SETTING_NOT_POWERSAVE;

	const uint8_t msg[2] = { SUB_ADDR_SETTINGS, settings_byte};

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
RGBLED::set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[6] = { SUB_ADDR_PWM0, (uint8_t)(b*15/255), SUB_ADDR_PWM1, (uint8_t)(g*15/255), SUB_ADDR_PWM2, (uint8_t)(r*15/255)};

	return transfer(msg, sizeof(msg), nullptr, 0);
}


int
RGBLED::get(bool &on, bool &not_powersave, uint8_t &r, uint8_t &g, uint8_t &b)
{
	uint8_t result[2];
	int ret;

	ret = transfer(nullptr, 0, &result[0], 2);

	if (ret == OK) {
		on = result[0] & SETTING_ENABLE;
		not_powersave = result[0] & SETTING_NOT_POWERSAVE;
		/* XXX check, looks wrong */
		r = (result[0] & 0x0f)*255/15;
		g = (result[1] & 0xf0)*255/15;
		b = (result[1] & 0x0f)*255/15;
	}

	return ret;
}

void rgbled_usage();


void rgbled_usage() {
	warnx("missing command: try 'start', 'test', 'info', 'off', 'rgb'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	errx(0, "    -a addr (0x%x)", ADDR);
}

int
rgbled_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int rgbledadr = ADDR; /* 7bit */

	int ch;
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
		}
	}
	argc -= optind;
	argv += optind;
	const char *verb = argv[0];

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
	    exit(0);
	}

	if (!strcmp(verb, "test")) {
		fd = open(RGBLED_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " RGBLED_DEVICE_PATH);
		}
		ret = ioctl(fd, RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_WHITE);

		if(ret != OK) {
			close(fd);
			exit(ret);
		}
		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_NORMAL);
		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_rgbled->info();
		exit(0);
	}

	if (!strcmp(verb, "off")) {
		fd = open(RGBLED_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " RGBLED_DEVICE_PATH);
		}
		ret = ioctl(fd, RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_OFF);
		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "rgb")) {
		fd = open(RGBLED_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " RGBLED_DEVICE_PATH);
		}
		if (argc < 4) {
			errx(1, "Usage: rgbled rgb <red> <green> <blue>");
		}
		rgbled_rgbset_t v;
		v.red   = strtol(argv[1], NULL, 0);
		v.green = strtol(argv[2], NULL, 0);
		v.blue  = strtol(argv[3], NULL, 0);
		ret = ioctl(fd, RGBLED_SET_RGB, (unsigned long)&v);
		close(fd);
		exit(ret);
	}

	rgbled_usage();
}
