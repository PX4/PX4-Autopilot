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


enum ledModes {
	RGBLED_MODE_TEST,
	RGBLED_MODE_SYSTEMSTATE,
	RGBLED_MODE_OFF,
	RGBLED_MODE_RGB
};

class RGBLED : public device::I2C
{
public:
	RGBLED(int bus, int rgbled);
	virtual ~RGBLED();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		setMode(enum ledModes mode);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	work_s			_work;

	rgbled_color_t		_led_colors[8];
	rgbled_blinkmode_t	_led_blinkmode;

	// RGB values for MODE_RGB 
	struct RGBLEDSet 	_rgb;

	int			_mode;
	int			_running;

	void 			setLEDColor(rgbled_color_t ledcolor);
	static void		led_trampoline(void *arg);
	void			led();

	int				set(bool on, uint8_t r, uint8_t g, uint8_t b);
	int				set_on(bool on);
	int				set_rgb(uint8_t r, uint8_t g, uint8_t b);
	int				get(bool &on, bool &not_powersave, uint8_t &r, uint8_t &g, uint8_t &b);
};

/* for now, we only support one RGBLED */
namespace
{
	RGBLED *g_rgbled;
}


extern "C" __EXPORT int rgbled_main(int argc, char *argv[]);

RGBLED::RGBLED(int bus, int rgbled) :
	I2C("rgbled", RGBLED_DEVICE_PATH, bus, rgbled, 100000),
	_led_colors({RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF,RGBLED_COLOR_OFF}),
	_led_blinkmode(RGBLED_BLINK_OFF),
	_mode(RGBLED_MODE_OFF),
	_running(false)
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
RGBLED::setMode(enum ledModes new_mode)
{
	switch (new_mode) {
		case RGBLED_MODE_SYSTEMSTATE:
		case RGBLED_MODE_TEST:
		case RGBLED_MODE_RGB:
			_mode = new_mode;
			if (!_running) {
				_running = true;
				set_on(true);
				work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, 1);
			}
			break;

		case RGBLED_MODE_OFF:

		default:
			if (_running) {
				_running = false;
				set_on(false);
			}
			_mode = RGBLED_MODE_OFF;
			break;
	}

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
	case RGBLED_SET: {
		/* set the specified RGB values */
		memcpy(&_rgb, (struct RGBLEDSet *)arg, sizeof(_rgb));
		setMode(RGBLED_MODE_RGB);
		return OK;
	}
	case RGBLED_SET_COLOR: {
		/* set the specified color name */
		setLEDColor((rgbled_color_t)arg);
	}

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
	static int led_thread_runcount=0;
	static int _led_interval = 1000;

	switch (_mode) {
		case RGBLED_MODE_TEST:
			/* Demo LED pattern for now */
			_led_colors[0] = RGBLED_COLOR_YELLOW;
			_led_colors[1] = RGBLED_COLOR_AMBER;
			_led_colors[2] = RGBLED_COLOR_RED;
			_led_colors[3] = RGBLED_COLOR_PURPLE;
			_led_colors[4] = RGBLED_COLOR_BLUE;
			_led_colors[5] = RGBLED_COLOR_GREEN;
			_led_colors[6] = RGBLED_COLOR_WHITE;
			_led_colors[7] = RGBLED_COLOR_OFF;
			_led_blinkmode = RGBLED_BLINK_ON;
			break;

		case RGBLED_MODE_SYSTEMSTATE:
			/* XXX TODO set pattern */
			_led_colors[0] = RGBLED_COLOR_OFF;
			_led_colors[1] = RGBLED_COLOR_OFF;
			_led_colors[2] = RGBLED_COLOR_OFF;
			_led_colors[3] = RGBLED_COLOR_OFF;
			_led_colors[4] = RGBLED_COLOR_OFF;
			_led_colors[5] = RGBLED_COLOR_OFF;
			_led_colors[6] = RGBLED_COLOR_OFF;
			_led_colors[7] = RGBLED_COLOR_OFF;
			_led_blinkmode = RGBLED_BLINK_OFF;

			break;

		case RGBLED_MODE_RGB:
			set_rgb(_rgb.red, _rgb.green, _rgb.blue);
			_running = false;
			return;

		case RGBLED_MODE_OFF:
		default:
			return;
			break;
	}


	if (led_thread_runcount & 1) {
		if (_led_blinkmode == RGBLED_BLINK_ON)
			setLEDColor(RGBLED_COLOR_OFF);
		_led_interval = RGBLED_OFFTIME;
	} else {
		setLEDColor(_led_colors[(led_thread_runcount/2) % 8]);
		_led_interval = RGBLED_ONTIME;
	}

	led_thread_runcount++;

	if(_running) {
		/* re-queue ourselves to run again later */
		work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, _led_interval);
	} else if (_mode == RGBLED_MODE_RGB) {
		// no need to run again until the colour changes
		set_on(true);
	} else {
		set_on(false);
	}
}

void RGBLED::setLEDColor(rgbled_color_t ledcolor) {
	switch (ledcolor) {
		case RGBLED_COLOR_OFF:	// off
			set_rgb(0,0,0);
			break;
		case RGBLED_COLOR_RED:	// red
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
		case RGBLED_COLOR_BLUE:	// blue
			set_rgb(0,0,255);
			break;
		case RGBLED_COLOR_WHITE:	// white
			set_rgb(255,255,255);
			break;
		case RGBLED_COLOR_AMBER:	// amber
			set_rgb(255,20,0);
			break;
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
	warnx("missing command: try 'start', 'systemstate', 'test', 'info', 'off', 'rgb'");
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
	    fprintf(stderr, "not started\n");
	    rgbled_usage();
	    exit(0);
	}

	if (!strcmp(verb, "test")) {
		g_rgbled->setMode(RGBLED_MODE_TEST);
		exit(0);
	}

	if (!strcmp(verb, "systemstate")) {
		g_rgbled->setMode(RGBLED_MODE_SYSTEMSTATE);
		exit(0);
	}

	if (!strcmp(verb, "info")) {
		g_rgbled->info();
		exit(0);
	}

	if (!strcmp(verb, "off")) {
		g_rgbled->setMode(RGBLED_MODE_OFF);
		exit(0);
	}

	if (!strcmp(verb, "rgb")) {
		int fd = open(RGBLED_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " RGBLED_DEVICE_PATH);
		}
		if (argc < 4) {
			errx(1, "Usage: rgbled rgb <red> <green> <blue>");
		}
		struct RGBLEDSet v;
		v.red   = strtol(argv[1], NULL, 0);
		v.green = strtol(argv[2], NULL, 0);
		v.blue  = strtol(argv[3], NULL, 0);
		int ret = ioctl(fd, RGBLED_SET, (unsigned long)&v);
		close(fd);
		exit(ret);
	}

	rgbled_usage();
}
