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
 * @file pca8574.cpp
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

#include <drivers/drv_io_expander.h>

#define PCA8574_ONTIME 120
#define PCA8574_OFFTIME 120
#define PCA8574_DEVICE_PATH "/dev/pca8574"

#define ADDR			0x20	/**< I2C adress of PCA8574 (default, A0-A2 pulled to GND) */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */


class PCA8574 : public device::I2C
{
public:
	PCA8574(int bus, int pca8574);
	virtual ~PCA8574();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	work_s			_work;

	uint8_t			_values[8];
	float			_brightness;

	enum IOX_MODE		_mode;
	bool			_running;
	int			_led_interval;
	bool			_should_run;
	int			_counter;

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(bool enable);
	int			send_led_values();
};

/* for now, we only support one PCA8574 */
namespace
{
PCA8574 *g_pca8574;
}

void pca8574_usage();

extern "C" __EXPORT int pca8574_main(int argc, char *argv[]);

PCA8574::PCA8574(int bus, int pca8574) :
	I2C("pca8574", PCA8574_DEVICE_PATH, bus, pca8574, 100000),
	_mode(IOX_MODE_OFF),
	_values({}),
	_brightness(1.0f),
	_running(false),
	_led_interval(0),
	_should_run(false),
	_counter(0)
{
	memset(&_work, 0, sizeof(_work));
}

PCA8574::~PCA8574()
{
}

int
PCA8574::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);

	/* kick it in */
	_should_run = true;
	_led_interval = 80;
	work_queue(LPWORK, &_work, (worker_t)&PCA8574::led_trampoline, this, 1);

	return OK;
}

int
PCA8574::probe()
{

	return send_led_enable(false);
}

int
PCA8574::info()
{
	int ret = OK;

	return ret;
}

int
PCA8574::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case IOX_SET_VALUE ... (IOX_SET_VALUE + 8):
	{
		/* set the specified color */
		unsigned prev = _values[cmd - IOX_SET_VALUE];
		_values[cmd - IOX_SET_VALUE] = arg;
		if (_values[cmd - IOX_SET_VALUE] != prev) {
			// XXX will be done with a change flag
			send_led_values();
		}
		return OK;
	}

	case IOX_ENABLE:
		send_led_enable(arg);
		return OK;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}


void
PCA8574::led_trampoline(void *arg)
{
	PCA8574 *rgbl = reinterpret_cast<PCA8574 *>(arg);

	rgbl->led();
}

/**
 * Main loop function
 */
void
PCA8574::led()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	// switch (_mode) {
	// case PCA8574_MODE_BLINK_SLOW:
	// case PCA8574_MODE_BLINK_NORMAL:
	// case PCA8574_MODE_BLINK_FAST:
	// 	if (_counter >= 2)
	// 		_counter = 0;

	// 	send_led_enable(_counter == 0);

	// 	break;

	// case PCA8574_MODE_BREATHE:

	// 	if (_counter >= 62)
	// 		_counter = 0;

	// 	int n;

	// 	if (_counter < 32) {
	// 		n = _counter;

	// 	} else {
	// 		n = 62 - _counter;
	// 	}

	// 	_brightness = n * n / (31.0f * 31.0f);
	// 	send_led_rgb();
	// 	break;

	// case PCA8574_MODE_PATTERN:

	// 	/* don't run out of the pattern array and stop if the next frame is 0 */
	// 	if (_counter >= PCA8574_PATTERN_LENGTH || _pattern.duration[_counter] <= 0)
	// 		_counter = 0;

	// 	set_color(_pattern.color[_counter]);
	// 	send_led_rgb();
	// 	_led_interval = _pattern.duration[_counter];
	// 	break;

	// default:
	// 	break;
	// }


	// we count only seven states
	_counter &= 0xF;
	_counter++;

	for (int i = 0; i < 8; i++) {
		if (i < _counter) {
			_values[i] = 1;
		} else {
			_values[i] = 0;
		}
	}

	send_led_values();

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&PCA8574::led_trampoline, this, _led_interval);
}

// /**
//  * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
//  */
// void
// PCA8574::set_mode(pca8574_mode_t mode)
// {
// 	if (mode != _mode) {
// 		_mode = mode;

// 		switch (mode) {
// 		// case PCA8574_MODE_OFF:
// 		// 	_should_run = false;
// 		// 	send_led_enable(false);
// 		// 	break;

// 		// case PCA8574_MODE_ON:
// 		// 	_brightness = 1.0f;
// 		// 	send_led_rgb();
// 		// 	send_led_enable(true);
// 		// 	break;

// 		// case PCA8574_MODE_BLINK_SLOW:
// 		// 	_should_run = true;
// 		// 	_counter = 0;
// 		// 	_led_interval = 2000;
// 		// 	_brightness = 1.0f;
// 		// 	send_led_rgb();
// 		// 	break;

// 		// case PCA8574_MODE_BLINK_NORMAL:
// 		// 	_should_run = true;
// 		// 	_counter = 0;
// 		// 	_led_interval = 500;
// 		// 	_brightness = 1.0f;
// 		// 	send_led_rgb();
// 		// 	break;

// 		// case PCA8574_MODE_BLINK_FAST:
// 		// 	_should_run = true;
// 		// 	_counter = 0;
// 		// 	_led_interval = 100;
// 		// 	_brightness = 1.0f;
// 		// 	send_led_rgb();
// 		// 	break;

// 		// case PCA8574_MODE_BREATHE:
// 		// 	_should_run = true;
// 		// 	_counter = 0;
// 		// 	_led_interval = 25;
// 		// 	send_led_enable(true);
// 		// 	break;

// 		// case PCA8574_MODE_PATTERN:
// 		// 	_should_run = true;
// 		// 	_counter = 0;
// 		// 	_brightness = 1.0f;
// 		// 	send_led_enable(true);
// 		// 	break;

// 		default:
// 			warnx("mode unknown");
// 			break;
// 		}

// 		/* if it should run now, start the workq */
// 		if (_should_run && !_running) {
// 			_running = true;
// 			work_queue(LPWORK, &_work, (worker_t)&PCA8574::led_trampoline, this, 1);
// 		}

// 	}
// }

/**
 * Sent ENABLE flag to LED driver
 */
int
PCA8574::send_led_enable(bool enable)
{
	uint8_t msg;

	if (enable) {
		/* active low */
		msg = 0x00;
	} else {
		/* active low, so off */
		msg = 0xFF;
	}

	int ret = transfer(&msg, sizeof(msg), nullptr, 0);

	return ret;
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
PCA8574::send_led_values()
{
	uint8_t msg = 0;

	for (int i = 0; i < 8; i++) {
		if (_values[i]) {
			msg |= (1 << i);
		}
	}

	int ret = transfer(&msg, sizeof(msg), nullptr, 0);
}

// int
// PCA8574::get(bool &on, bool &powersave, uint8_t &r, uint8_t &g, uint8_t &b)
// {
// 	uint8_t result[2];
// 	int ret;

// 	ret = transfer(nullptr, 0, &result[0], 2);

// 	if (ret == OK) {
// 		on = result[0] & SETTING_ENABLE;
// 		powersave = !(result[0] & SETTING_NOT_POWERSAVE);
// 		/* XXX check, looks wrong */
// 		r = (result[0] & 0x0f) << 4;
// 		g = (result[1] & 0xf0);
// 		b = (result[1] & 0x0f) << 4;
// 	}

// 	return ret;
// }

void
pca8574_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'val 0 100'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", ADDR);
}

int
pca8574_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int pca8574adr = ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			pca8574adr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;

		default:
			pca8574_usage();
			exit(0);
		}
	}

        if (optind >= argc) {
            pca8574_usage();
            exit(1);
        }

	const char *verb = argv[optind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_pca8574 != nullptr)
			errx(1, "already started");

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_pca8574 = new PCA8574(PX4_I2C_BUS_EXPANSION, pca8574adr);

			if (g_pca8574 != nullptr && OK != g_pca8574->init()) {
				delete g_pca8574;
				g_pca8574 = nullptr;
			}

			if (g_pca8574 == nullptr) {
				// fall back to default bus
				if (PX4_I2C_BUS_LED == PX4_I2C_BUS_EXPANSION) {
					errx(1, "init failed");
				}
				i2cdevice = PX4_I2C_BUS_LED;
			}
		}

		if (g_pca8574 == nullptr) {
			g_pca8574 = new PCA8574(i2cdevice, pca8574adr);

			if (g_pca8574 == nullptr)
				errx(1, "new failed");

			if (OK != g_pca8574->init()) {
				delete g_pca8574;
				g_pca8574 = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_pca8574 == nullptr) {
		warnx("not started");
		pca8574_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_VALUE, 255);
		// ret = ioctl(fd, PCA8574_SET_MODE, (unsigned long)PCA8574_MODE_PATTERN);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_pca8574->info();
		exit(0);
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_MODE, (unsigned long)IOX_MODE_OFF);
		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "stop")) {
		delete g_pca8574;
		g_pca8574 = nullptr;
		exit(0);
	}

	if (!strcmp(verb, "val")) {
		if (argc < 4) {
			errx(1, "Usage: pca8574 val <channel> <value>");
		}

		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		unsigned channel = strtol(argv[2], NULL, 0);
		unsigned val = strtol(argv[3], NULL, 0);
		ret = ioctl(fd, (IOX_SET_VALUE+channel), val);
		ret = ioctl(fd, IOX_ENABLE, 1);
		close(fd);
		exit(ret);
	}

	pca8574_usage();
	exit(0);
}
