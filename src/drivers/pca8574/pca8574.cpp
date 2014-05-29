/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * Driver for an 8 I/O controller (PC8574) connected via I2C.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
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

#define ADDR			0x20	///< I2C adress of PCA8574 (default, A0-A2 pulled to GND)

class PCA8574 : public device::I2C
{
public:
	PCA8574(int bus, int pca8574);
	virtual ~PCA8574();


	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	bool			is_running() { return _running; }

private:
	work_s			_work;

	uint8_t			_values_out;
	uint8_t			_values_in;
	uint8_t			_blinking;
	uint8_t			_blink_phase;

	enum IOX_MODE		_mode;
	bool			_running;
	int			_led_interval;
	bool			_should_run;
	bool			_update_out;
	int			_counter;

	static void		led_trampoline(void *arg);
	void			led();

	int			send_led_enable(uint8_t arg);
	int			send_led_values();

	int			get(uint8_t &vals);
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
	_values_out(0),
	_values_in(0),
	_blinking(0),
	_blink_phase(0),
	_mode(IOX_MODE_OFF),
	_running(false),
	_led_interval(80),
	_should_run(false),
	_update_out(false),
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

	return OK;
}

int
PCA8574::probe()
{
	uint8_t val;
	return get(val);
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
	case IOX_SET_VALUE ...(IOX_SET_VALUE + 8): {
			// set the specified on / off state
			uint8_t position = (1 << (cmd - IOX_SET_VALUE));
			uint8_t prev = _values_out;

			if (arg) {
				_values_out |= position;

			} else {
				_values_out &= ~(position);
			}

			if (_values_out != prev) {
				if (_values_out) {
					_mode = IOX_MODE_ON;
				}
				send_led_values();
			}

			return OK;
		}

	case IOX_SET_MASK:
		send_led_enable(arg);
		return OK;

	case IOX_GET_MASK: {
			uint8_t val;
			ret = get(val);

			if (ret == OK) {
				return val;

			} else {
				return -1;
			}
		}

	case IOX_SET_MODE:

		if (_mode != (IOX_MODE)arg) {

			switch ((IOX_MODE)arg) {
			case IOX_MODE_OFF:
				_values_out = 0xFF;
				break;

			case IOX_MODE_ON:
				_values_out = 0;
				break;

			case IOX_MODE_TEST_OUT:
				break;

			default:
				return -1;
			}

			_mode = (IOX_MODE)arg;
			send_led_values();
		}

		return OK;

	default:
		// see if the parent class can make any use of it
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
	if (_mode == IOX_MODE_TEST_OUT) {

		// we count only seven states
		_counter &= 0xF;
		_counter++;

		for (int i = 0; i < 8; i++) {
			if (i < _counter) {
				_values_out |= (1 << i);

			} else {
				_values_out &= ~(1 << i);
			}
		}

		_update_out = true;
		_should_run = true;
	} else if (_mode == IOX_MODE_OFF) {
		_update_out = true;
		_should_run = false;
	} else {

		// Any of the normal modes
		if (_blinking > 0) {
			/* we need to be running to blink */
			_should_run = true;
		} else {
			_should_run = false;
		}
	}

	if (_update_out) {
		uint8_t msg;

		if (_blinking) {
			msg = (_values_out & _blinking & _blink_phase);

			// wipe out all positions that are marked as blinking
			msg &= ~(_blinking);

			// fill blink positions
			msg |= ((_blink_phase) ? _blinking : 0);

			_blink_phase = !_blink_phase;
		} else {
			msg = _values_out;
		}

		int ret = transfer(&msg, sizeof(msg), nullptr, 0);

		if (!ret) {
			_update_out = false;
		}
	}

	// check if any activity remains, else stp
	if (!_should_run) {
		_running = false;
		return;
	}

	// re-queue ourselves to run again later
	_running = true;
	work_queue(LPWORK, &_work, (worker_t)&PCA8574::led_trampoline, this, _led_interval);
}

/**
 * Sent ENABLE flag to LED driver
 */
int
PCA8574::send_led_enable(uint8_t arg)
{

	int ret = transfer(&arg, sizeof(arg), nullptr, 0);

	return ret;
}

/**
 * Send 8 outputs
 */
int
PCA8574::send_led_values()
{
	_update_out = true;

	// if not active, kick it
	if (!_running) {
		_running = true;
		work_queue(LPWORK, &_work, (worker_t)&PCA8574::led_trampoline, this, 1);
	}

	return 0;
}

int
PCA8574::get(uint8_t &vals)
{
	uint8_t result;
	int ret;

	ret = transfer(nullptr, 0, &result, 1);

	if (ret == OK) {
		_values_in = result;
		vals = result;
	}

	return ret;
}

void
pca8574_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'val 0 1'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", ADDR);
}

int
pca8574_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int pca8574adr = ADDR; // 7bit

	int ch;

	// jump over start/off/etc and look at options first
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
		if (g_pca8574 != nullptr) {
			errx(1, "already started");
		}

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

			if (g_pca8574 == nullptr) {
				errx(1, "new failed");
			}

			if (OK != g_pca8574->init()) {
				delete g_pca8574;
				g_pca8574 = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	// need the driver past this point
	if (g_pca8574 == nullptr) {
		warnx("not started, run pca8574 start");
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_MODE, (unsigned long)IOX_MODE_TEST_OUT);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_pca8574->info();
		exit(0);
	}

	if (!strcmp(verb, "off")) {
		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd < 0) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_MODE, (unsigned long)IOX_MODE_OFF);
		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "stop")) {
		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_MODE, (unsigned long)IOX_MODE_OFF);
		close(fd);

		// wait until we're not running any more
		for (unsigned i = 0; i < 15; i++) {
			if (!g_pca8574->is_running()) {
				break;
			}

			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");
		fflush(stdout);

		if (!g_pca8574->is_running()) {
			delete g_pca8574;
			g_pca8574 = nullptr;
			exit(0);
		} else {
			warnx("stop failed.");
			exit(1);
		}
	}

	if (!strcmp(verb, "val")) {
		if (argc < 4) {
			errx(1, "Usage: pca8574 val <channel> <0 or 1>");
		}

		fd = open(PCA8574_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " PCA8574_DEVICE_PATH);
		}

		unsigned channel = strtol(argv[2], NULL, 0);
		unsigned val = strtol(argv[3], NULL, 0);

		if (channel < 8) {
			ret = ioctl(fd, (IOX_SET_VALUE + channel), val);
		} else {
			ret = -1;
		}
		close(fd);
		exit(ret);
	}

	pca8574_usage();
	exit(0);
}
