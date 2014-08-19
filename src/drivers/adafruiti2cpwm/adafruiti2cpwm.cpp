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
 * @file adafruiti2cpwm.cpp
 *
 * Driver for the adafruit I2C PWM converter based on the PCA9685
 * https://www.adafruit.com/product/815
 *
 * some code is adapted from the arduino library for the board
 * https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 * see the arduino_Adafruit_PWM_Servo_Driver_Library_license.txt file
 * see https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library for contributors
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
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
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>
#include <drivers/drv_io_expander.h>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define ADDR PCA9685_SUBADR1	///< I2C adress

#define ADAFRUITI2CPWM_DEVICE_PATH "/dev/adafruiti2cpwm"
#define ADAFRUITI2CPWM_BUS PX4_I2C_BUS_EXPANSION

class ADAFRUITI2CPWM : public device::I2C
{
public:
	ADAFRUITI2CPWM(int bus=ADAFRUITI2CPWM_BUS, uint8_t address=ADDR);
	virtual ~ADAFRUITI2CPWM();


	virtual int		init();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	bool			is_running() { return _running; }

private:
	work_s			_work;


	enum IOX_MODE		_mode;
	bool			_running;
	int			_i2cpwm_interval;
	bool			_should_run;

	static void		i2cpwm_trampoline(void *arg);
	void			i2cpwm();

};

/* for now, we only support one board */
namespace
{
ADAFRUITI2CPWM *g_adafruiti2cpwm;
}

void adafruiti2cpwm_usage();

extern "C" __EXPORT int adafruiti2cpwm_main(int argc, char *argv[]);

ADAFRUITI2CPWM::ADAFRUITI2CPWM(int bus, uint8_t address) :
	I2C("adafruiti2cpwm", ADAFRUITI2CPWM_DEVICE_PATH, bus, address, 100000),
	_mode(IOX_MODE_OFF),
	_running(false),
	_i2cpwm_interval(SEC2TICK(1.0f/60.0f)),
	_should_run(false)
{
	memset(&_work, 0, sizeof(_work));
}

ADAFRUITI2CPWM::~ADAFRUITI2CPWM()
{
}

int
ADAFRUITI2CPWM::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	return OK;
}

int
ADAFRUITI2CPWM::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	switch (cmd) {

	case IOX_SET_MODE:

		if (_mode != (IOX_MODE)arg) {

			switch ((IOX_MODE)arg) {
			case IOX_MODE_OFF:
			case IOX_MODE_ON:
			case IOX_MODE_TEST_OUT:
				break;

			default:
				return -1;
			}

			_mode = (IOX_MODE)arg;
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
ADAFRUITI2CPWM::i2cpwm_trampoline(void *arg)
{
	ADAFRUITI2CPWM *i2cpwm = reinterpret_cast<ADAFRUITI2CPWM *>(arg);

	i2cpwm->i2cpwm();
}

/**
 * Main loop function
 */
void
ADAFRUITI2CPWM::i2cpwm()
{

	if (_mode == IOX_MODE_TEST_OUT) {

		_should_run = true;
	} else if (_mode == IOX_MODE_OFF) {
		_should_run = false;
	} else {

	}


	// check if any activity remains, else stop
	if (!_should_run) {
		_running = false;
		return;
	}

	// re-queue ourselves to run again later
	_running = true;
	work_queue(LPWORK, &_work, (worker_t)&ADAFRUITI2CPWM::i2cpwm_trampoline, this, _i2cpwm_interval);
}

void
print_usage()
{
	warnx("missing command: try 'start', 'test', 'stop'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_EXPANSION);
	warnx("    -a addr (0x%x)", ADDR);
}

int
adafruiti2cpwm_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int i2caddr = ADDR; // 7bit

	int ch;

	// jump over start/off/etc and look at options first
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			i2caddr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;

		default:
			print_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		print_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_adafruiti2cpwm != nullptr) {
			errx(1, "already started");
		}

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_adafruiti2cpwm = new ADAFRUITI2CPWM(PX4_I2C_BUS_EXPANSION, i2caddr);

			if (g_adafruiti2cpwm != nullptr && OK != g_adafruiti2cpwm->init()) {
				delete g_adafruiti2cpwm;
				g_adafruiti2cpwm = nullptr;
			}

			if (g_adafruiti2cpwm == nullptr) {
				errx(1, "init failed");
			}
		}

		if (g_adafruiti2cpwm == nullptr) {
			g_adafruiti2cpwm = new ADAFRUITI2CPWM(i2cdevice, i2caddr);

			if (g_adafruiti2cpwm == nullptr) {
				errx(1, "new failed");
			}

			if (OK != g_adafruiti2cpwm->init()) {
				delete g_adafruiti2cpwm;
				g_adafruiti2cpwm = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	// need the driver past this point
	if (g_adafruiti2cpwm == nullptr) {
		warnx("not started, run adafruiti2cpwm start");
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(ADAFRUITI2CPWM_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " ADAFRUITI2CPWM_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_MODE, (unsigned long)IOX_MODE_TEST_OUT);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "stop")) {
		fd = open(ADAFRUITI2CPWM_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " ADAFRUITI2CPWM_DEVICE_PATH);
		}

		ret = ioctl(fd, IOX_SET_MODE, (unsigned long)IOX_MODE_OFF);
		close(fd);

		// wait until we're not running any more
		for (unsigned i = 0; i < 15; i++) {
			if (!g_adafruiti2cpwm->is_running()) {
				break;
			}

			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");
		fflush(stdout);

		if (!g_adafruiti2cpwm->is_running()) {
			delete g_adafruiti2cpwm;
			g_adafruiti2cpwm= nullptr;
			warnx("stopped, exiting");
			exit(0);
		} else {
			warnx("stop failed.");
			exit(1);
		}
	}

	print_usage();
	exit(0);
}
