/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <px4_posix.h>
#include <drivers/drv_rgbled.h>

#include "navio_rgbled.h"

#define GPIO_LED_CNF    (GPIO_CNF_OUTPUT)
#define GPIO_LED_R      (GPIO_PIN4)
#define GPIO_LED_G      (GPIO_PIN27)
#define GPIO_LED_B      (GPIO_PIN6)

using namespace DriverFramework;


int RGBLED::start()
{
	int res;

	res = DevObj::init();

	if (res != 0) {
		DF_LOG_ERR("error: could not init DevObj");
		return res;
	}

	_gpio.start();

	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_R);
	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_G);
	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_B);

	return 0;
}

int RGBLED::stop()
{
	int res;

	_gpio.stop();

	res = DevObj::stop();

	if (res < 0) {
		DF_LOG_ERR("error: could not stop DevObj");
		//this may not be an error for this device
		return res;
	}

	return 0;
}

int RGBLED::devIOCTL(unsigned long request, unsigned long arg)
{
	int ret = ENOTTY;
	rgbled_rgbset_t *rgb;

	switch (request) {
	case RGBLED_SET_RGB:
		ret = 0;
		rgb = (rgbled_rgbset_t *)arg;
		_rgb.red = (rgb->red != 0) ? LED_ON : LED_OFF;
		_rgb.green = (rgb->green != 0) ? LED_ON : LED_OFF;
		_rgb.blue = (rgb->blue != 0) ? LED_ON : LED_OFF;
		_gpio.gpiowrite(GPIO_LED_R, _rgb.red);
		_gpio.gpiowrite(GPIO_LED_G, _rgb.green);
		_gpio.gpiowrite(GPIO_LED_B, _rgb.blue);
		break;

	case RGBLED_SET_COLOR:
		if (arg > _max_color) {
			ret = ENOTSUP;

		} else {
			_rgb = _rgbsets[arg];
			_gpio.gpiowrite(GPIO_LED_R, _rgb.red);
			_gpio.gpiowrite(GPIO_LED_G, _rgb.green);
			_gpio.gpiowrite(GPIO_LED_B, _rgb.blue);
			ret = 0;
		}

		break;

	case RGBLED_SET_MODE:
		ret = 0;

		switch (arg) {
		case RGBLED_MODE_ON:
			DevObj::setSampleInterval(0);
			break;

		case RGBLED_MODE_BLINK_SLOW:
			DevObj::setSampleInterval(2000 * 1000);
			break;

		case RGBLED_MODE_BLINK_NORMAL:
			DevObj::setSampleInterval(500 * 1000);
			break;

		case RGBLED_MODE_BLINK_FAST:
			DevObj::setSampleInterval(100 * 1000);
			break;

		case RGBLED_MODE_BREATHE:
			DevObj::setSampleInterval(1500 * 1000);
			break;

		default:
			ret = ENOTSUP;
		}

		if (!m_work_handle.isValid()) {
			// this can fail
			DevObj::start();
		}

		break;

	case RGBLED_PLAY_SCRIPT_NAMED:
	case RGBLED_PLAY_SCRIPT:
	case RGBLED_SET_USER_SCRIPT:
	case RGBLED_SET_PATTERN:
		ret = ENOTSUP;
		break;

	default:
		ret = DevObj::devIOCTL(request, arg);
		break;
	}

	return ret;
}

void RGBLED::_measure()
{
	if (_turn) {
		_gpio.gpiowrite(GPIO_LED_R, LED_OFF);
		_gpio.gpiowrite(GPIO_LED_G, LED_OFF);
		_gpio.gpiowrite(GPIO_LED_B, LED_OFF);
		_turn = false;

	} else {
		_gpio.gpiowrite(GPIO_LED_R, _rgb.red);
		_gpio.gpiowrite(GPIO_LED_G, _rgb.green);
		_gpio.gpiowrite(GPIO_LED_B, _rgb.blue);
		_turn = true;
	}
}

extern "C" { __EXPORT int navio_rgbled_main(int argc, char *argv[]); }

namespace navio_rgbled
{
int start();
int stop();
void usage();

RGBLED *g_dev = nullptr;

int start()
{
	g_dev = new RGBLED("navio_rgbled");

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating RGBLED");
		return -1;
	}

	return g_dev->start();
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("not running");
		return -1;
	}

	g_dev->stop();

	delete g_dev;
	g_dev = nullptr;

	return 0;
}

void usage()
{
	PX4_WARN("Usage: navio_rgbled 'start', 'stop'");
}

} //namespace navio_rgbled

int navio_rgbled_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		navio_rgbled::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = navio_rgbled::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = navio_rgbled::stop();
	}

	else {
		navio_rgbled::usage();
		return 1;
	}

	return ret;
}

