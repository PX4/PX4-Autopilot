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
#include <drivers/drv_led.h>

#include "navio_rgbled.h"

#define GPIO_LED_CNF    (GPIO_CNF_OUTPUT)
#define GPIO_LED_R      (GPIO_PIN4)
#define GPIO_LED_G      (GPIO_PIN27)
#define GPIO_LED_B      (GPIO_PIN6)

#define RGBLED_BASE_DEVICE_PATH "/dev/rgbled"

// inverted
#define LED_ON  0
#define LED_OFF 1

using namespace DriverFramework;


RGBLED::RGBLED(const char *name)
	: DevObj(name,
		 RGBLED0_DEVICE_PATH,
		 RGBLED_BASE_DEVICE_PATH,
		 DeviceBusType_UNKNOWN,
		 0)
{
};

RGBLED::~RGBLED()
{
	if (_led_controller.led_control_subscription() >= 0) {
		orb_unsubscribe(_led_controller.led_control_subscription());
	}
};

int RGBLED::start()
{
	int res = DevObj::init();

	if (res != 0) {
		DF_LOG_ERR("could not init DevObj (%i)", res);
		return res;
	}

	_gpio.start();

	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_R);
	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_G);
	_gpio.configgpio(GPIO_LED_CNF | GPIO_LED_B);

	// update at fixed interval
	DevObj::setSampleInterval(_led_controller.maximum_update_interval());

	res = DevObj::start();

	if (res != 0) {
		DF_LOG_ERR("could not start DevObj (%i)", res);
		return res;
	}

	return res;
}

int RGBLED::stop()
{
	int res;

	_gpio.stop();

	res = DevObj::stop();

	if (res < 0) {
		DF_LOG_ERR("could not stop DevObj");
		//this may not be an error for this device
		return res;
	}

	return 0;
}

void RGBLED::_measure()
{
	if (!_led_controller.is_init()) {
		int led_control_sub = orb_subscribe(ORB_ID(led_control));
		_led_controller.init(led_control_sub);
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_gpio.gpiowrite(GPIO_LED_R, LED_ON);
			_gpio.gpiowrite(GPIO_LED_G, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_B, LED_OFF);
			break;

		case led_control_s::COLOR_GREEN:
			_gpio.gpiowrite(GPIO_LED_R, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_G, LED_ON);
			_gpio.gpiowrite(GPIO_LED_B, LED_OFF);
			break;

		case led_control_s::COLOR_BLUE:
			_gpio.gpiowrite(GPIO_LED_R, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_G, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_B, LED_ON);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_gpio.gpiowrite(GPIO_LED_R, LED_ON);
			_gpio.gpiowrite(GPIO_LED_G, LED_ON);
			_gpio.gpiowrite(GPIO_LED_B, LED_OFF);
			break;

		case led_control_s::COLOR_PURPLE:
			_gpio.gpiowrite(GPIO_LED_R, LED_ON);
			_gpio.gpiowrite(GPIO_LED_G, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_B, LED_ON);
			break;

		case led_control_s::COLOR_CYAN:
			_gpio.gpiowrite(GPIO_LED_R, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_G, LED_ON);
			_gpio.gpiowrite(GPIO_LED_B, LED_ON);
			break;

		case led_control_s::COLOR_WHITE:
			_gpio.gpiowrite(GPIO_LED_R, LED_ON);
			_gpio.gpiowrite(GPIO_LED_G, LED_ON);
			_gpio.gpiowrite(GPIO_LED_B, LED_ON);
			break;

		default: // led_control_s::COLOR_OFF
			_gpio.gpiowrite(GPIO_LED_R, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_G, LED_OFF);
			_gpio.gpiowrite(GPIO_LED_B, LED_OFF);
			break;
		}
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
	PX4_INFO("Usage: navio_rgbled 'start', 'stop'");
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

