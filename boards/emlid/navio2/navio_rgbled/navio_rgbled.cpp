/****************************************************************************
 *
 *   Copyright (c) 2016 - 2017 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/posix.h>
#include <drivers/drv_led.h>

#include <string.h>

#include "navio_rgbled.h"

#define RGBLED_BASE_DEVICE_PATH "/dev/rgbled"

using namespace DriverFramework;


RGBLED::RGBLED(const char *name)
	: DevObj(name,
		 RGBLED0_DEVICE_PATH,
		 RGBLED_BASE_DEVICE_PATH,
		 DeviceBusType_UNKNOWN,
		 0)
	, _gpioR(4)
	, _gpioG(27)
	, _gpioB(6)
{
};

int RGBLED::start()
{
	int res = DevObj::init();

	if (res != 0) {
		DF_LOG_ERR("could not init DevObj (%i)", res);
		return res;
	}

	res = _gpioR.exportPin();

	if (res != 0) {
		PX4_ERR("red led: failed to export");
		goto cleanup;
	}

	res = _gpioR.setDirection(LinuxGPIO::Direction::OUT);

	if (res != 0) {
		PX4_ERR("red led: failed to set direction");
		goto cleanup;
	}

	res = _gpioG.exportPin();

	if (res != 0) {
		PX4_ERR("green led: failed to export");
		goto cleanup;
	}

	res = _gpioG.setDirection(LinuxGPIO::Direction::OUT);

	if (res != 0) {
		PX4_ERR("green led: failed to set direction");
		goto cleanup;
	}

	res = _gpioB.exportPin();

	if (res != 0) {
		PX4_ERR("blue led: failed to export");
		goto cleanup;
	}

	res = _gpioB.setDirection(LinuxGPIO::Direction::OUT);

	if (res != 0) {
		PX4_ERR("blue led: failed to set direction");
		goto cleanup;
	}

	// update at fixed interval
	DevObj::setSampleInterval(_led_controller.maximum_update_interval());

	res = DevObj::start();

	if (res != 0) {
		DF_LOG_ERR("could not start DevObj (%i)", res);
		return res;
	}

	return res;

cleanup:
	_gpioR.unexportPin();
	_gpioG.unexportPin();
	_gpioB.unexportPin();

	return res;
}

int
RGBLED::stop()
{
	_gpioR.unexportPin();
	_gpioG.unexportPin();
	_gpioB.unexportPin();

	int res = DevObj::stop();

	if (res < 0) {
		DF_LOG_ERR("could not stop DevObj");
		//this may not be an error for this device
		return res;
	}

	return 0;
}

void
RGBLED::_measure()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_gpioR.writeValue(LinuxGPIO::Value::LOW);
			_gpioG.writeValue(LinuxGPIO::Value::HIGH);
			_gpioB.writeValue(LinuxGPIO::Value::HIGH);
			break;

		case led_control_s::COLOR_GREEN:
			_gpioR.writeValue(LinuxGPIO::Value::HIGH);
			_gpioG.writeValue(LinuxGPIO::Value::LOW);
			_gpioB.writeValue(LinuxGPIO::Value::HIGH);
			break;

		case led_control_s::COLOR_BLUE:
			_gpioR.writeValue(LinuxGPIO::Value::HIGH);
			_gpioG.writeValue(LinuxGPIO::Value::HIGH);
			_gpioB.writeValue(LinuxGPIO::Value::LOW);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_gpioR.writeValue(LinuxGPIO::Value::LOW);
			_gpioG.writeValue(LinuxGPIO::Value::LOW);
			_gpioB.writeValue(LinuxGPIO::Value::HIGH);
			break;

		case led_control_s::COLOR_PURPLE:
			_gpioR.writeValue(LinuxGPIO::Value::LOW);
			_gpioG.writeValue(LinuxGPIO::Value::HIGH);
			_gpioB.writeValue(LinuxGPIO::Value::LOW);
			break;

		case led_control_s::COLOR_CYAN:
			_gpioR.writeValue(LinuxGPIO::Value::HIGH);
			_gpioG.writeValue(LinuxGPIO::Value::LOW);
			_gpioB.writeValue(LinuxGPIO::Value::LOW);
			break;

		case led_control_s::COLOR_WHITE:
			_gpioR.writeValue(LinuxGPIO::Value::LOW);
			_gpioG.writeValue(LinuxGPIO::Value::LOW);
			_gpioB.writeValue(LinuxGPIO::Value::LOW);
			break;

		default: // led_control_s::COLOR_OFF
			_gpioR.writeValue(LinuxGPIO::Value::HIGH);
			_gpioG.writeValue(LinuxGPIO::Value::HIGH);
			_gpioB.writeValue(LinuxGPIO::Value::HIGH);
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

