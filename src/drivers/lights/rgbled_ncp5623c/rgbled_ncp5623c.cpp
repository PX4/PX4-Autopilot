/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file rgbled_ncp5623c.cpp
 *
 * Driver for the onboard RGB LED controller (NCP5623B or NCP5623C)
 * connected via I2C.
 *
 * @author CUAVcaijie <caijie@cuav.net>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

#define NCP5623B_ADDR       0x38  /**< I2C address of NCP5623B */
#define NCP5623C_ADDR       0x39  /**< I2C address of NCP5623C */

#define NCP5623_LED_CURRENT 0x20  /**< Current register */
#define NCP5623_LED_PWM0    0x40  /**< pwm0 register */
#define NCP5623_LED_PWM1    0x60  /**< pwm1 register */
#define NCP5623_LED_PWM2    0x80  /**< pwm2 register */

#define NCP5623_LED_BRIGHT  0x1f  /**< full brightness */
#define NCP5623_LED_OFF     0x00  /**< off */


class RGBLED_NCP5623C : public device::I2C, public I2CSPIDriver<RGBLED_NCP5623C>
{
public:
	RGBLED_NCP5623C(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_NCP5623C() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();
	virtual int8_t  get_i2c_address() {return get_device_address();}

private:
	int			send_led_rgb();

	int			write(uint8_t reg, uint8_t data);

	float			_brightness{1.0f};

	uint8_t		_r{0};
	uint8_t		_g{0};
	uint8_t		_b{0};
	volatile bool		_running{false};

	LedController		_led_controller;

	uint8_t		_red{NCP5623_LED_PWM0};
	uint8_t		_green{NCP5623_LED_PWM1};
	uint8_t		_blue{NCP5623_LED_PWM2};
};

RGBLED_NCP5623C::RGBLED_NCP5623C(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	int ordering = config.custom1;
	// ordering is RGB: Hundreds is Red, Tens is green and ones is Blue
	// 123 would drive the
	//      R LED from = NCP5623_LED_PWM0
	//      G LED from = NCP5623_LED_PWM1
	//      B LED from = NCP5623_LED_PWM2
	// 321 would drive the
	//      R LED from = NCP5623_LED_PWM2
	//      G LED from = NCP5623_LED_PWM1
	//      B LED from = NCP5623_LED_PWM0
	const uint8_t sig[] = {NCP5623_LED_PWM0, NCP5623_LED_PWM1, NCP5623_LED_PWM2};
	// Process ordering in lsd to msd order.(BGR)
	uint8_t *color[] = {&_blue, &_green, &_red };
	unsigned int s = 0;

	for (unsigned int i = 0; i < arraySize(color); i++) {
		s = (ordering % 10) - 1;

		if (s < arraySize(sig)) {
			*color[i] = sig[s];
		}

		ordering /= 10;
	}
}

int
RGBLED_NCP5623C::write(uint8_t reg, uint8_t data)
{
	uint8_t msg[1] = { 0x00 };
	msg[0] = ((reg & 0xe0) | (data & 0x1f));

	int ret = transfer(&msg[0], 1, nullptr, 0);

	return ret;
}

int
RGBLED_NCP5623C::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	_running = true;

	ScheduleNow();

	return OK;
}

int
RGBLED_NCP5623C::probe()
{
	_retries = 2;
	int status = write(NCP5623_LED_CURRENT, NCP5623_LED_OFF);

	if (status == PX4_ERROR) {
		set_device_address(NCP5623B_ADDR);
		status = write(NCP5623_LED_CURRENT, NCP5623_LED_OFF);
	}

	return status;
}

void
RGBLED_NCP5623C::RunImpl()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = NCP5623_LED_BRIGHT; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = NCP5623_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = NCP5623_LED_BRIGHT; _g = NCP5623_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = NCP5623_LED_BRIGHT; _g = 0; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = NCP5623_LED_BRIGHT; _b = NCP5623_LED_BRIGHT;
			break;

		case led_control_s::COLOR_WHITE:
			_r = NCP5623_LED_BRIGHT; _g = NCP5623_LED_BRIGHT; _b = NCP5623_LED_BRIGHT;
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;
		send_led_rgb();

	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_NCP5623C::send_led_rgb()
{
	uint8_t msg[7] = {0x20, 0x70, 0x40, 0x70, 0x60, 0x70, 0x80};
	uint8_t brightness = UINT8_MAX;

	msg[0] = NCP5623_LED_CURRENT | (brightness & 0x1f);
	msg[2] = _red | (uint8_t(_r * _brightness) & 0x1f);
	msg[4] = _green | (uint8_t(_g * _brightness) & 0x1f);
	msg[6] = _blue | (uint8_t(_b * _brightness) & 0x1f);

	return transfer(&msg[0], 7, nullptr, 0);
}

void
RGBLED_NCP5623C::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x39);
	PRINT_MODULE_USAGE_PARAM_INT('o', 123, 123, 321, "RGB PWM Assignment", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_ncp5623c_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_NCP5623C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = NCP5623C_ADDR;
	cli.custom1 = 123;
	int ch;

	while ((ch = cli.getOpt(argc, argv, "o:")) != EOF) {
		switch (ch) {
		case 'o':
			cli.custom1 = atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli,
				     DRV_LED_DEVTYPE_RGBLED_NCP5623C);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
