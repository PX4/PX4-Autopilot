/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file rgbled_lp5562.cpp
 *
 * Driver for the RGB LED controller Texas Instruments LP5562 connected via I2C.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <stdint.h>
#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>


using namespace time_literals;

// The addresses are 0x60, 0x62, 0x64, 0x66 according to the datasheet page 27.
// We specify 7bit addresses, hence 0x60 becomes 0x30.
#define I2C_ADDR 0x30

// Unfortunately, there is no WHO_AM_I or device id register, so
// instead we query the W_CURRENT which has a certain pattern
// after reset, and we don't use it or change it, so we don't have
// to reset it and therefore don't mess with a device that we're
// not sure what it is.
static constexpr uint8_t LED_MAP_ADDR = 0x70;
static constexpr uint8_t LED_MAP_ALL_PWM = 0b00000000;

static constexpr uint8_t ENABLE_ADDR = 0x00;
static constexpr uint8_t ENABLE_CHIP_EN = 0b01000000;

static constexpr uint8_t CONFIG_ADDR = 0x08;
static constexpr uint8_t CONFIG_ENABLE_INTERNAL_CLOCK = 0b00000001;

static constexpr uint8_t RESET_ADDR = 0x0D;
static constexpr uint8_t RESET_DO_RESET = 0xFF;

static constexpr uint8_t B_PWM_ADDR = 0x02;

static constexpr uint8_t B_CURRENT_ADDR = 0x05;

static constexpr uint8_t W_CURRENT_ADDR = 0x0F;
static constexpr uint8_t W_CURRENT_DEFAULT = 0b10101111;


class RGBLED_LP5562: public device::I2C, public I2CSPIDriver<RGBLED_LP5562>
{
public:
	RGBLED_LP5562(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_LP5562() = default;

	static void print_usage();

	int init() override;
	int probe() override;

	void RunImpl();

private:
	int read(uint8_t address, uint8_t *data, unsigned count);
	int write(uint8_t address, uint8_t *data, unsigned count);
	int send_led_rgb(uint8_t r, uint8_t g, uint8_t b);

	LedController _led_controller;
	uint8_t _current = 175; // matching default current of 17.5mA
};

RGBLED_LP5562::RGBLED_LP5562(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	_current = config.custom1;
}

int
RGBLED_LP5562::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	uint8_t command[1] = {ENABLE_CHIP_EN};
	ret = write(ENABLE_ADDR, command, sizeof(command));

	if (ret != OK) {
		return ret;
	}

	// We have to wait 500us after enable.
	px4_usleep(500);

	command[0] = CONFIG_ENABLE_INTERNAL_CLOCK;
	ret = write(CONFIG_ADDR, command, sizeof(command));

	if (ret != OK) {
		return ret;
	}

	command[0] = LED_MAP_ALL_PWM;
	ret = write(LED_MAP_ADDR, command, sizeof(command));

	if (ret != OK) {
		return ret;
	}

	// Write all 3 colors at once.
	uint8_t currents[3] = {_current, _current, _current};
	ret = write(B_CURRENT_ADDR, currents, sizeof(currents));

	if (ret != OK) {
		return ret;
	}

	ScheduleNow();

	return OK;
}

int
RGBLED_LP5562::probe()
{
	uint8_t result[1] = {0};
	int ret = read(W_CURRENT_ADDR, result, sizeof(result));

	if (ret != OK) {
		return ret;
	}

	_retries = 1;

	return (result[0] == W_CURRENT_DEFAULT) ? OK : ERROR;
}

int
RGBLED_LP5562::read(uint8_t address, uint8_t *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int
RGBLED_LP5562::write(uint8_t address, uint8_t *data, unsigned count)
{
	uint8_t buf[4];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

void
RGBLED_LP5562::RunImpl()
{
	if (should_exit()) {
		send_led_rgb(0, 0, 0);
		return;
	}

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {

		const uint8_t on = led_control_data.leds[0].brightness;

		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			send_led_rgb(on, 0, 0);
			break;

		case led_control_s::COLOR_GREEN:
			send_led_rgb(0, on, 0);
			break;

		case led_control_s::COLOR_BLUE:
			send_led_rgb(0, 0, on);
			break;

		case led_control_s::COLOR_AMBER: // same as yellow
		case led_control_s::COLOR_YELLOW:
			send_led_rgb(on, on, 0);
			break;

		case led_control_s::COLOR_PURPLE:
			send_led_rgb(on, 0, on);
			break;

		case led_control_s::COLOR_CYAN:
			send_led_rgb(0, on, on);
			break;

		case led_control_s::COLOR_WHITE:
			send_led_rgb(on, on, on);
			break;

		case led_control_s::COLOR_OFF:
		default:
			send_led_rgb(0, 0, 0);
			break;
		}

	}

	ScheduleDelayed(_led_controller.maximum_update_interval());
}

int
RGBLED_LP5562::send_led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t leds[3] = {b, g, r};
	return write(B_PWM_ADDR, leds, sizeof(leds));
}

void
RGBLED_LP5562::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for [LP5562](https://www.ti.com/product/LP5562) LED driver connected via I2C.

This used in some GPS modules by Holybro for [PX4 status notification](../getting_started/led_meanings.md)

The driver is included by default in firmware (KConfig key DRIVERS_LIGHTS_RGBLED_LP5562) and is always enabled.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("rgbled_lp5562", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(I2C_ADDR);
	PRINT_MODULE_USAGE_PARAM_FLOAT('u', 17.5f, 0.1f, 25.5f, "Current in mA", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_lp5562_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = RGBLED_LP5562;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = I2C_ADDR;
	cli.custom1 = 175;

	while ((ch = cli.getOpt(argc, argv, "u:")) != EOF) {
		switch (ch) {
		case 'u':
			float v = atof(cli.optArg());

			if (v >= 0.1f && v <= 25.5f) {
				cli.custom1 = ((uint8_t)(v * 10.f));

			} else {
				PX4_ERR("current out of range");
				return -1;
			}

			break;
		}
	}


	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED_LP5562);

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
