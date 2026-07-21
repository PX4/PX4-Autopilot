/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file rgbled_apa102.cpp
 *
 * Driver for the onboard RGB LED controller (APA102) connected via SPI.
 *
 * @author Andrew Brahim <brahim@ascendengineer.com>
 */

#include <string.h>

#include <lib/led/led.h>
#include <lib/drivers/device/spi.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

static constexpr uint32_t SPI_SPEED = 1 * 1000 * 1000; // 1MHz SPI serial interface

using namespace time_literals;

class RGBLED_APA102 : public device::SPI, public I2CSPIDriver<RGBLED_APA102>
{
public:
	RGBLED_APA102(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_APA102() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

private:
	int			send_led_enable(uint8_t color_);
	int			send_led_rgb(uint8_t r, uint8_t g, uint8_t b);

	bool			_led_opened{true};
	int start_frame();
	void end_frame(uint16_t count);
	LedController		_led_controller;
};

RGBLED_APA102::RGBLED_APA102(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config)
{
}

int
RGBLED_APA102::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);

	// kick off work queue
	ScheduleNow();

	return OK;
}

int
RGBLED_APA102::probe()
{
	// Check if device responds by reading RSTR register (should return 0x09)

	int ret = start_frame();
	return ret;
}

int
RGBLED_APA102::start_frame()
{
	uint8_t data = 0;

	int ret = 0;

	for (int i = 0; i < 4; i++) {
		ret |= transfer(&data, nullptr, 1);

	}

	return ret;
}

void
RGBLED_APA102::end_frame(uint16_t count)
{

	uint8_t ef = 0xFF;

	for (uint16_t i = 0; i < (count + 14) / 16; i++) {
		transfer(&ef, nullptr, 1);
	}
}

void
RGBLED_APA102::RunImpl()
{

	const uint16_t led_count = BOARD_MAX_LEDS;

	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		int ret = start_frame();

		if (ret != PX4_OK) {
			PX4_ERR("Unable to init");
		}


		for (uint16_t i = 0; i < led_count; i++) {
			const float scale = (float)led_control_data.leds[i].brightness / 255.f;
			const uint8_t on = (uint8_t)(scale * 255.f + 0.5f);

			switch (led_control_data.leds[i].color) {
			case led_control_s::COLOR_RED:
				send_led_rgb(on, 0, 0);
				break;

			case led_control_s::COLOR_GREEN:
				send_led_rgb(0, on, 0);
				break;

			case led_control_s::COLOR_BLUE:
				send_led_rgb(0, 0, on);
				break;

			case led_control_s::COLOR_AMBER: //make it the same as yellow
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

			default: // led_control_s::COLOR_OFF
				send_led_rgb(0, 0, 0);
				break;
			}

		}

		end_frame(led_count);
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Sent ENABLE flag to LED driveer
 */
int
RGBLED_APA102::send_led_enable(uint8_t color_)
{
	if (color_ == 0) {
		send_led_rgb(0, 0, 0);
	}

	return 0;
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_APA102::send_led_rgb(uint8_t red, uint8_t green, uint8_t blue)
{

	int ret = 0;
	uint8_t global_setting = 0xE0 | 0x1F; // 111b marker + 5-bit global brightness (31 = max); dimming done per-channel
	ret = transfer(&global_setting, nullptr, 1);
	ret |= transfer(&blue, nullptr, 1);
	ret |= transfer(&green, nullptr, 1);
	ret |= transfer(&red, nullptr, 1);

	return ret;
}

void
RGBLED_APA102::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled_apa102", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_apa102_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_APA102;
	BusCLIArguments cli{false, true};
	cli.custom1 = -1;
	cli.spi_mode = SPIDEV_MODE3;
	cli.default_spi_frequency = SPI_SPEED;
	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED_APA102);

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
