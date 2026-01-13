/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file rgbled_aw2023.cpp
 *
 * Driver for the onboard RGB LED controller (AW2023) connected via I2C.
 *
 * @author Amovlab Lv Guofei <service@amovauto.com>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

/**
 * @brief register address
 *
 */
#define AW2023_REG_RSTR                      0x00      /**< chip id and software reset register*/
#define AW2023_REG_GCR1                      0x01      /**< global control register*/

#define AW2023_REG_LCTR                      0x30      /**< led control register*/
#define AW2023_REG_LCFG0                     0x31      /**< led0 mode configuration register*/
#define AW2023_REG_LCFG1                     0x32      /**< led1 mode configuration register*/
#define AW2023_REG_LCFG2                     0x33      /**< led2 mode configuration register*/
#define AW2023_REG_PWM0		             0x34      /** Ppwm dimming register*/
#define AW2023_REG_PWM1	                     0x35      /** pwm dimming register*/
#define AW2023_REG_PWM2		             0x36      /** pwm dimming register*/

#define AW2023_ADDR                          0x45      /**< I2C address of AW2023   */

/**
 * @brief register bits
 *
 */

#define AW2023_CHIPEN			     0X01  /**< enable, the device enters active state*/
#define AW2023_CHIPID       		     0x09  /**< chip ID*/
#define AW2023_LED0_ENABLE   	             0x01  /**< LED0 Enable (blue)*/
#define AW2023_LED2_ENABLE  	             0x02  /**< LED2 Enable (green)*/
#define AW2023_LED1_ENABLE    	   	     0x04  /**< LED1 Enable (red)*/
#define AW2023_LED2_CUR  	             0x0F  /**< LED output current*/
#define AW2023_LED_PWM  		     0XFF  /**< full brightness */

class RGBLED_AW2023 : public device::I2C, public I2CSPIDriver<RGBLED_AW2023>
{
public:
	RGBLED_AW2023(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_AW2023() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

private:
	int			send_led_enable(uint8_t color_);
	int			send_led_rgb();

	bool			_led_opened{true};

	LedController		_led_controller;
};

RGBLED_AW2023::RGBLED_AW2023(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

int
RGBLED_AW2023::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/* switch off LED on start */
	send_led_enable(false);
	send_led_rgb();

	// kick off work queue
	ScheduleNow();

	return OK;
}

int
RGBLED_AW2023::probe()
{
	// Check if device responds by reading RSTR register (should return 0x09)
	uint8_t data = 0;
	const uint8_t reg = AW2023_REG_RSTR;

	for (id_t i = 0; i < 3; i++) {
		if (transfer(&reg, 1, &data, 1) == PX4_OK) {

			// Expected chip ID is 0x09
			if (data == AW2023_CHIPID) {
				return PX4_OK;
			}
		}
	}

	return PX4_ERROR;
}

void
RGBLED_AW2023::RunImpl()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			send_led_enable(AW2023_LED1_ENABLE);
			break;

		case led_control_s::COLOR_GREEN:
			send_led_enable(AW2023_LED2_ENABLE);
			break;

		case led_control_s::COLOR_BLUE:
			send_led_enable(AW2023_LED0_ENABLE);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			send_led_enable(AW2023_LED1_ENABLE | AW2023_LED2_ENABLE);
			break;

		case led_control_s::COLOR_PURPLE:
			send_led_enable(AW2023_LED1_ENABLE | AW2023_LED0_ENABLE);
			break;

		case led_control_s::COLOR_CYAN:
			send_led_enable(AW2023_LED2_ENABLE | AW2023_LED0_ENABLE);
			break;

		case led_control_s::COLOR_WHITE:
			send_led_enable(AW2023_LED1_ENABLE | AW2023_LED2_ENABLE | AW2023_LED0_ENABLE);
			break;

		default: // led_control_s::COLOR_OFF
			send_led_enable(0);
			break;
		}
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Sent ENABLE flag to LED driveer
 */
int
RGBLED_AW2023::send_led_enable(uint8_t color_)
{
	uint8_t msg[2] = {AW2023_REG_LCTR, color_};
	return transfer(msg, sizeof(msg), nullptr, 0);
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_AW2023::send_led_rgb()
{
	uint8_t msg[2] = {AW2023_REG_GCR1, AW2023_CHIPEN};
	int ret = transfer(msg, 2, nullptr, 0);

	if (ret == PX4_OK) {
		uint8_t msg0[2] = {0, 0};
		uint8_t msg1[2] = {0, 0};
		msg0[1] = AW2023_LED2_CUR;
		msg1[1] = AW2023_LED_PWM;

		msg0[0] = AW2023_REG_LCFG0;
		transfer(msg0, 2, nullptr, 0);
		msg0[0] = AW2023_REG_LCFG1;
		transfer(msg0, 2, nullptr, 0);
		msg0[0] = AW2023_REG_LCFG2;
		transfer(msg0, 2, nullptr, 0);


		msg1[0] = AW2023_REG_PWM2;
		transfer(msg1, 2, nullptr, 0);
		msg1[0] = AW2023_REG_PWM1;
		transfer(msg1, 2, nullptr, 0);
		msg1[0] = AW2023_REG_PWM0;
		transfer(msg1, 2, nullptr, 0);

	} else {
		PX4_DEBUG("chipen failed");
	}

	return 0;
}

void
RGBLED_AW2023::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled_aw2023", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x45);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_aw2023_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_AW2023;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = AW2023_ADDR;
	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED_AW2023);

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
