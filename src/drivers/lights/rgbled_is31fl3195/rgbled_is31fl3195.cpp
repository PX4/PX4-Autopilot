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
 * @file rgbled_is31fl3195.cpp
 *
 * Driver for the RGB LED controller (IS31FL3195) connected via I2C.
 *
 * @author David_Sidrane <David.Sidrane@nscdg.com>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

#define ADDR			                        0x54	/* I2C adress of IS31FL3195 */
#define PRODUCT_ID                        0x00

#define SHUTDOWN_CTRL                     0x01
#define   SHUTDOWN_CTRL_SSD_SHUTDOWN      0x00
#define   SHUTDOWN_CTRL_SSD_NORMAL        0x01

#define   SHUTDOWN_CTRL_SLE_SHIFT         1
#define   SHUTDOWN_CTRL_SLE_MASK          0x03
#define   SHUTDOWN_CTRL_SLE_DISABLE       (0 << SHUTDOWN_CTRL_SLE_SHIFT)
#define   SHUTDOWN_CTRL_SLE_SLEEP1        (1 << SHUTDOWN_CTRL_SLE_SHIFT)
#define   SHUTDOWN_CTRL_SLE_SLEEP2        (2 << SHUTDOWN_CTRL_SLE_SHIFT)

#define   SHUTDOWN_CTRL_CPPM_SHIFT        3
#define   SHUTDOWN_CTRL_CPPM_1X           (0 << SHUTDOWN_CTRL_CPPM_SHIFT)
#define   SHUTDOWN_CTRL_CPPM_1P5X         (1 << SHUTDOWN_CTRL_CPPM_SHIFT)
#define   SHUTDOWN_CTRL_EN_SHIFT          4
#define   SHUTDOWN_CTRL_EN_MASK           0xf
#define   SHUTDOWN_CTRL_EN1               (1 << SHUTDOWN_CTRL_EN_SHIFT)
#define   SHUTDOWN_CTRL_EN2               (2 << SHUTDOWN_CTRL_EN_SHIFT)
#define   SHUTDOWN_CTRL_EN3               (4 << SHUTDOWN_CTRL_EN_SHIFT)
#define   SHUTDOWN_CTRL_EN4               (8 << SHUTDOWN_CTRL_EN_SHIFT)

#define MODE_CONFIG                       0x02
#define   MODE_CONFIG_LM_SHIFT            0
#define   MODE_CONFIG_LM_MASK             0x3
#define   MODE_CONFIG_LM_SINGLE           (0 << MODE_CONFIG_LM_SHIFT)
#define   MODE_CONFIG_LM_RGBW             (1 << MODE_CONFIG_LM_SHIFT)
#define   MODE_CONFIG_LM_SRGBY            (2 << MODE_CONFIG_LM_SHIFT)

#define   MODE_CONFIG_OUT_MODE_SHIFT      4
#define   MODE_CONFIG_OUT_MODE_MASK       0xf
#define   MODE_CONFIG_OUT_MODE_CURRENT    0
#define   MODE_CONFIG_OUT_MODE_PATTERN    1
#define   MODE_CONFIG_OUT_MODE(n,m)       ((m) << ((n-1) + MODE_CONFIG_OUT_MODE_SHIFT))
#define   MODE_CONFIG_OUT1_MODE(m)        MODE_CONFIG_OUT_MODE(1,(m))
#define   MODE_CONFIG_OUT2_MODE(m)        MODE_CONFIG_OUT_MODE(2,(m))
#define   MODE_CONFIG_OUT3_MODE(m)        MODE_CONFIG_OUT_MODE(3,(m))
#define   MODE_CONFIG_OUT4_MODE(m)        MODE_CONFIG_OUT_MODE(4,(m))

#define CHARGE_PUMP1                      0x3
#define     CHARGE_PUMP1_CPM_SHIFT        0
#define     CHARGE_PUMP1_CPM_MASK         0x3
#define     CHARGE_PUMP1_CPM_AUTO         (0 << CHARGE_PUMP1_CPM_SHIFT)
#define     CHARGE_PUMP1_CPM_1X           (1 << CHARGE_PUMP1_CPM_SHIFT)
#define     CHARGE_PUMP1_CPM_1P5X         (2 << CHARGE_PUMP1_CPM_SHIFT)
#define     CHARGE_PUMP1_DEFAULT          (8 << 2)

#define CHARGE_PUMP2                      0x4
#define     CHARGE_PUMP2_CPDE_SHIFT       0
#define     CHARGE_PUMP2_CPDE_MASK        0xf
#define     CHARGE_PUMP2_CPDE_ENABLE      0
#define     CHARGE_PUMP2_CPDE_DISABLE     1
#define     CHARGE_PUMP2_CPDE(n,m)        ((m) << ((n-1) + CHARGE_PUMP2_CPDE_SHIFT))

#define     CHARGE_PUMP2_HRT_SHIFT        4
#define     CHARGE_PUMP2_MASK             0x3
#define     CHARGE_PUMP2_HRT(m)           ((m) << + CHARGE_PUMP2_HRT_SHIFT)
#define     CHARGE_PUMP2_HRT_50MV         CHARGE_PUMP2_HRT(0)
#define     CHARGE_PUMP2_HRT_100MV        CHARGE_PUMP2_HRT(1)
#define     CHARGE_PUMP2_HRT_125MV        CHARGE_PUMP2_HRT(2)
#define     CHARGE_PUMP2_HRT_150MV        CHARGE_PUMP2_HRT(3)
#define     CHARGE_PUMP2_HRT_175MV        CHARGE_PUMP2_HRT(4)
#define     CHARGE_PUMP2_HRT_200MV        CHARGE_PUMP2_HRT(5)
#define     CHARGE_PUMP2_HRT_250MV        CHARGE_PUMP2_HRT(6)
#define     CHARGE_PUMP2_HRT_300MV        CHARGE_PUMP2_HRT(7)

#define CURRENT_BAND  0x5
#define   CURRENT_BAND_CB_SHIFT           0
#define   CURRENT_BAND_CB_MASK_           0x3
#define   CURRENT_BAND_CB2_SHIFT          2

#define   CURRENT_BAND_CB_P25             0
#define   CURRENT_BAND_CB_P5              1
#define   CURRENT_BAND_CB_P75             2
#define   CURRENT_BAND_CB_1P0             3
#define CURRENT_BAND_CB(n,m)              (m) << ((n-1) * CURRENT_BAND_CB2_SHIFT)

#define OUT_CURRENT1                      0x10
#define OUT_CURRENT2                      0x21
#define OUT_CURRENT3                      0x32
#define OUT_CURRENT4                      0x40

#define COLOR_UPDATE                      0x50
#define COLOR_UPDATE_KEY                  0xC5


#define RUN_MODE (SHUTDOWN_CTRL_SSD_NORMAL  | \
		  SHUTDOWN_CTRL_SLE_DISABLE | \
		  SHUTDOWN_CTRL_CPPM_1P5X)

#define L1_L3_EN  (SHUTDOWN_CTRL_EN1         | \
		   SHUTDOWN_CTRL_EN2         | \
		   SHUTDOWN_CTRL_EN3)


class RGBLED_IS31FL3195: public device::I2C, public I2CSPIDriver<RGBLED_IS31FL3195>
{
public:
	RGBLED_IS31FL3195(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_IS31FL3195() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

private:
	int			send_led_enable(bool enable);
	int			send_led_rgb();

	float			_brightness{1.0f};

	uint8_t			_r{0};
	uint8_t			_g{0};
	uint8_t			_b{0};
	bool			_leds_enabled{false};

	LedController		_led_controller;

	uint8_t   _red_output{OUT_CURRENT1};
	uint8_t   _green_output{OUT_CURRENT2};
	uint8_t   _blue_output{OUT_CURRENT3};
	uint8_t   _current_band{CURRENT_BAND_CB_P5};

};

RGBLED_IS31FL3195::RGBLED_IS31FL3195(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	_current_band = config.custom2;


	int ordering = config.custom1;
	/*
	 * ordering is in RGB order
	 *
	 *    Hundreds is Red,
	 *    Tens is green
	 *    Ones is Blue
	 *
	 *  123 would drive the
	 *       R LED from = OUT_CURRENT1
	 *       G LED from = OUT_CURRENT2
	 *       B LED from = OUT_CURRENT3
	 *
	 *  321 would drive the
	 *       R LED from = OUT_CURRENT3
	 *       G LED from = OUT_CURRENT2
	 *       B LED from = OUT_CURRENT1
	 *
	 */

	const uint8_t outputs[] = {OUT_CURRENT1, OUT_CURRENT2, OUT_CURRENT3};

	// Process ordering in lsd to msd order.(BGR)
	uint8_t *color[] = {&_blue_output, &_green_output, &_red_output };

	unsigned int s = 0;

	for (unsigned int i = 0; i < arraySize(color); i++) {
		s = (ordering % 10) - 1;

		if (s < arraySize(outputs)) {
			*color[i] = outputs[s];
		}

		ordering /= 10;
	}


}

int
RGBLED_IS31FL3195::init()
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
RGBLED_IS31FL3195::probe()
{
	int ret = OK;
	const uint8_t msg[1] = {PRODUCT_ID};
	uint8_t result[1] = {0};

	ret = transfer(msg, sizeof(msg), result, sizeof(result));

	if (result[0] != ADDR << 1) {
		return -1;
	}

	_retries = 1;

	return ret;
}

void
RGBLED_IS31FL3195::RunImpl()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = 255; _g = 0; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = 255; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = 255; _g = 255; _b = 0;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_PURPLE:
			_r = 255; _g = 0; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = 255; _b = 255;
			send_led_enable(true);
			break;

		case led_control_s::COLOR_WHITE:
			_r = 255; _g = 255; _b = 255;
			send_led_enable(true);
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			send_led_enable(false);
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;

		send_led_rgb();
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED_IS31FL3195::send_led_enable(bool enable)
{
	int ret = 0;

	if ((_leds_enabled ^ enable) == 0) {
		return ret;
	}

	_leds_enabled = enable;
	uint8_t shutdown_ctrl = RUN_MODE;

	if (enable) {
		shutdown_ctrl |= L1_L3_EN;
	}

	const uint8_t mode = MODE_CONFIG_LM_SINGLE |
			     MODE_CONFIG_OUT1_MODE(MODE_CONFIG_OUT_MODE_CURRENT) |
			     MODE_CONFIG_OUT2_MODE(MODE_CONFIG_OUT_MODE_CURRENT) |
			     MODE_CONFIG_OUT3_MODE(MODE_CONFIG_OUT_MODE_CURRENT);

	const uint8_t cb = CURRENT_BAND_CB(1, _current_band) |
			   CURRENT_BAND_CB(2, _current_band) |
			   CURRENT_BAND_CB(3, _current_band);

	const uint8_t init[][2] = {
		{ SHUTDOWN_CTRL, shutdown_ctrl},
		{ MODE_CONFIG,  mode },
		{ CHARGE_PUMP1, CHARGE_PUMP1_DEFAULT | CHARGE_PUMP1_CPM_1P5X },
		{ CHARGE_PUMP2, CHARGE_PUMP2_HRT_150MV | CHARGE_PUMP2_CPDE(4, CHARGE_PUMP2_CPDE_DISABLE) },
		{ CURRENT_BAND, cb },
	};

	for (unsigned i = 0; i < arraySize(init); i++) {

		ret = transfer(init[i], sizeof(init[i]), nullptr, 0);

		if (ret < 0) {
			break;
		}
	}

	return ret;
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_IS31FL3195::send_led_rgb()
{
	int ret = OK;
	const uint8_t leds[][2] = {
		{ _red_output,   static_cast<uint8_t>(_r * _brightness)},
		{ _green_output, static_cast<uint8_t>(_g * _brightness)},
		{ _blue_output,  static_cast<uint8_t>(_b * _brightness)},
		{ COLOR_UPDATE, COLOR_UPDATE_KEY},
	};

	for (unsigned i = 0; i < arraySize(leds); i++) {

		ret = transfer(leds[i], sizeof(leds[i]), nullptr, 0);

		if (ret < 0) {
			break;
		}
	}

	return ret;
}

void
RGBLED_IS31FL3195::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled_is31fl3195", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(ADDR);
	PRINT_MODULE_USAGE_PARAM_INT('o', 123, 123, 321, "RGB PWM Assignment", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('i', 0.5f, 0.25f, 1.0f, "Current Band", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_is31fl3195_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = RGBLED_IS31FL3195;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = ADDR;
	cli.custom1 = 123;
	cli.custom2 = CURRENT_BAND_CB_P5;


	while ((ch = cli.getOpt(argc, argv, "o:i:")) != EOF) {
		switch (ch) {
		case 'o':
			cli.custom1 = atoi(cli.optArg());
			break;

		case 'i':
			float v = atof(cli.optArg());
			cli.custom2 = ((uint8_t)(v / 0.25f)) - 1;
			break;
		}
	}


	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED_IS31FL3195);

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
