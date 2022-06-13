/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 *
 * Driver for RGB LP55231 LED controller connected via I2C.
 */

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

static constexpr uint8_t ADDRESS = 0x32;

// register stuff
static constexpr uint8_t REG_CNTRL1 = 0x00;
static constexpr uint8_t REG_CNTRL2 = 0x01;
static constexpr uint8_t REG_RATIO_MSB = 0x02;
static constexpr uint8_t REG_RATIO_LSB = 0x03;
static constexpr uint8_t REG_OUTPUT_ONOFF_MSB = 0x04;
static constexpr uint8_t REG_OUTPUT_ONOFF_LSB = 0x05;

// Per LED control channels - fader channel assig, log dimming enable, temperature compensation
static constexpr uint8_t REG_D1_CTRL = 0x06;
static constexpr uint8_t REG_D2_CTRL = 0x07;
static constexpr uint8_t REG_D3_CTRL = 0x08;
static constexpr uint8_t REG_D4_CTRL = 0x09;
static constexpr uint8_t REG_D5_CTRL = 0x0a;
static constexpr uint8_t REG_D6_CTRL = 0x0b;
static constexpr uint8_t REG_D7_CTRL = 0x0c;
static constexpr uint8_t REG_D8_CTRL = 0x0d;
static constexpr uint8_t REG_D9_CTRL = 0x0e;

// 0x0f to 0x15 reserved

// Direct PWM control registers
static constexpr uint8_t REG_D1_PWM  = 0x16;
static constexpr uint8_t REG_D2_PWM  = 0x17;
static constexpr uint8_t REG_D3_PWM  = 0x18;
static constexpr uint8_t REG_D4_PWM  = 0x19;
static constexpr uint8_t REG_D5_PWM  = 0x1a;
static constexpr uint8_t REG_D6_PWM  = 0x1b;
static constexpr uint8_t REG_D7_PWM  = 0x1c;
static constexpr uint8_t REG_D8_PWM  = 0x1d;
static constexpr uint8_t REG_D9_PWM  = 0x1e;

// 0x1f to 0x25 reserved

// Drive current registers
static constexpr uint8_t REG_D1_I_CTL = 0x26;
static constexpr uint8_t REG_D2_I_CTL  = 0x27;
static constexpr uint8_t REG_D3_I_CTL  = 0x28;
static constexpr uint8_t REG_D4_I_CTL  = 0x29;
static constexpr uint8_t REG_D5_I_CTL  = 0x2a;
static constexpr uint8_t REG_D6_I_CTL  = 0x2b;
static constexpr uint8_t REG_D7_I_CTL  = 0x2c;
static constexpr uint8_t REG_D8_I_CTL  = 0x2d;
static constexpr uint8_t REG_D9_I_CTL  = 0x2e;

// 0x2f to 0x35 reserved

static constexpr uint8_t REG_MISC     = 0x36;
static constexpr uint8_t REG_PC1      = 0x37;
static constexpr uint8_t REG_PC2      = 0x38;
static constexpr uint8_t REG_PC3      = 0x39;
static constexpr uint8_t REG_STATUS_IRQ = 0x3A;
static constexpr uint8_t REG_INT_GPIO   = 0x3B;
static constexpr uint8_t REG_GLOBAL_VAR = 0x3C;
static constexpr uint8_t REG_RESET      = 0x3D;
static constexpr uint8_t REG_TEMP_CTL   = 0x3E;
static constexpr uint8_t REG_TEMP_READ  = 0x3F;
static constexpr uint8_t REG_TEMP_WRITE = 0x40;
static constexpr uint8_t REG_TEST_CTL   = 0x41;
static constexpr uint8_t REG_TEST_ADC   = 0x42;

// 0x43 to 0x44 reserved

static constexpr uint8_t REG_ENGINE_A_VAR = 0x45;
static constexpr uint8_t REG_ENGINE_B_VAR = 0x46;
static constexpr uint8_t REG_ENGINE_C_VAR = 0x47;

static constexpr uint8_t REG_MASTER_FADE_1 = 0x48;
static constexpr uint8_t REG_MASTER_FADE_2 = 0x49;
static constexpr uint8_t REG_MASTER_FADE_3 = 0x4A;

// 0x4b Reserved

static constexpr uint8_t REG_PROG1_START = 0x4C;
static constexpr uint8_t REG_PROG2_START = 0x4D;
static constexpr uint8_t REG_PROG3_START = 0x4E;
static constexpr uint8_t REG_PROG_PAGE_SEL = 0x4f;

// Memory is more confusing - there are 6 pages, sel by addr 4f
static constexpr uint8_t REG_PROG_MEM_BASE = 0x50;
static constexpr uint8_t REG_PROG_MEM_END  = 0x6f;

static constexpr uint8_t REG_ENG1_MAP_MSB = 0x70;
static constexpr uint8_t REG_ENG1_MAP_LSB = 0x71;
static constexpr uint8_t REG_ENG2_MAP_MSB = 0x72;
static constexpr uint8_t REG_ENG2_MAP_LSB = 0x73;
static constexpr uint8_t REG_ENG3_MAP_MSB = 0x74;
static constexpr uint8_t REG_ENG3_MAP_LSB = 0x75;

static constexpr uint8_t REG_GAIN_CHANGE = 0x76;

// Colors on eval board
static constexpr uint8_t CHANNEL_L0_RED = 0;
static constexpr uint8_t CHANNEL_L0_GREEN = 3;
static constexpr uint8_t CHANNEL_L0_BLUE = 4;
static constexpr uint8_t CHANNEL_L1_RED = 1;
static constexpr uint8_t CHANNEL_L1_GREEN = 5;
static constexpr uint8_t CHANNEL_L1_BLUE = 6;
static constexpr uint8_t CHANNEL_L2_RED = 2;
static constexpr uint8_t CHANNEL_L2_GREEN = 7;
static constexpr uint8_t CHANNEL_L2_BLUE = 8;

class RGBLED_LP55231 : public device::I2C, public I2CSPIDriver<RGBLED_LP55231>
{
public:
	RGBLED_LP55231(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_LP55231() override;

	static void print_usage();

	int init() override;
	int probe() override;

	void RunImpl();

private:
	void print_status() override;

	int write(uint8_t address, uint8_t value);
	int read(uint8_t address, uint8_t &value);

	int setChannelPWM(uint8_t channel, uint8_t value);

	int setLed(uint8_t channel_red, uint8_t channel_green, uint8_t channel_blue,
		   const LedControlDataSingle &led);

	int reset();
	int enable();

	LedController		_led_controller;

	// uint8_t _channel = 0;
};

RGBLED_LP55231::RGBLED_LP55231(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

RGBLED_LP55231::~RGBLED_LP55231()
{
	reset();
}


int RGBLED_LP55231::write(uint8_t address, uint8_t value)
{
	uint8_t data[2] = {address, value};
	return transfer(data, sizeof(data), nullptr, 0);
}

int RGBLED_LP55231::read(uint8_t address, uint8_t &value)
{
	return transfer(&address, 1, (uint8_t *)&value, 1);
}


int RGBLED_LP55231::enable()
{
	int ret = write(REG_CNTRL1, 0x40);
	// enable charge pump, internal oscillator, auto increment
	ret = write(REG_MISC, 0x53);
	return ret;
}

int RGBLED_LP55231::reset()
{
	return write(REG_RESET, 0xFF);
}


int RGBLED_LP55231::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	ret = reset();
	ret = enable();

	// kick off work queue
	ScheduleNow();

	return OK;
}

int RGBLED_LP55231::setChannelPWM(uint8_t channel, uint8_t value)
{
	assert(channel < 9);
	return write(REG_D1_PWM + channel, value);
}

int RGBLED_LP55231::setLed(uint8_t channel_red, uint8_t channel_green, uint8_t channel_blue,
			   const LedControlDataSingle &led)
{
	uint8_t r{0}, g{0}, b{0};
	uint8_t brightness = led.brightness;

	switch (led.color) {
	case led_control_s::COLOR_RED:
		r = brightness; g = 0; b = 0;
		break;

	case led_control_s::COLOR_GREEN:
		r = 0; g = brightness; b = 0;
		break;

	case led_control_s::COLOR_BLUE:
		r = 0; g = 0; b = brightness;
		break;

	case led_control_s::COLOR_AMBER: //make it the same as yellow
	case led_control_s::COLOR_YELLOW:
		r = brightness / 2; g = brightness / 2; b = 0;
		break;

	case led_control_s::COLOR_PURPLE:
		r = brightness / 2; g = 0; b = brightness / 2;
		break;

	case led_control_s::COLOR_CYAN:
		r = 0; g = brightness / 2; b = brightness / 2;
		break;

	case led_control_s::COLOR_WHITE:
		r = brightness / 3; g = brightness / 3; b = brightness / 3;
		break;

	default: // led_control_s::COLOR_OFF
		r = 0; g = 0; b = 0;
		break;
	}

	int ret = 0;
	ret = setChannelPWM(channel_red, r);
	ret = setChannelPWM(channel_green, g);
	ret = setChannelPWM(channel_blue, b);
	return ret;
}

int RGBLED_LP55231::probe()
{
	// try read output on / off (should be 0xff)
	uint8_t on_off;
	int ret = read(REG_OUTPUT_ONOFF_LSB, on_off);

	if (on_off != 0xFF) {
		return -1;
	}

	// try read current control (should be 0xAF)
	uint8_t current_control;
	ret = read(REG_D1_I_CTL, current_control);

	if (current_control != 0xAF) {
		return -1;
	}

	_retries = 1;

	return ret;
}

void
RGBLED_LP55231::print_status()
{
	PX4_INFO("No status implemented");
}



void RGBLED_LP55231::RunImpl()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		setLed(CHANNEL_L0_RED, CHANNEL_L0_GREEN, CHANNEL_L0_BLUE, led_control_data.leds[0]);
		setLed(CHANNEL_L1_RED, CHANNEL_L1_GREEN, CHANNEL_L1_BLUE, led_control_data.leds[1]);
		setLed(CHANNEL_L2_RED, CHANNEL_L2_GREEN, CHANNEL_L2_BLUE, led_control_data.leds[2]);
	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());

	// setChannelPWM(_channel, 0);
	// _channel++;
	// _channel = _channel % 9;
	// setChannelPWM(_channel, 255);
	// ScheduleDelayed(500_ms);
}


void
RGBLED_LP55231::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled_lp55231", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x32);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_lp55231_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_LP55231;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = ADDRESS;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED_LP55231);

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
