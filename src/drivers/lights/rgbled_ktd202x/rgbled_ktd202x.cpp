/****************************************************************************
 *
 *   Copyright (c) 2024 AirMind Development Team. All rights reserved.
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
 * @file rgbled_ktd202x.cpp
 *
 * Driver for the RGB LED controller (KTD202x series) connected via I2C.
 *
 * @author Roland <ning.roland@mindpx.net>
 */

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

/** KTD202x line has 5 variations with different address as
 *  defined below.
 *  KTD2027 has an extra White channel (D4).
 */
#define KTD2026EWE_ADDR         0x30
#define KTD2026BEWE_ADDR        0x31
#define KTD2026CEWE_ADDR        0x32
#define KTD2027EWE_ADDR         0x30
#define KTD2027BEWE_ADDR        0x31

#define CTRL_REG0_ADDR          0x0000
#define CTRL_REG1_ADDR          0x0001
#define CTRL_REG2_ADDR          0x0002
#define CTRL_REG3_ADDR          0x0003
#define CTRL_REG4_ADDR          0x0004
#define CTRL_REG5_ADDR          0x0005
#define CTRL_REG6_ADDR          0x0006
#define CTRL_REG7_ADDR          0x0007
#define CTRL_REG8_ADDR          0x0008
#define CTRL_REG9_ADDR          0x0009

#define CH_CTL_REG_BASE         CTRL_REG6_ADDR
#define CH_CTL_IDX_R            0x00
#define CH_CTL_IDX_G            0x01
#define CH_CTL_IDX_B            0x02
#define CH_CTL_IDX_W            0x03

#define CMD_CHIP_RESET          0x07
#define CMD_ENCTL_AON           (0x03 << 3)
#define CMD_RAMP_SCALE          0x0         //default 1x
#define CMD_RAMP_TRISE          0x02        //256ms Rise
#define CMD_RAMP_TFALL          0x02        //256ms Fall
#define CMD_CHANNEL_AON         0x01        //for each LED channel
#define CMD_CHANNEL_AOFF        0x00        //for each LED channel
#define CURRENT_OUT_LIMIT       0xFF        //Specify maximum output current for each channel in step of 0.125mA. 0xFF = 24mA max.


class RGBLED_KTD202X: public device::I2C, public I2CSPIDriver<RGBLED_KTD202X>
{
public:
	RGBLED_KTD202X(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_KTD202X() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();

private:
	int         chip_reset();
	int         reg_reset();
	int         set_channel_enable(uint8_t ch_reg, uint8_t ch_mode);
	int         send_chip_enable(uint8_t chip_mode);
	int         set_all_channel(uint8_t channel_mode);
	int			send_led_rgb();
	int         w8_reg(uint8_t reg, uint8_t data);
	int         r8_reg(uint8_t reg, uint8_t *data);
	int         set_nominal_current_out(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

	float			_brightness{1.0f};

	uint8_t			_r{0};
	uint8_t			_g{0};
	uint8_t			_b{0};
	uint8_t         _w{0};

	uint8_t         _out_cur_limit;
	bool            _use_white_channel{false};

	LedController		_led_controller;

	uint8_t   _red_reg{CH_CTL_REG_BASE + CH_CTL_IDX_R};
	uint8_t   _green_reg{CH_CTL_REG_BASE + CH_CTL_IDX_G};
	uint8_t   _blue_reg{CH_CTL_REG_BASE + CH_CTL_IDX_B};
	uint8_t   _white_reg{CH_CTL_REG_BASE + CH_CTL_IDX_W};

	//Cache status for channels mode;
	uint8_t   _ch_mode[4] {CMD_CHANNEL_AOFF};

	//All alternative chip addresses;
	uint8_t   _alt_addrs[3] {KTD2026EWE_ADDR, KTD2026BEWE_ADDR, KTD2026CEWE_ADDR};

};

RGBLED_KTD202X::RGBLED_KTD202X(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{

	_use_white_channel = (bool)config.custom2;
	_out_cur_limit = *((uint8_t *)(config.custom_data));

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

	_blue_reg = CH_CTL_REG_BASE + ordering % 10 - 1;
	_green_reg = CH_CTL_REG_BASE + (ordering / 10) % 10 - 1;
	_red_reg = CH_CTL_REG_BASE + (ordering / 100) % 10 - 1;

	//white channel always goes to CH4(D4);
	_white_reg = CH_CTL_REG_BASE + CH_CTL_IDX_W;
}

int
RGBLED_KTD202X::init()
{
	//call probe from inside init;
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	// kick off work queue
	ScheduleNow();

	return PX4_OK;
}

int
RGBLED_KTD202X::chip_reset()
{
	return w8_reg(CTRL_REG0_ADDR, CMD_CHIP_RESET);
}

int
RGBLED_KTD202X::probe()
{
	int ret = PX4_OK;
	uint8_t addr_idx = 0;
#ifdef CONFIG_I2C
	uint8_t default_addr = get_device_address();
#endif
	bool no_more_alt_addr = false;

	//Scan through all alternative addresses;
	do {
		if (addr_idx >= sizeof(_alt_addrs)) {
			no_more_alt_addr = true;
		}

		//Reset chip upon initialization; Note this cmd will never get ACK response;
		chip_reset();

		//wait 200us before sending next cmd per datasheet required.
		up_udelay(199);

		//set chip enable control mode;
		ret = set_all_channel(CMD_CHANNEL_AOFF);

		if (ret != PX4_OK) {
#ifdef CONFIG_I2C
			PX4_DEBUG("no insetance at addr 0x%2x", get_device_address());

			//try next alternative address;
			for (; addr_idx < sizeof(_alt_addrs); addr_idx++) {
				if (default_addr != _alt_addrs[addr_idx]) {
					PX4_DEBUG("try alt addr 0x%2x", _alt_addrs[addr_idx]);
					set_device_address(_alt_addrs[addr_idx]);
					addr_idx++;
					break;
				}
			}

#endif

		} else {
			//Device found. Set chip en mode to ALWAYS_ON;
			send_chip_enable(CMD_ENCTL_AON);
		}
	} while ((ret != PX4_OK) && (!no_more_alt_addr));

	if (ret != PX4_OK) {
		//probe failed;
		PX4_DEBUG("ktd202x probe failed.");
		return -1;
	}

	return ret;
}

int
RGBLED_KTD202X::set_nominal_current_out(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	int ret = PX4_OK;

	//r channel
	if (0 == r) {
		//Need to disable channel as '0' value in current register does NOT mean zero sink current.
		set_channel_enable(_red_reg, CMD_CHANNEL_AOFF);

	} else {
		set_channel_enable(_red_reg, CMD_CHANNEL_AON);
		w8_reg(_red_reg, r);
	}

	//g channel
	if (0 == g) {
		set_channel_enable(_green_reg, CMD_CHANNEL_AOFF);

	} else {
		set_channel_enable(_green_reg, CMD_CHANNEL_AON);
		w8_reg(_green_reg, g);
	}

	//b channel;
	if (0 == b) {
		ret = set_channel_enable(_blue_reg, CMD_CHANNEL_AOFF);

	} else {
		set_channel_enable(_blue_reg, CMD_CHANNEL_AON);
		ret = w8_reg(_blue_reg, b);
	}

	if (_use_white_channel) {
		if (0 == w) {
			ret = set_channel_enable(_white_reg, CMD_CHANNEL_AOFF);

		} else {
			ret = set_channel_enable(_white_reg, CMD_CHANNEL_AON);
			ret = w8_reg(_white_reg, w);
		}
	}

	return ret;
}

void
RGBLED_KTD202X::RunImpl()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = _out_cur_limit; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = _out_cur_limit; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = _out_cur_limit;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = _out_cur_limit; _g = _out_cur_limit; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = _out_cur_limit; _g = 0; _b = _out_cur_limit;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = _out_cur_limit; _b = _out_cur_limit;
			break;

		case led_control_s::COLOR_WHITE:
			if (_use_white_channel) {
				_r = 0; _g = 0; _b = 0; _w = _out_cur_limit;

			} else {
				_r = _out_cur_limit; _g = _out_cur_limit; _b = _out_cur_limit;
			}

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

int
RGBLED_KTD202X::w8_reg(uint8_t reg, uint8_t data)
{
	uint8_t msg[2];
	msg[0] = reg;
	msg[1] = data;

	int ret = transfer(msg, 2, nullptr, 0);

	return ret;
}

int
RGBLED_KTD202X::r8_reg(uint8_t reg, uint8_t *data)
{
	uint8_t msg[2] = {0};
	msg[0] = reg;

	int ret = transfer(nullptr, 0, msg, 2);
	*data = msg[1];

	return ret;
}

/**
 * Set enable control mode of the chip
 */
int
RGBLED_KTD202X::send_chip_enable(uint8_t chip_mode)
{
	uint8_t reg_val = 0;
	//set chip enable control mode;
	r8_reg(CTRL_REG0_ADDR, &reg_val);
	int ret = w8_reg(CTRL_REG0_ADDR, (reg_val & 0x67) | chip_mode);

	return ret;
}

/**
 * Set enable control mode for each individual channel
 *
 */
int
RGBLED_KTD202X::set_channel_enable(uint8_t ch_reg, uint8_t ch_mode)
{
	int ret = PX4_OK;
	uint8_t reg_val = 0;

	uint8_t ch_idx = ch_reg - CH_CTL_REG_BASE;

	if (ch_mode == _ch_mode[ch_idx]) {
		return PX4_OK;
	}

	//set channel mode;
	ret = r8_reg(CTRL_REG4_ADDR, &reg_val);

	if (ret != PX4_OK) {
		return ret;
	}

	reg_val &= ~(0x03 << (ch_idx * 2));
	reg_val |= ch_mode << (ch_idx * 2);
	ret = w8_reg(CTRL_REG4_ADDR, reg_val);

	if (ret == PX4_OK) {
		//update cached channel status;
		_ch_mode[ch_idx] = ch_mode;
	}

	return ret;
}

/**
 * Set enable control mode for all channels
 */
int
RGBLED_KTD202X::set_all_channel(uint8_t channel_mode)
{
	uint8_t reg_val = 0;

	//set channel mode;
	reg_val = channel_mode | (channel_mode << 2) | (channel_mode << 4);

	if (_use_white_channel) {
		reg_val |= channel_mode << 6 ;
	}

	int ret = w8_reg(CTRL_REG4_ADDR, reg_val);

	if (ret == PX4_OK) {
		for (uint8_t i = 0; i < (sizeof(_ch_mode) - 1); i++) {
			_ch_mode[i] = channel_mode;
		}

		if (_use_white_channel) {
			_ch_mode[CH_CTL_IDX_W] = channel_mode;
		}
	}

	return ret;
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_KTD202X::send_led_rgb()
{
	int ret = PX4_OK;
	uint8_t wb = 0;

	if (_use_white_channel) {
		wb = static_cast<uint8_t>(_w * _brightness);

	} else {
		wb = 0;
	}

	ret = set_nominal_current_out(static_cast<uint8_t>(_r * _brightness), static_cast<uint8_t>(_g * _brightness),
				      static_cast<uint8_t>(_b * _brightness), wb);

	return ret;
}

void
RGBLED_KTD202X::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled_ktd202x", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(KTD2026EWE_ADDR);
	PRINT_MODULE_USAGE_PARAM_FLAG('w', "use white channel for ktd2027", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 123, 123, 321, "RGB PWM Assignment", true);
	PRINT_MODULE_USAGE_PARAM_INT('z', 255, 1, 255,
				     "Specify the limit of output current in step of 0.125mA, valid between 1(0.25mA) and 255(24mA)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_ktd202x_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = RGBLED_KTD202X;
	//limit of output current;
	uint8_t cur_limit = CURRENT_OUT_LIMIT;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 1000000;    //1MHz
	cli.i2c_address = KTD2026EWE_ADDR;      //default addr
	cli.custom1 = 123;
	cli.custom2 = false;                    //NOT use white channel as default;
	cli.custom_data = &cur_limit;

	while ((ch = cli.getOpt(argc, argv, "w:o:z:")) != EOF) {
		switch (ch) {
		case 'w':
			cli.custom2 = true;             //use white channel if requested;
			break;

		case 'o':
			cli.custom1 = atoi(cli.optArg());
			break;

		case 'z':
			cur_limit = atoi(cli.optArg());
			break;
		}
	}


	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_LED_DEVTYPE_RGBLED_KTD202X);

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
