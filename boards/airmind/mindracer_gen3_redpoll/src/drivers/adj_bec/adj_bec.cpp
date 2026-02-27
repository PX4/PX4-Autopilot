/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file adj_bec.cpp
 *
 * Driver for AirMind's adjustable output BEC/MRPDB and 4-in-1 ESC V3.
 *
 * @author Roland <ning.roland@mindpx.net>
 */

#include <stdlib.h>
#include <stdbool.h>
#include <drivers/drv_adc.h>
#include "board_config.h"
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c.h>
#include <px4_platform_common/getopt.h>

#define BECV_BUS_ADDR 0x24
#define BECV_SWRST_ADDR 0x00
#define BECV_DEVID_ADDR 0x7C
#define BECV_RD_OP 1
#define BECV_WR_OP 0

namespace adj_bec
{
static int battery_voltage_sense(uint32_t base_address, unsigned channel, float div, float *p_vbat)
{
	//float bat_voltage_adc_readings;
	uint32_t result = px4_arch_adc_sample(base_address, channel);

	if (result == UINT32_MAX) {
		//syslog(LOG_ERR, "[boot] Battery ADC sample failed.\n");
		return PX4_ERROR;
	}

	volatile float v_ref = px4_arch_adc_reference_v();
	volatile uint32_t resolution = px4_arch_adc_dn_fullcount();

	*p_vbat = result * v_ref * div / resolution;

	return PX4_OK;
}

static int bec_enable_high_voltage(uint8_t bus, bool yon)
{
	int ret = PX4_ERROR;

	//find param;
	param_t en = param_find("ADJ_BEC_EN");
	param_t div = param_find("BAT1_V_DIV");

	if (PARAM_INVALID == en || PARAM_INVALID == div) {
		PX4_ERR("Parameter missing.\n");
		return PX4_ERROR;
	}

	int32_t adj_en = 0;
	param_get(en, &adj_en);

	if (adj_en < 1) {
		PX4_ERR("[ADJ_BEC] adjustable BEC been disabled.\n");
		return PX4_ERROR;
	}

	float v_div = 0.0f;
	param_get(div, &v_div);

	bool port_high;
	float vbat = 0.0f;
	battery_voltage_sense(SYSTEM_ADC_BASE, ADC_BATTERY_VOLTAGE_CHANNEL, v_div, &vbat);

	if (vbat >= 14.0f) {
		port_high = yon;

	} else {
		if (yon) {
			PX4_ERR("Not high enough battery voltage.\n");
			return PX4_ERROR;

		} else {
			port_high = yon;
		}
	}

	// attach to the i2c bus (external)
	struct i2c_master_s *i2c = px4_i2cbus_initialize(bus);

	if (i2c == NULL) {
		PX4_ERR("[ADJ_BEC] Can not open BEC.\n");
		return PX4_ERROR;
	}

	// Check device ID;
	uint8_t txdata[] = {0}; //0x5100, 0x2100 MSB to LSB here.

	struct i2c_msg_s msgv;

	//enable / disable high voltage supply;
	msgv.frequency = 1000000;
	msgv.addr = BECV_BUS_ADDR;
	msgv.flags = BECV_WR_OP;
	msgv.buffer = &txdata[0];
	msgv.length = sizeof(txdata);

	txdata[0] = !port_high;
	ret = I2C_TRANSFER(i2c, &msgv, 1);

	if (PX4_OK == ret) {
		PX4_INFO("BEC output adjust done.");

	} else {
		PX4_ERR("Failed to adjust BEC output voltage.\n");
	}

	px4_i2cbus_uninitialize(i2c);

	return ret;

}

static void usage()
{
	PRINT_MODULE_DESCRIPTION("Adjust BEC output voltage.");

	PRINT_MODULE_USAGE_NAME("adj_bec", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("high", "Set BEC output to higher voltage.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("norm", "Set BEC output to normal voltage.");
	PRINT_MODULE_USAGE_PARAM_FLAG('I', "on Internal I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('X', "on External I2C bus", true);
}
} //namespace adj_bec

extern "C" __EXPORT int adj_bec_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;

	uint8_t i2c_bus = 0;
	uint8_t i2c_bus_i = 0;
	I2CBusIterator i2c_bus_iterator {I2CBusIterator::FilterType::All};

	while (i2c_bus_iterator.next()) {
		if (i2c_bus_iterator.bus().is_external) {
			//external bus is default;
			i2c_bus = i2c_bus_iterator.bus().bus;

		} else {
			i2c_bus_i = i2c_bus_iterator.bus().bus;
		}
	}

	while ((ch = px4_getopt(argc, argv, "I:X:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'I':
			i2c_bus = i2c_bus_i;
			break;

		case 'X':
			//nned to do nothing;
			break;

		default:
			break;
		}
	}

	if (i2c_bus <= 0) {
		//no i2c bus available;
		return PX4_ERROR;
	}

	const char *verb = myoptarg;

	if (!verb) {
		adj_bec::usage();
		return PX4_ERROR;
	}

	if (!strcmp(verb, "high")) {
		return adj_bec::bec_enable_high_voltage(i2c_bus, true);
	}

	if (!strcmp(verb, "norm")) {
		return adj_bec::bec_enable_high_voltage(i2c_bus, false);
	}

	//if (!strcmp(verb, "status")) {
	//return adj_bec::bec_enable_high_voltage(i2c_bus, false);
	//}

	adj_bec::usage();
	return PX4_ERROR;
}
