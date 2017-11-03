/****************************************************************************
 *
 *   Copyright (C) 2017  Intel Corporation. All rights reserved.
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

#include "aerofc_configuration.h"

#include <stdint.h>

#include <px4_i2c.h>
#include <drivers/device/i2c.h>

#include "board_config.h"

#define I2C_CLOCK_FREQUENCY 400000
#define SLAVE_ADDR 0x51
#define VERSION_REG 0x00

#define TELEMETRY_CONNECTOR_SEL_REG 0x02
#define TELEMETRY_CONNECTOR_SEL_I2C_VALUE 0x01

static px4_i2c_dev_t *_dev = NULL;

static int _read_uint8_reg(uint8_t reg, uint8_t *data)
{
	struct i2c_msg_s msgv[2];

	msgv[0].frequency = I2C_CLOCK_FREQUENCY;
	msgv[0].addr = SLAVE_ADDR;
	msgv[0].flags = 0;
	msgv[0].buffer = &reg;
	msgv[0].length = 1;

	msgv[1].frequency = I2C_CLOCK_FREQUENCY;
	msgv[1].addr = SLAVE_ADDR;
	msgv[1].flags = I2C_M_READ;
	msgv[1].buffer = data;
	msgv[1].length = 1;

	return I2C_TRANSFER(_dev, msgv, 2) == 2 ? 0 : -1;
}

static int _write_uint8_reg(uint8_t reg, uint8_t data)
{
	uint8_t buffer[2] = { reg, data };
	struct i2c_msg_s msgv[1];

	msgv[0].frequency = I2C_CLOCK_FREQUENCY;
	msgv[0].addr = SLAVE_ADDR;
	msgv[0].flags = 0;
	msgv[0].buffer = buffer;
	msgv[0].length = 2;

	return I2C_TRANSFER(_dev, msgv, 1) == 1 ? 0 : -1;
}

static int _init()
{
	uint8_t buffer;

	_dev = px4_i2cbus_initialize(PX4_I2C_BUS_EXPANSION);

	if (!_dev) {
		return -1;
	}

	if (_read_uint8_reg(VERSION_REG, &buffer)) {
		px4_i2cbus_uninitialize(_dev);
		_dev = NULL;
		return -1;
	}

	return 0;
}

int aerofc_fpga_config_telem_connector(bool enable_i2c)
{
	if (!_dev && _init()) {
		return -1;
	}

	int r = _write_uint8_reg(TELEMETRY_CONNECTOR_SEL_REG, enable_i2c ?
				 TELEMETRY_CONNECTOR_SEL_I2C_VALUE : 0);

	if (enable_i2c) {
		stm32_unconfiggpio(GPIO_UART8_RX);
		stm32_unconfiggpio(GPIO_UART8_TX);

	} else {
		stm32_configgpio(GPIO_UART8_RX);
		stm32_configgpio(GPIO_UART8_TX);
	}

	device::I2C::enable_runtime_bus(PX4_I2C_BUS_EXPANSION1, enable_i2c);

	return r;
}

static int _invert_uart(uint8_t uart, bool invert)
{
	uint8_t buffer;

	if (_read_uint8_reg(UART_INVERTED_REG, &buffer)) {
		return -1;
	}

	if (invert) {
		buffer |= uart;

	} else {
		buffer &= ~uart;
	}

	return _write_uint8_reg(UART_INVERTED_REG, buffer);
}
