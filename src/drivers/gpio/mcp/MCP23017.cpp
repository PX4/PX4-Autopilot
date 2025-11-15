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

#include "MCP23017.hpp"

#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>


MCP23017::MCP23017(const I2CSPIDriverConfig &config) :
	//I2C(config),
	//I2CSPIDriver(config),
	MCP(config)
	//_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{


}

MCP23017::~MCP23017()
{
	ScheduleClear();
	//perf_free(_cycle_perf);
}


int MCP23017::read_reg(Register address, uint8_t &data)
{
	int ret = transfer((uint8_t *)&address, 1, &data, 1);
	return ret;
}

int MCP23017::write_reg(Register address, uint8_t value)
{
	uint8_t data[2] = {(uint8_t)address, value};
	return transfer(data, 2, nullptr, 0);
}



int MCP23017::init()
{
	printf("MCP23017.cpp: init_dummy called \n");
    // Provide default values for direction, state, pull_up
    uint16_t default_direction = 0xFFFF; // all pins input by default
    uint16_t default_state = 0x0000;     // all outputs low
    uint16_t default_pull_up = 0x0000;   // no pull-ups

    return init(default_direction, default_state, default_pull_up);
}

int MCP23017::init(uint16_t direction, uint16_t state, uint16_t pull_up)
{
	printf("MCP23017.cpp: init called \n");
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	// buffer the new initial states
	_iodirA = (uint8_t)(direction & 0x00FF);
	_olatA = (uint8_t)(state & 0x00FF);
	_gppuA = (uint8_t)(pull_up & 0x00FF);

	_iodirB = (uint8_t)(direction >> 8);
	_olatB = (uint8_t)(state >> 8);
	_gppuB = (uint8_t)(pull_up >> 8);

	// Write the initial state to the device
	ret = write_reg(Register::OLATA, _olatA);
	ret |= write_reg(Register::OLATB, _olatB);

	//Set pins as input/output
	ret |= write_reg(Register::IODIRA, _iodirA);
	ret |= write_reg(Register::IODIRB, _iodirB);

	//Set pins as pullup/pulldown
	ret |= write_reg(Register::GPPUA, _gppuA);
	ret |= write_reg(Register::GPPUB, _gppuB);

	//Enable interrupts
	//ret |= write_reg(Register::GPINTENA, (uint8_t)(int_en & 0x00FF));
	//ret |= write_reg(Register::GPINTENB, (uint8_t)(int_en >> 8));

	//Set reference values
	//ret |= write_reg(Register::DEFVALA, (uint8_t)(ref_vals & 0x00FF));
	//ret |= write_reg(Register::DEFVALB, (uint8_t)(ref_vals >> 8));

	//Set interrupt type
	//ret |= write_reg(Register::INTCONA, 0xFF);
	//ret |= write_reg(Register::INTCONB, 0xFF);

	//if (!split_int) {
	//	ret |= write_reg(Register::IOCONA, 0x40);
	//}

	if (ret != PX4_OK) {
		PX4_ERR("Device init failed (%i)", ret);
		return ret;
	}

	return PX4_OK;//init_uorb();
}

int MCP23017::probe()
{
	printf("MCP23017.cpp: probe called \n");
	// no whoami, try to read IOCONA
	uint8_t data;
	return read_reg(Register::IOCONA, data);
}

int MCP23017::read(uint16_t *mask)
{
	//printf("MCP23017.cpp: read called \n");
	uint8_t maskA;
	uint8_t maskB;

	int ret = read_reg(Register::GPIOA, maskA);
	ret |= read_reg(Register::GPIOB, maskB);

	*mask = ((uint16_t) maskA & 0x00FF) | ((uint16_t) maskB << 8);

	return ret;
}

int MCP23017::write(uint16_t mask_set, uint16_t mask_clear)
{
	printf("MCP23017.cpp: write called \n");
	// no need to read, we can use the buffered register value
	uint8_t mask_setA = (uint8_t)(mask_set & 0x00FF);
	uint8_t mask_clearA = (uint8_t)(mask_clear & 0x00FF);

	uint8_t mask_setB = (uint8_t)(mask_set >> 8);
	uint8_t mask_clearB = (uint8_t)(mask_clear >> 8);

	_olatA = (_olatA & ~mask_clearA) | mask_setA;
	_olatB = (_olatB & ~mask_clearB) | mask_setB;

	int ret = write_reg(Register::OLATA, _olatA);
	ret |= write_reg(Register::OLATB, _olatB);

	return ret;
}

int MCP23017::configure(uint16_t mask, PinType type)
{
	printf("MCP23017.cpp: configure called \n");
	uint8_t maskA = (uint8_t)(mask & 0x00FF);
	uint8_t maskB = (uint8_t)(mask >> 8);

	// no need to read, we can use the buffered register values
	switch (type) {
	case PinType::Input:
		_iodirA |= maskA;
		_iodirB |= maskB;
		_gppuA &= ~maskA;
		_gppuB &= ~maskB;
		break;

	case PinType::InputPullUp:
		_iodirA |= maskA;
		_iodirB |= maskB;
		_gppuA |= maskA;
		_gppuB |= maskB;
		break;

	case PinType::Output:
		_iodirA &= ~maskA;
		_iodirB &= ~maskB;
		break;

	default:
		return -EINVAL;
	}

	int ret = write_reg(Register::GPPUA, _gppuA);
	ret |= write_reg(Register::GPPUB, _gppuB);
	ret |= write_reg(Register::IODIRA, _iodirA);
	ret |= write_reg(Register::IODIRB, _iodirB);

	if (ret != 0) {
		PX4_ERR("Configuring MCP23017 failed");
		return ret;
	}

	return ret;
}
