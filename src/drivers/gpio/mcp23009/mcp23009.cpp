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

#include "mcp23009.h"

int MCP23009::read_reg(Register address, uint8_t &data)
{
	return transfer((uint8_t *)&address, 1, &data, 1);
}

int MCP23009::write_reg(Register address, uint8_t value)
{
	uint8_t data[2] = {(uint8_t)address, value};
	return transfer(data, sizeof(data), nullptr, 0);
}

int MCP23009::init(uint8_t direction, uint8_t state, uint8_t pull_up)
{
	// do I2C init (and probe) first
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	// buffer the new initial states
	_iodir = direction;
	_olat = state;
	_gppu = pull_up;

	// Write the initial state to the device
	ret = write_reg(Register::OLAT, _olat);
	ret |= write_reg(Register::IODIR, _iodir);
	ret |= write_reg(Register::GPPU, _gppu);

	if (ret != PX4_OK) {
		PX4_ERR("Device init failed (%i)", ret);
		return ret;
	}

	return init_uorb();
}

int MCP23009::probe()
{
	// no whoami, try to read IOCON
	uint8_t data;
	return read_reg(Register::IOCON, data);
}

int MCP23009::read(uint8_t *mask)
{
	return read_reg(Register::GPIO, *mask);
}

int MCP23009::write(uint8_t mask_set, uint8_t mask_clear)
{
	// no need to read, we can use the buffered register value
	_olat = (_olat & ~mask_clear) | mask_set;
	return write_reg(Register::OLAT, _olat);
}

int MCP23009::configure(uint8_t mask, PinType type)
{
	// no need to read, we can use the buffered register values
	switch (type) {
	case PinType::Input:
		_iodir |= mask;
		_gppu &= ~mask;
		break;

	case PinType::InputPullUp:
		_iodir |= mask;
		_gppu |= mask;
		break;

	case PinType::Output:
		_iodir &= ~mask;
		break;

	default:
		return -EINVAL;
	}

	int ret = write_reg(Register::GPPU, _gppu);

	if (ret != 0) {
		return ret;
	}

	return write_reg(Register::IODIR, _iodir);
}
