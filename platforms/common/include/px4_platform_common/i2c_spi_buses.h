/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include "i2c.h"
#include "spi.h"

#include <stdint.h>

#include <board_config.h>

enum class I2CSPIBusOption : uint8_t {
	All = 0, ///< select all runnning instances
	I2CInternal,
	I2CExternal,
	SPIInternal,
	SPIExternal,
};

/**
 * @class I2CSPIInstance
 * I2C/SPI driver instance used by BusInstanceIterator to find running instances.
 */
class I2CSPIInstance
{
public:
	I2CSPIInstance(I2CSPIBusOption bus_option, int bus)
		: _bus_option(bus_option), _bus(bus) {}

	virtual ~I2CSPIInstance() = default;

private:
	friend class BusInstanceIterator;

	const I2CSPIBusOption _bus_option;
	const int _bus;
};

struct BusCLIArguments {
	I2CSPIBusOption bus_option{I2CSPIBusOption::All};
	int requested_bus{-1};
	int chipselect_index{1};
	Rotation rotation{ROTATION_NONE};
	int custom1; ///< driver-specific custom argument
	int custom2; ///< driver-specific custom argument
};

/**
 * @class BusInstanceIterator
 * Iterate over running instances and/or configured I2C/SPI buses with given filter options.
 */
class BusInstanceIterator
{
public:
	BusInstanceIterator(I2CSPIInstance **instances, int max_num_instances,
			    const BusCLIArguments &cli_arguments, uint16_t devid_driver_index);
	~BusInstanceIterator() = default;

	I2CSPIBusOption configuredBusOption() const { return _bus_option; }

	int nextFreeInstance() const;

	bool next();

	I2CSPIInstance *instance() const;
	void resetInstance();
	board_bus_types busType() const;
	int bus() const;
	uint32_t devid() const;
	bool external() const;

	static I2CBusIterator::FilterType i2cFilter(I2CSPIBusOption bus_option);
	static SPIBusIterator::FilterType spiFilter(I2CSPIBusOption bus_option);
private:
	I2CSPIInstance **_instances;
	const int _max_num_instances;
	const I2CSPIBusOption _bus_option;
	SPIBusIterator _spi_bus_iterator;
	I2CBusIterator _i2c_bus_iterator;
	int _current_instance{-1};
};
