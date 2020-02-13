/****************************************************************************
 *
 * Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#ifndef BOARD_DISABLE_I2C_SPI

#ifndef MODULE_NAME
#define MODULE_NAME "SPI_I2C"
#endif

#include <px4_platform_common/i2c_spi_buses.h>

BusInstanceIterator::BusInstanceIterator(I2CSPIInstance **instances, int max_num_instances,
		const BusCLIArguments &cli_arguments, uint16_t devid_driver_index)
	: _instances(instances), _max_num_instances(max_num_instances),
	  _bus_option(cli_arguments.bus_option),
	  _spi_bus_iterator(spiFilter(cli_arguments.bus_option),
			    cli_arguments.bus_option == I2CSPIBusOption::SPIExternal ? cli_arguments.chipselect_index : devid_driver_index,
			    cli_arguments.requested_bus),
	  _i2c_bus_iterator(i2cFilter(cli_arguments.bus_option), cli_arguments.requested_bus)
{
}

bool BusInstanceIterator::next()
{
	int bus = -1;

	if (busType() == BOARD_INVALID_BUS) {
		while (++_current_instance < _max_num_instances && _instances[_current_instance] == nullptr) {}

		return _current_instance < _max_num_instances;

	} else if (busType() == BOARD_SPI_BUS) {
		if (_spi_bus_iterator.next()) {
			bus = _spi_bus_iterator.bus().bus;
		}

	} else {
		if (_i2c_bus_iterator.next()) {
			bus = _i2c_bus_iterator.bus().bus;
		}
	}

	if (bus != -1) {
		// find matching runtime instance
		_current_instance = -1;

		for (int i = 0; i < _max_num_instances; ++i) {
			if (!_instances[i]) {
				continue;
			}

			if (_bus_option == _instances[i]->_bus_option && bus == _instances[i]->_bus) {
				_current_instance = i;
			}
		}

		return true;
	}

	return false;
}

int BusInstanceIterator::nextFreeInstance() const
{
	for (int i = 0; i < _max_num_instances; ++i) {
		if (_instances[i] == nullptr) {
			return i;
		}
	}

	return -1;
}

I2CSPIInstance *BusInstanceIterator::instance() const
{
	if (_current_instance < 0 || _current_instance >= _max_num_instances) {
		return nullptr;
	}

	return _instances[_current_instance];
}

void BusInstanceIterator::resetInstance()
{
	if (_current_instance >= 0 && _current_instance < _max_num_instances) {
		_instances[_current_instance] = nullptr;
	}
}

board_bus_types BusInstanceIterator::busType() const
{
	switch (_bus_option) {
	case I2CSPIBusOption::All:
		return BOARD_INVALID_BUS;

	case I2CSPIBusOption::I2CInternal:
	case I2CSPIBusOption::I2CExternal:
		return BOARD_I2C_BUS;

	case I2CSPIBusOption::SPIInternal:
	case I2CSPIBusOption::SPIExternal:
		return BOARD_SPI_BUS;
	}

	return BOARD_INVALID_BUS;
}

int BusInstanceIterator::bus() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return -1;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.bus().bus;

	} else {
		return _i2c_bus_iterator.bus().bus;
	}
}

uint32_t BusInstanceIterator::devid() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return 0;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.devid();

	} else {
		return 0;
	}
}

bool BusInstanceIterator::external() const
{
	if (busType() == BOARD_INVALID_BUS) {
		return false;

	} else if (busType() == BOARD_SPI_BUS) {
		return _spi_bus_iterator.external();

	} else {
		return _i2c_bus_iterator.external();
	}
}

I2CBusIterator::FilterType BusInstanceIterator::i2cFilter(I2CSPIBusOption bus_option)
{
	switch (bus_option) {
	case I2CSPIBusOption::All: return I2CBusIterator::FilterType::All;

	case I2CSPIBusOption::I2CExternal: return I2CBusIterator::FilterType::ExternalBus;

	case I2CSPIBusOption::I2CInternal: return I2CBusIterator::FilterType::InternalBus;

	default: break;
	}

	return I2CBusIterator::FilterType::All;
}

SPIBusIterator::FilterType BusInstanceIterator::spiFilter(I2CSPIBusOption bus_option)
{
	switch (bus_option) {
	case I2CSPIBusOption::SPIExternal: return SPIBusIterator::FilterType::ExternalBus;

	case I2CSPIBusOption::SPIInternal: return SPIBusIterator::FilterType::InternalBus;

	default: break;
	}

	return SPIBusIterator::FilterType::InternalBus;
}

#endif /* BOARD_DISABLE_I2C_SPI */
