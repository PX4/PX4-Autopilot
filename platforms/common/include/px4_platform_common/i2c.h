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

#include <board_config.h>
#include <px4_platform_common/bus_topology.h>

#if defined(CONFIG_I2C)

#define I2C_BUS_MAX_BUS_ITEMS PX4_NUMBER_I2C_BUSES

struct px4_i2c_bus_t {
	int bus{-1}; ///< physical bus number (1, ...) (-1 means this is unused)
	BusTopology topology{BusTopology::External};
};

__EXPORT extern const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS]; ///< board-specific I2C bus configuration

static inline constexpr px4_i2c_bus_t initI2CBusInternal(int bus)
{
	px4_i2c_bus_t ret{};
	ret.bus = bus;
	ret.topology = BusTopology::Internal;
	return ret;
}

static inline constexpr px4_i2c_bus_t initI2CBusExternal(int bus)
{
	px4_i2c_bus_t ret{};
	ret.bus = bus;
	ret.topology = BusTopology::External;
	return ret;
}

static inline constexpr px4_i2c_bus_t initI2CBusShared(int bus)
{
	px4_i2c_bus_t ret{};
	ret.bus = bus;
	ret.topology = BusTopology::Shared;
	return ret;
}

/**
 * Bus topology for a physical I2C bus.
 * Unknown bus numbers are External (a connector we have not listed).
 * Boards may override this (BOARD_OVERRIDE_I2C_BUS_TOPOLOGY) for runtime HW variants.
 */
__EXPORT BusTopology px4_i2c_bus_topology(int bus);

static inline bool px4_i2c_bus_has_external(int bus)
{
	return bus_topology_has_external(px4_i2c_bus_topology(bus));
}

static inline bool px4_i2c_bus_has_internal(int bus)
{
	return bus_topology_has_internal(px4_i2c_bus_topology(bus));
}

/**
 * @class I2CBusIterator
 * Iterate over configured I2C buses by the board.
 *
 * InternalBus: Internal buses, plus a Shared bus when -b pins that bus.
 * ExternalBus: External and Shared buses (probe for unknown devices).
 *
 * -b is the physical bus number (1, 2, ...), not an external-bus index.
 * external() is the sensor classification implied by the filter (-I vs -X),
 * not the bus topology.
 */
class I2CBusIterator
{
public:
	enum class FilterType {
		All, ///< specific or all buses
		InternalBus, ///< onboard sensors: Internal buses, Shared if -b is given
		ExternalBus, ///< connector sensors: External and Shared buses
	};

	I2CBusIterator(FilterType filter, int bus = -1)
		: _filter(filter), _bus(bus) {}

	bool next();

	const px4_i2c_bus_t &bus() const { return px4_i2c_buses[_index]; }

	int externalBusIndex() const { return _external_bus_counter; }

	bool external() const { return _filter == FilterType::ExternalBus; }

private:
	const FilterType _filter;
	const int _bus;
	int _index{-1};
	int _external_bus_counter{0};
};

#endif // CONFIG_I2C
