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

#include <px4_platform_common/i2c.h>

#if defined(CONFIG_I2C)

#ifndef BOARD_OVERRIDE_I2C_BUS_EXTERNAL
bool px4_i2c_bus_external(int bus)
{
	if (px4_i2c_buses) {
		for (int i = 0; i < I2C_BUS_MAX_BUS_ITEMS; ++i) {
			if ((px4_i2c_buses[i].bus != -1) && (px4_i2c_buses[i].bus == bus)) {
				return px4_i2c_buses[i].is_external;
			}
		}
	}

	return true;
}
#endif // BOARD_OVERRIDE_I2C_BUS_EXTERNAL

#ifndef BOARD_OVERRIDE_I2C_DEVICE_EXTERNAL
#include <drivers/device/Device.hpp>
bool px4_i2c_device_external(const uint32_t device_id)
{
	device::Device::DeviceId dev_id{};
	dev_id.devid = device_id;
	return px4_i2c_bus_external(dev_id.devid_s.bus);
}
#endif // BOARD_OVERRIDE_I2C_DEVICE_EXTERNAL

bool I2CBusIterator::next()
{
	while (++_index < I2C_BUS_MAX_BUS_ITEMS && px4_i2c_buses[_index].bus != -1) {
		const px4_i2c_bus_t &bus_data = px4_i2c_buses[_index];

		if (!board_has_bus(BOARD_I2C_BUS, bus_data.bus)) {
			continue;
		}

		switch (_filter) {
		case FilterType::All:
			if (_bus == bus_data.bus || _bus == -1) {
				return true;
			}

			break;

		case FilterType::InternalBus:
			if (!px4_i2c_bus_external(bus_data.bus)) {
				if (_bus == bus_data.bus || _bus == -1) {
					return true;
				}
			}

			break;

		case FilterType::ExternalBus:
			if (px4_i2c_bus_external(bus_data.bus)) {
				++_external_bus_counter;

				if (_bus == bus_data.bus || _bus == -1) {
					return true;
				}
			}

			break;
		}
	}

	return false;
}

#endif // CONFIG_I2C
