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

#if defined(CONFIG_SPI)

#include <px4_platform_common/spi.h>

#ifndef GPIO_PIN_MASK
#define GPIO_PIN_MASK 0
#endif

#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1
void px4_set_spi_buses_from_hw_version()
{
#if defined(BOARD_HAS_SIMPLE_HW_VERSIONING)
	int hw_version_revision = board_get_hw_version();
#else
	int hw_version_revision = HW_VER_REV(board_get_hw_version(), board_get_hw_revision());
#endif


	for (int i = 0; i < BOARD_NUM_SPI_CFG_HW_VERSIONS; ++i) {
		if (!px4_spi_buses && px4_spi_buses_all_hw[i].board_hw_version_revision == 0) {
			px4_spi_buses = px4_spi_buses_all_hw[i].buses;
		}

		if (px4_spi_buses_all_hw[i].board_hw_version_revision == hw_version_revision) {
			px4_spi_buses = px4_spi_buses_all_hw[i].buses;
		}
	}

	if (!px4_spi_buses) { // fallback
		px4_spi_buses = px4_spi_buses_all_hw[0].buses;
	}
}

const px4_spi_bus_t *px4_spi_buses{nullptr};
#endif

int px4_find_spi_bus(uint32_t devid)
{
	for (int i = 0; ((px4_spi_bus_t *) px4_spi_buses) != nullptr && i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		const px4_spi_bus_t &bus_data = px4_spi_buses[i];

		if (bus_data.bus == -1) {
			break;
		}

		if (px4_spi_bus_external(bus_data)) {
			continue;
		}

		for (int j = 0; j < SPI_BUS_MAX_DEVICES; ++j) {
			if (PX4_SPIDEVID_TYPE(devid) == PX4_SPIDEVID_TYPE(bus_data.devices[j].devid) &&
			    PX4_SPI_DEV_ID(devid) == bus_data.devices[j].devtype_driver) {
				return bus_data.bus;
			}
		}
	}

	return -1;
}

bool px4_spi_bus_requires_locking(int bus)
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		const px4_spi_bus_t &bus_data = px4_spi_buses[i];

		if (bus_data.bus == bus) {
			return bus_data.requires_locking;
		}
	}

	return true;
}


bool px4_spi_bus_external(const px4_spi_bus_t &bus)
{
	return bus.is_external;
}

bool SPIBusIterator::next()
{
	while (_index < SPI_BUS_MAX_BUS_ITEMS && px4_spi_buses[_index].bus != -1) {
		const px4_spi_bus_t &bus_data = px4_spi_buses[_index];

		if (board_has_bus(BOARD_SPI_BUS, bus_data.bus)) {

			// Note: we use bus_data.is_external here instead of px4_spi_bus_external(),
			// otherwise the chip-select matching does not work if a bus is configured as
			// external/internal, but at runtime the other way around.
			// (On boards where a bus can be internal/external at runtime, it should be
			// configured as external.)
			switch (_filter) {
			case FilterType::InternalBus:
				if (!bus_data.is_external) {
					if (_bus == bus_data.bus || _bus == -1) {
						// Note: if chipselect < 0, it's not defined and used in filter
						// find device
						for (int i = _bus_device_index + 1; i < SPI_BUS_MAX_DEVICES; ++i) {
							if (PX4_SPI_DEVICE_ID == PX4_SPIDEVID_TYPE(bus_data.devices[i].devid) &&
							    _devid_driver_index == bus_data.devices[i].devtype_driver &&
							    (_chipselect < 0 || _chipselect == (int16_t)(bus_data.devices[i].cs_gpio & GPIO_PIN_MASK))) {
								_bus_device_index = i;
								return true;
							}
						}
					}
				}

				break;

			case FilterType::ExternalBus:
				if (bus_data.is_external) {
					// Note: chip-select index is starting from 1 in CLI, -1 means not defined
					uint16_t cs_index = _chipselect < 1 ? 0 : _chipselect - 1;

					if (_bus == _external_bus_counter && cs_index < SPI_BUS_MAX_DEVICES &&
					    bus_data.devices[cs_index].cs_gpio != 0 && cs_index != _bus_device_index) {
						// we know that bus_data.devices[cs_index].devtype_driver == cs_index
						_bus_device_index = cs_index;
						return true;
					}
				}

				break;
			}

			if (bus_data.is_external) {
				++_external_bus_counter;
			}
		}

		++_index;
		_bus_device_index = -1;
	}

	return false;
}

#endif // CONFIG_SPI
