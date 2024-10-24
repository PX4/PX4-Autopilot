/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include <px4_arch/hw_description.h>
#include <px4_platform_common/spi.h>

#include <mpfs_gpio.h>

#if defined(CONFIG_SPI)

static inline constexpr px4_spi_bus_device_t initSPIDevice(uint32_t devid, SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	px4_spi_bus_device_t ret{};

	ret.cs_gpio = getGPIOBank(cs_gpio.bank) | getGPIOPin(cs_gpio.pin) | (GPIO_OUTPUT | GPIO_BUFFER_ENABLE);

	if (drdy_gpio.bank != GPIO::BankInvalid) {
		ret.drdy_gpio = getGPIOBank(drdy_gpio.bank) | getGPIOPin(drdy_gpio.pin) | GPIO_INPUT;
	}

	if (PX4_SPIDEVID_TYPE(devid) == 0) { // it's a PX4 device (internal or external)
		// Construct device id including cs_gpio pin information into address field
		ret.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, devid) | (cs_gpio.pin << 8);

	} else { // it's a NuttX device (e.g. SPIDEV_FLASH(0))
		ret.devid = devid;
	}

	ret.devtype_driver = PX4_SPI_DEV_ID(devid);

	return ret;
}

static inline constexpr px4_spi_bus_t initSPIBusInternal(SPI::Bus bus, const px4_spi_bus_devices_t &devices,
		GPIO::GPIOPin power_enable = {})
{
	px4_spi_bus_t ret{};

	ret.requires_locking = false;

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		ret.devices[i] = devices.devices[i];

		// check that the same device is configured only once (the chip-select code depends on that)
		for (int j = i + 1; j < SPI_BUS_MAX_DEVICES; ++j) {
			if (ret.devices[j].cs_gpio != 0) {
				constexpr_assert(ret.devices[i].devid != ret.devices[j].devid, "Same device configured multiple times");
			}
		}

		if (ret.devices[i].cs_gpio != 0) {
			// A bus potentially requires locking if it is accessed by non-PX4 devices (i.e. NuttX drivers)
			if (PX4_SPI_DEVICE_ID != PX4_SPIDEVID_TYPE(ret.devices[i].devid)) {
				ret.requires_locking = true;
			}
		}
	}

	ret.bus = (int)bus;
	ret.is_external = false;

	if (power_enable.bank != GPIO::BankInvalid) {
		ret.power_enable_gpio = getGPIOBank(power_enable.bank) | getGPIOPin(power_enable.pin) | GPIO_OUTPUT;
	}

	return ret;
}

// just a wrapper since we cannot pass brace-enclosed initialized arrays directly as arguments
struct bus_device_external_cfg_array_t {
	SPI::bus_device_external_cfg_t devices[SPI_BUS_MAX_DEVICES];
};

static inline constexpr px4_spi_bus_t initSPIBusExternal(SPI::Bus bus, const bus_device_external_cfg_array_t &devices)
{
	px4_spi_bus_t ret{};

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (devices.devices[i].cs_gpio.bank == GPIO::BankInvalid) {
			break;
		}

		ret.devices[i] = initSPIDevice(i, devices.devices[i].cs_gpio, devices.devices[i].drdy_gpio);
	}

	ret.bus = (int)bus;
	ret.is_external = true;
	ret.requires_locking = false; // external buses are never accessed by NuttX drivers
	return ret;

	return ret;
}

static inline constexpr SPI::bus_device_external_cfg_t initSPIConfigExternal(SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	SPI::bus_device_external_cfg_t ret{};
	ret.cs_gpio = cs_gpio;
	ret.drdy_gpio = drdy_gpio;
	return ret;
}

struct px4_spi_bus_array_t {
	px4_spi_bus_t item[SPI_BUS_MAX_BUS_ITEMS];
};
static inline constexpr px4_spi_bus_all_hw_t initSPIHWVersion(int hw_version_revision,
		const px4_spi_bus_array_t &bus_items)
{
	px4_spi_bus_all_hw_t ret{};

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		ret.buses[i] = bus_items.item[i];
	}

	ret.board_hw_version_revision = hw_version_revision;
	return ret;
}
#endif // CONFIG_SPI
