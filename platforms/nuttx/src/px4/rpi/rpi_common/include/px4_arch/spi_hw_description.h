/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
#include <px4_arch/micro_hal.h>

static inline constexpr px4_spi_bus_device_t initSPIDevice(uint32_t devid, SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	px4_spi_bus_device_t ret{};
	ret.cs_gpio = getGPIOPin(cs_gpio.pin) | (GPIO_OUT | GPIO_SET);
	ret.drdy_gpio = getGPIOPin(drdy_gpio.pin) | GPIO_PU;	// GPIO_PU taken from kinetis

	if (PX4_SPIDEVID_TYPE(devid) == 0) { // it's a PX4 device (internal or external)
		ret.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, devid);

	} else { // it's a NuttX device (e.g. SPIDEV_FLASH(0))
		ret.devid = devid;
	}

	ret.devtype_driver = PX4_SPI_DEV_ID(devid);
	return ret;
}

static inline constexpr px4_spi_bus_t initSPIBus(SPI::Bus bus, const px4_spi_bus_devices_t &devices,
		GPIO::GPIOPin power_enable = {})
{
	px4_spi_bus_t ret{};
	ret.requires_locking = false;

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		ret.devices[i] = devices.devices[i];


		if (ret.devices[i].cs_gpio != 0) {
			if (PX4_SPI_DEVICE_ID == PX4_SPIDEVID_TYPE(ret.devices[i].devid)) {
				int same_devices_count = 0;

				for (int j = 0; j < i; ++j) {
					if (ret.devices[j].cs_gpio != 0) {
						same_devices_count += (ret.devices[i].devid & 0xff) == (ret.devices[j].devid & 0xff);
					}
				}

				// increment the 2. LSB byte to allow multiple devices of the same type
				ret.devices[i].devid |= same_devices_count << 8;

			} else {
				// A bus potentially requires locking if it is accessed by non-PX4 devices (i.e. NuttX drivers)
				ret.requires_locking = true;
			}
		}
	}

	ret.bus = (int)bus;
	ret.is_external = false;

	ret.power_enable_gpio = getGPIOPin(power_enable.pin) | GPIO_OUT;

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
		ret.devices[i] = initSPIDevice(i, devices.devices[i].cs_gpio, devices.devices[i].drdy_gpio);
	}

	ret.bus = (int)bus;
	ret.is_external = true;
	ret.requires_locking = false; // external buses are never accessed by NuttX drivers
	return ret;
}

static inline constexpr SPI::bus_device_external_cfg_t initSPIConfigExternal(SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	SPI::bus_device_external_cfg_t ret{};
	ret.cs_gpio = cs_gpio;
	ret.drdy_gpio = drdy_gpio;
	return ret;
}

// struct px4_spi_bus_array_t {
// 	px4_spi_bus_t item[SPI_BUS_MAX_BUS_ITEMS];
// };
// static inline constexpr px4_spi_bus_all_hw_t initSPIHWVersion(int hw_version_revision,
// 		const px4_spi_bus_array_t &bus_items)
// {
// 	px4_spi_bus_all_hw_t ret{};

// 	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
// 		ret.buses[i] = bus_items.item[i];
// 	}

// 	ret.board_hw_version_revision = hw_version_revision;
// 	return ret;
// }
constexpr bool validateSPIConfig(const px4_spi_bus_t spi_buses_conf[SPI_BUS_MAX_BUS_ITEMS]);

// constexpr bool validateSPIConfig(const px4_spi_bus_all_hw_t spi_buses_conf[BOARD_NUM_SPI_CFG_HW_VERSIONS])
// {
// 	for (int ver = 0; ver < BOARD_NUM_SPI_CFG_HW_VERSIONS; ++ver) {
// 		validateSPIConfig(spi_buses_conf[ver].buses);
// 	}

// 	for (int ver = 1; ver < BOARD_NUM_SPI_CFG_HW_VERSIONS; ++ver) {
// 		for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
// 			const bool equal_power_enable_gpio = spi_buses_conf[ver].buses[i].power_enable_gpio == spi_buses_conf[ver -
// 							     1].buses[i].power_enable_gpio;
// 			// currently board_control_spi_sensors_power_configgpio() depends on that - this restriction can be removed
// 			// by ensuring board_control_spi_sensors_power_configgpio() is called after the hw version is determined
// 			// and SPI config is initialized.
// 			constexpr_assert(equal_power_enable_gpio, "All HW versions must define the same power enable GPIO");
// 		}
// 	}

// 	return false;
// }
