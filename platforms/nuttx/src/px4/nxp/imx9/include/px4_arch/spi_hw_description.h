/****************************************************************************
 *
 *   Copyright (C) 2024 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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

#if defined(CONFIG_SPI)

constexpr bool validateSPIConfig(const px4_spi_bus_t spi_busses_conf[SPI_BUS_MAX_BUS_ITEMS])
{
	const bool nuttx_enabled_spi_buses[] = {
#ifdef CONFIG_IMX9_LPSPI1
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI2
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI3
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI4
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI5
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI6
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI7
		true,
#else
		false,
#endif
#ifdef CONFIG_IMX9_LPSPI8
		true,
#else
		false,
#endif
	};

	for (unsigned i = 0; i < sizeof(nuttx_enabled_spi_buses) / sizeof(nuttx_enabled_spi_buses[0]); ++i) {
		bool found_bus = false;

		for (int j = 0; j < SPI_BUS_MAX_BUS_ITEMS; ++j) {
			if (spi_busses_conf[j].bus == (int)i + 1) {
				found_bus = true;
			}
		}

		// Either the bus is enabled in NuttX and configured in spi_busses_conf, or disabled and not configured
		constexpr_assert(found_bus == nuttx_enabled_spi_buses[i], "SPI bus config mismatch (CONFIG_IMX9_LPSPIx)");
	}

	return false;
}

static inline constexpr px4_spi_bus_device_t initSPIDevice(uint32_t devid, SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	px4_spi_bus_device_t ret{};

	ret.cs_gpio = getGPIOPort(cs_gpio.port) | getGPIOPin(cs_gpio.pin) | GPIO_OUTPUT | GPIO_OUTPUT_ONE;

	if (drdy_gpio.port != GPIO::PortInvalid) {
		ret.drdy_gpio = getGPIOPort(drdy_gpio.port) | getGPIOPin(drdy_gpio.pin) | GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE;
	}

	if (PX4_SPIDEVID_TYPE(devid) == 0) { // it's a PX4 device (internal or external)
		uint32_t address = ((cs_gpio.port << 5) | cs_gpio.pin) & 0xff; // address on the bus, 3 bits for port 5 for pin
		ret.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, devid | (address << 8));

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
	int i = 0;

	ret.bus = (int)bus;
	ret.is_external = false;

	for (; i < SPI_BUS_MAX_DEVICES; ++i) {
		ret.devices[i] = devices.devices[i];

		if (devices.devices[i].cs_gpio == 0) {
			break;
		}
	}

	/* Locking is required if there are more than 1 device on the bus */
	ret.requires_locking = i > 1 ? true : false;

	if (power_enable.port != GPIO::PortInvalid) {
		ret.power_enable_gpio = 0; //TODO!
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
		if (devices.devices[i].cs_gpio.port == GPIO::PortInvalid) {
			break;
		}

		ret.devices[i] = initSPIDevice(i, devices.devices[i].cs_gpio, devices.devices[i].drdy_gpio);
	}

	ret.bus = (int)bus;
	ret.is_external = true;
	ret.requires_locking = true;
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

constexpr bool validateSPIConfig(const px4_spi_bus_t spi_buses_conf[SPI_BUS_MAX_BUS_ITEMS]);

constexpr bool validateSPIConfig(const px4_spi_bus_all_hw_t spi_buses_conf[BOARD_NUM_SPI_CFG_HW_VERSIONS])
{
	for (int ver = 0; ver < BOARD_NUM_SPI_CFG_HW_VERSIONS; ++ver) {
		validateSPIConfig(spi_buses_conf[ver].buses);
	}

	for (int ver = 1; ver < BOARD_NUM_SPI_CFG_HW_VERSIONS; ++ver) {
		for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
			const bool equal_power_enable_gpio = spi_buses_conf[ver].buses[i].power_enable_gpio == spi_buses_conf[ver -
							     1].buses[i].power_enable_gpio;
			// currently board_control_spi_sensors_power_configgpio() depends on that - this restriction can be removed
			// by ensuring board_control_spi_sensors_power_configgpio() is called after the hw version is determined
			// and SPI config is initialized.
			constexpr_assert(equal_power_enable_gpio, "All HW versions must define the same power enable GPIO");
		}
	}

	return false;
}

#endif // CONFIG_SPI
