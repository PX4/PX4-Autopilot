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

#include <board_config.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/spi.h>
#include <px4_arch/micro_hal.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <arm_arch.h>
#include <chip.h>
// #include <rp2040_gpio.h>

static const px4_spi_bus_t *_spi_bus0;
static const px4_spi_bus_t *_spi_bus1;

static void spi_bus_configgpio_cs(const px4_spi_bus_t *bus)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio != 0) {
			px4_arch_configgpio(bus->devices[i].cs_gpio | GPIO_FUN(RP2040_GPIO_FUNC_SIO));
		}
	}
}

__EXPORT void rp2040_spiinitialize()
{
	px4_set_spi_buses_from_hw_version();
	board_control_spi_sensors_power_configgpio();
	board_control_spi_sensors_power(true, 0xffff);

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case PX4_BUS_NUMBER_TO_PX4(0): _spi_bus0 = &px4_spi_buses[i]; break;

		case PX4_BUS_NUMBER_TO_PX4(1): _spi_bus1 = &px4_spi_buses[i]; break;
		}
	}

	/* Set default SPI pin */
	#if defined(CONFIG_RP2040_SPI0) && defined(GPIO_SPI0_SCLK) && defined(GPIO_SPI0_MISO) && defined(GPIO_SPI0_MOSI)
	px4_arch_configgpio(GPIO_SPI0_SCLK);
	px4_arch_configgpio(GPIO_SPI0_MISO);
	px4_arch_configgpio(GPIO_SPI0_MOSI);
	#endif

	#if defined(CONFIG_RP2040_SPI1) && defined(GPIO_SPI1_SCLK) && defined(GPIO_SPI1_MISO) && defined(GPIO_SPI1_MOSI)
	px4_arch_configgpio(GPIO_SPI1_SCLK);
	px4_arch_configgpio(GPIO_SPI1_MISO);
	px4_arch_configgpio(GPIO_SPI1_MOSI);
	#endif

	#ifdef CONFIG_RP2040_SPI0
	ASSERT(_spi_bus0);
	if (board_has_bus(BOARD_SPI_BUS, PX4_BUS_NUMBER_TO_PX4(0))) {
		spi_bus_configgpio_cs(_spi_bus0);
	}
	#endif // CONFIG_RP2040_SPI0

	#ifdef CONFIG_RP2040_SPI1
	ASSERT(_spi_bus1);
	if (board_has_bus(BOARD_SPI_BUS, PX4_BUS_NUMBER_TO_PX4(1))) {
		spi_bus_configgpio_cs(_spi_bus1);
	}
	#endif // CONFIG_RP2040_SPI1
}

static inline void rp2040_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			px4_arch_gpiowrite(bus->devices[i].cs_gpio, !selected);
		}
	}
}


/****************************************************************************
 * Name:  rp2040_spi0/1select and rp2040_spi0/1status
 *
 * Description:
 *   The external functions, rp2040_spi0/1select and rp2040_spi0/1status
 *   must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including rp2040_spibus_initialize()) are provided by
 *   common RP2040 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in rp2040_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide rp2040_spi0/1select() and rp2040_spi0/1status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to rp2040_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by rp2040_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/
#ifdef CONFIG_RP2040_SPI0
void rp2040_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  rp2040_spixselect(_spi_bus0, dev, devid, selected);
}

uint8_t rp2040_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_RP2040_SPI1
void rp2040_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");
  rp2040_spixselect(_spi_bus1, dev, devid, selected);
}

uint8_t rp2040_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}
#endif

void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
	const px4_spi_bus_t *buses = px4_spi_buses;
	// this might be called very early on boot where we have not yet determined the hw version
	// (we expect all versions to have the same power GPIO)
// #if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

// 	if (!buses) {
// 		buses = &px4_spi_buses_all_hw[0].buses[0];
// 	}

// #endif

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (buses[bus].bus == -1) {
			break;
		}

		const bool bus_matches = bus_mask & (1 << (buses[bus].bus - 1));

		if (buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, buses[bus].bus) ||
		    !bus_matches) {
			continue;
		}

		px4_arch_gpiowrite(buses[bus].power_enable_gpio, enable_power ? 1 : 0);
	}
}

void board_control_spi_sensors_power_configgpio()
{
	const px4_spi_bus_t *buses = px4_spi_buses;
	// this might be called very early on boot where we have yet not determined the hw version
	// (we expect all versions to have the same power GPIO)
// #if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

// 	if (!buses) {
// 		buses = &px4_spi_buses_all_hw[0].buses[0];
// 	}

// #endif

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (buses[bus].bus == -1) {
			break;
		}

		if (buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, buses[bus].bus)) {
			continue;
		}

		px4_arch_configgpio(buses[bus].power_enable_gpio);
	}
}

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
	bool has_power_enable = false;

	// disable SPI bus
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

		if (px4_spi_buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_requested) {
			continue;
		}

		has_power_enable = true;

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(PX4_GPIO_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(PX4_GPIO_PIN_OFF(px4_spi_buses[bus].devices[i].drdy_gpio));
			}
		}

#if defined(CONFIG_RP2040_SPI0)

		if (px4_spi_buses[bus].bus == 1) {
			px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SPI0_SCLK));
			px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SPI0_MISO));
			px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SPI0_MOSI));
		}

#endif
#if defined(CONFIG_RP2040_SPI1)

		if (px4_spi_buses[bus].bus == 2) {
			px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SPI1_SCLK));
			px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SPI1_MISO));
			px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SPI1_MOSI));
		}

#endif
	}

	if (!has_power_enable) {
		// board does not have power control over any of the sensor buses
		return;
	}

	// set the sensor rail(s) off
	board_control_spi_sensors_power(false, bus_mask);

	// wait for the sensor rail to reach GND
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	/* re-enable power */

	// switch the sensor rail back on
	board_control_spi_sensors_power(true, bus_mask);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

		if (px4_spi_buses[bus].power_enable_gpio == 0 ||
		    !board_has_bus(BOARD_SPI_BUS, px4_spi_buses[bus].bus) ||
		    !bus_requested) {
			continue;
		}

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].drdy_gpio);
			}
		}

#if defined(CONFIG_RP2040_SPI0)

		if (px4_spi_buses[bus].bus == 1) {
			px4_arch_configgpio(GPIO_SPI0_SCLK);
			px4_arch_configgpio(GPIO_SPI0_MISO);
			px4_arch_configgpio(GPIO_SPI0_MOSI);
		}

#endif
#if defined(CONFIG_RP2040_SPI1)

		if (px4_spi_buses[bus].bus == 2) {
			px4_arch_configgpio(GPIO_SPI1_SCLK);
			px4_arch_configgpio(GPIO_SPI1_MISO);
			px4_arch_configgpio(GPIO_SPI1_MOSI);
		}

#endif
	}
}
