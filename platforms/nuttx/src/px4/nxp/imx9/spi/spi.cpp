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

#include <board_config.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/spi.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <arm_internal.h>
#include <chip.h>
#include <imx9_gpio.h>

static const px4_spi_bus_t *_spi_bus1;
static const px4_spi_bus_t *_spi_bus2;
static const px4_spi_bus_t *_spi_bus3;
static const px4_spi_bus_t *_spi_bus4;
static const px4_spi_bus_t *_spi_bus5;
static const px4_spi_bus_t *_spi_bus6;
static const px4_spi_bus_t *_spi_bus7;
static const px4_spi_bus_t *_spi_bus8;

static void spi_bus_configgpio_cs(const px4_spi_bus_t *bus)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio != 0) {
			px4_arch_configgpio(bus->devices[i].cs_gpio);
		}
	}
}

__EXPORT void imx9_spiinitialize()
{
	px4_set_spi_buses_from_hw_version();
	board_control_spi_sensors_power_configgpio();
	board_control_spi_sensors_power(true, 0xffff);

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 1: _spi_bus1 = &px4_spi_buses[i]; break;

		case 2: _spi_bus2 = &px4_spi_buses[i]; break;

		case 3: _spi_bus3 = &px4_spi_buses[i]; break;

		case 4: _spi_bus4 = &px4_spi_buses[i]; break;

		case 5: _spi_bus5 = &px4_spi_buses[i]; break;

		case 6: _spi_bus6 = &px4_spi_buses[i]; break;

		case 7: _spi_bus7 = &px4_spi_buses[i]; break;

		case 8: _spi_bus8 = &px4_spi_buses[i]; break;
		}
	}

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		if (px4_spi_buses[i].bus > 0 && board_has_bus(BOARD_SPI_BUS, px4_spi_buses[i].bus)) {
			spi_bus_configgpio_cs(&px4_spi_buses[i]);
		}
	}
}

static inline void imx9_lpspixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			imx9_gpio_write(bus->devices[i].cs_gpio, !selected);
		}
	}
}


/************************************************************************************
 * Name: imx9_lpspi1select and imx9_lpspi1status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 1.
 *
 ************************************************************************************/
#ifdef CONFIG_IMX9_LPSPI1

__EXPORT void imx9_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus1, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI1

/************************************************************************************
 * Name: imx9_lpspi2select and imx9_lpspi2status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 2.
 *
 ************************************************************************************/
#if defined(CONFIG_IMX9_LPSPI2)
__EXPORT void imx9_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus2, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI2

/************************************************************************************
 * Name: imx9_lpspi3select and imx9_lpspi3status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 3.
 *
 ************************************************************************************/
#if defined(CONFIG_IMX9_LPSPI3)
__EXPORT void imx9_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus3, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI3

/************************************************************************************
 * Name: imx9_lpspi4select and imx9_lpspi4status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 4.
 *
 ************************************************************************************/
#ifdef CONFIG_IMX9_LPSPI4

__EXPORT void imx9_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus4, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI4

/************************************************************************************
 * Name: imx9_lpspi5select and imx9_lpspi5status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 5.
 *
 ************************************************************************************/
#ifdef CONFIG_IMX9_LPSPI5

__EXPORT void imx9_lpspi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus5, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI5

/************************************************************************************
 * Name: imx9_lpspi6select and imx9_lpspi6status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 6.
 *
 ************************************************************************************/
#ifdef CONFIG_IMX9_LPSPI6

__EXPORT void imx9_lpspi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus6, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi6status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI6

/************************************************************************************
 * Name: imx9_lpspi7select and imx9_lpspi7status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 7.
 *
 ************************************************************************************/
#ifdef CONFIG_IMX9_LPSPI7

__EXPORT void imx9_lpspi7select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus7, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi7status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI7

/************************************************************************************
 * Name: imx9_lpspi8select and imx9_lpspi8status
 *
 * Description:
 *   Called by the imx9 LPSPI driver on bus 8.
 *
 ************************************************************************************/
#ifdef CONFIG_IMX9_LPSPI8

__EXPORT void imx9_lpspi8select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imx9_lpspixselect(_spi_bus8, dev, devid, selected);
}

__EXPORT uint8_t imx9_lpspi8status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_IMX9_LPSPI8

void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
	const px4_spi_bus_t *buses = px4_spi_buses;
	// this might be called very early on boot where we have not yet determined the hw version
	// (we expect all versions to have the same power GPIO)
#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

	if (!buses) {
		buses = &px4_spi_buses_all_hw[0].buses[0];
	}

#endif

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
#if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

	if (!buses) {
		buses = &px4_spi_buses_all_hw[0].buses[0];
	}

#endif

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
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
			}

			if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].drdy_gpio));
			}
		}

#if defined(CONFIG_IMX9_LPSPI1)

		if (px4_spi_buses[bus].bus == 1) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI1_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI1_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI1_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI2)

		if (px4_spi_buses[bus].bus == 2) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI2_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI2_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI2_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI3)

		if (px4_spi_buses[bus].bus == 3) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI3_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI3_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI3_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI4)

		if (px4_spi_buses[bus].bus == 4) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI4_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI4_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI4_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI5)

		if (px4_spi_buses[bus].bus == 5) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI5_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI5_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI5_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI6)

		if (px4_spi_buses[bus].bus == 6) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI6_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI6_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI6_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI7)

		if (px4_spi_buses[bus].bus == 7) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI7_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI7_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI7_MOSI));
		}

#endif
#if defined(CONFIG_IMX9_LPSPI8)

		if (px4_spi_buses[bus].bus == 8) {
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI8_SCK));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI8_MISO));
			px4_arch_configgpio(_PIN_OFF(GPIO_LPSPI8_MOSI));
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

#if defined(CONFIG_IMX9_LPSPI1)

		if (px4_spi_buses[bus].bus == 1) {
			px4_arch_configgpio(GPIO_LPSPI1_SCK);
			px4_arch_configgpio(GPIO_LPSPI1_MISO);
			px4_arch_configgpio(GPIO_LPSPI1_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI2)

		if (px4_spi_buses[bus].bus == 2) {
			px4_arch_configgpio(GPIO_LPSPI2_SCK);
			px4_arch_configgpio(GPIO_LPSPI2_MISO);
			px4_arch_configgpio(GPIO_LPSPI2_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI3)

		if (px4_spi_buses[bus].bus == 3) {
			px4_arch_configgpio(GPIO_LPSPI3_SCK);
			px4_arch_configgpio(GPIO_LPSPI3_MISO);
			px4_arch_configgpio(GPIO_LPSPI3_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI4)

		if (px4_spi_buses[bus].bus == 4) {
			px4_arch_configgpio(GPIO_LPSPI4_SCK);
			px4_arch_configgpio(GPIO_LPSPI4_MISO);
			px4_arch_configgpio(GPIO_LPSPI4_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI5)

		if (px4_spi_buses[bus].bus == 5) {
			px4_arch_configgpio(GPIO_LPSPI5_SCK);
			px4_arch_configgpio(GPIO_LPSPI5_MISO);
			px4_arch_configgpio(GPIO_LPSPI5_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI6)

		if (px4_spi_buses[bus].bus == 6) {
			px4_arch_configgpio(GPIO_LPSPI6_SCK);
			px4_arch_configgpio(GPIO_LPSPI6_MISO);
			px4_arch_configgpio(GPIO_LPSPI6_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI7)

		if (px4_spi_buses[bus].bus == 7) {
			px4_arch_configgpio(GPIO_LPSPI7_SCK);
			px4_arch_configgpio(GPIO_LPSPI7_MISO);
			px4_arch_configgpio(GPIO_LPSPI7_MOSI);
		}

#endif
#if defined(CONFIG_IMX9_LPSPI8)

		if (px4_spi_buses[bus].bus == 8) {
			px4_arch_configgpio(GPIO_LPSPI8_SCK);
			px4_arch_configgpio(GPIO_LPSPI8_MISO);
			px4_arch_configgpio(GPIO_LPSPI8_MOSI);
		}

#endif
	}
}
