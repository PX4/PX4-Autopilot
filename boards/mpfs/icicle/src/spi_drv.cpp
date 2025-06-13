/****************************************************************************
 *
 *   Copyright (C) 2021 Technology Innovation Institute. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <px4_platform_common/spi.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

static const px4_spi_bus_t *_spi_bus0;
static const px4_spi_bus_t *_spi_bus1;

static inline void board_spix_select(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			mpfs_gpiowrite(bus->devices[i].cs_gpio, !selected);
		}
	}
}

#if defined(CONFIG_MPFS_SPI0)
__EXPORT void mpfs_spi0_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	board_spix_select(_spi_bus0, dev, devid, selected);
}
#endif

#if defined(CONFIG_MPFS_SPI1)
__EXPORT void mpfs_spi1_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	board_spix_select(_spi_bus1, dev, devid, selected);
}
#endif


__EXPORT void board_spidev_initialize(void)
{
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
			}
		}
	}
}

__EXPORT int board_spibus_initialize(void)
{
	// NOTE: MPFS uses 0based bus numbering

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 1: _spi_bus0 = &px4_spi_buses[i]; break;

		case 2: _spi_bus1 = &px4_spi_buses[i]; break;
		}
	}

	struct spi_dev_s *spi_bus0 = px4_spibus_initialize(1);

	if (!spi_bus0) {
		return -ENODEV;
	}

	struct spi_dev_s *spi_bus1 = px4_spibus_initialize(2);

	if (!spi_bus1) {
		return -ENODEV;
	}

	/* deselect all */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				SPI_SELECT(spi_bus0, px4_spi_buses[bus].devices[i].devid, false);
				SPI_SELECT(spi_bus1, px4_spi_buses[bus].devices[i].devid, false);
			}
		}
	}

	return OK;
}
