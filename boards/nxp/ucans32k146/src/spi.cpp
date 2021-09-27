/************************************************************************************
 *
 *   Copyright (C) 2016, 2018, 2021 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 * 			  Landon Haugh <landon.haugh@nxp.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#include <nuttx/config.h>

#include <px4_arch/spi_hw_description.h>
#include <px4_platform_common/px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include <systemlib/px4_macros.h>

#include "s32k1xx_config.h"
#include "s32k1xx_lpspi.h"
#include "s32k1xx_pin.h"
#include "board_config.h"

#if defined(CONFIG_S32K1XX_LPSPI0) 
// Only LPSPI0 is defined in defconfig. cont. || defined(CONFIG_S32K1XX_LPSPI1) || defined(CONFIG_S32K1XX_LPSPI2)

// UCANS32K146 has only one external SPI @ PTB5
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBusExternal(SPI::Bus::SPI0, {
		// Going to assume PTB5 means PortB, Pin5
		initSPIConfigExternal(SPI::CS{GPIO::PortB, GPIO::Pin5})
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

#define PX4_MK_GPIO(pin_ftmx, io)    ((((uint32_t)(pin_ftmx)) & ~(_PIN_MODE_MASK | _PIN_OPTIONS_MASK)) |(io))


/************************************************************************************
 * Public Functions
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
	/* Goal not to back feed the chips on the bus via IO lines */

	/* Next Change CS to inputs with pull downs */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == PX4_BUS_NUMBER_TO_PX4(1)) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					// s32k1xx_pinconfig is defined
					// Only one argument, (uint32_t cfgset)
					s32k1xx_pinconfig(PX4_MK_GPIO(px4_spi_buses[bus].devices[i].cs_gpio, GPIO_PULLDOWN));
				}
			}
		}
	}

	/* Restore all the CS to ouputs inactive */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == PX4_BUS_NUMBER_TO_PX4(1)) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					s32k1xx_pinconfig(px4_spi_buses[bus].devices[i].cs_gpio);
				}
			}
		}
	}
}

/************************************************************************************
 * Name: s32k1xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP UCANS32K146 board.
 *
 ************************************************************************************/

void s32k1xx_spidev_initialize(void)
{
	board_spi_reset(10, 0xffff);

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				s32k1xx_pinconfig(px4_spi_buses[bus].devices[i].cs_gpio);
			}
		}
	}
}

/************************************************************************************
 * Name: s32k1xx_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP UCANS32K146 board.
 *
 ************************************************************************************/

static const px4_spi_bus_t *_spi_bus0;

__EXPORT int s32k1xx_spi_bus_initialize(void)
{
	
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case PX4_BUS_NUMBER_TO_PX4(1): _spi_bus0 = &px4_spi_buses[i]; break;
		}
	}
	
	struct spi_dev_s *spi_ext = px4_spibus_initialize(PX4_BUS_NUMBER_TO_PX4(1));

	if (!spi_ext) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", 2);
		return -ENODEV;
	}

	/* Default external bus to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_ext, 8 * 1000 * 1000);
	SPI_SETBITS(spi_ext, 8);
	SPI_SETMODE(spi_ext, SPIDEV_MODE3);

	/* deselect all */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				SPI_SELECT(spi_ext, px4_spi_buses[bus].devices[i].devid, false);
			}
		}
	}

	return OK;

}

/************************************************************************************
 * Name: s32k1xx_spi[n]select, s32k1xx_spi[n]status, and s32k1xx_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including s32k1xx_spibus_initialize()) are provided by common s32k1xx logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in s32k1xx_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide s32k1xx_spi[n]select() and s32k1xx_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      s32k1xx_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to s32k1xx_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by s32k1xx_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

static inline void s32k1xx_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			// s32k1xx_gpiowrite is defined (uint32_t pinset, bool value)
			s32k1xx_gpiowrite(bus->devices[i].cs_gpio, !selected);
		}
	}
}


void s32k1xx_lpspi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
	s32k1xx_spixselect(_spi_bus0, dev, devid, selected);
}

uint8_t s32k1xx_lpspi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

#endif /* CONFIG_S32K1XX_LPSPI0 */
