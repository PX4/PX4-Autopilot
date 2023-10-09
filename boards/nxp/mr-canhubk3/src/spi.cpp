/************************************************************************************
 *
 *   Copyright (C) 2016, 2018, 2021 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *            Landon Haugh <landon.haugh@nxp.com>
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

#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include <systemlib/px4_macros.h>

#include "s32k3xx_config.h"
#include "s32k3xx_lpspi.h"
#include "s32k3xx_pin.h"
#include "board_config.h"

#if defined(CONFIG_S32K3XX_LPSPI)


constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBusExternal(SPI::Bus::SPI1, {
		initSPIConfigExternal(SPI::CS{GPIO::PortB, GPIO::Pin5})
	}),
	initSPIBusExternal(SPI::Bus::SPI2, { // SD Card
		initSPIConfigExternal(SPI::CS{GPIO::PortB, GPIO::Pin5})
	}),
	initSPIBus(SPI::Bus::SPI3, { // SPI3 is ignored only used for FS26 by a NuttX driver
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin17})
	}),
	initSPIBusExternal(SPI::Bus::SPI4, {
		initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin16}, SPI::DRDY{PIN_WKPU20}),
		initSPIConfigExternal(SPI::CS{GPIO::PortB, GPIO::Pin8}, SPI::DRDY{PIN_WKPU56})
	}),
	initSPIBusExternal(SPI::Bus::SPI5, {
		initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin14}, SPI::DRDY{PIN_WKPU4})
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
					// s32k3xx_pinconfig is defined
					// Only one argument, (uint32_t cfgset)
					s32k3xx_pinconfig(PX4_MK_GPIO(px4_spi_buses[bus].devices[i].cs_gpio, GPIO_PULLDOWN));
				}
			}
		}
	}

	/* Restore all the CS to ouputs inactive */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == PX4_BUS_NUMBER_TO_PX4(1)) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					s32k3xx_pinconfig(px4_spi_buses[bus].devices[i].cs_gpio);
				}
			}
		}
	}
}

/************************************************************************************
 * Name: s32k3xx_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP UCANS32K146 board.
 *
 ************************************************************************************/

void s32k3xx_spidev_initialize(void)
{
	board_spi_reset(10, 0xffff);

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				s32k3xx_pinconfig(px4_spi_buses[bus].devices[i].cs_gpio);
			}
		}
	}
}

/************************************************************************************
 * Name: s32k3xx_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP UCANS32K146 board.
 *
 ************************************************************************************/


__EXPORT int s32k3xx_spi_bus_initialize(void)
{

	struct spi_dev_s *spi_ext;

	spi_ext = px4_spibus_initialize(2);

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

	spi_ext = px4_spibus_initialize(3);

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

	spi_ext = px4_spibus_initialize(5);

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

	spi_ext = px4_spibus_initialize(6);

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

/****************************************************************************
 * Name: s32k3xx_lpspiNselect and s32k3xx_lpspiNstatus
 *
 * Description:
 *   The external functions, s32k3xx_lpspiNselect and s32k3xx_lpspiNstatus
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).  All other methods (including
 *   s32k3xx_lpspibus_initialize()) are provided by common logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in s32k3xx_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide s32k3xx_lpspiNselect() and s32k3xx_lpspiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to s32k3xx_lpspibus_initialize() in your low level
 *      application initialization logic.
 *   4. The handle returned by s32k3xx_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_S32K3XX_LPSPI1
/* LPSPI1 *******************************************************************/

void s32k3xx_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid,
			  bool selected)
{
	spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
		selected ? "assert" : "de-assert");

	s32k3xx_gpiowrite(PIN_LPSPI1_PCS, !selected);
}

uint8_t s32k3xx_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_S32K3XX_LPSPI1 */

#ifdef CONFIG_S32K3XX_LPSPI2
/* LPSPI2 *******************************************************************/

void s32k3xx_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid,
			  bool selected)
{
	spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
		selected ? "assert" : "de-assert");

	s32k3xx_gpiowrite(PIN_LPSPI2_PCS, !selected);
}

uint8_t s32k3xx_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_S32K3XX_LPSPI2 */

#ifdef CONFIG_S32K3XX_LPSPI3
/* LPSPI3 *******************************************************************/

void s32k3xx_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid,
			  bool selected)
{
	spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
		selected ? "assert" : "de-assert");

	s32k3xx_gpiowrite(PIN_LPSPI3_PCS, !selected);
}

uint8_t s32k3xx_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_S32K3XX_LPSPI3 */

#ifdef CONFIG_S32K3XX_LPSPI4
/* LPSPI4 *******************************************************************/

void s32k3xx_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid,
			  bool selected)
{
	spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
		selected ? "assert" : "de-assert");

	devid = ((devid) & 0xF);

	if (devid == 0) {
		s32k3xx_gpiowrite(PIN_LPSPI4_CS_P26, !selected);

	} else if (devid == 1) {
		s32k3xx_gpiowrite(PIN_LPSPI4_CS_P8B, !selected);
	}
}

uint8_t s32k3xx_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_S32K3XX_LPSPI4 */

#ifdef CONFIG_S32K3XX_LPSPI5
/* LPSPI5 *******************************************************************/

void s32k3xx_lpspi5select(FAR struct spi_dev_s *dev, uint32_t devid,
			  bool selected)
{
	spiinfo("devid: %" PRId32 ", CS: %s\n", devid,
		selected ? "assert" : "de-assert");

	s32k3xx_gpiowrite(PIN_LPSPI5_PCS, !selected);
}

uint8_t s32k3xx_lpspi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif /* CONFIG_S32K3XX_LPSPI5 */


#endif /* CONFIG_S32K3XX_LPSPI0 */
