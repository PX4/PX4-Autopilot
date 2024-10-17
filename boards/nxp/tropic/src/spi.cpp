/************************************************************************************
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

#include <px4_platform_common/px4_config.h>
#include <px4_log.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>
#include <px4_platform/gpio.h>

#include <arm_internal.h>
#include <chip.h>
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "board_config.h"
#include <systemlib/err.h>

#if defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)


constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::LPSPI3, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::Port1, GPIO::Pin28}), /* GPIO_AD_B1_12 GPIO1_IO28 */
	}),
	initSPIBus(SPI::Bus::LPSPI4, {
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::Port2, GPIO::Pin18}),  /* GPIO_B1_02 GPIO2_IO18 */
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::Port2, GPIO::Pin0}), /* GPIO_B0_00 GPIO2_IO00 */
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_PULL_DOWN_100K | IOMUX_CMOS_INPUT))

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/************************************************************************************
 * Name: fmurt1062_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUKRT1062-V1 board.
 *
 ************************************************************************************/

void imxrt_spidev_initialize(void)
{
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
			}
		}
	}
}

/************************************************************************************
 * Name: imxrt_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUKRT1062-V1 board.
 *
 ************************************************************************************/

static const px4_spi_bus_t *_spi_bus3;
static const px4_spi_bus_t *_spi_bus4;

__EXPORT int imxrt1062_spi_bus_initialize(void)
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 3: _spi_bus3 = &px4_spi_buses[i]; break;

		case 4: _spi_bus4 = &px4_spi_buses[i]; break;
		}
	}

	/* Configure SPI-based devices */

	struct spi_dev_s *spi_icm = px4_spibus_initialize(3);

	if (!spi_icm) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", 1);
		return -ENODEV;
	}

	/* Default bus 1 to 8MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_icm, 8 * 1000 * 1000);
	SPI_SETBITS(spi_icm, 8);
	SPI_SETMODE(spi_icm, SPIDEV_MODE3);

	/* Get the SPI port for the BMI088 */

	struct spi_dev_s *spi_bmi = px4_spibus_initialize(4);

	if (!spi_bmi) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", 4);
		return -ENODEV;
	}

	/* Default ext bus to 8MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_bmi, 8 * 1000 * 1000);
	SPI_SETBITS(spi_bmi, 8);
	SPI_SETMODE(spi_bmi, SPIDEV_MODE3);

	/* deselect all */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				SPI_SELECT(spi_bmi, px4_spi_buses[bus].devices[i].devid, false);
			}
		}
	}

	return OK;

}

/****************************************************************************
 * Name:  imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *
 * Description:
 *   The external functions, imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including imxrt_lpspibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imxrt_lpspi1/2/3select() and imxrt_lpspi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

static inline void imxrt_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			// SPI select is active low, so write !selected to select the device
			imxrt_gpio_write(bus->devices[i].cs_gpio, !selected);
		}
	}
}

#if defined(CONFIG_IMXRT_LPSPI3)
__EXPORT void imxrt_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imxrt_spixselect(_spi_bus3, dev, devid, selected);
}

__EXPORT uint8_t imxrt_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#if defined(CONFIG_IMXRT_LPSPI4)
__EXPORT void imxrt_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imxrt_spixselect(_spi_bus4, dev, devid, selected);
}

__EXPORT uint8_t imxrt_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *
 *
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms, int bus_mask)
{
#ifdef CONFIG_IMXRT_LPSPI1

	/* Goal not to back feed the chips on the bus via IO lines */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == 1 || px4_spi_buses[bus].bus == 3) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					imxrt_config_gpio(_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
				}

				if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
					imxrt_config_gpio(_PIN_OFF(px4_spi_buses[bus].devices[i].drdy_gpio));
				}
			}
		}
	}

	imxrt_config_gpio(GPIO_SPI1_SCK_OFF);
	imxrt_config_gpio(GPIO_SPI1_MISO_OFF);
	imxrt_config_gpio(GPIO_SPI1_MOSI_OFF);

	imxrt_config_gpio(GPIO_SPI3_SCK_OFF);
	imxrt_config_gpio(GPIO_SPI3_MISO_OFF);
	imxrt_config_gpio(GPIO_SPI3_MOSI_OFF);


	imxrt_config_gpio(_PIN_OFF(GPIO_LPI2C3_SDA_RESET));
	imxrt_config_gpio(_PIN_OFF(GPIO_LPI2C3_SCL_RESET));

	/* set the sensor rail off */
	imxrt_gpio_write(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	imxrt_gpio_write(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == 1 || px4_spi_buses[bus].bus == 3) {
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
				if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
					imxrt_config_gpio(px4_spi_buses[bus].devices[i].cs_gpio);
				}

				if (px4_spi_buses[bus].devices[i].drdy_gpio != 0) {
					imxrt_config_gpio(px4_spi_buses[bus].devices[i].drdy_gpio);
				}
			}
		}
	}

	imxrt_config_gpio(GPIO_LPSPI1_SCK);
	imxrt_config_gpio(GPIO_LPSPI1_MISO);
	imxrt_config_gpio(GPIO_LPSPI1_MOSI);

	imxrt_config_gpio(GPIO_LPSPI3_SCK);
	imxrt_config_gpio(GPIO_LPSPI3_MISO);
	imxrt_config_gpio(GPIO_LPSPI3_MOSI);

	imxrt_config_gpio(GPIO_LPI2C3_SDA);
	imxrt_config_gpio(GPIO_LPI2C3_SCL);

#endif /* CONFIG_IMXRT_LPSPI1 */

}

#endif /* CONFIG_IMXRT_LPSPI3 || CONFIG_IMXRT_LPSPI4  */
