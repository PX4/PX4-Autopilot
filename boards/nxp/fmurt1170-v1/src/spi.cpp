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

#ifdef CONFIG_IMXRT_LPSPI


constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::LPSPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::Port2, GPIO::Pin11}, SPI::DRDY{GPIO::Port3, GPIO::Pin19}), /* GPIO_EMC_B2_01 GPIO2_IO11, GPIO_AD_20, GPIO3_IO19 */
	}, {GPIO::Port2, GPIO::Pin1}), // Power GPIO_EMC_B1_33  GPIO2_IO01

	initSPIBus(SPI::Bus::LPSPI2, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::Port3, GPIO::Pin24}, SPI::DRDY{GPIO::Port2, GPIO::Pin7}), /* GPIO_AD_25 GPIO3_IO24, GPIO_EMC_B1_39 GPIO2_IO07 */
	}, {GPIO::Port1, GPIO::Pin22}), // Power GPIO_EMC_B1_22  GPIO1_IO22

	initSPIBus(SPI::Bus::LPSPI3, {
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::Port2, GPIO::Pin18}, SPI::DRDY{GPIO::Port2, GPIO::Pin28}), /* GPIO_EMC_B2_08 GPIO2_IO18, GPIO_EMC_B2_18 GPIO2_IO28 */
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::Port2, GPIO::Pin15}),                                      /* GPIO_EMC_B2_05 GPIO2_IO15 */
	}, {GPIO::Port1, GPIO::Pin14}), // Power GPIO_EMC_B1_14  GPIO1_IO14

	initSPIBusExternal(SPI::Bus::LPSPI6, {
		initSPIConfigExternal(SPI::CS{GPIO::Port6, GPIO::Pin9}, SPI::DRDY{GPIO::Port1, GPIO::Pin5}), /* GPIO_LPSR_09 GPIO6_IO09 GPIO_EMC_B1_05 GPIO1_IO05*/
		initSPIConfigExternal(SPI::CS{GPIO::Port6, GPIO::Pin8}, SPI::DRDY{GPIO::Port1, GPIO::Pin7}), /* GPIO_LPSR_08 GPIO6_IO08  GPIO_EMC_B1_07  GPIO1_IO07*/
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

//#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_PULL_DOWN_100K | IOMUX_CMOS_INPUT))

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/************************************************************************************
 * Name: fmurt1170_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUKRT1170-V1 board.
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
 *   Called to configure SPI chip select GPIO pins for the NXP FMUKRT1170-V1 board.
 *
 ************************************************************************************/

static const px4_spi_bus_t *_spi_bus1;
static const px4_spi_bus_t *_spi_bus2;
static const px4_spi_bus_t *_spi_bus3;
static const px4_spi_bus_t *_spi_bus6;

__EXPORT int imxrt1176_spi_bus_initialize(void)
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 1: _spi_bus1 = &px4_spi_buses[i]; break;

		case 6: _spi_bus6 = &px4_spi_buses[i]; break;
		}
	}

	/* Configure SPI-based devices */

	struct spi_dev_s *spi_sensors = px4_spibus_initialize(1);

	if (!spi_sensors) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", 1);
		return -ENODEV;
	}

	/* Default bus 1 to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_sensors, 1 * 1000 * 1000);
	SPI_SETBITS(spi_sensors, 8);
	SPI_SETMODE(spi_sensors, SPIDEV_MODE3);

	/* Get the SPI port for the Memory */

	struct spi_dev_s *spi_memory = px4_spibus_initialize(6);

	if (!spi_memory) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", 2);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_MEMORY to 12MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_memory, 12 * 1000 * 1000);
	SPI_SETBITS(spi_memory, 8);
	SPI_SETMODE(spi_memory, SPIDEV_MODE3);


	/* deselect all */
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				//SPI_SELECT(spi_ext, px4_spi_buses[bus].devices[i].devid, false);
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

#if defined(CONFIG_IMXRT_LPSPI1)
__EXPORT void imxrt_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imxrt_spixselect(_spi_bus1, dev, devid, selected);
}

__EXPORT uint8_t imxrt_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#if defined(CONFIG_IMXRT_LPSPI2)
__EXPORT void imxrt_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imxrt_spixselect(_spi_bus2, dev, devid, selected);
}

__EXPORT uint8_t imxrt_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

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

#if defined(CONFIG_IMXRT_LPSPI5)
__EXPORT void imxrt_lpspi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imxrt_spixselect(_spi_bus5, dev, devid, selected);
}

__EXPORT uint8_t imxrt_lpspi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#if defined(CONFIG_IMXRT_LPSPI6)
__EXPORT void imxrt_lpspi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	imxrt_spixselect(_spi_bus6, dev, devid, selected);
}

__EXPORT uint8_t imxrt_lpspi6status(FAR struct spi_dev_s *dev, uint32_t devid)
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


	//imxrt_config_gpio(_PIN_OFF(GPIO_LPI2C3_SDA_RESET));
	//imxrt_config_gpio(_PIN_OFF(GPIO_LPI2C3_SCL_RESET));

	/* set the sensor rail off */
	//imxrt_gpio_write(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	//imxrt_gpio_write(GPIO_VDD_3V3_SENSORS_EN, 1);

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

	//imxrt_config_gpio(GPIO_LPSPI3_SCK);
	//imxrt_config_gpio(GPIO_LPSPI3_MISO);
	//imxrt_config_gpio(GPIO_LPSPI3_MOSI);

	//imxrt_config_gpio(GPIO_LPI2C3_SDA);
	//imxrt_config_gpio(GPIO_LPI2C3_SCL);

#endif /* CONFIG_IMXRT_LPSPI */

}

#endif /* CONFIG_IMXRT_LPSPI  */
