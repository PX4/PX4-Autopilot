/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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

/**
 * @file spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *   mask - is bus selection
 *   1 - 1 << 0
 *   2 - 1 << 1
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize()
{
	stm32_configgpio(GPIO_SPI_CS_MEMS);
	stm32_configgpio(GPIO_SPI_CS_SDCARD);
	stm32_configgpio(GPIO_SPI3_CS_BARO);
	stm32_configgpio(GPIO_SPI3_CS_OSD);
}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	UNUSED(devid);
	px4_arch_gpiowrite(GPIO_SPI_CS_MEMS, !selected);
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	UNUSED(devid);
	px4_arch_gpiowrite(GPIO_SPI_CS_SDCARD, !selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	UNUSED(devid);
	/* SPI select is active low, so write !selected to select the device */
	px4_arch_gpiowrite(GPIO_SPI3_CS_BARO, !selected);
	px4_arch_gpiowrite(GPIO_SPI3_CS_OSD, !selected);
}

__EXPORT uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}

__EXPORT void board_spi_reset(int ms)
{
	// TODO: DRDY
	///* disable SPI bus 1  DRDY */
	//stm32_configgpio(GPIO_DRDY_OFF_PORTD_PIN15);
	//stm32_configgpio(GPIO_DRDY_OFF_PORTC_PIN14);
	//stm32_configgpio(GPIO_DRDY_OFF_PORTE_PIN12);

	//stm32_gpiowrite(GPIO_DRDY_OFF_PORTD_PIN15, 0);
	//stm32_gpiowrite(GPIO_DRDY_OFF_PORTC_PIN14, 0);
	//stm32_gpiowrite(GPIO_DRDY_OFF_PORTE_PIN12, 0);

	/* disable SPI bus 1  CS */
	stm32_configgpio(GPIO_SPI1_CS_MEMS_OFF);
	stm32_gpiowrite(GPIO_SPI1_CS_MEMS_OFF, 0);

	/* disable SPI bus 2  CS */
	stm32_configgpio(GPIO_SPI2_CS_SDCARD_OFF);
	stm32_gpiowrite(GPIO_SPI2_CS_SDCARD_OFF, 0);

	/* disable SPI bus 3  CS */
	stm32_configgpio(GPIO_SPI3_CS_BARO_OFF);
	stm32_gpiowrite(GPIO_SPI3_CS_BARO_OFF, 0);
	stm32_configgpio(GPIO_SPI3_CS_OSD_OFF);
	stm32_gpiowrite(GPIO_SPI3_CS_OSD_OFF, 0);

	/* disable SPI bus 1*/
	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);

	/* disable SPI bus 2*/
	stm32_configgpio(GPIO_SPI2_SCK_OFF);
	stm32_configgpio(GPIO_SPI2_MISO_OFF);
	stm32_configgpio(GPIO_SPI2_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI2_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI2_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI2_MOSI_OFF, 0);

	/* disable SPI bus 3*/
	stm32_configgpio(GPIO_SPI3_SCK_OFF);
	stm32_configgpio(GPIO_SPI3_MISO_OFF);
	stm32_configgpio(GPIO_SPI3_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI3_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI3_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI3_MOSI_OFF, 0);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	stm32_spiinitialize();

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	// TODO: why do we not enable SPI2 here?

	stm32_configgpio(GPIO_SPI3_SCK);
	stm32_configgpio(GPIO_SPI3_MISO);
	stm32_configgpio(GPIO_SPI3_MOSI);
}
