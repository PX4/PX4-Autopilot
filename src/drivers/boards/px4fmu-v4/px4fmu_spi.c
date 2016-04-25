/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4fmu_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
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
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI1
	stm32_configgpio(GPIO_SPI_CS_MPU9250);
	stm32_configgpio(GPIO_SPI_CS_HMC5983);
	stm32_configgpio(GPIO_SPI_CS_MS5611);
	stm32_configgpio(GPIO_SPI_CS_ICM_20608_G);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
	stm32_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
	stm32_gpiowrite(GPIO_SPI_CS_MS5611, 1);
	stm32_gpiowrite(GPIO_SPI_CS_ICM_20608_G, 1);

	stm32_configgpio(GPIO_DRDY_MPU9250);
	stm32_configgpio(GPIO_DRDY_HMC5983);
	stm32_configgpio(GPIO_DRDY_ICM_20608_G);
#endif

#ifdef CONFIG_STM32_SPI2
	stm32_configgpio(GPIO_SPI_CS_FRAM);
	stm32_gpiowrite(GPIO_SPI_CS_FRAM, 1);
#endif

}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_ICM:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ICM_20608_G, !selected);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MS5611, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_ICM_20608_G, 1);
		break;

	case PX4_SPIDEV_HMC:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		stm32_gpiowrite(GPIO_SPI_CS_HMC5983, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ICM_20608_G, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_MPU9250, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ICM_20608_G, 1);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}


#ifdef CONFIG_STM32_SPI2
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case SPIDEV_FLASH:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		stm32_gpiowrite(GPIO_SPI_CS_FRAM, !selected);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_FRAM, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MS5611, !selected);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
#endif
