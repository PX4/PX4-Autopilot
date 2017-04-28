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
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"
#include <systemlib/err.h>

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

__EXPORT void stm32_spiinitialize(int mask)
{
#ifdef CONFIG_STM32_SPI1

	if (mask & PX4_SPI_BUS_SENSORS) {
		stm32_configgpio(GPIO_SPI1_CS_PORTC_PIN2);
		stm32_configgpio(GPIO_SPI1_CS_PORTC_PIN15);
		stm32_configgpio(GPIO_SPI1_CS_PORTE_PIN15);

		stm32_configgpio(GPIO_DRDY_PORTD_PIN15);
		stm32_configgpio(GPIO_DRDY_PORTC_PIN14);
		stm32_configgpio(GPIO_DRDY_PORTE_PIN12);
	}

#endif

#ifdef CONFIG_STM32_SPI2

	if (mask & (PX4_SPI_BUS_RAMTRON | PX4_SPI_BUS_BARO)) {
		stm32_configgpio(GPIO_SPI2_CS_MS5611);
		stm32_configgpio(GPIO_SPI2_CS_FRAM);
	}

#endif

}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {

	/* Shared PC2 CS devices */

	case PX4_SPIDEV_BMI:
	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2,  !selected);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);
		break;

	/* Shared PC15 CS devices */

	case PX4_SPIDEV_ICM:
	case PX4_SPIDEV_ICM_20602:
	case PX4_SPIDEV_ICM_20608:
	case PX4_SPIDEV_BMI055_ACC:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, !selected);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);
		break;

	/* Shared PE15 CS devices */

	case PX4_SPIDEV_HMC:
	case PX4_SPIDEV_BMI055_GYR:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, !selected);
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
		stm32_gpiowrite(GPIO_SPI2_CS_MS5611, 1);
		stm32_gpiowrite(GPIO_SPI2_CS_FRAM, !selected);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI2_CS_FRAM, 1);
		stm32_gpiowrite(GPIO_SPI2_CS_MS5611, !selected);
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

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus 1  DRDY */

	stm32_configgpio(GPIO_DRDY_OFF_PORTD_PIN15);
	stm32_configgpio(GPIO_DRDY_OFF_PORTC_PIN14);
	stm32_configgpio(GPIO_DRDY_OFF_PORTE_PIN12);

	stm32_gpiowrite(GPIO_DRDY_OFF_PORTD_PIN15, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_PORTC_PIN14, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_PORTE_PIN12, 0);

	/* disable SPI bus 1  CS */

	stm32_configgpio(GPIO_SPI1_CS_OFF_PORTC_PIN2);
	stm32_configgpio(GPIO_SPI1_CS_OFF_PORTC_PIN15);
	stm32_configgpio(GPIO_SPI1_CS_OFF_PORTE_PIN15);

	stm32_gpiowrite(GPIO_SPI1_CS_OFF_PORTC_PIN2, 0);
	stm32_gpiowrite(GPIO_SPI1_CS_OFF_PORTC_PIN15, 0);
	stm32_gpiowrite(GPIO_SPI1_CS_OFF_PORTE_PIN15, 0);

	/* disable SPI bus 1*/

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);


	/* N.B we do not have control over the SPI 2 buss powered devices
	 * so the the ms5611 is not resetable.
	 */


	/* set the sensor rail off (default) */
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	stm32_spiinitialize(PX4_SPI_BUS_SENSORS);
	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);


}
