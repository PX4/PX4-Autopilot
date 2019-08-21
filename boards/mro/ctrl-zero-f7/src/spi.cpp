/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <board_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32_gpio.h>
#include "board_config.h"

/* Define CS GPIO array */
static constexpr uint32_t spi1selects_gpio[] = PX4_SPI_BUS_1_CS_GPIO;
static constexpr uint32_t spi2selects_gpio[] = PX4_SPI_BUS_2_CS_GPIO;
static constexpr uint32_t spi5selects_gpio[] = PX4_SPI_BUS_5_CS_GPIO;

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize()
{
#ifdef CONFIG_STM32F7_SPI1

	for (auto gpio : spi1selects_gpio) {
		px4_arch_configgpio(gpio);
	}

#endif // CONFIG_STM32F7_SPI1


#if defined(CONFIG_STM32F7_SPI2)

	for (auto gpio : spi2selects_gpio) {
		px4_arch_configgpio(gpio);
	}

#endif // CONFIG_STM32F7_SPI2

#ifdef CONFIG_STM32F7_SPI5

	for (auto gpio : spi5selects_gpio) {
		px4_arch_configgpio(gpio);
	}

#endif // CONFIG_STM32F7_SPI5
}

/************************************************************************************
 * Name: stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   Called by stm32 spi driver on bus 1.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32F7_SPI1
__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	ASSERT(PX4_SPI_BUS_ID(devid) == PX4_SPI_BUS_1);

	// Making sure the other peripherals are not selected
	for (auto cs : spi1selects_gpio) {
		stm32_gpiowrite(cs, 1);
	}

	// SPI select is active low, so write !selected to select the device
	stm32_gpiowrite(spi1selects_gpio[PX4_SPI_DEV_ID(devid)], !selected);
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI1

/************************************************************************************
 * Name: stm32_spi2select and stm32_spi2status
 *
 * Description:
 *   Called by stm32 spi driver on bus 2.
 *
 ************************************************************************************/
#if defined(CONFIG_STM32F7_SPI2)
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	if (devid == SPIDEV_FLASH(0)) {
		devid = PX4_SPIDEV_MEMORY;
	}

	ASSERT(PX4_SPI_BUS_ID(devid) == PX4_SPI_BUS_2);

	// Making sure the other peripherals are not selected
	for (auto cs : spi2selects_gpio) {
		stm32_gpiowrite(cs, 1);
	}

	// SPI select is active low, so write !selected to select the device
	stm32_gpiowrite(spi2selects_gpio[PX4_SPI_DEV_ID(devid)], !selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI2 && GPIO_SPI2_CS_FRAM

/************************************************************************************
 * Name: stm32_spi5select and stm32_spi5status
 *
 * Description:
 *   Called by stm32 spi driver on bus 5.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32F7_SPI5
__EXPORT void stm32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	ASSERT(PX4_SPI_BUS_ID(devid) == PX4_SPI_BUS_5);

	// Making sure the other peripherals are not selected
	for (auto cs : spi5selects_gpio) {
		stm32_gpiowrite(cs, 1);
	}

	// SPI select is active low, so write !selected to select the device
	stm32_gpiowrite(spi5selects_gpio[PX4_SPI_DEV_ID(devid)], !selected);
}

__EXPORT uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI5

/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *
 *
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms)
{
	// disable SPI bus

	// SPI1
	for (auto cs : spi1selects_gpio) {
		stm32_configgpio(_PIN_OFF(cs));
	}

	stm32_configgpio(_PIN_OFF(GPIO_SPI1_SCK));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MISO));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MOSI));

	// SPI5
	for (auto cs : spi5selects_gpio) {
		stm32_configgpio(_PIN_OFF(cs));
	}

	stm32_configgpio(_PIN_OFF(GPIO_SPI5_SCK));
	stm32_configgpio(_PIN_OFF(GPIO_SPI5_MISO));
	stm32_configgpio(_PIN_OFF(GPIO_SPI5_MOSI));
	stm32_configgpio(_PIN_OFF(GPIO_DRDY_BMI088_INT1_ACCEL));
	stm32_configgpio(_PIN_OFF(GPIO_DRDY_BMI088_INT2_ACCEL));
	stm32_configgpio(_PIN_OFF(GPIO_DRDY_BMI088_INT3_GYRO));
	stm32_configgpio(_PIN_OFF(GPIO_DRDY_BMI088_INT4_GYRO));

	/* set the sensor rail off */
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */

	// SPI1
	for (auto cs : spi1selects_gpio) {
		stm32_configgpio(cs);
	}

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	// SPI5
	for (auto cs : spi5selects_gpio) {
		stm32_configgpio(cs);
	}

	stm32_configgpio(GPIO_SPI5_SCK);
	stm32_configgpio(GPIO_SPI5_MISO);
	stm32_configgpio(GPIO_SPI5_MOSI);
	stm32_configgpio(GPIO_DRDY_BMI088_INT1_ACCEL);
	stm32_configgpio(GPIO_DRDY_BMI088_INT2_ACCEL);
	stm32_configgpio(GPIO_DRDY_BMI088_INT3_GYRO);
	stm32_configgpio(GPIO_DRDY_BMI088_INT4_GYRO);
}
