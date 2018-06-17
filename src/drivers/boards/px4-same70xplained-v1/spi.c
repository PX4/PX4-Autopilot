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
 * @file px4same70xplained_spi.c
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

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <sam_spi.h>
#include <sam_gpio.h>
#include "board_config.h"
/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: boad_spi_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4_SAME70XPLAINED_V1 board.
 *
 ************************************************************************************/

__EXPORT void board_spi_initialize(void)
{
#ifdef CONFIG_SAMV7_SPI0_MASTER
	sam_configgpio(GPIO_SPI_CS_GYRO);
	sam_configgpio(GPIO_SPI_CS_ACCEL_MAG);
	sam_configgpio(GPIO_SPI_CS_BARO);
	sam_configgpio(GPIO_SPI_CS_MPU);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	sam_gpiowrite(GPIO_SPI_CS_GYRO, 1);
	sam_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
	sam_gpiowrite(GPIO_SPI_CS_BARO, 1);
	sam_gpiowrite(GPIO_SPI_CS_MPU, 1);

	sam_configgpio(GPIO_EXTI_GYRO_DRDY);
	sam_configgpio(GPIO_EXTI_MAG_DRDY);
	sam_configgpio(GPIO_EXTI_ACCEL_DRDY);
	sam_configgpio(GPIO_EXTI_MPU_DRDY);
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
	sam_configgpio(GPIO_SPI_CS_EXT0);
	sam_configgpio(GPIO_SPI_CS_EXT1);
	sam_configgpio(GPIO_SPI_CS_EXT2);
	sam_configgpio(GPIO_SPI_CS_EXT3);
	sam_gpiowrite(GPIO_SPI_CS_EXT0, 1);
	sam_gpiowrite(GPIO_SPI_CS_EXT1, 1);
	sam_gpiowrite(GPIO_SPI_CS_EXT2, 1);
	sam_gpiowrite(GPIO_SPI_CS_EXT3, 1);
#endif
}

/****************************************************************************
 * Name: sam_spi[0|1]select
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   PIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the PIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SPI0_MASTER
void sam_spi0select(uint32_t devid, bool selected)
{
	switch (devid) {
	case PX4_SPIDEV_GYRO:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_GYRO, !selected);
		sam_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		sam_gpiowrite(GPIO_SPI_CS_BARO, 1);
		sam_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		sam_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, !selected);
		sam_gpiowrite(GPIO_SPI_CS_BARO, 1);
		sam_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		sam_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		sam_gpiowrite(GPIO_SPI_CS_BARO, !selected);
		sam_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		sam_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		sam_gpiowrite(GPIO_SPI_CS_BARO, 1);
		sam_gpiowrite(GPIO_SPI_CS_MPU, !selected);
		break;

	default:
		break;
	}
}
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
void sam_spi1select(uint32_t devid, bool selected)
{
	switch (devid) {
	case PX4_SPIDEV_EXT0:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_EXT0, !selected);
		sam_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT2, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT3, 1);
		break;

	case PX4_SPIDEV_EXT1:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT1, !selected);
		sam_gpiowrite(GPIO_SPI_CS_EXT2, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT3, 1);
		break;

	case PX4_SPIDEV_EXT2:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT2, !selected);
		sam_gpiowrite(GPIO_SPI_CS_EXT3, 1);
		break;

	case PX4_SPIDEV_EXT3:
		/* Making sure the other peripherals are not selected */
		sam_gpiowrite(GPIO_SPI_CS_EXT0, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT1, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT2, 1);
		sam_gpiowrite(GPIO_SPI_CS_EXT3, !selected);
		break;

	default:
		break;

	}
}
#endif

/****************************************************************************
 * Name: sam_spi[0|1]status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   Bit-encoded SPI status (see include/nuttx/spi/spi.h.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_SPI0_MASTER
uint8_t sam_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_SAMV7_SPI1_MASTER
uint8_t sam_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif
