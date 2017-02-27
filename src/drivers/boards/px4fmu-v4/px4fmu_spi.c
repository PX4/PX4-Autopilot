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
 *
 ************************************************************************************/

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI1
	px4_arch_configgpio(GPIO_SPI_CS_MPU9250);
	px4_arch_configgpio(GPIO_SPI_CS_HMC5983);
	px4_arch_configgpio(GPIO_SPI_CS_MS5611);
	px4_arch_configgpio(GPIO_SPI_CS_ICM_2060X);
	px4_arch_configgpio(GPIO_SPI1_CS_PORTC_PIN2);   //BMI160
	px4_arch_configgpio(GPIO_SPI1_CS_PORTC_PIN15);  //BMI055 ACC
	px4_arch_configgpio(GPIO_SPI1_CS_PORTE_PIN15);  //BMI055 GYRO

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
	px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
	px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);
	px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);
	px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);

	px4_arch_configgpio(GPIO_DRDY_MPU9250);
	px4_arch_configgpio(GPIO_DRDY_HMC5983);
	px4_arch_configgpio(GPIO_DRDY_ICM_2060X);
#endif

#ifdef CONFIG_STM32_SPI2
	stm32_configgpio(GPIO_SPI_CS_FRAM);
#endif

}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_ICM:

	/* intended fallthrough */
	case PX4_SPIDEV_ICM_20602:

	/* intended fallthrough */
	case PX4_SPIDEV_ICM_20608:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, !selected);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);     //BMI160
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);    //BMI055 ACC
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);    //BMI055 GYRO
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
		break;

	case PX4_SPIDEV_HMC:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);     //BMI160
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);    //BMI055 ACC
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);    //BMI055 GYRO
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);     //BMI160
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);    //BMI055 ACC
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);    //BMI055 GYRO
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, !selected);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
		break;

	case PX4_SPIDEV_BMI:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);    //BMI055 ACC
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);    //BMI055 GYRO
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, !selected); //BMI160
		break;

	case PX4_SPIDEV_BMI055_ACC:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);     //BMI160
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, 1);    //BMI055 GYRO
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, !selected); //BMI055 ACC
		break;

	case PX4_SPIDEV_BMI055_GYR:
		/* Making sure the other peripherals are not selected */
		px4_arch_gpiowrite(GPIO_SPI_CS_MPU9250, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_HMC5983, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_MS5611, 1);
		px4_arch_gpiowrite(GPIO_SPI_CS_ICM_2060X, 1);
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN2, 1);     //BMI160
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTC_PIN15, 1);    //BMI055 ACC
		px4_arch_gpiowrite(GPIO_SPI1_CS_PORTE_PIN15, !selected);    //BMI055 GYRO
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

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	px4_arch_configgpio(GPIO_SPI_CS_OFF_MPU9250);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_HMC5983);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_MS5611);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_ICM_2060X);
	px4_arch_configgpio(GPIO_SPI_CS_OFF_BMI160);   // BMI160
	px4_arch_configgpio(GPIO_SPI_CS_OFF_BMI055_ACC);  // BMI055 ACC
	px4_arch_configgpio(GPIO_SPI_CS_OFF_BMI055_GYR);  // BMI055 GYRO

	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_MPU9250, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_HMC5983, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_MS5611, 0);
	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_ICM_2060X, 0);
    px4_arch_gpiowrite(GPIO_SPI_CS_OFF_BMI160, 0);     // BMI160
    px4_arch_gpiowrite(GPIO_SPI_CS_OFF_BMI055_ACC, 0);    // BMI055 ACC
    px4_arch_gpiowrite(GPIO_SPI_CS_OFF_BMI055_GYR, 0);    // BMI055 GYRO

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);

	stm32_configgpio(GPIO_DRDY_OFF_MPU9250);
	stm32_configgpio(GPIO_DRDY_OFF_HMC5983);
	stm32_configgpio(GPIO_DRDY_OFF_ICM_2060X);

	stm32_gpiowrite(GPIO_DRDY_OFF_MPU9250, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_HMC5983, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_ICM_2060X, 0);

	/* set the sensor rail off */
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
#ifdef CONFIG_STM32_SPI1
	px4_arch_configgpio(GPIO_SPI_CS_MPU9250);
	px4_arch_configgpio(GPIO_SPI_CS_HMC5983);
	px4_arch_configgpio(GPIO_SPI_CS_MS5611);
	px4_arch_configgpio(GPIO_SPI_CS_ICM_2060X);
    px4_arch_configgpio(GPIO_SPI_CS_OFF_BMI160);		// BMI160
    px4_arch_configgpio(GPIO_SPI_CS_OFF_BMI055_ACC);	// BMI055 ACC
    px4_arch_configgpio(GPIO_SPI_CS_OFF_BMI055_GYR);	// BMI055 GYRO

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	// // XXX bring up the EXTI pins again
	// stm32_configgpio(GPIO_GYRO_DRDY);
	// stm32_configgpio(GPIO_MAG_DRDY);
	// stm32_configgpio(GPIO_ACCEL_DRDY);
	// stm32_configgpio(GPIO_EXTI_MPU_DRDY);

#endif

}
