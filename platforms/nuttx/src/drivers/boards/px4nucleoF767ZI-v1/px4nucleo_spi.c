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
 * @file px4nucleo_spi.c
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
#include <systemlib/px4_macros.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32_gpio.h>
#include "board_config.h"
#include <systemlib/err.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message syslog
#  else
#    define message printf
#  endif
#endif

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
#ifdef CONFIG_STM32F7_SPI1
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

#ifdef CONFIG_STM32F7_SPI2
	stm32_configgpio(GPIO_SPI_CS_FRAM);
	stm32_gpiowrite(GPIO_SPI_CS_FRAM, 1);
#endif

}

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_sensors;
static struct spi_dev_s *spi_fram;
static struct spi_dev_s *spi_baro;
static struct spi_dev_s *spi_icm;

__EXPORT int stm32_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */

	spi_sensors = stm32_spibus_initialize(PX4_SPI_BUS_SENSORS);

	if (!spi_sensors) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSORS);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi_sensors, 10000000);
	SPI_SETBITS(spi_sensors, 8);
	SPI_SETMODE(spi_sensors, SPIDEV_MODE3);

	for (int cs = PX4_SENSORS_BUS_FIRST_CS; cs <= PX4_SENSORS_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensors, cs, false);
	}

	/* Get the SPI port for the FRAM */

	spi_fram = stm32_spibus_initialize(PX4_SPI_BUS_RAMTRON);

	if (!spi_fram) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_RAMTRON);
		return -ENODEV;
	}

	/* Default SPI_BUS_RAMTRON to 12MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_fram, 12 * 1000 * 1000);
	SPI_SETBITS(spi_fram, 8);
	SPI_SETMODE(spi_fram, SPIDEV_MODE3);

	for (int cs = PX4_RAMTRON_BUS_FIRST_CS; cs <= PX4_RAMTRON_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_fram, cs, false);
	}

	/* Get the SPI port for the BARO */

	spi_baro = stm32_spibus_initialize(PX4_SPI_BUS_BARO);

	if (!spi_baro) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_BARO);
		return -ENODEV;
	}

	/* MS5611 has max SPI clock speed of 20MHz
	 */

	SPI_SETFREQUENCY(spi_baro, 20 * 1000 * 1000);
	SPI_SETBITS(spi_baro, 8);
	SPI_SETMODE(spi_baro, SPIDEV_MODE3);

	for (int cs = PX4_BARO_BUS_FIRST_CS; cs <= PX4_BARO_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_baro, cs, false);
	}

	/* Get the SPI port for the PX4_SPI_BUS_ICM */

	spi_icm = stm32_spibus_initialize(PX4_SPI_BUS_ICM);

	if (!spi_icm) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_ICM);
		return -ENODEV;
	}

	/* ICM 20608 G has max SPI clock speed of 8MHz
	 */

	SPI_SETFREQUENCY(spi_icm, 8 * 1000 * 1000);
	SPI_SETBITS(spi_icm, 8);
	SPI_SETMODE(spi_icm, SPIDEV_MODE3);

	for (int cs = PX4_ICM_BUS_FIRST_CS; cs <= PX4_ICM_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_icm, cs, false);
	}

	return OK;

}

/* Define CS GPIO array */

static const uint32_t spi1selects_gpio[] = PX4_SENSOR_BUS_CS_GPIO;

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSORS);

	/* Making sure the other peripherals are not selected */

	for (int cs = 0;  arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_gpiowrite(spi1selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi1selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}


/* Define CS GPIO array */

static const uint32_t spi4selects_gpio[] = PX4_RAMTRON_BUS_CS_GPIO;

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	int sel = (int) devid;

	if (devid == SPIDEV_FLASH) {
		sel = PX4_SPIDEV_FRAM;
	}

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_RAMTRON);

	/* Making sure the other peripherals are not selected */
	for (int cs = 0; arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		stm32_gpiowrite(spi4selects_gpio[cs], 1);
	}

	uint32_t gpio = spi4selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
static const uint32_t spi5selects_gpio[] = PX4_BARO_BUS_CS_GPIO;

__EXPORT void stm32_spi5select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_BARO);

	/* Making sure the other peripherals are not selected */
	for (int cs = 0; arraySize(spi5selects_gpio) > 1 && cs < arraySize(spi5selects_gpio); cs++) {
		stm32_gpiowrite(spi5selects_gpio[cs], 1);
	}

	uint32_t gpio = spi5selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}

static const uint32_t spi6selects_gpio[] = PX4_ICM_BUS_CS_GPIO;

__EXPORT void stm32_spi6select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_ICM);

	/* Making sure the other peripherals are not selected */
	for (int cs = 0; arraySize(spi6selects_gpio) > 1 && cs < arraySize(spi6selects_gpio); cs++) {
		stm32_gpiowrite(spi6selects_gpio[cs], 1);
	}

	uint32_t gpio = spi6selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi6status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	stm32_configgpio(GPIO_SPI_CS_OFF_MPU9250);
	stm32_configgpio(GPIO_SPI_CS_OFF_HMC5983);
	stm32_configgpio(GPIO_SPI_CS_OFF_MS5611);
	stm32_configgpio(GPIO_SPI_CS_OFF_ICM_20608_G);

	stm32_gpiowrite(GPIO_SPI_CS_OFF_MPU9250, 0);
	stm32_gpiowrite(GPIO_SPI_CS_OFF_HMC5983, 0);
	stm32_gpiowrite(GPIO_SPI_CS_OFF_MS5611, 0);
	stm32_gpiowrite(GPIO_SPI_CS_OFF_ICM_20608_G, 0);

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);

	stm32_configgpio(GPIO_DRDY_OFF_MPU9250);
	stm32_configgpio(GPIO_DRDY_OFF_HMC5983);
	stm32_configgpio(GPIO_DRDY_OFF_ICM_20608_G);

	stm32_gpiowrite(GPIO_DRDY_OFF_MPU9250, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_HMC5983, 0);
	stm32_gpiowrite(GPIO_DRDY_OFF_ICM_20608_G, 0);

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
#ifdef CONFIG_STM32F7_SPI1
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
