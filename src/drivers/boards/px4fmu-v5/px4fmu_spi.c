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

/* Define CS GPIO array */
static const uint32_t spi1selects_gpio[] = PX4_SENSOR_BUS_CS_GPIO;
static const uint32_t spi2selects_gpio[] = PX4_MEMORY_BUS_CS_GPIO;
#ifdef CONFIG_STM32F7_SPI3
static const uint32_t spi3selects_gpio[] = {FIXME};
#error Need to define SPI3 Usage
#endif
static const uint32_t spi4selects_gpio[] = PX4_BARO_BUS_CS_GPIO;
#ifdef CONFIG_STM32F7_SPI3
static const uint32_t spi5selects_gpio[] = {FIXME};;
#error Need to define SPI5 Usage
#endif
static const uint32_t spi6selects_gpio[] = PX4_EXTERNAL_BUS_CS_GPIO;


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

	for (int cs = 0; cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_configgpio(spi1selects_gpio[cs]);
		}
	}

#endif

#ifdef CONFIG_STM32F7_SPI2

	for (int cs = 0; cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			stm32_configgpio(spi2selects_gpio[cs]);
		}
	}

#endif
#ifdef CONFIG_STM32F7_SPI3

	for (int cs = 0; cs < arraySize(spi3selects_gpio); cs++) {
		if (spi3selects_gpio[cs] != 0) {
			stm32_configgpio(spi3selects_gpio[cs]);
		}
	}

#endif
#ifdef CONFIG_STM32F7_SPI4

	for (int cs = 0; cs < arraySize(spi4selects_gpio); cs++) {
		if (spi4selects_gpio[cs] != 0) {
			stm32_configgpio(spi4selects_gpio[cs]);
		}
	}

#endif
#ifdef CONFIG_STM32F7_SPI5

	for (int cs = 0; cs < arraySize(spi5selects_gpio); cs++) {
		if (spi5selects_gpio[cs] != 0) {
			stm32_configgpio(spi5selects_gpio[cs]);
		}
	}

#endif
#ifdef CONFIG_STM32F7_SPI6

	for (int cs = 0; cs < arraySize(spi6selects_gpio); cs++) {
		if (spi6selects_gpio[cs] != 0) {
			stm32_configgpio(spi6selects_gpio[cs]);
		}
	}

#endif

}

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI buses on PX4FMU board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_sensors;
static struct spi_dev_s *spi_memory;
static struct spi_dev_s *spi_baro;
static struct spi_dev_s *spi_ext;

__EXPORT int stm32_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */

	/* Get the SPI port for the Sensors */

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

	/* Get the SPI port for the Memory */

	spi_memory = stm32_spibus_initialize(PX4_SPI_BUS_MEMORY);

	if (!spi_memory) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_MEMORY);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_MEMORY to 12MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_memory, 12 * 1000 * 1000);
	SPI_SETBITS(spi_memory, 8);
	SPI_SETMODE(spi_memory, SPIDEV_MODE3);

	for (int cs = PX4_MEMORY_BUS_FIRST_CS; cs <= PX4_MEMORY_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_memory, cs, false);
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

	/* Get the SPI port for the PX4_SPI_EXTERNAL */

	spi_ext = stm32_spibus_initialize(PX4_SPI_BUS_EXTERNAL);

	if (!spi_ext) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_EXTERNAL);
		return -ENODEV;
	}

	SPI_SETFREQUENCY(spi_ext, 8 * 1000 * 1000);
	SPI_SETBITS(spi_ext, 8);
	SPI_SETMODE(spi_ext, SPIDEV_MODE3);

	for (int cs = PX4_EXTERNAL_BUS_FIRST_CS; cs <= PX4_EXTERNAL_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_ext, cs, false);
	}

	return OK;

}

/************************************************************************************
 * Name: stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   Called by stm32 spi driver on bus 1.
 *
 ************************************************************************************/

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


/************************************************************************************
 * Name: stm32_spi2select and stm32_spi2status
 *
 * Description:
 *   Called by stm32 spi driver on bus 2.
 *
 ************************************************************************************/

__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;

	if (devid == SPIDEV_FLASH) {
		sel = PX4_SPIDEV_MEMORY;
	}

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_MEMORY);

	/* Making sure the other peripherals are not selected */

	for (int cs = 0;  arraySize(spi2selects_gpio) > 1 && cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			stm32_gpiowrite(spi2selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi2selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}

/************************************************************************************
 * Name: stm32_spi3select and stm32_spi2status
 *
 * Description:
 *   Called by stm32 spi driver on bus 3.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32F7_SPI3
__EXPORT void stm32_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == FIXME);

	/* Making sure the other peripherals are not selected */

	for (int cs = 0;  arraySize(spi3selects_gpio) > 1 && cs < arraySize(spi3selects_gpio); cs++) {
		if (spi3selects_gpio[cs] != 0) {
			stm32_gpiowrite(spi3selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi3selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}
#endif
/************************************************************************************
 * Name: stm32_spi4select and stm32_spi4status
 *
 * Description:
 *   Called by stm32 spi driver on bus 4.
 *
 ************************************************************************************/

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_BARO);

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
	return SPI_STATUS_PRESENT;
}

/************************************************************************************
 * Name: stm32_spi5select and stm32_spi5status
 *
 * Description:
 *   Called by stm32 spi driver on bus 5.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32F7_SPI5
__EXPORT void stm32_spi5select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == FIXME);

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
	return SPI_STATUS_PRESENT;
}
#endif

/************************************************************************************
 * Name: stm32_spi6select and stm32_spi6status
 *
 * Description:
 *   Called by stm32 spi driver on bus 6.
 *
 ************************************************************************************/

__EXPORT void stm32_spi6select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_EXTERNAL);

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
	return SPI_STATUS_PRESENT;
}

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	for (int cs = 0;  arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_configgpio(_PIN_OFF(spi1selects_gpio[cs]));
		}
	}

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);


#if BOARD_USE_DRDY
	stm32_configgpio(GPIO_DRDY_OFF_ICM20689_DRDY1);
	stm32_configgpio(GPIO_DRDY_OFF_BMI055_DRDY2_GYRO);
	stm32_configgpio(GPIO_DRDY_OFF_BMI055_DRDY3_ACC);
	stm32_configgpio(GPIO_DRDY_OFF_ICM20602_DRDY4);
	stm32_configgpio(GPIO_DRDY_OFF_BMI055_DRDY5_GYRO);
	stm32_configgpio(GPIO_DRDY_OFF_BMI055_DRDY6_ACC);
#endif
	/* set the sensor rail off */
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
	for (int cs = 0;  arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_configgpio(spi1selects_gpio[cs]);
		}
	}

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

#if BOARD_USE_DRDY
	stm32_configgpio(GPIO_ICM20689_DRDY1);
	stm32_configgpio(GPIO_BMI055_DRDY2_GYRO);
	stm32_configgpio(GPIO_BMI055_DRDY3_ACC);
	stm32_configgpio(GPIO_ICM20602_DRDY4);
	stm32_configgpio(GPIO_BMI055_DRDY5_GYRO);
	stm32_configgpio(GPIO_BMI055_DRDY6_ACC);
#endif

}
