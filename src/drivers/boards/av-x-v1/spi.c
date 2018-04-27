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
 * @file px4fmu_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>
#include <px4_log.h>

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

/* Define CS GPIO array */
static const uint32_t spi1selects_gpio[] = PX4_SENSOR1_BUS_CS_GPIO;
static const uint32_t spi2selects_gpio[] = PX4_SENSOR2_BUS_CS_GPIO;
static const uint32_t spi4selects_gpio[] = PX4_SENSOR4_BUS_CS_GPIO;
static const uint32_t spi5selects_gpio[] = PX4_SENSOR5_BUS_CS_GPIO;


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
	board_gpio_init(spi1selects_gpio, arraySize(spi1selects_gpio));
	board_gpio_init(spi2selects_gpio, arraySize(spi2selects_gpio));
	board_gpio_init(spi4selects_gpio, arraySize(spi4selects_gpio));
	board_gpio_init(spi5selects_gpio, arraySize(spi5selects_gpio));
}

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI buses on PX4FMU board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_sensor1;
static struct spi_dev_s *spi_sensor2;
static struct spi_dev_s *spi_sensor4;
static struct spi_dev_s *spi_sensor5;

__EXPORT int stm32_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */


	/* Get the SPI port for the Sensors */
	spi_sensor1 = stm32_spibus_initialize(PX4_SPI_BUS_SENSOR1);

	if (!spi_sensor1) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSOR1);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi_sensor1, 1000000);
	SPI_SETBITS(spi_sensor1, 16);
	SPI_SETMODE(spi_sensor1, SPIDEV_MODE3);

	for (int cs = PX4_SENSOR1_BUS_FIRST_CS; cs <= PX4_SENSOR1_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensor1, cs, false);
	}


	/* Get the SPI port for the Memory */
	spi_sensor2 = stm32_spibus_initialize(PX4_SPI_BUS_SENSOR2);

	if (!spi_sensor2) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSOR2);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSOR2 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi_sensor2, 1000000);
	SPI_SETBITS(spi_sensor2, 8);
	SPI_SETMODE(spi_sensor2, SPIDEV_MODE3);

	for (int cs = PX4_SENSOR2_BUS_FIRST_CS; cs <= PX4_SENSOR2_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensor2, cs, false);
	}


	/* Get the SPI port for the BARO */
	spi_sensor4 = stm32_spibus_initialize(PX4_SPI_BUS_SENSOR4);

	if (!spi_sensor4) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSOR4);
		return -ENODEV;
	}

	SPI_SETFREQUENCY(spi_sensor4, 1 * 1000 * 1000);
	SPI_SETBITS(spi_sensor4, 8);
	SPI_SETMODE(spi_sensor4, SPIDEV_MODE3);

	for (int cs = PX4_SENSOR4_BUS_FIRST_CS; cs <= PX4_SENSOR4_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensor4, cs, false);
	}


	/* Get the SPI port for the PX4_SPI_EXTERNAL1 */
	spi_sensor5 = stm32_spibus_initialize(PX4_SPI_BUS_SENSOR5);

	if (!spi_sensor5) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSOR5);
		return -ENODEV;
	}

	SPI_SETFREQUENCY(spi_sensor5, 1 * 1000 * 1000);
	SPI_SETBITS(spi_sensor5, 8);
	SPI_SETMODE(spi_sensor5, SPIDEV_MODE3);

	for (int cs = PX4_SENSOR5_BUS_FIRST_CS; cs <= PX4_SENSOR5_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensor5, cs, false);
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

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSOR1);

	/* Making sure the other peripherals are not selected */
	for (size_t cs = 0; arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_gpiowrite(spi1selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi1selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
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

__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSOR2);

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

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}


/************************************************************************************
 * Name: stm32_spi4select and stm32_spi4status
 *
 * Description:
 *   Called by stm32 spi driver on bus 4.
 *
 ************************************************************************************/

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSOR4);

	/* Making sure the other peripherals are not selected */
	for (size_t cs = 0; arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		stm32_gpiowrite(spi4selects_gpio[cs], 1);
	}

	uint32_t gpio = spi4selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
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

__EXPORT void stm32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSOR5);

	/* Making sure the other peripherals are not selected */
	for (size_t cs = 0; arraySize(spi5selects_gpio) > 1 && cs < arraySize(spi5selects_gpio); cs++) {
		stm32_gpiowrite(spi5selects_gpio[cs], 1);
	}

	uint32_t gpio = spi5selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		stm32_gpiowrite(gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}


/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *
 *
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */

	// SPI1
	for (size_t cs = 0;  arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_configgpio(_PIN_OFF(spi1selects_gpio[cs]));
		}
	}

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	// SPI2
	for (size_t cs = 0;  arraySize(spi2selects_gpio) > 1 && cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			stm32_configgpio(_PIN_OFF(spi2selects_gpio[cs]));
		}
	}

	stm32_configgpio(GPIO_SPI2_SCK_OFF);
	stm32_configgpio(GPIO_SPI2_MISO_OFF);
	stm32_configgpio(GPIO_SPI2_MOSI_OFF);

	// SPI4
	for (size_t cs = 0;  arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		if (spi4selects_gpio[cs] != 0) {
			stm32_configgpio(_PIN_OFF(spi4selects_gpio[cs]));
		}
	}

	stm32_configgpio(GPIO_SPI4_SCK_OFF);
	stm32_configgpio(GPIO_SPI4_MISO_OFF);
	stm32_configgpio(GPIO_SPI4_MOSI_OFF);

	// SPI5
	for (size_t cs = 0;  arraySize(spi5selects_gpio) > 1 && cs < arraySize(spi5selects_gpio); cs++) {
		if (spi5selects_gpio[cs] != 0) {
			stm32_configgpio(_PIN_OFF(spi5selects_gpio[cs]));
		}
	}

	stm32_configgpio(GPIO_SPI5_SCK_OFF);
	stm32_configgpio(GPIO_SPI5_MISO_OFF);
	stm32_configgpio(GPIO_SPI5_MOSI_OFF);


	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	PX4_INFO("reset done, %d ms", ms);

	/* re-enable power */

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */

	// SPI1
	for (size_t cs = 0; arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			stm32_configgpio(spi1selects_gpio[cs]);
		}
	}

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	// SPI2
	for (size_t cs = 0; arraySize(spi2selects_gpio) > 1 && cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			stm32_configgpio(spi2selects_gpio[cs]);
		}
	}

	stm32_configgpio(GPIO_SPI2_SCK);
	stm32_configgpio(GPIO_SPI2_MISO);
	stm32_configgpio(GPIO_SPI2_MOSI);

	// SPI4
	for (size_t cs = 0; arraySize(spi4selects_gpio) > 1 && cs < arraySize(spi4selects_gpio); cs++) {
		if (spi4selects_gpio[cs] != 0) {
			stm32_configgpio(spi4selects_gpio[cs]);
		}
	}

	stm32_configgpio(GPIO_SPI4_SCK);
	stm32_configgpio(GPIO_SPI4_MISO);
	stm32_configgpio(GPIO_SPI4_MOSI);

	// SPI5
	for (size_t cs = 0; arraySize(spi5selects_gpio) > 1 && cs < arraySize(spi5selects_gpio); cs++) {
		if (spi5selects_gpio[cs] != 0) {
			stm32_configgpio(spi5selects_gpio[cs]);
		}
	}

	stm32_configgpio(GPIO_SPI5_SCK);
	stm32_configgpio(GPIO_SPI5_MISO);
	stm32_configgpio(GPIO_SPI5_MOSI);

}
