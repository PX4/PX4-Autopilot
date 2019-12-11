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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#include <px4_platform_common/px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
//#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include <kinetis.h>
#include "board_config.h"
#include <systemlib/px4_macros.h>

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || defined(CONFIG_KINETIS_SPI2)

/* Define CS GPIO array */
static const uint32_t spi0selects_gpio[] = PX4_MEMORY_BUS_CS_GPIO;
static const uint32_t spi1selects_gpio[] = PX4_SENSOR_BUS_CS_GPIO;
static const uint32_t spi2selects_gpio[] = PX4_EXTERNAL_BUS_CS_GPIO;

/************************************************************************************
 * Public Functions
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms)
{
	/* Goal not to back feed the chips on the bus via IO lines */

	/* First float the A_MISO line tied to SA0 to ensure SPI Auto selection*/

	kinetis_pinconfig(PX4_MK_GPIO(PIN_SPI1_SIN, GPIO_OPENDRAIN));
	kinetis_gpiowrite(PIN_SPI1_SIN, 1);

	/* Next Change CS to inputs with pull downs */

	for (unsigned int cs = 0; cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			kinetis_pinconfig(PX4_MK_GPIO(spi1selects_gpio[cs], GPIO_PULLDOWN));
		}
	}

	/* Turn all the int inputs to inputs with pull down  */

	kinetis_pinconfig(PX4_MK_GPIO(GPIO_EXTI_GYRO_INT1, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_EXTI_GYRO_INT2, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_EXTI_ACCEL_MAG_INT1, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_EXTI_ACCEL_MAG_INT2, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_EXTI_BARO_INT1, GPIO_PULLDOWN));
	kinetis_pinconfig(PX4_MK_GPIO(GPIO_EXTI_BARO_INT2, GPIO_PULLDOWN));

	/* Drive the Reset Pins LOW
	 * For the Gyro FXAS21002C this is RESET
	 * for the Accel FXOS8700CQ this is not RESET */

	kinetis_gpiowrite(GPIO_GM_nRST, false);
	kinetis_gpiowrite(GPIO_A_RST, false);

	/* Power Down The Sensors */

	VDD_3V3_SENSORS_EN(false);
	up_mdelay(ms);

	/* Power Up The Sensors */
	VDD_3V3_SENSORS_EN(true);
	up_mdelay(2);

	/* Restore all the CS to ouputs inactive */

	for (unsigned int cs = 0; cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			kinetis_pinconfig(spi1selects_gpio[cs]);
		}
	}

	/* Restore all the int inputs to inputs */

	kinetis_pinconfig(GPIO_EXTI_GYRO_INT1);
	kinetis_pinconfig(GPIO_EXTI_GYRO_INT2);
	kinetis_pinconfig(GPIO_EXTI_ACCEL_MAG_INT1);
	kinetis_pinconfig(GPIO_EXTI_ACCEL_MAG_INT2);
	kinetis_pinconfig(GPIO_EXTI_BARO_INT1);
	kinetis_pinconfig(GPIO_EXTI_BARO_INT2);

	/* Set Rests Active  */

	/* Accel Assert Reset to reset the FXOS8700CQ */
	/* The Gyro was reset above */

	kinetis_gpiowrite(GPIO_A_RST, true);
	up_mdelay(ms);

	/* Accel & Gyro release Reset */

	kinetis_gpiowrite(GPIO_A_RST, false);
	kinetis_gpiowrite(GPIO_GM_nRST, true);

	/* Allow the Accel time to see PIN_SPI1_SIN as a float */

	up_mdelay(2);

	kinetis_pinconfig(PIN_SPI1_SIN);
}

/************************************************************************************
 * Name: fmuk66_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUK66-V3 board.
 *
 ************************************************************************************/

void fmuk66_spidev_initialize(void)
{
	board_spi_reset(10);

	for (unsigned int cs = 0; cs < arraySize(spi0selects_gpio); cs++) {
		if (spi0selects_gpio[cs] != 0) {
			kinetis_pinconfig(spi0selects_gpio[cs]);
		}
	}

	for (unsigned int cs = 0; cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			kinetis_pinconfig(spi1selects_gpio[cs]);
		}
	}

	for (unsigned int cs = 0; cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			kinetis_pinconfig(spi2selects_gpio[cs]);
		}
	}
}

/************************************************************************************
 * Name: kinetis_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUK66 v3 board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_sensors;
static struct spi_dev_s *spi_memory;
static struct spi_dev_s *spi_ext;

__EXPORT int fmuk66_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */

	spi_sensors = px4_spibus_initialize(PX4_SPI_BUS_SENSORS);

	if (!spi_sensors) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSORS);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_sensors, 1 * 1000 * 1000);
	SPI_SETBITS(spi_sensors, 8);
	SPI_SETMODE(spi_sensors, SPIDEV_MODE0);

	for (int cs = PX4_SENSOR_BUS_FIRST_CS; cs <= PX4_SENSOR_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensors, cs, false);
	}

	/* Get the SPI port for the Memory */

	spi_memory = px4_spibus_initialize(PX4_SPI_BUS_MEMORY);

	if (!spi_memory) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_MEMORY);
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

	/* Configure EXTERNAL SPI-based devices */

	spi_ext = px4_spibus_initialize(PX4_SPI_BUS_EXTERNAL);

	if (!spi_ext) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_EXTERNAL);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_ext, 8 * 1000 * 1000);
	SPI_SETBITS(spi_ext, 8);
	SPI_SETMODE(spi_ext, SPIDEV_MODE3);

	for (int cs = PX4_EXTERNAL_BUS_FIRST_CS; cs <= PX4_EXTERNAL_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_ext, cs, false);
	}


	return OK;

}

/************************************************************************************
 * Name:  kinetis_spi[n]select, kinetis_spi[n]status, and kinetis_spi[n]cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All other methods
 *   including kinetis_spibus_initialize()) are provided by common Kinetis logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide kinetis_spi[n]select() and kinetis_spi[n]status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      kinetis_spi[n]cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to kinetis_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by kinetis_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

void kinetis_spi0select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

	/* SPI select is active low, so write !selected to select the device */

	uint32_t sel = devid;

	if (devid == SPIDEV_FLASH(0)) {
		sel = PX4_SPIDEV_MEMORY;
	}

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_MEMORY);

	/* Making sure the other peripherals are not selected */

	for (unsigned int cs = 0;  arraySize(spi0selects_gpio) > 1 && cs < arraySize(spi0selects_gpio); cs++) {
		if (spi0selects_gpio[cs] != 0) {
			kinetis_gpiowrite(spi0selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi0selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		kinetis_gpiowrite(gpio, !selected);
	}
}

uint8_t kinetis_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

void kinetis_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSORS);

	/* Making sure the other peripherals are not selected */

	for (unsigned int cs = 0;  arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			kinetis_gpiowrite(spi1selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi1selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		kinetis_gpiowrite(gpio, !selected);
	}
}

uint8_t kinetis_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}

void kinetis_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_EXTERNAL);

	/* Making sure the other peripherals are not selected */

	for (unsigned int cs = 0;  arraySize(spi2selects_gpio) > 1 && cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			kinetis_gpiowrite(spi2selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi2selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		kinetis_gpiowrite(gpio, !selected);
	}
}

uint8_t kinetis_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}


#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 || CONFIG_KINETIS_SPI2 */
