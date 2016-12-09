/************************************************************************************
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <px4_config.h>

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
#include <systemlib/err.h>
#include <systemlib/px4_macros.h>

#if defined(CONFIG_KINETIS_SPI0) || defined(CONFIG_KINETIS_SPI1) || \
	defined(CONFIG_KINETIS_SPI2)

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

__EXPORT void board_spi_reset(int ms)
{

}

/************************************************************************************
 * Name: nxphlite_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXPhlite-v1 board.
 *
 ************************************************************************************/

void nxphlite_spidev_initialize(void)
{
	kinetis_pinconfig(GPIO_SPI_CS_SDCARD);
	kinetis_pinconfig(GPIO_SPI_CS_FXOS8700CQ_ACCEL_MAG);
	kinetis_pinconfig(GPIO_SPI_CS_FXAS21002CQ_GYRO);

	kinetis_pinconfig(GPIO_GM_RST);
	kinetis_pinconfig(GPIO_A_RST);

	kinetis_pinconfig(GPIO_EXTI_GYRO_INT1);
	kinetis_pinconfig(GPIO_EXTI_GYRO_INT2);
	kinetis_pinconfig(GPIO_EXTI_ACCEL_MAG_INT1);
	kinetis_pinconfig(GPIO_EXTI_ACCEL_MAG_INT2);
	kinetis_pinconfig(GPIO_EXTI_BARO_INT1);
	kinetis_pinconfig(GPIO_EXTI_BARO_INT2);
}

/************************************************************************************
 * Name: kinetis_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXPHLITEV1 board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_accel_mag;
static struct spi_dev_s *spi_baro;

__EXPORT int nxphlite_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */

	spi_accel_mag = kinetis_spibus_initialize(PX4_SPI_BUS_ACCEL_MAG);

	if (!spi_accel_mag) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_ACCEL_MAG);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_ACCEL_MAG to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_accel_mag, 1 * 1000 * 1000);
	SPI_SETBITS(spi_accel_mag, 8);
	SPI_SETMODE(spi_accel_mag, SPIDEV_MODE3);

	for (int cs = PX4_ACCEL_MAG_BUS_FIRST_CS; cs <= PX4_ACCEL_MAG_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_accel_mag, cs, false);
	}

	/* Get the SPI port for the GYRO */

	spi_baro = kinetis_spibus_initialize(PX4_SPI_BUS_GYRO);

	if (!spi_baro) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_GYRO);
		return -ENODEV;
	}

	/* FXAS21002CQ has max SPI clock speed of 2MHz and uses MODE 0 (CPOL = 0, and CPHA = 0)
	 */

	SPI_SETFREQUENCY(spi_baro, 2 * 1000 * 1000);
	SPI_SETBITS(spi_baro, 8);
	SPI_SETMODE(spi_baro, SPIDEV_MODE0);

	for (int cs = PX4_GYRO_BUS_FIRST_CS; cs <= PX4_GYRO_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_baro, cs, false);
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

static const uint32_t spi0selects_gpio[] = PX4_SDCARD_BUS_CS_GPIO;

void kinetis_spi0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SDCARD);

	/* Making sure the other peripherals are not selected */

	for (int cs = 0;  arraySize(spi0selects_gpio) > 1 && cs < arraySize(spi0selects_gpio); cs++) {
		if (spi0selects_gpio[cs] != 0) {
			kinetis_gpiowrite(spi0selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi0selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		kinetis_gpiowrite(gpio, !selected);
	}
}

uint8_t kinetis_spi0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}

static const uint32_t spi1selects_gpio[] = PX4_ACCEL_MAG_BUS_CS_GPIO;

void kinetis_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_ACCEL_MAG);

	/* Making sure the other peripherals are not selected */

	for (int cs = 0;  arraySize(spi1selects_gpio) > 1 && cs < arraySize(spi1selects_gpio); cs++) {
		if (spi1selects_gpio[cs] != 0) {
			kinetis_gpiowrite(spi1selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi1selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		kinetis_gpiowrite(gpio, !selected);
	}
}

uint8_t kinetis_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}

static const uint32_t spi2selects_gpio[] = PX4_GYRO_BUS_CS_GPIO;

void kinetis_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_GYRO);

	/* Making sure the other peripherals are not selected */

	for (int cs = 0;  arraySize(spi2selects_gpio) > 1 && cs < arraySize(spi2selects_gpio); cs++) {
		if (spi2selects_gpio[cs] != 0) {
			kinetis_gpiowrite(spi2selects_gpio[cs], 1);
		}
	}

	uint32_t gpio = spi2selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		kinetis_gpiowrite(gpio, !selected);
	}
}

uint8_t kinetis_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}


#endif /* CONFIG_KINETIS_SPI0 || CONFIG_KINETIS_SPI1 || CONFIG_KINETIS_SPI2 */
