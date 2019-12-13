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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_log.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>
#include <px4_platform/gpio.h>

#include <up_arch.h>
#include <chip.h>
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "board_config.h"
#include <systemlib/err.h>

#if defined(CONFIG_IMXRT_LPSPI1) || defined(CONFIG_IMXRT_LPSPI2) || \
    defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)

/* Define CS GPIO array */

#if defined(CONFIG_IMXRT_LPSPI1)
static const uint32_t spi1selects_gpio[] = PX4_SENSOR_BUS_CS_GPIO;
#endif
#if defined(CONFIG_IMXRT_LPSPI2)
static const uint32_t spi2selects_gpio[] = PX4_MEMORY_BUS_CS_GPIO;
#endif
#if defined(CONFIG_IMXRT_LPSPI3)
static const uint32_t spi3selects_gpio[] = PX4_BARO_BUS_CS_GPIO;
#endif
#if defined(CONFIG_IMXRT_LPSPI4)
static const uint32_t spi4selects_gpio[] = PX4_EXTERNAL1_BUS_CS_GPIO;
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/************************************************************************************
 * Name: fmurt1062_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUKRT1062-V1 board.
 *
 ************************************************************************************/

void imxrt_spidev_initialize(void)
{
#if defined(CONFIG_IMXRT_LPSPI1)
	px4_gpio_init(spi1selects_gpio, arraySize(spi1selects_gpio));
#endif
#if defined(CONFIG_IMXRT_LPSPI2)
	px4_gpio_init(spi2selects_gpio, arraySize(spi2selects_gpio));
#endif
#if defined(CONFIG_IMXRT_LPSPI3)
	px4_gpio_init(spi3selects_gpio, arraySize(spi3selects_gpio));
#endif
#if defined(CONFIG_IMXRT_LPSPI4)
	px4_gpio_init(spi4selects_gpio, arraySize(spi4selects_gpio));
#endif
}

/************************************************************************************
 * Name: imxrt_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP FMUKRT1062-V1 board.
 *
 ************************************************************************************/
static struct spi_dev_s *spi_sensors;
static struct spi_dev_s *spi_memory;
static struct spi_dev_s *spi_baro;
static struct spi_dev_s *spi_ext;

__EXPORT int imxrt1062_spi_bus_initialize(void)
{
	/* Configure SPI-based devices */

	spi_sensors = px4_spibus_initialize(PX4_SPI_BUS_SENSORS);

	if (!spi_sensors) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSORS);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_sensors, 1 * 1000 * 1000);
	SPI_SETBITS(spi_sensors, 8);
	SPI_SETMODE(spi_sensors, SPIDEV_MODE3);

	for (int cs = PX4_SENSORS_BUS_FIRST_CS; cs <= PX4_SENSORS_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_sensors, cs, false);
	}

	/* Get the SPI port for the Memory */

	spi_memory = px4_spibus_initialize(PX4_SPI_BUS_MEMORY);

	if (!spi_memory) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_MEMORY);
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

	spi_baro = px4_spibus_initialize(PX4_SPI_BUS_BARO);

	if (!spi_baro) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_BARO);
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

	/* Get the SPI port for the PX4_SPI_EXTERNAL1 */

	spi_ext = px4_spibus_initialize(PX4_SPI_BUS_EXTERNAL1);

	if (!spi_ext) {
		PX4_ERR("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_EXTERNAL1);
		return -ENODEV;
	}

	/* Default PX4_SPI_BUS_SENSORS to 1MHz and de-assert the known chip selects.
	 */

	SPI_SETFREQUENCY(spi_ext, 8 * 1000 * 1000);
	SPI_SETBITS(spi_ext, 8);
	SPI_SETMODE(spi_ext, SPIDEV_MODE3);

	for (int cs = PX4_EXTERNAL1_BUS_FIRST_CS; cs <= PX4_EXTERNAL1_BUS_LAST_CS; cs++) {
		SPI_SELECT(spi_ext, cs, false);
	}

	return OK;

}

/****************************************************************************
 * Name:  imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *
 * Description:
 *   The external functions, imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including imxrt_lpspibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imxrt_lpspi1/2/3select() and imxrt_lpspi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#if defined(CONFIG_IMXRT_LPSPI1)
__EXPORT void imxrt_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_SENSORS);

	/* Making sure the other peripherals are not selected */

	for (auto cs : spi1selects_gpio) {
		imxrt_gpio_write(cs, 1);
	}

	uint32_t gpio = spi1selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		imxrt_gpio_write(gpio, !selected);
	}
}

__EXPORT uint8_t imxrt_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#if defined(CONFIG_IMXRT_LPSPI2)
__EXPORT void imxrt_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;

	if (devid == SPIDEV_FLASH(0)) {
		sel = PX4_SPIDEV_MEMORY;
	}

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_MEMORY);

	/* Making sure the other peripherals are not selected */

	for (auto cs : spi2selects_gpio) {
		imxrt_gpio_write(cs, 1);
	}

	uint32_t gpio = spi2selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		imxrt_gpio_write(gpio, !selected);
	}
}

__EXPORT uint8_t imxrt_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#if defined(CONFIG_IMXRT_LPSPI3)
__EXPORT void imxrt_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	int sel = (int) devid;
	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_BARO);

	/* Making sure the other peripherals are not selected */

	for (auto cs : spi3selects_gpio) {
		imxrt_gpio_write(cs, 1);
	}

	uint32_t gpio = spi3selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		imxrt_gpio_write(gpio, !selected);
	}
}

__EXPORT uint8_t imxrt_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

#if defined(CONFIG_IMXRT_LPSPI4)
__EXPORT void imxrt_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	int sel = (int) devid;

	ASSERT(PX4_SPI_BUS_ID(sel) == PX4_SPI_BUS_EXTERNAL1);

	/* Making sure the other peripherals are not selected */
	for (auto cs : spi4selects_gpio) {
		imxrt_gpio_write(cs, 1);
	}

	uint32_t gpio = spi4selects_gpio[PX4_SPI_DEV_ID(sel)];

	if (gpio) {
		imxrt_gpio_write(gpio, !selected);
	}
}

__EXPORT uint8_t imxrt_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif

/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *
 *
 ************************************************************************************/

__EXPORT void board_spi_reset(int ms)
{
#ifdef CONFIG_IMXRT_LPSPI1

	/* Goal not to back feed the chips on the bus via IO lines */
	for (auto cs : spi1selects_gpio) {
		imxrt_config_gpio(_PIN_OFF(cs));
	}

	imxrt_config_gpio(GPIO_SPI1_SCK_OFF);
	imxrt_config_gpio(GPIO_SPI1_MISO_OFF);
	imxrt_config_gpio(GPIO_SPI1_MOSI_OFF);

	for (auto cs : spi3selects_gpio) {
		imxrt_config_gpio(_PIN_OFF(cs));
	}

	imxrt_config_gpio(GPIO_SPI3_SCK_OFF);
	imxrt_config_gpio(GPIO_SPI3_MISO_OFF);
	imxrt_config_gpio(GPIO_SPI3_MOSI_OFF);


	imxrt_config_gpio(_PIN_OFF(GPIO_LPI2C3_SDA_RESET));
	imxrt_config_gpio(_PIN_OFF(GPIO_LPI2C3_SCL_RESET));

#  if BOARD_USE_DRDY
	imxrt_config_gpio(GPIO_DRDY_OFF_SPI1_DRDY1_ICM20689);
	imxrt_config_gpio(GPIO_DRDY_OFF_SPI1_DRDY2_BMI055_GYRO);
	imxrt_config_gpio(GPIO_DRDY_OFF_SPI1_DRDY3_BMI055_ACC);
	imxrt_config_gpio(GPIO_DRDY_OFF_SPI1_DRDY4_ICM20602);
#  endif
	/* set the sensor rail off */
	imxrt_gpio_write(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	imxrt_gpio_write(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
	for (auto cs : spi1selects_gpio) {
		imxrt_config_gpio(cs);
	}

	imxrt_config_gpio(GPIO_LPSPI1_SCK);
	imxrt_config_gpio(GPIO_LPSPI1_MISO);
	imxrt_config_gpio(GPIO_LPSPI1_MOSI);

	/* reconfigure the SPI pins */
	for (auto cs : spi3selects_gpio) {
		imxrt_config_gpio(cs);
	}

	imxrt_config_gpio(GPIO_LPSPI3_SCK);
	imxrt_config_gpio(GPIO_LPSPI3_MISO);
	imxrt_config_gpio(GPIO_LPSPI3_MOSI);

	imxrt_config_gpio(GPIO_LPI2C3_SDA);
	imxrt_config_gpio(GPIO_LPI2C3_SCL);

#  if BOARD_USE_DRDY
	imxrt_config_gpio(GPIO_SPI1_DRDY1_ICM20689);
	imxrt_config_gpio(GPIO_SPI1_DRDY2_BMI055_GYRO);
	imxrt_config_gpio(GPIO_SPI1_DRDY3_BMI055_ACC);
	imxrt_config_gpio(GPIO_SPI1_DRDY4_ICM20602);
#  endif
#endif /* CONFIG_IMXRT_LPSPI1 */

}

#endif /* CONFIG_IMXRT_LPSPI1 || CONFIG_IMXRT_LPSPI2 || CONFIG_IMXRT_LPSPI3 || CONFIG_IMXRT_LPSPI4  */
