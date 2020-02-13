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

#include <nuttx/spi/spi.h>
#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(1, {
		initSPIDevice(DRV_GYR_DEVTYPE_ICM20689, SPI::CS{GPIO::PortF, GPIO::Pin2}, SPI::DRDY{GPIO::PortB, GPIO::Pin4}),
		initSPIDevice(DRV_GYR_DEVTYPE_ICM20602, SPI::CS{GPIO::PortF, GPIO::Pin3}, SPI::DRDY{GPIO::PortC, GPIO::Pin5}),
		initSPIDevice(DRV_GYR_DEVTYPE_BMI055, SPI::CS{GPIO::PortF, GPIO::Pin4}, SPI::DRDY{GPIO::PortB, GPIO::Pin14}),
		initSPIDevice(DRV_ACC_DEVTYPE_BMI055, SPI::CS{GPIO::PortG, GPIO::Pin10}, SPI::DRDY{GPIO::PortB, GPIO::Pin15}),
	}, true),
	initSPIBus(2, {
		initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortF, GPIO::Pin5})
	}),
	initSPIBus(4, {
		initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortF, GPIO::Pin10}),
	}),
	initSPIBusExternal(5, {
		SPI::CS{GPIO::PortI, GPIO::Pin4},
		SPI::CS{GPIO::PortI, GPIO::Pin10},
		SPI::CS{GPIO::PortI, GPIO::Pin11}
	}),
	initSPIBusExternal(6, {
		SPI::CS{GPIO::PortI, GPIO::Pin6},
		SPI::CS{GPIO::PortI, GPIO::Pin7},
		SPI::CS{GPIO::PortI, GPIO::Pin8}
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);


// TODO: move to i2c.cpp
#include <px4_arch/i2c_hw_description.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusExternal(1),
	initI2CBusExternal(2),
	initI2CBusInternal(3),
	initI2CBusExternal(4),
};


//////////////////////////////////////////////////////////////////////////
// TODO: all of the things below are generic and can be moved under px4_arch/f7...

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

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/
#include <px4_platform_common/spi.h>

static const px4_spi_bus_t *_spi_bus1;
static const px4_spi_bus_t *_spi_bus2;
static const px4_spi_bus_t *_spi_bus3;
static const px4_spi_bus_t *_spi_bus4;
static const px4_spi_bus_t *_spi_bus5;
static const px4_spi_bus_t *_spi_bus6;

static void spi_bus_configgpio_cs(const px4_spi_bus_t *bus)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio != 0) {
			px4_arch_configgpio(bus->devices[i].cs_gpio);
		}
	}
}

__EXPORT void stm32_spiinitialize()
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 1: _spi_bus1 = &px4_spi_buses[i]; break;

		case 2: _spi_bus2 = &px4_spi_buses[i]; break;

		case 3: _spi_bus3 = &px4_spi_buses[i]; break;

		case 4: _spi_bus4 = &px4_spi_buses[i]; break;

		case 5: _spi_bus5 = &px4_spi_buses[i]; break;

		case 6: _spi_bus6 = &px4_spi_buses[i]; break;
		}
	}

#ifdef CONFIG_STM32F7_SPI1
	ASSERT(_spi_bus1);

	spi_bus_configgpio_cs(_spi_bus1);
#endif // CONFIG_STM32F7_SPI1


#if defined(CONFIG_STM32F7_SPI2)
	ASSERT(_spi_bus2);

	spi_bus_configgpio_cs(_spi_bus2);

#endif // CONFIG_STM32F7_SPI2

#ifdef CONFIG_STM32F7_SPI3
	ASSERT(_spi_bus3);

	spi_bus_configgpio_cs(_spi_bus3);

#endif // CONFIG_STM32F7_SPI3

#ifdef CONFIG_STM32F7_SPI4
	ASSERT(_spi_bus4);

	spi_bus_configgpio_cs(_spi_bus4);

#endif // CONFIG_STM32F7_SPI4


#ifdef CONFIG_STM32F7_SPI5
	ASSERT(_spi_bus5);

	spi_bus_configgpio_cs(_spi_bus5);

#endif // CONFIG_STM32F7_SPI5


#ifdef CONFIG_STM32F7_SPI6
	ASSERT(_spi_bus6);

	spi_bus_configgpio_cs(_spi_bus6);

#endif // CONFIG_STM32F7_SPI6
}

static inline void stm32_spixselect(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	int matched_dev_idx = -1;

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			matched_dev_idx = i;

		} else {
			// Making sure the other peripherals are not selected
			stm32_gpiowrite(bus->devices[i].cs_gpio, 1);
		}
	}

	// different devices might use the same CS, so make sure to configure the one we want last
	if (matched_dev_idx != -1) {
		// SPI select is active low, so write !selected to select the device
		stm32_gpiowrite(bus->devices[matched_dev_idx].cs_gpio, !selected);
	}
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
	stm32_spixselect(_spi_bus1, dev, devid, selected);
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
	stm32_spixselect(_spi_bus2, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI2 && GPIO_SPI2_CS_FRAM

/************************************************************************************
 * Name: stm32_spi4select and stm32_spi4status
 *
 * Description:
 *   Called by stm32 spi driver on bus 4.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32F7_SPI4

__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus4, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI4

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
	stm32_spixselect(_spi_bus5, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI5

/************************************************************************************
 * Name: stm32_spi6select and stm32_spi6status
 *
 * Description:
 *   Called by stm32 spi driver on bus 6.
 *
 ************************************************************************************/
#ifdef CONFIG_STM32F7_SPI6

__EXPORT void stm32_spi6select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	stm32_spixselect(_spi_bus6, dev, devid, selected);
}

__EXPORT uint8_t stm32_spi6status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32F7_SPI6

/************************************************************************************
 * Name: board_spi_reset
 *
 * Description:
 *
 *
 ************************************************************************************/

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

__EXPORT void board_spi_reset(int ms)
{
	// TODO: make this fully generic...

	// disable SPI bus
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		if (!px4_spi_buses[bus].should_be_reset) {
			continue;
		}

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
			}
		}
	}

	stm32_configgpio(_PIN_OFF(GPIO_SPI1_SCK));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MISO));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MOSI));


#if BOARD_USE_DRDY
	stm32_configgpio(GPIO_DRDY_OFF_SPI1_DRDY1_ICM20689);
	stm32_configgpio(GPIO_DRDY_OFF_SPI1_DRDY2_BMI055_GYRO);
	stm32_configgpio(GPIO_DRDY_OFF_SPI1_DRDY3_BMI055_ACC);
	stm32_configgpio(GPIO_DRDY_OFF_SPI1_DRDY4_ICM20602);
	stm32_configgpio(GPIO_DRDY_OFF_SPI1_DRDY5_BMI055_GYRO);
	stm32_configgpio(GPIO_DRDY_OFF_SPI1_DRDY6_BMI055_ACC);
#endif
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
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
		if (px4_spi_buses[bus].bus == -1) {
			break;
		}

		if (!px4_spi_buses[bus].should_be_reset) {
			continue;
		}

		for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
			if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
				px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
			}
		}
	}

	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

#if BOARD_USE_DRDY
	stm32_configgpio(GPIO_SPI1_DRDY1_ICM20689);
	stm32_configgpio(GPIO_SPI1_DRDY2_BMI055_GYRO);
	stm32_configgpio(GPIO_SPI1_DRDY3_BMI055_ACC);
	stm32_configgpio(GPIO_SPI1_DRDY4_ICM20602);
	stm32_configgpio(GPIO_SPI1_DRDY5_BMI055_GYRO);
	stm32_configgpio(GPIO_SPI1_DRDY6_BMI055_ACC);
#endif
}

