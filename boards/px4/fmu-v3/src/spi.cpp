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

constexpr px4_spi_bus_all_hw_t px4_spi_buses_all_hw[BOARD_NUM_HW_VERSIONS] = {
	initSPIHWVersion(HW_VER_FMUV2, {
		initSPIBus(1, {
			initSPIDevice(DRV_GYR_DEVTYPE_MPU6000, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_GYR_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}, SPI::DRDY{GPIO::PortD, GPIO::Pin15}),
			initSPIDevice(DRV_GYR_DEVTYPE_L3GD20, SPI::CS{GPIO::PortC, GPIO::Pin13}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
			initSPIDevice(DRV_ACC_DEVTYPE_LSM303D, SPI::CS{GPIO::PortC, GPIO::Pin15}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		}, true),
		initSPIBus(2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBusExternal(4, {
			SPI::CS{GPIO::PortC, GPIO::Pin14},
			SPI::CS{GPIO::PortE, GPIO::Pin4},
		}),
	}),

	initSPIHWVersion(HW_VER_FMUV3, {
		initSPIBus(1, {
			initSPIDevice(DRV_GYR_DEVTYPE_MPU6000, SPI::CS{GPIO::PortC, GPIO::Pin2}),
			initSPIDevice(DRV_GYR_DEVTYPE_MPU9250, SPI::CS{GPIO::PortC, GPIO::Pin2}),
			initSPIDevice(DRV_MAG_DEVTYPE_HMC5883, SPI::CS{GPIO::PortC, GPIO::Pin1}), // HMC5983
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		}, true),
		initSPIBus(2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBus(4, {
			// TODO: which devices are running here?
			initSPIDevice(DRV_ACC_DEVTYPE_ACCELSIM, SPI::CS{GPIO::PortC, GPIO::Pin13}),
			initSPIDevice(DRV_ACC_DEVTYPE_ACCELSIM, SPI::CS{GPIO::PortC, GPIO::Pin14}),
			initSPIDevice(DRV_ACC_DEVTYPE_ACCELSIM, SPI::CS{GPIO::PortC, GPIO::Pin15}),
			initSPIDevice(DRV_ACC_DEVTYPE_ACCELSIM, SPI::CS{GPIO::PortE, GPIO::Pin4}),
		}),
	}),

	initSPIHWVersion(HW_VER_FMUV2MINI, {
		initSPIBus(1, {
			initSPIDevice(DRV_GYR_DEVTYPE_ICM20608, SPI::CS{GPIO::PortC, GPIO::Pin15}, SPI::DRDY{GPIO::PortC, GPIO::Pin14}),
			initSPIDevice(DRV_BARO_DEVTYPE_MS5611, SPI::CS{GPIO::PortD, GPIO::Pin7}),
		}, true),
		initSPIBus(2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBusExternal(4, {
			SPI::CS{GPIO::PortC, GPIO::Pin13},
			SPI::CS{GPIO::PortC, GPIO::Pin15},
			SPI::CS{GPIO::PortE, GPIO::Pin4},
		}),
	}),

	initSPIHWVersion(HW_VER_FMUV2X, {
		initSPIBus(1, {
			// TODO
		}, true),
		initSPIBus(2, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortD, GPIO::Pin10})
		}),
		initSPIBusExternal(4, {
			// TODO
		}),
	}),

};
static constexpr bool unused = validateSPIConfig(px4_spi_buses_all_hw);


static const px4_spi_bus_t *_spi_bus1;
static const px4_spi_bus_t *_spi_bus2;
static const px4_spi_bus_t *_spi_bus4;


/**
 * @file px4fmu_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>

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

#ifdef CONFIG_STM32_SPI1
/*   Verification
 *        PA5 PA6 PA7 PB0 PB1 PB4 PC1 PC2 PC13 PC14 PC15 PD7 PD15 PE2 PE4 PE5 PE6
 * driver  X   X   X                                               X       X   X
 * local               V2  v2   V2  V3  a   V2  V2M  V2x   a    a        4
 */
static void stm32_spi1_initialize(void)
{
	stm32_configgpio(GPIO_SPI1_CS_PC2);
	stm32_configgpio(GPIO_SPI1_CS_PD7);

	stm32_configgpio(GPIO_SPI1_EXTI_DRDY_PD15);

	if (HW_VER_FMUV2MINI == board_get_hw_version()) {
		stm32_configgpio(GPIO_SPI1_EXTI_20608_DRDY_PC14);
		stm32_configgpio(GPIO_SPI1_CS_PC15);

	} else if (HW_VER_FMUV3 == board_get_hw_version()) {
		stm32_configgpio(GPIO_SPI1_CS_PC1);

	} else {
		stm32_configgpio(GPIO_SPI1_EXTI_DRDY_PB0);
		stm32_configgpio(GPIO_SPI1_EXTI_DRDY_PB1);
		stm32_configgpio(GPIO_SPI1_EXTI_DRDY_PB4);
		stm32_configgpio(GPIO_SPI1_CS_PC13);
		stm32_configgpio(GPIO_SPI1_CS_PC15);
	}
}
#endif // CONFIG_STM32_SPI1

#ifdef CONFIG_STM32_SPI4
/*   Verification
 *        PA5 PA6 PA7 PB0 PB1 PB4 PC1 PC2 PC13 PC14 PC15 PD7 PD15 PE2 PE4 PE5 PE6
 * driver   X   X   X                                               X       X   X
 * local               V3 V3   -  V3  a   V3    V23  V3   -    -
 */
static void stm32_spi4_initialize(void)
{
	stm32_configgpio(GPIO_SPI4_NSS_PE4);

	if (HW_VER_FMUV3 == board_get_hw_version()) {
		stm32_configgpio(GPIO_SPI4_EXTI_DRDY_PB0);
		stm32_configgpio(GPIO_SPI4_CS_PB1);
		stm32_configgpio(GPIO_SPI4_CS_PC13);
		stm32_configgpio(GPIO_SPI4_CS_PC15);
	}

	if (HW_VER_FMUV2MINI != board_get_hw_version()) {
		stm32_configgpio(GPIO_SPI4_GPIO_PC14);
	}
}
#endif //CONFIG_STM32_SPI4

__EXPORT void stm32_spiinitialize(void)
{
	px4_set_spi_buses_from_hw_version();

	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
		switch (px4_spi_buses[i].bus) {
		case 1: _spi_bus1 = &px4_spi_buses[i]; break;

		case 2: _spi_bus2 = &px4_spi_buses[i]; break;

		case 4: _spi_bus4 = &px4_spi_buses[i]; break;
		}
	}

#ifdef CONFIG_STM32_SPI1
	stm32_spi1_initialize();
#endif

#ifdef CONFIG_STM32_SPI2
	stm32_configgpio(GPIO_SPI2_CS_PD10);
#endif

#ifdef CONFIG_STM32_SPI4
	stm32_spi4_initialize();
#endif
}

#ifdef CONFIG_STM32_SPI1

static void spi1_gpiowrite(uint32_t pinset, bool value)
{
	uint32_t pinport = pinset & (GPIO_PORT_MASK | GPIO_PIN_MASK);

	if (pinport == (GPIO_PORTC | GPIO_PIN13)) {
		if (HW_VER_FMUV2 == board_get_hw_version()) {
			stm32_gpiowrite(pinset, value);
		}

	} else if (pinport == (GPIO_PORTC | GPIO_PIN15)) {
		if (HW_VER_FMUV3 != board_get_hw_version()) {
			stm32_gpiowrite(pinset, value);
		}

	} else if (pinport == (GPIO_PORTC | GPIO_PIN1)) {
		if (HW_VER_FMUV3 == board_get_hw_version()) {
			stm32_gpiowrite(pinset, value);
		}

	} else {
		stm32_gpiowrite(pinset, value);
	}
}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	/*   Verification
	 *        PA5 PA6 PA7 PB0 PB1 PB4 PC1 PC2 PC13 PC14 PC15 PD7 PD15 PE2 PE4 PE5 PE6
	 * driver  X   X   X                                               X       X   X
	 * local               -   -   -  V3   a   V2   -   V2M   a    -       -
	 */
	int matched_dev_idx = -1;
	const px4_spi_bus_t *bus = _spi_bus1;

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			matched_dev_idx = i;

		} else {
			// Making sure the other peripherals are not selected
			spi1_gpiowrite(bus->devices[i].cs_gpio, 1);
		}
	}

	// different devices might use the same CS, so make sure to configure the one we want last
	if (matched_dev_idx != -1) {
		// SPI select is active low, so write !selected to select the device
		spi1_gpiowrite(bus->devices[matched_dev_idx].cs_gpio, !selected);
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI1

#ifdef CONFIG_STM32_SPI2
__EXPORT void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* there can only be one device on this bus, so always select it */
	stm32_gpiowrite(GPIO_SPI2_CS_PD10, !selected);
}

__EXPORT uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
#endif


#ifdef CONFIG_STM32_SPI4
static void spi4_gpiowrite(uint32_t pinset, bool value)
{
	uint32_t pinport = pinset & (GPIO_PORT_MASK | GPIO_PIN_MASK);

	if (pinport == (GPIO_PORTC | GPIO_PIN14)) {
		if (HW_VER_FMUV2MINI != board_get_hw_version()) {
			stm32_gpiowrite(pinset, value);
		}

	} else if (pinport == (GPIO_PORTC | GPIO_PIN13) || pinport == (GPIO_PORTC | GPIO_PIN15)) {
		if (HW_VER_FMUV3 == board_get_hw_version()) {
			stm32_gpiowrite(pinset, value);
		}

	} else {
		stm32_gpiowrite(pinset, value);
	}
}
__EXPORT void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */
	/*   Verification
	 *        PA5 PA6 PA7 PB0 PB1 PB4 PC1 PC2 PC13 PC14 PC15 PD7 PD15 PE2 PE4 PE5 PE6
	 * driver  X   X   X                                               X       X   X
	 * local               -   -   -  -    -   V3  !V2M  V3   -   -        a
	 */

	int matched_dev_idx = -1;
	const px4_spi_bus_t *bus = _spi_bus4;

	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
		if (bus->devices[i].cs_gpio == 0) {
			break;
		}

		if (devid == bus->devices[i].devid) {
			matched_dev_idx = i;

		} else {
			// Making sure the other peripherals are not selected
			spi4_gpiowrite(bus->devices[i].cs_gpio, 1);
		}
	}

	// different devices might use the same CS, so make sure to configure the one we want last
	if (matched_dev_idx != -1) {
		// SPI select is active low, so write !selected to select the device
		spi4_gpiowrite(bus->devices[matched_dev_idx].cs_gpio, !selected);
	}
}
__EXPORT uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI4

/* V2, V2M SPI1 All signals SPI4, V3 ALL signals */
/*   Verification
 *        PA5 PA6 PA7 PB0 PB1 PB4 PC1 PC2 PC13 PC14 PC15 PD7 PD15 PE2 PE4 PE5 PE6
 * local   A   A   A   A   A   A  V3   A   A    !V2    A   A   A  V3  V3  V3   V3
 */

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC2));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC13));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC15));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PD7));

	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC2), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC13), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC15), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PD7), 0);

	stm32_configgpio(_PIN_OFF(GPIO_SPI1_SCK));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MISO));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MOSI));

	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_SCK), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_MISO), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_MOSI), 0);

	stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB0));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB1));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB4));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PD15));

	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB0), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB1), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB4), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PD15), 0);

	if (HW_VER_FMUV2 != board_get_hw_version()) {
		stm32_configgpio(_PIN_OFF(GPIO_SPI4_CS_PC14));
		stm32_gpiowrite(_PIN_OFF(GPIO_SPI4_CS_PC14), 0);
	}

	if (HW_VER_FMUV3 == board_get_hw_version()) {
		stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC1));
		stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC1), 0);

		stm32_configgpio(_PIN_OFF(GPIO_SPI4_NSS_PE4));
		stm32_gpiowrite(_PIN_OFF(GPIO_SPI4_NSS_PE4), 0);

		stm32_configgpio(_PIN_OFF(GPIO_SPI4_SCK));
		stm32_configgpio(_PIN_OFF(GPIO_SPI4_MISO));
		stm32_configgpio(_PIN_OFF(GPIO_SPI4_MOSI));

		stm32_gpiowrite(_PIN_OFF(GPIO_SPI4_SCK), 0);
		stm32_gpiowrite(_PIN_OFF(GPIO_SPI4_MISO), 0);
		stm32_gpiowrite(_PIN_OFF(GPIO_SPI4_MOSI), 0);
	}

	/* set the sensor rail off */
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
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
	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	if (HW_VER_FMUV3 == board_get_hw_version()) {
		stm32_configgpio(GPIO_SPI4_SCK);
		stm32_configgpio(GPIO_SPI4_MISO);
		stm32_configgpio(GPIO_SPI4_MOSI);
		stm32_spi4_initialize();
	}

	stm32_spi1_initialize();
}

