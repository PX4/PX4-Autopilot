/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

/*
 * SPI buses and devices for CIPHERWING F4SD (custom)
 * - SPI1: IMU on PA4 (probe MPU6000 or ICM20689)
 * - SPI2: microSD card on PB12
 * - SPI3: Barometer (BMP280) + OSD (ATXXXX)
 */

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	// SPI1: IMU on PA4
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_MPU6000,  SPI::CS{GPIO::PortA, GPIO::Pin4}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20689, SPI::CS{GPIO::PortA, GPIO::Pin4}),
	}),

	// SPI2: SD card
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(SPIDEV_MMCSD(0), SPI::CS{GPIO::PortB, GPIO::Pin12})
	}),

	// SPI3: Barometer + OSD (if populated)
	initSPIBus(SPI::Bus::SPI3, {
		initSPIDevice(DRV_BARO_DEVTYPE_BMP280,  SPI::CS{GPIO::PortB, GPIO::Pin3}),
		initSPIDevice(DRV_OSD_DEVTYPE_ATXXXX,   SPI::CS{GPIO::PortA, GPIO::Pin15}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);

