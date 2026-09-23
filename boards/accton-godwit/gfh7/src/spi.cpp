/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_OSD_DEVTYPE_ATXXXX, SPI::CS{GPIO::PortA, GPIO::Pin4}),
	}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(DRV_IMU_DEVTYPE_ST_LSM6DSV, SPI::CS{GPIO::PortB, GPIO::Pin9},
			      SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
	}),
	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortE, GPIO::Pin4},
			      SPI::DRDY{GPIO::PortD, GPIO::Pin10}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
