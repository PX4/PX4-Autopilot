/****************************************************************************
 *
 *   Copyright (C) 2020, 2022 PX4 Development Team. All rights reserved.
 *   Copyright (C) 2025 Lectron. All rights reserved.
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
 * @file spi.cpp
 *
 * Lectron Pi5 Autopilot SPI Configuration
 *
 * Hardware Configuration:
 * - SPI1: ICM-42670-P (FMU Board)     - CS: PI9,  INT: PF2
 * - SPI2: ICM-42670-P (Sensor Board)  - CS: PH5,  INT: PA10
 * - SPI3: BMI270 (IMU)                - CS: PI4,  INT1: PI6, INT2: PI7
 * - SPI5: Flash Memory (Internal)     - CS: PG7
 * - SPI6: External Expansion          - CS: PI10, PA15
 */

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_all_hw_t px4_spi_buses_all_hw[BOARD_NUM_SPI_CFG_HW_VERSIONS] = {
	initSPIFmumID(V6X_0, {
// SPI1: ICM-42670-P on FMU Board (Primary IMU)
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42670P, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
// SPI2: ICM-42670-P on Sensor Board (Secondary IMU)
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42670P, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
// SPI3: BMI270 (6-axis IMU: Gyro + Accel)
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_IMU_DEVTYPE_BMI270, SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortI, GPIO::Pin6}),
		}, {GPIO::PortE, GPIO::Pin7}),
// SPI5: Internal Flash Memory
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
// SPI6: External Expansion (PMW3901 Optical Flow + 1 free slot)
// PMW3901: DRDY removed - using polling mode (DRDY pin floating issue)
		initSPIBus(SPI::Bus::SPI6, {
			initSPIDevice(DRV_FLOW_DEVTYPE_PMW3901, SPI::CS{GPIO::PortI, GPIO::Pin10}),
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses_all_hw);

// Runtime SPI bus configuration (points to V6X_0 by default)
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	px4_spi_buses_all_hw[0].buses[0],  // SPI1
	px4_spi_buses_all_hw[0].buses[1],  // SPI2
	px4_spi_buses_all_hw[0].buses[2],  // SPI3
	px4_spi_buses_all_hw[0].buses[3],  // SPI4
	px4_spi_buses_all_hw[0].buses[4],  // SPI5
	px4_spi_buses_all_hw[0].buses[5],  // SPI6
};
