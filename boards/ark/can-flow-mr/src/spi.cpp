/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortA, GPIO::Pin4}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin4}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
		initSPIDevice(DRV_IMU_DEVTYPE_IIM42652, SPI::CS{GPIO::PortA, GPIO::Pin4}, SPI::DRDY{GPIO::PortB, GPIO::Pin0}),
		initSPIDevice(DRV_FLOW_DEVTYPE_PAW3902, SPI::CS{GPIO::PortB, GPIO::Pin5}, SPI::DRDY{GPIO::PortB, GPIO::Pin2}),
		initSPIDevice(DRV_FLOW_DEVTYPE_PAA3905, SPI::CS{GPIO::PortB, GPIO::Pin5}, SPI::DRDY{GPIO::PortB, GPIO::Pin2}),
	}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(DRV_DEVTYPE_UNUSED, SPI::CS{GPIO::PortB, GPIO::Pin12}, SPI::DRDY{GPIO::PortB, GPIO::Pin4}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
