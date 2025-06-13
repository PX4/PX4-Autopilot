/****************************************************************************
 *
 *   Copyright (C) 2021 Technology Innovation Institute. All rights reserved.
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
	initSPIBusInternal(SPI::Bus::SPI0, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P,
		SPI::CS{GPIO::Bank2, GPIO::Pin8},
		SPI::DRDY{}),
		/* NOTE: Not in use
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P,
		SPI::CS{GPIO::Bank2, GPIO::Pin11},
		SPI::DRDY{}),
		*/
		initSPIDevice(DRV_IMU_DEVTYPE_ICM20649,
		SPI::CS{GPIO::Bank2, GPIO::Pin9},
		SPI::DRDY{GPIO::Bank2, GPIO::Pin1})
	}),
	initSPIBusInternal(SPI::Bus::SPI1, {
		initSPIDevice(SPIDEV_FLASH(0),
		SPI::CS{GPIO::Bank2, GPIO::Pin15},
		SPI::DRDY{})
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
