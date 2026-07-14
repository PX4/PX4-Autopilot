/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/*
 * SPI bus config for CORVON 743V2.
 *
 * SPI3 = sensor bus, 3 devices:
 *   - ICM-42688P (primary IMU)   CS: PA15  DRDY: PD10 (EXTI10)
 *   - BMI088 Gyro  (secondary)   CS: PD5   DRDY: PD11 (EXTI11)
 *   - BMI088 Accel (secondary)   CS: PD4   DRDY: PD6  (EXTI6)
 *
 * SPI3 alt-func mapping (in nuttx-config/include/board.h):
 *   SCK  = PB3 (AF6) via GPIO_SPI3_SCK_1
 *   MISO = PB4 (AF6) via GPIO_SPI3_MISO_1
 *   MOSI = PB5 (AF7) via GPIO_SPI3_MOSI_4   (NOT _2 -- that one is PC12 AF6)
 *
 * SPI clock targets: 24 MHz for ICM-42688P, 10 MHz for BMI088 (driver-set).
 */

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI3, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P,   SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin10}),
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088,      SPI::CS{GPIO::PortD, GPIO::Pin5},  SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088,      SPI::CS{GPIO::PortD, GPIO::Pin4},  SPI::DRDY{GPIO::PortD, GPIO::Pin6}),
	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
