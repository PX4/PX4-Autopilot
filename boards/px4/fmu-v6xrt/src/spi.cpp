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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

#include <px4_platform_common/px4_config.h>
#include <px4_log.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <systemlib/px4_macros.h>
#include <px4_platform/gpio.h>

#include <arm_internal.h>
#include <chip.h>
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "board_config.h"
#include <systemlib/err.h>

#ifdef CONFIG_IMXRT_LPSPI


constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::LPSPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42686P, SPI::CS{GPIO::Port2, GPIO::Pin11}, SPI::DRDY{GPIO::Port3, GPIO::Pin19}), /* GPIO_EMC_B2_01 GPIO2_IO11, GPIO_AD_20, GPIO3_IO19 */
	}, {GPIO::Port2, GPIO::Pin1}), // Power GPIO_EMC_B1_33  GPIO2_IO01

	initSPIBus(SPI::Bus::LPSPI2, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::Port3, GPIO::Pin24}, SPI::DRDY{GPIO::Port2, GPIO::Pin7}), /* GPIO_AD_25 GPIO3_IO24, GPIO_EMC_B1_39 GPIO2_IO07 */
	}, {GPIO::Port1, GPIO::Pin22}), // Power GPIO_EMC_B1_22  GPIO1_IO22

	initSPIBus(SPI::Bus::LPSPI3, {
		initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::Port2, GPIO::Pin18}, SPI::DRDY{GPIO::Port2, GPIO::Pin28}), /* GPIO_EMC_B2_08 GPIO2_IO18, GPIO_EMC_B2_18 GPIO2_IO28 */
		initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::Port2, GPIO::Pin15}),                                      /* GPIO_EMC_B2_05 GPIO2_IO15 */
	}, {GPIO::Port1, GPIO::Pin14}), // Power GPIO_EMC_B1_14  GPIO1_IO14

	initSPIBusExternal(SPI::Bus::LPSPI6, {
		initSPIConfigExternal(SPI::CS{GPIO::Port6, GPIO::Pin9}, SPI::DRDY{GPIO::Port1, GPIO::Pin5}), /* GPIO_LPSR_09 GPIO6_IO09 GPIO_EMC_B1_05 GPIO1_IO05*/
		initSPIConfigExternal(SPI::CS{GPIO::Port6, GPIO::Pin8}, SPI::DRDY{GPIO::Port1, GPIO::Pin7}), /* GPIO_LPSR_08 GPIO6_IO08  GPIO_EMC_B1_07  GPIO1_IO07*/
	}),
};

#endif
static constexpr bool unused = validateSPIConfig(px4_spi_buses);
