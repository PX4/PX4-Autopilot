/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <px4_arch/board_init.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

#define SPI_PIN_OFF(def)   (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))
#define SDMMC_PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_FLOAT   |GPIO_SPEED_2MHz))

void px4_arch_pin_initialize()
{
	// Reset PWM first thing
	board_on_reset(-1);

	// configure pins
#if defined(PX4_GPIO_INIT_LIST)
	const uint32_t list[] = PX4_GPIO_INIT_LIST;

	for (auto &gpio : list) {
		if (gpio != 0) {
			px4_arch_configgpio(gpio);
		}
	}

#endif // PX4_GPIO_INIT_LIST




// CAN

#if defined(GPIO_CAN1_RX)
	px4_arch_configgpio(GPIO_CAN1_RX);
#endif // GPIO_CAN1_RX
#if defined(GPIO_CAN1_TX)
	px4_arch_configgpio(GPIO_CAN1_TX);
#endif // GPIO_CAN1_TX
#if defined(GPIO_CAN1_SILENT_S0)
	px4_arch_configgpio(GPIO_CAN1_SILENT_S0);
#endif // GPIO_CAN1_SILENT_S0

#if defined(GPIO_CAN2_RX)
	px4_arch_configgpio(GPIO_CAN2_RX);
#endif // GPIO_CAN2_RX
#if defined(GPIO_CAN2_TX)
	px4_arch_configgpio(GPIO_CAN2_TX);
#endif // GPIO_CAN2_TX
#if defined(GPIO_CAN2_SILENT_S0)
	px4_arch_configgpio(GPIO_CAN2_SILENT_S0);
#endif // GPIO_CAN2_SILENT_S0

#if defined(GPIO_CAN3_RX)
	px4_arch_configgpio(GPIO_CAN3_RX);
#endif // GPIO_CAN3_RX
#if defined(GPIO_CAN3_TX)
	px4_arch_configgpio(GPIO_CAN3_TX);
#endif // GPIO_CAN3_TX
#if defined(GPIO_CAN3_SILENT_S0)
	px4_arch_configgpio(GPIO_CAN3_SILENT_S0);
#endif // GPIO_CAN3_SILENT_S0




// I2C

#if defined(CONFIG_STM32H7_I2C1)
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL));
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA));
#endif // CONFIG_STM32H7_I2C1

#if defined(CONFIG_STM32H7_I2C2)
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL));
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA));
#endif // CONFIG_STM32H7_I2C2

#if defined(CONFIG_STM32H7_I2C3)
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SCL));
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SDA));
#endif // CONFIG_STM32H7_I2C3

#if defined(CONFIG_STM32H7_I2C4)
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL));
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA));
#endif // CONFIG_STM32H7_I2C4




// SPI

#if defined(CONFIG_STM32H7_SPI1)
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI1_SCK));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI1_MISO));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI1_MOSI));
#endif // CONFIG_STM32_SPI1
#if defined(CONFIG_STM32H7_SPI2)
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI2_SCK));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI2_MISO));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI2_MOSI));
#endif // CONFIG_STM32_SPI2
#if defined(CONFIG_STM32H7_SPI3)
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI3_SCK));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI3_MISO));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI3_MOSI));
#endif // CONFIG_STM32_SPI3
#if defined(CONFIG_STM32H7_SPI4)
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI4_SCK));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI4_MISO));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI4_MOSI));
#endif // CONFIG_STM32_SPI4
#if defined(CONFIG_STM32H7_SPI5)
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI5_SCK));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI5_MISO));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI5_MOSI));
#endif // CONFIG_STM32_SPI5
#if defined(CONFIG_STM32H7_SPI6)
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI6_SCK));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI6_MISO));
	px4_arch_configgpio(SPI_PIN_OFF(GPIO_SPI6_MOSI));
#endif // CONFIG_STM32_SPI6




// SDMMC

#if defined(CONFIG_STM32H7_SDMMC1)
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D1));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D2));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D3));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD));
#endif // CONFIG_STM32H7_SDMMC1
#if defined(CONFIG_STM32H7_SDMMC2)
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC2_D0));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC2_D1));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC2_D2));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC2_D3));
	px4_arch_configgpio(PX4_GPIO_PIN_OFF(GPIO_SDMMC2_CMD));
#endif // CONFIG_STM32H7_SDMMC2




// USB
#if defined(CONFIG_STM32H7_OTGFS)
	px4_arch_configgpio(GPIO_OTGFS_VBUS);
#endif

}

