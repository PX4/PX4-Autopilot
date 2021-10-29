/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)
#define BOARD_PLL_SYS_FREQ      (125 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (125 * MHZ)
#define BOARD_PERI_FREQ         (125 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_RTC_FREQ          46875

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

#define BOARD_TICK_CLOCK        (1 * MHZ)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
 * omnibusf4sd.  The following definitions describe how NuttX controls the LEDs:
 */

// #define LED_STARTED       0  /* LED1 */
// #define LED_HEAPALLOCATE  1  /* LED2 */
// #define LED_IRQSENABLED   2  /* LED1 */
// #define LED_STACKCREATED  3  /* LED1 + LED2 */
// #define LED_INIRQ         4  /* LED1 */
// #define LED_SIGNAL        5  /* LED2 */
// #define LED_ASSERTION     6  /* LED1 + LED2 */
// #define LED_PANIC         7  /* LED1 + LED2 */

/* Alternate function pin selections ************************************************/

/*
 * UARTs.
 * UART0TX: GPIO0
 * UART0RX: GPIO1
 * UART1TX: GPIO8
 * UART1RX: GPIO9
 */
#define CONFIG_RP2040_UART0_GPIO	0	/* TELEM */

#define CONFIG_RP2040_UART1_GPIO	8	/* GPS */

/*
 * I2C (external)
 *
 * I2C1SCL: GPIO7
 * I2C1SDA: GPIO6
 *
 * TODO:
 *   The optional _GPIO configurations allow the I2C driver to manually
 *   reset the bus to clear stuck slaves.  They match the pin configuration,
 *   but are normally-high GPIOs.
 */
#define CONFIG_RP2040_I2C1_GPIO		6

/* SPI0:
 *  SPIDEV_FLASH (probably micro sd card)
 *  CS: GPIO5 -- should be configured in sec/spi.cpp (probably)
 *  CLK: GPIO2
 *  MISO: GPIO4
 *  MOSI: GPIO3
 */

#define GPIO_SPI0_SCLK  ( 2 | GPIO_FUN(RP2040_GPIO_FUNC_SPI) )
#define GPIO_SPI0_MISO ( 4 | GPIO_FUN(RP2040_GPIO_FUNC_SPI) )
#define GPIO_SPI0_MOSI ( 3 | GPIO_FUN(RP2040_GPIO_FUNC_SPI) )

/* SPI1:
 *  MPU9250 and BMP280
 *  CS: GPIO13 for MPU9250, GPIO14 for BMP280 -- should be configured in sec/spi.cpp (probably)
 *  CLK: GPIO10
 *  MISO: GPIO12
 *  MOSI: GPIO11
 */

#define GPIO_SPI1_SCLK	( 10 | GPIO_FUN(RP2040_GPIO_FUNC_SPI) )
#define GPIO_SPI1_MISO	( 12 | GPIO_FUN(RP2040_GPIO_FUNC_SPI) )
#define GPIO_SPI1_MOSI	( 11 | GPIO_FUN(RP2040_GPIO_FUNC_SPI) )


#endif  /* __ARCH_BOARD_BOARD_H */
