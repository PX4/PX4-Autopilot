/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef __NUTTX_CONFIG_DAKEFPV_H743_INCLUDE_BOARD_H
#define __NUTTX_CONFIG_DAKEFPV_H743_INCLUDE_BOARD_H

#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"

/****************************************************************************
 * Clocking — 8 MHz HSE → 480 MHz CPU (same as all STM32H743 PX4 boards)
 ****************************************************************************/

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

#define STM32_BOARD_USEHSE

#define STM32_PLLCFG_PLLSRC     RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1: VCO = (8 MHz / 1) * 120 = 960 MHz → P=480, Q=240, R=120 */
#define STM32_PLLCFG_PLL1CFG    (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP1EN | \
                                  RCC_PLLCFGR_DIVQ1EN | \
                                  RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M      RCC_PLLCKSELR_DIVM1(1)
#define STM32_PLLCFG_PLL1N      RCC_PLL1DIVR_N1(120)
#define STM32_PLLCFG_PLL1P      RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q      RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R      RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY    ((STM32_HSE_FREQUENCY / 1) * 120)
#define STM32_PLL1P_FREQUENCY   (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY   (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY   (STM32_VCO1_FREQUENCY / 8)

/* PLL2: VCO = (8 MHz / 2) * 48 = 192 MHz → P=96, Q=96, R=96 */
#define STM32_PLLCFG_PLL2CFG    (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP2EN | \
                                  RCC_PLLCFGR_DIVQ2EN | \
                                  RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M      RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N      RCC_PLL2DIVR_N2(48)
#define STM32_PLLCFG_PLL2P      RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q      RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R      RCC_PLL2DIVR_R2(2)

#define STM32_VCO2_FREQUENCY    ((STM32_HSE_FREQUENCY / 2) * 48)
#define STM32_PLL2P_FREQUENCY   (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY   (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2R_FREQUENCY   (STM32_VCO2_FREQUENCY / 2)

/* PLL3: VCO = (8 MHz / 2) * 48 = 192 MHz → Q=48 MHz (USB) */
#define STM32_PLLCFG_PLL3CFG   (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL3RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVQ3EN)
#define STM32_PLLCFG_PLL3M      RCC_PLLCKSELR_DIVM3(2)
#define STM32_PLLCFG_PLL3N      RCC_PLL3DIVR_N3(48)
#define STM32_PLLCFG_PLL3P      RCC_PLL3DIVR_P3(2)
#define STM32_PLLCFG_PLL3Q      RCC_PLL3DIVR_Q3(4)
#define STM32_PLLCFG_PLL3R      RCC_PLL3DIVR_R3(2)

#define STM32_VCO3_FREQUENCY    ((STM32_HSE_FREQUENCY / 2) * 48)
#define STM32_PLL3P_FREQUENCY   (STM32_VCO3_FREQUENCY / 2)
#define STM32_PLL3Q_FREQUENCY   (STM32_VCO3_FREQUENCY / 4)
#define STM32_PLL3R_FREQUENCY   (STM32_VCO3_FREQUENCY / 2)

/* SYSCLK = PLL1P = 480 MHz, CPUCLK = 480 MHz */
#define STM32_RCC_D1CFGR_D1CPRE (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY  (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY  (STM32_SYSCLK_FREQUENCY / 1)

/* AHB/HCLK = SYSCLK/2 = 240 MHz */
#define STM32_RCC_D1CFGR_HPRE  RCC_D1CFGR_HPRE_SYSCLKd2
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY

/* APB1 = HCLK/2 = 120 MHz */
#define STM32_RCC_D2CFGR_D2PPRE1 RCC_D2CFGR_D2PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)

/* APB2 = HCLK/2 = 120 MHz */
#define STM32_RCC_D2CFGR_D2PPRE2 RCC_D2CFGR_D2PPRE2_HCLKd2
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)

/* APB3 = HCLK/2 = 120 MHz */
#define STM32_RCC_D1CFGR_D1PPRE  RCC_D1CFGR_D1PPRE_HCLKd2
#define STM32_PCLK3_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)

/* APB4 = HCLK/2 = 120 MHz */
#define STM32_RCC_D3CFGR_D3PPRE  RCC_D3CFGR_D3PPRE_HCLKd2
#define STM32_PCLK4_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)

/* Timer clocks (APB1 timers × 2, APB2 timers × 2) */
#define STM32_APB1_TIM2_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2 * STM32_PCLK1_FREQUENCY)

#define STM32_APB2_TIM1_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2 * STM32_PCLK2_FREQUENCY)

/* Kernel clock sources */
#define STM32_RCC_D2CCIP2R_I2C123SRC  RCC_D2CCIP2R_I2C123SEL_HSI
#define STM32_RCC_D3CCIPR_I2C4SRC     RCC_D3CCIPR_I2C4SEL_HSI
#define STM32_RCC_D2CCIP1R_SPI123SRC  RCC_D2CCIP1R_SPI123SEL_PLL2
#define STM32_RCC_D2CCIP1R_SPI45SRC   RCC_D2CCIP1R_SPI45SEL_PLL2
#define STM32_RCC_D3CCIPR_SPI6SRC     RCC_D3CCIPR_SPI6SEL_PLL2
#define STM32_RCC_D2CCIP2R_USBSRC     RCC_D2CCIP2R_USBSEL_PLL3
#define STM32_RCC_D3CCIPR_ADCSEL      RCC_D3CCIPR_ADCSEL_PLL2
#define STM32_RCC_D2CCIP1R_FDCANSEL   RCC_D2CCIP1R_FDCANSEL_HSE
#define STM32_FDCANCLK                 STM32_HSE_FREQUENCY

#define BOARD_FLASH_WAITSTATES 2

/****************************************************************************
 * LEDs
 ****************************************************************************/

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_RED     BOARD_LED1
#define BOARD_LED_GREEN   BOARD_LED2
#define BOARD_LED_BLUE    BOARD_LED3

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

#define LED_STARTED        0
#define LED_HEAPALLOCATE   1
#define LED_IRQSENABLED    2
#define LED_STACKCREATED   3
#define LED_INIRQ          4
#define LED_SIGNAL         5
#define LED_ASSERTION      6
#define LED_PANIC          7
#define LED_IDLE           8

/****************************************************************************
 * UART / serial pin assignments
 ****************************************************************************/

#define GPIO_USART1_TX   GPIO_USART1_TX_2   /* PA9  */
#define GPIO_USART1_RX   GPIO_USART1_RX_2   /* PA10 */

#define GPIO_USART2_TX   GPIO_USART2_TX_2   /* PD5  */
#define GPIO_USART2_RX   GPIO_USART2_RX_2   /* PD6  */

#define GPIO_USART3_TX   GPIO_USART3_TX_3   /* PD8  */
#define GPIO_USART3_RX   GPIO_USART3_RX_3   /* PD9  */

#define GPIO_UART4_TX    GPIO_UART4_TX_5    /* PD1  */
#define GPIO_UART4_RX    GPIO_UART4_RX_5    /* PD0  */

#define GPIO_UART5_TX    GPIO_UART5_TX_2    /* PB6  */
#define GPIO_UART5_RX    GPIO_UART5_RX_2    /* PB5  */

#define GPIO_USART6_TX   GPIO_USART6_TX_1   /* PC6  */
#define GPIO_USART6_RX   GPIO_USART6_RX_1   /* PC7  */

#define GPIO_UART7_TX    GPIO_UART7_TX_3    /* PE8  */
#define GPIO_UART7_RX    GPIO_UART7_RX_3    /* PE7  */

#define GPIO_UART8_TX    GPIO_UART8_TX_1    /* PE1  */
#define GPIO_UART8_RX    GPIO_UART8_RX_1    /* PE0  */

/****************************************************************************
 * SPI pin assignments
 ****************************************************************************/

#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_2MHz))

/* SPI1 (IMU1 ICM-42688P): PA5/PA6/PA7 */
#define GPIO_SPI1_SCK    ADJ_SLEW_RATE(GPIO_SPI1_SCK_1)  /* PA5 */
#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1                 /* PA6 */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1                 /* PA7 */

/* SPI2 (AT7456E OSD): PB13/PB14/PB15 */
#define GPIO_SPI2_SCK    ADJ_SLEW_RATE(GPIO_SPI2_SCK_3)  /* PB13 */
#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1                 /* PB14 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1                 /* PB15 */

/* SPI3 (dataflash): PC10/PC11/PC12 */
#define GPIO_SPI3_SCK    ADJ_SLEW_RATE(GPIO_SPI3_SCK_2)  /* PC10 */
#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_2                 /* PC11 */
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_2                 /* PC12 */

/* SPI4 (IMU2 ICM-42688P): PE12/PE13/PE14 */
#define GPIO_SPI4_SCK    ADJ_SLEW_RATE(GPIO_SPI4_SCK_1)  /* PE12 */
#define GPIO_SPI4_MISO   GPIO_SPI4_MISO_1                 /* PE13 */
#define GPIO_SPI4_MOSI   GPIO_SPI4_MOSI_1                 /* PE14 */

/****************************************************************************
 * I2C pin assignments
 ****************************************************************************/

/* I2C2 (SPL06 barometer): PB10/PB11 */
#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1    /* PB10 */
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_1    /* PB11 */

#define GPIO_I2C2_SCL_GPIO  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN11)

#define PROBE_INIT(mask)
#define PROBE(n,s)
#define PROBE_MARK(n)

#endif /* __NUTTX_CONFIG_DAKEFPV_H743_INCLUDE_BOARD_H */
