/************************************************************************************
 * configs/px4fmu/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
#pragma once

#include "board_dma_map.h"

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - 16 MHz Crystal
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        16000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     0

/* Main PLL Configuration.
 *
 * PLL source is HSE = 16,000,000
 *
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     2 <= PLLM <= 63
 *   192 <= PLLN <= 432
 *   192 MHz <= PLL_VCO <= 432MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * Subject to
 *
 *   PLLP = {2, 4, 6, 8}
 *   SYSCLK <= 216 MHz
 *
 * USB OTG FS, SDMMC and RNG Clock = PLL_VCO / PLLQ
 * Subject to
 *   The USB OTG FS requires a 48 MHz clock to work correctly. The SDMMC
 *   and the random number generator need a frequency lower than or equal
 *   to 48 MHz to work correctly.
 *
 * 2 <= PLLQ <= 15
 */

/* Highest SYSCLK with USB OTG FS clock = 48 MHz
 *
 * PLL_VCO = (16,000,000 / 8) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(9)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 8) * 216)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 9)

/* Configure factors for  PLLSAI clock */

#define CONFIG_STM32F7_PLLSAI 1
#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(8)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR1_PLLI2SDIVQ  RCC_DCKCFGR1_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVQ  RCC_DCKCFGR1_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVR  RCC_DCKCFGR1_PLLSAIDIVR(0)
#define STM32_RCC_DCKCFGR1_SAI1SRC     RCC_DCKCFGR1_SAI1SEL(0)
#define STM32_RCC_DCKCFGR1_SAI2SRC     RCC_DCKCFGR1_SAI2SEL(0)
#define STM32_RCC_DCKCFGR1_TIMPRESRC   0
#define STM32_RCC_DCKCFGR1_DFSDM1SRC   0
#define STM32_RCC_DCKCFGR1_ADFSDM1SRC  0



/* Configure factors for  PLLI2S clock */

#define CONFIG_STM32F7_PLLI2S 1
#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_USART1SRC  RCC_DCKCFGR2_USART1SEL_APB
#define STM32_RCC_DCKCFGR2_USART2SRC  RCC_DCKCFGR2_USART2SEL_APB
#define STM32_RCC_DCKCFGR2_UART4SRC   RCC_DCKCFGR2_UART4SEL_APB
#define STM32_RCC_DCKCFGR2_UART5SRC   RCC_DCKCFGR2_UART5SEL_APB
#define STM32_RCC_DCKCFGR2_USART6SRC  RCC_DCKCFGR2_USART6SEL_APB
#define STM32_RCC_DCKCFGR2_UART7SRC   RCC_DCKCFGR2_UART7SEL_APB
#define STM32_RCC_DCKCFGR2_UART8SRC   RCC_DCKCFGR2_UART8SEL_APB
#define STM32_RCC_DCKCFGR2_I2C1SRC    RCC_DCKCFGR2_I2C1SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C2SRC    RCC_DCKCFGR2_I2C2SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C3SRC    RCC_DCKCFGR2_I2C3SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C4SRC    RCC_DCKCFGR2_I2C4SEL_HSI
#define STM32_RCC_DCKCFGR2_LPTIM1SRC  RCC_DCKCFGR2_LPTIM1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSRC     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_PHY


/* Several prescalers allow the configuration of the two AHB buses, the
 * high-speed APB (APB2) and the low-speed APB (APB1) domains. The maximum
 * frequency of the two AHB buses is 216 MHz while the maximum frequency of
 * the high-speed APB domains is 108 MHz. The maximum allowed frequency of
 * the low-speed APB domain is 54 MHz.
 */

/* AHB clock (HCLK) is SYSCLK (216 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (108MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)


/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 7

/* LED definitions ******************************************************************/
/* The board has numerous LEDs but 1, RED LED

 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */


/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red
 *   ----------------------  --------------------------  ------ */

#define LED_STARTED        0 /* NuttX has been started   OFF    */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    */
#define LED_INIRQ          4 /* In an interrupt          N/C    */
#define LED_SIGNAL         5 /* In a signal handler      N/C    */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */


/* UARTs */

#define GPIO_USART2_TX   GPIO_USART2_TX_1   /* PA2  */
#define GPIO_USART2_RX   GPIO_USART2_RX_1   /* PA3  */

#define GPIO_USART3_TX   GPIO_USART3_TX_1   /* PB10 */
#define GPIO_USART3_RX   GPIO_USART3_RX_1   /* PB11 */


/* CAN */

#define GPIO_CAN1_RX	GPIO_CAN1_RX_2      /* PB8  */
#define GPIO_CAN1_TX	GPIO_CAN1_TX_2      /* PB9  */


/* I2C */

#define GPIO_I2C1_SCL	GPIO_I2C1_SCL_1     /* PB6  */
#define GPIO_I2C1_SDA	GPIO_I2C1_SDA_1     /* PB7  */

#define GPIO_I2C3_SCL	GPIO_I2C3_SCL_1     /* PA8  */
#define GPIO_I2C3_SDA	GPIO_I2C3_SDA_1     /* PC9  */


/* SPI */

#define GPIO_SPI3_MISO  GPIO_SPI3_MISO_2    /* PC11 */
#define GPIO_SPI3_MOSI  GPIO_SPI3_MOSI_3    /* PC12 */
#define GPIO_SPI3_SCK   GPIO_SPI3_SCK_1     /* PB3  */

