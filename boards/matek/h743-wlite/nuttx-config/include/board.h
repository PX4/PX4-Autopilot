/************************************************************************************
 * nuttx-configs/px4_fmu-v6u/include/board.h
 *
 *   Copyright (C) 2016-2019 Gregory Nutt. All rights reserved.
 *   Authors: David Sidrane <david.sidrane@nscdg.com>
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
#ifndef __NUTTX_CONFIG_MATEKH743SLIM_INCLUDE_BOARD_H
#define __NUTTX_CONFIG_MATEKH743SLIM_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdmmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The MatekH743-Slim  board provides the following clock sources:
 *
 *   X1: 8 MHz crystal for HSE
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 64 MHz RC factory-trimmed
 *   HSE:  8 MHz crystal for HSE
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 8,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 480 MHz
 */

#define STM32_BOARD_USEHSE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (8,000,000 / 1) * 120 = 960 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz
 *   PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
 *   PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
 */

#define STM32_PLLCFG_PLL1CFG    (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
				 RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
				 RCC_PLLCFGR_DIVP1EN | \
				 RCC_PLLCFGR_DIVQ1EN | \
				 RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(1)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(120)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 1) * 120)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG     (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
				  RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
				  RCC_PLLCFGR_DIVP2EN | \
				  RCC_PLLCFGR_DIVQ2EN | \
				  RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(48)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(2)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 48)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)

/* PLL3 */

#define STM32_PLLCFG_PLL3CFG    (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
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

/* SYSCLK = PLL1P = 480MHz
 * CPUCLK = SYSCLK / 1 = 480 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (240 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4 = 240
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY            /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/2 (120 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2       /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB2 clock (PCLK2) is HCLK/2 (120 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2       /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 (120 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2        /* PCLK3 = HCLK / 2 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB4 clock (PCLK4) is HCLK/4 (120 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2       /* PCLK4 = HCLK / 2 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* Timer clock frequencies */

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

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C123 clock source */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2

/* SPI45 clock source */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_PLL2

/* SPI6 clock source */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PLL2

/* USB 1 and 2 clock source */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_PLL3

/* ADC 1 2 3 clock source */

#define STM32_RCC_D3CCIPR_ADCSRC     RCC_D3CCIPR_ADCSEL_PLL2

/* FDCAN 1 clock source */

#define STM32_RCC_D2CCIP1R_FDCANSEL  RCC_D2CCIP1R_FDCANSEL_HSE   /* FDCAN 1 2 clock source */

#define STM32_FDCANCLK               STM32_HSE_FREQUENCY

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 2

/* SDMMC definitions ********************************************************/

/* Init 480kHz, freq = PLL1Q/(2*div)  div =  PLL1Q/(2*freq) */

#define STM32_SDMMC_INIT_CLKDIV     (300 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* 20 MHz Max for now - more reliable on some boards than 25 MHz
 * 20 MHz = PLL1Q/(2*div), div =  PLL1Q/(2*freq), div = 6 = 240 / 40
 */

#if defined(CONFIG_STM32H7_SDMMC_XDMA) || defined(CONFIG_STM32H7_SDMMC_IDMA)
#  define STM32_SDMMC_MMCXFR_CLKDIV   (6 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV   (100 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif
#if defined(CONFIG_STM32H7_SDMMC_XDMA) || defined(CONFIG_STM32H7_SDMMC_IDMA)
#  define STM32_SDMMC_SDXFR_CLKDIV    (6 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV    (100 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE

/* LED definitions ******************************************************************/
/* The board has two, LED_GREEN a Green LED and LED_BLUE a Blue LED,
 * that can be controlled by software.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_BLUE    BOARD_LED1
#define BOARD_LED_GREEN   BOARD_LED2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ----*/

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Alternate function pin selections ************************************************/

#define GPIO_USART1_RX   GPIO_USART1_RX_2   /* PA10 */
#define GPIO_USART1_TX   GPIO_USART1_TX_2   /* PA9  */

#define GPIO_USART2_RX   GPIO_USART2_RX_2   /* PD6  */
#define GPIO_USART2_TX   GPIO_USART2_TX_2   /* PD5  */

#define GPIO_USART3_RX   GPIO_USART3_RX_3   /* PD9  */
#define GPIO_USART3_TX   GPIO_USART3_TX_3   /* PD8  */

#define GPIO_UART4_RX    GPIO_UART4_RX_3    /* PB8  */
#define GPIO_UART4_TX    GPIO_UART4_TX_3    /* PB9  */

#define GPIO_USART6_RX   GPIO_USART6_RX_1   /* PC7  */
#define GPIO_USART6_TX   GPIO_USART6_TX_1   /* PC6  */

#define GPIO_UART7_RX    GPIO_UART7_RX_3    /* PE7  */
#define GPIO_UART7_TX    GPIO_UART7_TX_3    /* PE8  */
#define GPIO_UART7_RTS   GPIO_UART7_RTS_1   /* PE9  */
#define GPIO_UART7_CTS   GPIO_UART7_CTS_1   /* PE10 */

#define GPIO_UART8_RX    GPIO_UART8_RX_1    /* PE0  */
#define GPIO_UART8_TX    GPIO_UART8_TX_1    /* PE1  */


/* CAN
 *
 * CAN1 is routed to transceiver.
 */

#define GPIO_CAN1_RX     GPIO_CAN1_RX_3     /* PD0  */
#define GPIO_CAN1_TX     GPIO_CAN1_TX_3     /* PD1  */

/* SPI
 *
 * SPI1 is MPU6000
 * SPI2 is MAX7456
 * SPI3 is extern with PD4 and PE2 as CS
 * SPI4 is ICM20602
 */

#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_2MHz))

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1               /* PA6  */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_3               /* PD7  */
#define GPIO_SPI1_SCK    ADJ_SLEW_RATE(GPIO_SPI1_SCK_1) /* PA5  */

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1               /* PB14 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1               /* PB15 */
#define GPIO_SPI2_SCK    ADJ_SLEW_RATE(GPIO_SPI2_SCK_4) /* PB13 */

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_1               /* PB4  */
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_4               /* PB5  */
#define GPIO_SPI3_SCK    ADJ_SLEW_RATE(GPIO_SPI3_SCK_1) /* PB3  */

#define GPIO_SPI4_MISO   GPIO_SPI4_MISO_1               /* PE13 */
#define GPIO_SPI4_MOSI   GPIO_SPI4_MOSI_1               /* PE14 */
#define GPIO_SPI4_SCK    ADJ_SLEW_RATE(GPIO_SPI4_SCK_1) /* PE12 */

/* I2C
 *
 *   The optional _GPIO configurations allow the I2C driver to manually
 *   reset the bus to clear stuck slaves.  They match the pin configuration,
 *   but are normally-high GPIOs.
 *
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_1       /* PB6  */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_1       /* PB7  */

#define GPIO_I2C1_SCL_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN6)
#define GPIO_I2C1_SDA_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN7)

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1       /* PB10 */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1       /* PB11 */

#define GPIO_I2C2_SCL_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO                  (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN11)

/* SDMMC1
 *
 *      SDMMC1_D0                          PC8
 *      SDMMC1_D1                          PC9
 *      SDMMC1_D2                          PC10
 *      SDMMC1_D3                          PC11
 *      SDMMC1_CK                          PC12
 *      SDMMC1_CMD                         PD2
 */

// #define GPIO_SDMMC1_D0   GPIO_SDMMC1_D0    /* PC8  */
// #define GPIO_SDMMC1_D1   GPIO_SDMMC1_D1    /* PC9  */
// #define GPIO_SDMMC1_D2   GPIO_SDMMC1_D2    /* PC10 */
// #define GPIO_SDMMC1_D3   GPIO_SDMMC1_D3    /* PC11 */
// #define GPIO_SDMMC1_CK   GPIO_SDMMC1_CK    /* PC12 */
// #define GPIO_SDMMC1_CMD  GPIO_SDMMC1_CMD   /* PD2  */


/* USB
 *
 *      OTG_FS_DM                           PA11
 *      OTG_FS_DP                           PA12
 *      VBUS                                PA9
 */


/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# include "stm32_gpio.h"
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN0)   /* PI0 AUX1 */
# define PROBE_2    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN12)  /* PH12 AUX2 */
# define PROBE_3    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN11)  /* PH11 AUX3 */
# define PROBE_4    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN10)  /* PH10 AUX4 */
# define PROBE_5    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)  /* PD13 AUX5 */
# define PROBE_6    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)  /* PD14 AUX6 */
# define PROBE_7    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN6)   /* PH6  AUX7 */
# define PROBE_8    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN9)   /* PH9  AUX8 */
# define PROBE_9    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)  /* PE11  CAP1 */

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { stm32_configgpio(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { stm32_configgpio(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { stm32_configgpio(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { stm32_configgpio(PROBE_4); } \
		if ((mask)& PROBE_N(5)) { stm32_configgpio(PROBE_5); } \
		if ((mask)& PROBE_N(6)) { stm32_configgpio(PROBE_6); } \
		if ((mask)& PROBE_N(7)) { stm32_configgpio(PROBE_7); } \
		if ((mask)& PROBE_N(8)) { stm32_configgpio(PROBE_8); } \
		if ((mask)& PROBE_N(9)) { stm32_configgpio(PROBE_9); } \
	} while(0)

# define PROBE(n,s)  do {stm32_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif

#endif  /*__NUTTX_CONFIG_MATEKH743SLIM_INCLUDE_BOARD_H  */
