/************************************************************************************
 * nuttx-configs/px4nucleoF767ZI-v1/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: David Sidrane <david_s5@nscdg.com>
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
#ifndef __NUTTX_CONFIG_PX4NUCLEOF767ZI_V1_BOARD_HINCLUDE_BOARD_H
#define __NUTTX_CONFIG_PX4NUCLEOF767ZI_V1_BOARD_HINCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

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
/* The px4nucleoF767ZI-v1  board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock
 *   X2:  32.768 KHz crystal for LSE
 *   X3:  HSE crystal oscillator (not provided)
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 8 MHz from MCO output of ST-LINK
 *   LSE: 32.768 kHz
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
 * PLL_VCO = (8,000,000 / 4) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(9)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 4) * 216)
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
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_48MHZ


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

/* SDMMC dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(118+2)=400 KHz
 */

/* Use the Falling edge of the SDIO_CLK clock to change the edge the
 * data and commands are change on
 */
#define STM32_SDMMC_CLKCR_EDGE STM32_SDMMC_CLKCR_NEGEDGE

#define STM32_SDMMC_INIT_CLKDIV         (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV     (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV     (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */
//TODO #warning "Check Freq for 24mHz"

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV      (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV      (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA Channl/Stream Selections *****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * if we set aside more DMA channels/streams.
 *
 * SDMMC DMA is on DMA2
 *
 * SDMMC1 DMA
 *   DMAMAP_SDMMC1_1 = Channel 4, Stream 3 <- may later be used by SPI DMA
 *   DMAMAP_SDMMC1_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDMMC1  DMAMAP_SDMMC1_1


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
/* The px4nucleoF767ZI-v1 board has numerous LEDs but only three, LD1 a Green LED, LD2 a Blue
 * LED and LD3 a Red LED, that can be controlled by software. The following
 * definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

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

/* Button definitions ***************************************************************/
/* The STM32F7 Discovery supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PI11.  A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)
/* Alternate function pin selections ************************************************/

#define GPIO_USART1_RX   GPIO_USART1_RX_2    /* PB7[CN11-21] CONFLICT w/ BLUE LED*/
#define GPIO_USART1_TX   GPIO_USART1_TX_2    /* PB6[CN12-17] */

#define GPIO_USART2_RX   GPIO_USART2_RX_2   /* PD6[CN11-43]  */
#define GPIO_USART2_TX   GPIO_USART2_TX_2   /* PD5[CN11-41]  */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2  /* PD4[CN11-39]  */
#define GPIO_USART2_CTS  GPIO_USART2_CTS_2  /* PD3[CN11-40]  */

#define GPIO_USART3_RX   GPIO_USART3_RX_3   /* PD9[CN11-69]  */
#define GPIO_USART3_TX   GPIO_USART3_TX_3   /* PD8[CN12-10]  */
#define GPIO_USART3_RTS  GPIO_USART3_RTS_2  /* PD12[CN12-43] */
#define GPIO_USART3_CTS  GPIO_USART3_CTS_2  /* PD11[CN12-45] */
#if !defined(CONFIG_STM32F7_CAN1) && defined(CONFIG_STM32F7_UART4)
#  define GPIO_UART4_RX    GPIO_UART4_RX_4    /* PD0[CN11-57]  */
#  define GPIO_UART4_TX    GPIO_UART4_TX_4    /* PD1[CN11-55]  */
#endif
#define GPIO_USART6_RX   GPIO_USART6_RX_2   /* PG9[CN11-63]  */
#define GPIO_USART6_TX   GPIO_USART6_TX_2   /* PG14[CN12-61] */
#define GPIO_USART6_RTS  GPIO_USART6_RTS_2  /* PG8[CN12-66]  */
#define GPIO_USART6_CTT  GPIO_USART6_CTS_2  /* PG15[CN11-64  */

#define GPIO_UART7_RX    GPIO_UART7_RX_2    /* PF6[CN11-9]  */
#define GPIO_UART7_TX    GPIO_UART7_TX_1    /* PE8[CN12-40]  */

/* USART8:
 *
 * This configurations assume that you are connecting to the Morpho connector
 * with the serial interface with the adaptor's RX on pin CN11 pin 64 and
 * TX on pin CN11 pin 61
 *
 * USART8: has no remap
 *
 *      GPIO_UART7_RX                          PE0[CN12-64]
 *      GPIO_UART7_TX                          PE1[CN11-61]
 */

/* UART RX DMA configurations */

#define DMAMAP_USART1_RX DMAMAP_USART1_RX_2
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_2

/* CAN
 *
 * CAN1 is routed to the Morpho connector.
 * CAN2 is routed to the Morpho connector.
 * CAN3 is routed to the Morpho connector.
 */
/* The 144 pin package is conflicted with UART4 */
#if defined(CONFIG_STM32F7_CAN1) && !defined(CONFIG_STM32F7_UART4)
#  define GPIO_CAN1_RX     GPIO_CAN1_RX_3   /* PD0[CN11-57]  */
#  define GPIO_CAN1_TX     GPIO_CAN1_TX_3   /* PD1[CN11-55]  */
#endif
#define GPIO_CAN2_RX     GPIO_CAN2_RX_1     /* PB12[CN12-16] */
#define GPIO_CAN2_TX     GPIO_CAN2_TX_1     /* PB13[CN12-30] */
#define GPIO_CAN3_RX     GPIO_CAN3_RX_1     /* PA8[CN12-23]  */
#define GPIO_CAN3_TX     GPIO_CAN3_TX_1     /* PA15[CN11-17] */

#if defined(CONFIG_STM32F7_CAN1) && defined(CONFIG_STM32F7_UART4)
#warning "On The 144 pin package CAN1 is conflicted with UART4!"
#endif

/* SPI
 * N.B. 144 pinout limits access to SPI 2
 * There are sensors on SPI1, and SPI4 is connected to the FRAM.
 * BARO is on SPI5 for isolation.
 * SPI6 Reserved
 *
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1   /* PA6[CN12-13]  */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_3   /* PD7[CN11-45]  */
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_3    /* PG11[CN11-79s] */

#define GPIO_SPI4_MISO   GPIO_SPI4_MISO_2   /* PE13[CN12-55]  */
#define GPIO_SPI4_MOSI   GPIO_SPI4_MOSI_1   /* PE6[CN11-62]  */
#define GPIO_SPI4_SCK    GPIO_SPI4_SCK_1    /* PE2[CN11-46]  */

#define GPIO_SPI5_MISO   GPIO_SPI5_MISO_1   /* PF8[CN11-54]  */
#define GPIO_SPI5_MOSI   GPIO_SPI5_MOSI_1   /* PF9[CN11-56]  */
#define GPIO_SPI5_SCK    GPIO_SPI5_SCK_1    /* PF7[CN11-11]  */

#define GPIO_SPI6_MISO   GPIO_SPI6_MISO_1   /* PG12[CN11-65] */
#define GPIO_SPI6_MOSI   GPIO_SPI6_MOSI_3   /* PB5[CN12-29] */
#define GPIO_SPI6_SCK    GPIO_SPI6_SCK_1    /* PG13[CN11-68] */

/* I2C
 *
 *   Each I2C is associated with a U[S]ART
 *   hence the naming I2C2_SDA_UART4 in FMU USAGE spreadsheet
 *
 *   PF1 can not be used on Morpho connector without SB mods.
 *   see PH1/PF1, SP148, SB163 On schematic
 *
 *   I2C3 is not pined out on FMUv5 on 144 pin packages
 *
 *   The optional _GPIO configurations allow the I2C driver to manually
 *   reset the bus to clear stuck slaves.  They match the pin configuration,
 *   but are normally-high GPIOs.
 *
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2       /* PB8[CN12-3]   */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2       /* PB9[CN12-5]   */

#define GPIO_I2C1_SCL_GPIO                  (GPIO_OUTPUT     | \
                                             GPIO_OPENDRAIN  | \
                                             GPIO_SPEED_50MHz| \
                                             GPIO_OUTPUT_SET | \
                                             GPIO_PORTB      | \
                                             GPIO_PIN8)

#define GPIO_I2C1_SDA_GPIO                  (GPIO_OUTPUT     | \
                                             GPIO_OPENDRAIN  | \
                                             GPIO_SPEED_50MHz| \
                                             GPIO_OUTPUT_SET | \
                                             GPIO_PORTB      | \
                                             GPIO_PIN9)

/* PF1 can not be used on Morpho connector without SB mods. */
#if defined(CONFIG_STM32F7_I2C2)
#  warning "PF1 can not be used on Morpho connector without SB mods."
#endif
#define GPIO_I2C2_SCL GPIO_I2C2_SCL_2       /* PF1[CN11-51]  */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_2       /* PF0[CN11-53]  */

#define GPIO_I2C2_SCL_GPIO                  (GPIO_OUTPUT     | \
                                             GPIO_OPENDRAIN  | \
                                             GPIO_SPEED_50MHz| \
                                             GPIO_OUTPUT_SET | \
                                             GPIO_PORTF      | \
                                             GPIO_PIN1)

#define GPIO_I2C2_SDA_GPIO                  (GPIO_OUTPUT     | \
                                             GPIO_OPENDRAIN  | \
                                             GPIO_SPEED_50MHz| \
                                             GPIO_OUTPUT_SET | \
                                             GPIO_PORTF      | \
                                             GPIO_PIN0)

#define GPIO_I2C4_SCL GPIO_I2C4_SCL_2       /* PF14[CN12-50] */
#define GPIO_I2C4_SDA GPIO_I2C4_SDA_2       /* PF15[CN12-60] */

#define GPIO_I2C4_SCL_GPIO                  (GPIO_OUTPUT     | \
                                             GPIO_OPENDRAIN  | \
                                             GPIO_SPEED_50MHz| \
                                             GPIO_OUTPUT_SET | \
                                             GPIO_PORTF      | \
                                             GPIO_PIN14)

#define GPIO_I2C4_SDA_GPIO                  (GPIO_OUTPUT     | \
                                             GPIO_OPENDRAIN  | \
                                             GPIO_SPEED_50MHz| \
                                             GPIO_OUTPUT_SET | \
                                             GPIO_PORTF      | \
                                             GPIO_PIN14)

/* SDMMC1
 *
 *      VDD 3.3                             [CN11-5]
 *      GND                                 [CN11-8]
 *      SDMMC1_CK                           PC12[CN11-3]
 *      SDMMC1_CMD                          PD2[CN11-4]
 *      SDMMC1_D0                           PC8[CN12-2]
 *      SDMMC1_D1                           PC9[CN12-1]
 *      SDMMC1_D2                           PC10[CN11-1]
 *      SDMMC1_D3                           PC11[CN11-2]
 *      GPIO_SDMMC1_NCD                     PG0[CN11-69]
 */

/* USB
 *
 *      OTG_FS_DM                           PA11[CN12-14]
 *      OTG_FS_DP                           PA12[CN12-12]
 *      VBUS                                PA9[CN12-21]
 */


/* The STM32 F7 connects to a SMSC LAN8742A PHY using these pins:
 *
 *   STM32 F7 BOARD        LAN8742A
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PG11     RMII_TX_EN   TXEN
 *   PG13     RMII_TXD0    TXD0
 *   PG14     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   PG2      RMII_RXER    RXER/PHYAD0 -- Not used
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   N/A      NRST         nRST
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 *   N/A      OSC_25M      XTAL1/CLKIN
 *
 * The PHY address is either 0 or 1, depending on the state of PG2 on reset.
 * PG2 is not controlled but appears to result in a PHY address of 0.
 */

#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_2
#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_2
#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_2


/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14) /* PE14[CN12-51] */
# define PROBE_2    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13) /* PE13[CN12-55] */
# define PROBE_3    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11) /* PE11[CN12-56] */
# define PROBE_4    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)  /* PE9[CN12-52]  */
# define PROBE_5    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13) /* PD13[CN12-41] */
# define PROBE_6    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14) /* PD14[CN12-46] */

# define PROBE_INIT(mask) \
    do { \
        if ((mask)& PROBE_N(1)) { stm32_configgpio(PROBE_1); } \
        if ((mask)& PROBE_N(2)) { stm32_configgpio(PROBE_2); } \
        if ((mask)& PROBE_N(3)) { stm32_configgpio(PROBE_3); } \
        if ((mask)& PROBE_N(4)) { stm32_configgpio(PROBE_4); } \
        if ((mask)& PROBE_N(5)) { stm32_configgpio(PROBE_5); } \
        if ((mask)& PROBE_N(6)) { stm32_configgpio(PROBE_6); } \
    } while(0)

# define PROBE(n,s)  do {stm32_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/
#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /*__NUTTX_CONFIG_PX4NUCLEOF767ZI_V1_BOARD_HINCLUDE_BOARD_H  */
