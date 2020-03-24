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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include "board_dma_map.h"

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - 8  MHz Crystal
 * LSE - not installed
 */

#define STM32_BOARD_USEHSE		1
#define STM32_BOARD_XTAL		8000000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

// #define STM32_BOARD_USEHSI		1

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000

/* Main PLL Configuration */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

#define STM32_RCC_PLLI2SCFGR_PLLI2SM RCC_PLLI2SCFGR_PLLI2SM(16)
#define STM32_RCC_PLLI2SCFGR_PLLI2SN RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR RCC_PLLI2SCFGR_PLLI2SR(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SSRC RCC_PLLI2SCFGR_PLLI2SSRC(0) /* HSE or HSI depending on PLLSRC of PLLCFGR*/

#define STM32_RCC_DCKCFGR2_CK48MSEL RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_FMPI2C1SEL RCC_DCKCFGR2_FMPI2C1SEL_APB
#define STM32_RCC_DCKCFGR2_SDIOSEL RCC_DCKCFGR2_SDIOSEL_48MHZ

#define STM32_SYSCLK_FREQUENCY  96000000ul

/* AHB clock (HCLK) is SYSCLK (96MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY      /* Same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/2 (48MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 (see page 112 of reference manual) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (96MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY)

/* Timers driven from APB2 will be PCLK2 since no prescale division */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM2_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (2 * STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (2 * STM32_PCLK2_FREQUENCY)

/* Alternate function pin selections ************************************************/

/* USART2:
 *
 * Used for NSH console.
 *
 *  RXD: PD6    CN9 pin 4
 *  TXD: PD5    CN9 pin 6
 */

#  define GPIO_USART2_RX GPIO_USART2_RX_2
#  define GPIO_USART2_TX GPIO_USART2_TX_2

/* USART6:
 *  RXD: PG9    CN10 pin 16
 *  TXD: PG14   CN10 pin 14
 */

#define GPIO_USART6_RX   GPIO_USART6_RX_2
#define GPIO_USART6_TX   GPIO_USART6_TX_2


// JAKE: TODO: do we need to use DMA?
/* UART RX DMA configurations */

// #define DMAMAP_USART1_RX DMAMAP_USART1_RX_2
// #define DMAMAP_USART6_RX DMAMAP_USART6_RX_2

/* I2C1:
 *  SCL: PB8	CN7 pin2
 *  SDA: PB9	CN7 pin4
 */

#define GPIO_I2C1_SCL    GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA    GPIO_I2C1_SDA_2

/* SPI1:
 *  MISO: PA6 	CN7 pin 12
 *  MOSI: PA7	CN7 pin 14
 *  SCK:  PA5	CN7 pin 10
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

/* CAN1:
 *  RX: PD0		CN9 pin 25
 *  TX: PD1		CN9 pin 27
 */

#define GPIO_CAN1_RX GPIO_CAN1_RX_3
#define GPIO_CAN1_TX GPIO_CAN1_TX_3

// JAKE: TODO: USB STUFF

/* LEDs
 *
 * The NUCLEO-F412ZG board has 3 user leds.
 *  LD1: PB0	GREEN
 *  LD2: PB7	BLUE
 *  LD3: PB14	RED
 */

#define BOARD_NLEDS       3

#define GPIO_LD1 \
(GPIO_PORTB | GPIO_PIN0 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
GPIO_SPEED_50MHz)

#define GPIO_LD2 \
(GPIO_PORTB | GPIO_PIN7 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
GPIO_SPEED_50MHz)

#define GPIO_LD3 \
(GPIO_PORTB | GPIO_PIN14 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PULLUP | \
GPIO_SPEED_50MHz)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning
 *   -------------------  -----------------------
 *   LED_STARTED          NuttX has been started
 *   LED_HEAPALLOCATE     Heap has been allocated
 *   LED_IRQSENABLED      Interrupts enabled
 *   LED_STACKCREATED     Idle stack created
 *   LED_INIRQ            In an interrupt
 *   LED_SIGNAL           In a signal handler
 *   LED_ASSERTION        An assertion failed
 *   LED_PANIC            The system has crashed
 *   LED_IDLE             MCU is is sleep mode
 *
 * Thus if LD2, NuttX has successfully booted and is, apparently, running
 * normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      1
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 3
#define LED_INIRQ        0
#define LED_SIGNAL       0
#define LED_ASSERTION    1
#define LED_PANIC        1

/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
# define PROBE_2  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
# define PROBE_3  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
# define PROBE_4  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
# define PROBE_5  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
# define PROBE_6  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

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
extern "C" {
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

EXTERN void stm32_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
