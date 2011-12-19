/************************************************************************************
 * arch/arm/src/sam3u/sam3u_internal.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_INTERNAL_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#if defined(CONFIG_GPIOA_IRQ) || defined(CONFIG_GPIOB_IRQ) || defined(CONFIG_GPIOC_IRQ)
#  define CONFIG_GPIO_IRQ 1
#else
#  undef CONFIG_GPIO_IRQ
#endif

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_GPIO
#  undef CONFIG_DEBUG_DMA
#endif

/* Bit-encoded input to sam3u_configgpio() ******************************************/

/* 16-bit Encoding:
 * MMCC CII. VPPB BBBB
 */

/* Input/Output mode:
 *
 * MM.. .... .... ....
 */

#define GPIO_MODE_SHIFT            (14)        /* Bits 14-15: GPIO mode */
#define GPIO_MODE_MASK             (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT               (0 << GPIO_MODE_SHIFT) /* Input */
#  define GPIO_OUTPUT              (1 << GPIO_MODE_SHIFT) /* Output */
#  define GPIO_PERIPHA             (2 << GPIO_MODE_SHIFT) /* Controlled by periph A signal */
#  define GPIO_PERIPHB             (3 << GPIO_MODE_SHIFT) /* Controlled by periph B signal */

/* These bits set the configuration of the pin:
 * ..CC C... .... ....
 */

#define GPIO_CFG_SHIFT             (11)        /* Bits 11-13: GPIO configuration bits */
#define GPIO_CFG_MASK              (7 << GPIO_CFG_SHIFT)
#  define GPIO_CFG_DEFAULT         (0 << GPIO_CFG_SHIFT) /* Default, no attribute */
#  define GPIO_CFG_PULLUP          (1 << GPIO_CFG_SHIFT) /* Bit 11: Internal pull-up */
#  define GPIO_CFG_DEGLITCH        (2 << GPIO_CFG_SHIFT) /* Bit 12: Internal glitch filter */
#  define GPIO_CFG_OPENDRAIN       (4 << GPIO_CFG_SHIFT) /* Bit 13: Open drain */

/* Additional interrupt modes:
 * .... .II. .... ....
 */

#define GPIO_INT_SHIFT             (9)        /* Bits 9-10: GPIO configuration bits */
#define GPIO_INT_MASK              (3 << GPIO_INT_SHIFT)
#  define GPIO_INT_LEVEL           (1 << 10)   /* Bit 10: Level detection interrupt */
#  define GPIO_INT_EDGE            (0)         /*        (vs. Edge detection interrupt) */
#  define GPIO_INT_HIGHLEVEL       (1 << 9)    /* Bit 9: High level detection interrupt */
#  define GPIO_INT_LOWLEVEL        (0)         /*        (vs. Low level detection interrupt) */
#  define GPIO_INT_RISING          (1 << 9)    /* Bit 9: Rising edge detection interrupt */
#  define GPIO_INT_FALLING         (0)         /*        (vs. Falling edge detection interrupt) */

/* If the pin is an GPIO output, then this identifies the initial output value:
 * .... .... V... ....
 */

#define GPIO_OUTPUT_SET            (1 << 7)    /* Bit 7: Inital value of output */
#define GPIO_OUTPUT_CLEAR          (0) 

/* This identifies the GPIO port:
 * .... .... .PP. ....
 */

#define GPIO_PORT_SHIFT            (5)         /* Bit 5-6:  Port number */
#define GPIO_PORT_MASK             (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOA           (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOB           (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOC           (2 << GPIO_PORT_SHIFT)

/* This identifies the bit in the port:
 * .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             0        /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK              (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN0                  (0  << GPIO_PIN_SHIFT)
#define GPIO_PIN1                  (1  << GPIO_PIN_SHIFT)
#define GPIO_PIN2                  (2  << GPIO_PIN_SHIFT)
#define GPIO_PIN3                  (3  << GPIO_PIN_SHIFT)
#define GPIO_PIN4                  (4  << GPIO_PIN_SHIFT)
#define GPIO_PIN5                  (5  << GPIO_PIN_SHIFT)
#define GPIO_PIN6                  (6  << GPIO_PIN_SHIFT)
#define GPIO_PIN7                  (7  << GPIO_PIN_SHIFT)
#define GPIO_PIN8                  (8  << GPIO_PIN_SHIFT)
#define GPIO_PIN9                  (9  << GPIO_PIN_SHIFT)
#define GPIO_PIN10                 (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11                 (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12                 (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13                 (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14                 (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15                 (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16                 (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17                 (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18                 (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19                 (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20                 (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21                 (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22                 (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23                 (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24                 (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25                 (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26                 (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27                 (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28                 (28 << GPIO_PIN_SHIFT)
#define GPIO_PIN29                 (29 << GPIO_PIN_SHIFT)
#define GPIO_PIN30                 (30 << GPIO_PIN_SHIFT)
#define GPIO_PIN31                 (31 << GPIO_PIN_SHIFT)

/* GPIO pin definitions *************************************************************/

#define GPIO_ADC0_AD0             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN21)
#define GPIO_ADC0_AD1             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN30)
#define GPIO_ADC0_AD2             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN3)
#define GPIO_ADC0_AD3             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN4)
#define GPIO_ADC0_AD4             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN15)
#define GPIO_ADC0_AD5             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN16)
#define GPIO_ADC0_AD6             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN17)
#define GPIO_ADC0_AD7             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN18)

#define GPIO_CAN_XCVR_RS          (GPIO_OUTPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_OUTPUT_SET|GPIO_PIN23)
#define GPIO_CAN1_XCVR_TXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN27)
#define GPIO_CAN1_XCVR_RXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN26)
#define GPIO_CAN2_XCVR_TXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN29)
#define GPIO_CAN2_XCVR_RXD        (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN28)

#define GPIO_SMC_D0               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN9)  /* Check! */
#define GPIO_SMC_D1               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN10) /* Check! */
#define GPIO_SMC_D2               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN11) /* Check! */
#define GPIO_SMC_D3               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN12) /* Check! */
#define GPIO_SMC_D4               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN13) /* Check! */
#define GPIO_SMC_D5               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN14) /* Check! */
#define GPIO_SMC_D6               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN15) /* Check! */
#define GPIO_SMC_D7               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN16) /* Check! */
#define GPIO_SMC_D8               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN25) /* Check! */
#define GPIO_SMC_D9               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN26) /* Check! */
#define GPIO_SMC_D10              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN27) /* Check! */
#define GPIO_SMC_D11              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN28) /* Check! */
#define GPIO_SMC_D12              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN29) /* Check! */
#define GPIO_SMC_D13              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN30) /* Check! */
#define GPIO_SMC_D14              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|{GPIO_PIN31) /* Check! */
#define GPIO_SMC_D15              (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN6)   /* Check! */
#define GPIO_SMC_NCS0             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN20)
#define GPIO_SMC_NRD              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN19)
#define GPIO_SMC_NWE              (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN23)
#define GPIO_SMC_PSRAM_A0         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN0)  /* Check! */
#define GPIO_SMC_PSRAM_A1         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN1)  /* Check! */
#define GPIO_SMC_PSRAM_A2         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN2)  /* Check! */
#define GPIO_SMC_PSRAM_A3         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN3)  /* Check! */
#define GPIO_SMC_PSRAM_A4         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN4)  /* Check! */
#define GPIO_SMC_PSRAM_A5         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN5)  /* Check! */
#define GPIO_SMC_PSRAM_A6         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN6)  /* Check! */
#define GPIO_SMC_PSRAM_A7         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN7)  /* Check! */
#define GPIO_SMC_PSRAM_A8         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN8)  /* Check! */
#define GPIO_SMC_PSRAM_A9         (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN9)  /* Check! */
#define GPIO_SMC_PSRAM_A10        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN10) /* Check! */
#define GPIO_SMC_PSRAM_A11        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN11) /* Check! */
#define GPIO_SMC_PSRAM_A12        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN24) /* Check! */
#define GPIO_SMC_PSRAM_A13        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN25) /* Check! */
#define GPIO_SMC_PSRAM_A14        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN26) /* Check! */
#define GPIO_SMC_PSRAM_A15        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN27) /* Check! */
#define GPIO_SMC_PSRAM_A16        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN27) /* Check! */
#define GPIO_SMC_PSRAM_A17        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN28) /* Check! */
#define GPIO_SMC_PSRAM_A18        (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|{GPIO_PIN29) /* Check! */
#define GPIO_SMC_PSRAM_NBS0       (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN7)   /* Check! */
#define GPIO_SMC_PSRAM_NBS1       (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|GPIO_PIN15)
#define GPIO_SMC_A1               (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN8)
#define GPIO_SMC_NCS2             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOC|GPIO_PIN16)
#define GPIO_SMC_LCD_RS           (GPIO_PERIPHB|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN8)

#define GPIO_MCI_DAT0             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN5)
#define GPIO_MCI_DAT1             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN6)
#define GPIO_MCI_DAT2             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN7)
#define GPIO_MCI_DAT3             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN8)
#define GPIO_MCI_DAT4             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN28)
#define GPIO_MCI_DAT5             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN29)
#define GPIO_MCI_DAT6             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN30)
#define GPIO_MCI_DAT7             (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOB|GPIO_PIN31)
#define GPIO_MCI_CK               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN3)
#define GPIO_MCI_DA               (GPIO_PERIPHA|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN4)
#define GPIO_MCI_DAT0IN           (GPIO_INPUT|GPIO_CFG_PULLUP|GPIO_PORT_PIOA|GPIO_PIN5)

#define GPIO_PWMC_PWMH0           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN0)
#define GPIO_PWMC_PWML0           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN7)
#define GPIO_PWMC_PWMH1           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN1)
#define GPIO_PWMC_PWML1           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN8)
#define GPIO_PWMC_PWMH2           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN2)
#define GPIO_PWMC_PWML2           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN9)

#define GPIO_SPI0_MISO            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN13)
#define GPIO_SPI0_MOSI            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN14)
#define GPIO_SPI0_SPCK            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN15)
#define GPIO_SPI0_NPCS0           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN16)

#define GPIO_SPI0_NPCS1_1         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN0)
#define GPIO_SPI0_NPCS1_2         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN3)
#define GPIO_SPI0_NPCS1_3         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN19)
#define GPIO_SPI0_NPCS2_1         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN1)
#define GPIO_SPI0_NPCS2_2         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN4)
#define GPIO_SPI0_NPCS2_3         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN14)
#define GPIO_SPI0_NPCS3_1         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN19)
#define GPIO_SPI0_NPCS3_2         (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN5)

#define GPIO_SSC_TD               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN26)
#define GPIO_SSC_TK               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN28)
#define GPIO_SSC_TF               (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN30)

#define GPIO_PCK0                 (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN21)

#define GPIO_TWI_TWD0             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN9)
#define GPIO_TWI_TWCK0            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN10)
#define GPIO_TWI_TWD1             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN24)
#define GPIO_TWI_TWCK1            (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN25)

#define GPIO_UART_TXD             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN12)
#define GPIO_UART_RXD             (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN11)

#define GPIO_USART0_CTS           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN8)
#define GPIO_USART0_DCD           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN11)
#define GPIO_USART0_DSR           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN10)
#define GPIO_USART0_DTR           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN9)
#define GPIO_USART0_RI            (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN12)
#define GPIO_USART0_RTS           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN7)
#define GPIO_USART0_RXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN19)
#define GPIO_USART0_SCK           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN17)
#define GPIO_USART0_TXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN18)

#define GPIO_USART1_CTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN23)
#define GPIO_USART1_RTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN22)
#define GPIO_USART1_RXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN21)
#define GPIO_USART1_SCK           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN24)
#define GPIO_USART1_TXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN20)

#define GPIO_USART2_CTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN22)
#define GPIO_USART2_RTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOB|GPIO_PIN21)
#define GPIO_USART2_RXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN23)
#define GPIO_USART2_SCK           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN25)
#define GPIO_USART2_TXD           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN22)

#define GPIO_USART3_CTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN10)
#define GPIO_USART3_RTS           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN11)
#define GPIO_USART3_RXD           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN13)
#define GPIO_USART3_SCK           (GPIO_PERIPHA|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN19)
#define GPIO_USART3_TXD           (GPIO_PERIPHB|GPIO_CFG_DEFAULT|GPIO_PORT_PIOC|GPIO_PIN12)

#define GPIO_USB_VBUS             (GPIO_INPUT|GPIO_CFG_DEFAULT|GPIO_PORT_PIOA|GPIO_PIN0)

/* DMA ******************************************************************************/

/* Flags used to characterize the desired DMA channel.  The naming convention is that
 * one side is the peripheral and the other is memory (however, the interface could still
 * be used if, for example, both sides were memory although the naming would be awkward)
 */

/* Unchange-able properties of the channel */

#define DMACH_FLAG_FLOWCONTROL                (1 << 0)  /* Bit 0: Channel supports flow control */
#define DMACH_FLAG_FIFOSIZE_SHIFT             (1)       /* Bit 1: Size of DMA FIFO */
#define DMACH_FLAG_FIFOSIZE_MASK              (1 << DMACH_FLAG_FIFOSIZE_SHIFT)
#  define DMACH_FLAG_FIFO_8BYTES              (0 << DMACH_FLAG_FIFOSIZE_SHIFT) /* 8 bytes */
#  define DMACH_FLAG_FIFO_32BYTES             (1 << DMACH_FLAG_FIFOSIZE_SHIFT) /* 32 bytes */

/* Configurable properties of the channel */

#define DMACH_FLAG_BURST_LARGEST              0  /* Largest length AHB burst */
#define DMACH_FLAG_BURST_HALF                 1  /* Half FIFO size */
#define DMACH_FLAG_BURST_SINGLE               2  /* Single AHB access */

#define DMACH_FLAG_FIFOCFG_SHIFT              (2)       /* Bits 2-3: FIFO configuration */
#define DMACH_FLAG_FIFOCFG_MASK               (3 << DMACH_FLAG_FIFOCFG_SHIFT)
#  define DMACH_FLAG_FIFOCFG_LARGEST          (DMACH_FLAG_BURST_LARGEST << DMACH_FLAG_FIFOCFG_SHIFT)
#  define DMACH_FLAG_FIFOCFG_HALF             (DMACH_FLAG_BURST_HALF << DMACH_FLAG_FIFOCFG_SHIFT)
#  define DMACH_FLAG_FIFOCFG_SINGLE           (DMACH_FLAG_BURST_SINGLE << DMACH_FLAG_FIFOCFG_SHIFT)

/* Peripheral endpoint characteristics */

#define DMACH_FLAG_PERIPHPID_SHIFT            (4)       /* Bits 4-7: Peripheral PID */
#define DMACH_FLAG_PERIPHPID_MASK             (15 << DMACH_FLAG_PERIPHPID_SHIFT)
#define DMACH_FLAG_PERIPHH2SEL                (1 << 8)  /* Bits 8: HW handshaking */
#define DMACH_FLAG_PERIPHISPERIPH             (1 << 9)  /* Bits 9: 0=memory; 1=peripheral */
#define DMACH_FLAG_PERIPHWIDTH_SHIFT          (10)      /* Bits 10-11: Peripheral width */
#define DMACH_FLAG_PERIPHWIDTH_MASK           (3 << DMACH_FLAG_PERIPHWIDTH_SHIFT)
#  define DMACH_FLAG_PERIPHWIDTH_8BITS        (0 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 8 bits */
#  define DMACH_FLAG_PERIPHWIDTH_16BITS       (1 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 16 bits */
#  define DMACH_FLAG_PERIPHWIDTH_32BITS       (2 << DMACH_FLAG_PERIPHWIDTH_SHIFT) /* 32 bits */
#define DMACH_FLAG_PERIPHINCREMENT            (1 << 12) /* Bit 12: Autoincrement peripheral address */
#define DMACH_FLAG_PERIPHCHUNKSIZE            (1 << 13) /* Bit 13: Peripheral chunk size */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_1        (0)                        /* Peripheral chunksize = 1 */
#  define DMACH_FLAG_PERIPHCHUNKSIZE_4        DMACH_FLAG_PERIPHCHUNKSIZE /* Peripheral chunksize = 4 */

/* Memory endpoint characteristics */

#define DMACH_FLAG_MEMPID_SHIFT               (14)      /* Bits 14-17: Memory PID */
#define DMACH_FLAG_MEMPID_MASK                (15 << DMACH_FLAG_PERIPHPID_SHIFT)
#define DMACH_FLAG_MEMH2SEL                   (1 << 18) /* Bits 18: HW handshaking */
#define DMACH_FLAG_MEMISPERIPH                (1 << 19) /* Bits 19: 0=memory; 1=peripheral */
#define DMACH_FLAG_MEMWIDTH_SHIFT             (20)      /* Bits 20-21: Memory width */
#define DMACH_FLAG_MEMWIDTH_MASK              (3 << DMACH_FLAG_MEMWIDTH_SHIFT)
#  define DMACH_FLAG_MEMWIDTH_8BITS           (0 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 8 bits */
#  define DMACH_FLAG_MEMWIDTH_16BITS          (1 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 16 bits */
#  define DMACH_FLAG_MEMWIDTH_32BITS          (2 << DMACH_FLAG_MEMWIDTH_SHIFT) /* 32 bits */
#define DMACH_FLAG_MEMINCREMENT               (1 << 22) /* Bit 22: Autoincrement memory address */
#define DMACH_FLAG_MEMCHUNKSIZE               (1 << 22) /* Bit 23: Memory chunk size */
#  define DMACH_FLAG_MEMCHUNKSIZE_1           (0)                     /* Memory chunksize = 1 */
#  define DMACH_FLAG_MEMCHUNKSIZE_4           DMACH_FLAG_MEMCHUNKSIZE /* Memory chunksize = 4 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct sam3u_dmaregs_s
{
  /* Global Registers */

  uint32_t gcfg;    /* DMAC Global Configuration Register */
  uint32_t en;      /* DMAC Enable Register */
  uint32_t sreq;    /* DMAC Software Single Request Register */
  uint32_t creq;    /* DMAC Software Chunk Transfer Request Register */
  uint32_t last;    /* DMAC Software Last Transfer Flag Register */
  uint32_t ebcimr;  /* DMAC Error Mask */
  uint32_t ebcisr;  /* DMAC Error Status */
  uint32_t chsr;    /* DMAC Channel Handler Status Register */

  /* Channel Registers */

  uint32_t saddr;   /* DMAC Channel Source Address Register */
  uint32_t daddr;   /* DMAC Channel Destination Address Register */
  uint32_t dscr;    /* DMAC Channel Descriptor Address Register */
  uint32_t ctrla;   /* DMAC Channel Control A Register */
  uint32_t ctrlb;   /* DMAC Channel Control B Register */
  uint32_t cfg;     /* DMAC Channel Configuration Register */
};
#endif

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

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
 * Name: sam3u_clockconfig
 *
 * Description:
 *   Called to initialize the SAM3U.  This does whatever setup is needed to put the
 *   SoC in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.  (After power-on reset, the sam3u is initiallyrunning on
 *   a 4MHz internal RC clock).  This function also performs other low-level chip
 *   initialization of the chip including EFC, master clock, IRQ and watchdog
 *   configuration.
 *
 ************************************************************************************/

EXTERN void sam3u_clockconfig(void);

/************************************************************************************
 * Name: sam3u_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART done early so that the serial
 *   console is available for debugging very early in the boot sequence.
 *
 ************************************************************************************/

EXTERN void sam3u_lowsetup(void);

/****************************************************************************
 * Name: sam3u_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segements.
 *
 ****************************************************************************/

#ifdef CONFIG_NUTTX_KERNEL
EXTERN void sam3u_userspace(void);
#endif

/****************************************************************************
 * Name: sam3u_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted SAM3U
 *   resources.
 *
 ****************************************************************************/

#ifdef CONFIG_NUTTX_KERNEL
EXTERN void sam3u_mpuinitialize(void);
#else
#  define sam3u_mpuinitialize()
#endif

/****************************************************************************
 * Name: sam3u_mpuheap
 *
 * Description:
 *  Map a heap region.
 *
 ****************************************************************************/

#ifdef CONFIG_NUTTX_KERNEL
EXTERN void sam3u_mpuheap(uintptr_t start, size_t size);
#else
#  define sam3u_mpuheap(start,size)
#endif

/************************************************************************************
 * Name: sam3u_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void sam3u_gpioirqinitialize(void);
#else
#  define sam3u_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: sam3u_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int sam3u_configgpio(uint16_t cfgset);

/************************************************************************************
 * Name: sam3u_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void sam3u_gpiowrite(uint16_t pinset, bool value);

/************************************************************************************
 * Name: sam3u_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool sam3u_gpioread(uint16_t pinset);

/************************************************************************************
 * Name: sam3u_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void sam3u_gpioirq(uint16_t pinset);
#else
#  define sam3u_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: sam3u_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void sam3u_gpioirqenable(int irq);
#else
#  define sam3u_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: sam3u_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void sam3u_gpioirqdisable(int irq);
#else
#  define sam3u_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  sam3u_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
EXTERN int sam3u_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define sam3u_dumpgpio(p,m)
#endif

/****************************************************************************
 * Name: sam3u_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel with
 *   the required FIFO size and flow control capabilities (determined by
 *   dma_flags) then  gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  Howerver, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel if the required FIFO size is available, this function
 *   returns a non-NULL, void* DMA channel handle.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

EXTERN DMA_HANDLE sam3u_dmachannel(uint32_t dmach_flags);

/****************************************************************************
 * Name: sam3u_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam3u_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void sam3u_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: sam3u_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam3u_dmatxsetup() and sam3u_dmatxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

EXTERN int sam3u_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                            size_t nbytes);

/****************************************************************************
 * Name: sam3u_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam3u_dmatxsetup() and sam3u_dmatxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

EXTERN int sam3u_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                            size_t nbytes);

/****************************************************************************
 * Name: sam3u_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

EXTERN int sam3u_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: sam3u_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam3u_dmastop() is called, the DMA channel is
 *   reset and sam3u_dmasetup() must be called before sam3u_dmastart() can be
 *   called again
 *
 ****************************************************************************/

EXTERN void sam3u_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: sam3u_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
EXTERN void sam3u_dmasample(DMA_HANDLE handle, struct sam3u_dmaregs_s *regs);
#else
#  define sam3u_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: sam3u_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
EXTERN void sam3u_dmadump(DMA_HANDLE handle, const struct sam3u_dmaregs_s *regs,
                          const char *msg);
#else
#  define sam3u_dmadump(handle,regs,msg)
#endif

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See include/nuttx/sdio.h */
EXTERN FAR struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- posssible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a 
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect);

/****************************************************************************
 * Name:  sam3u_spicsnumber, sam3u_spiselect, sam3u_spistatus, and
 *        sam3u_spicmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   include:
 *
 *   o sam3u_spicsnumbe and sam3u_spiselect which are helper functions to
 *     manage the board-specific aspects of the unique SAM3U chip select
 *     architecture.
 *   o sam3u_spistatus and sam3u_spicmddata:  Implementations of the status
 *     and cmddata methods of the SPI interface defined by struct spi_ops_
 *     (see include/nuttx/spi.h). All other methods including
 *     up_spiinitialize()) are provided by common SAM3U logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam3u_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam3u_spicsnumber(), sam3u_spiselect() and sam3u_spistatus()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your board
 *      is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam3u_spicmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_SAM3U_SPI
struct spi_dev_s;
enum spi_dev_e;

/****************************************************************************
 * Name: sam3u_spicsnumber
 *
 * Description:
 *   The SAM3U has 4 CS registers for controlling device features.  This
 *   function must be provided by board-specific code.  Given a logical device
 *   ID, this function returns a number from 0 to 3 that identifies one of
 *   these SAM3U CS resources.
 *
 *   If CONFIG_SPI_OWNBUS is not defined and the GPIO is controlled by
 *   sam3u_spiselect, then the same CS register may be used to control
 *   multiple devices.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   On success, a CS number from 0 to 3 is returned; A negated errno may
 *   be returned on a failure.
 *
 ****************************************************************************/

EXTERN int sam3u_spicsnumber(enum spi_dev_e devid);

/****************************************************************************
 * Name: sam3u_spiselect
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   GPIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the GPIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

EXTERN void sam3u_spiselect(enum spi_dev_e devid, bool selected);

/****************************************************************************
 * Name: sam3u_spistatus
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   Bit-encoded SPI status (see include/nuttx/spi.h.
 *
 ****************************************************************************/

EXTERN uint8_t sam3u_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);

/****************************************************************************
 * Name: sam3u_spicmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data.  If CONFIG_SPI_CMDDATA then
 *   this function will be called to different be command and data transfers.
 *
 *   This is often needed, for example, by LCD drivers.  Some LCD hardware
 *   may be configured to use 9-bit data transfers with the 9th bit
 *   indicating command or data.  That same hardware may be configurable,
 *   instead, to use 8-bit data but to require an additional, board-
 *   specific GPIO control to distinguish command and data.  This function
 *   would be needed in that latter case.
 *
 * Input Parameters:
 *   dev - SPI device info
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
EXTERN int sam3u_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif /* CONFIG_SAM3U_SPI */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_INTERNAL_H */
