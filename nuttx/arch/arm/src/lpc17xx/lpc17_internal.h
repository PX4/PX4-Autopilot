/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_internal.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_INTERNAL_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#if defined(CONFIG_LPC17_SPI) || defined(CONFIG_LPC17_SSP0) || defined(CONFIG_LPC17_SSP1)
#  include <nuttx/spi.h>
#endif

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* Bit-encoded input to lpc17_configgpio() ******************************************/

/* Encoding: FFFx MMOV PPPN NNNN
 *
 *   Pin Function:   FFF
 *   Pin Mode bits:  MM
 *   Open drain:     O (output pins)
 *   Initial value:  V (output pins)
 *   Port number:    PPP (0-4)
 *   Pin number:     NNNNN (0-31)
 */
 
/* Pin Function bits: FFF
 * Only meaningful when the GPIO function is GPIO_PIN
 */

#define GPIO_FUNC_SHIFT      (13)       /* Bits 13-15: GPIO mode */
#define GPIO_FUNC_MASK       (7 << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT         (0 << GPIO_FUNC_SHIFT) /* 000 GPIO input pin */
#  define GPIO_INTFE         (1 << GPIO_FUNC_SHIFT) /* 001 GPIO interrupt falling edge */
#  define GPIO_INTRE         (2 << GPIO_FUNC_SHIFT) /* 010 GPIO interrupt rising edge */
#  define GPIO_INTBOTH       (3 << GPIO_FUNC_SHIFT) /* 011 GPIO interrupt both edges */
#  define GPIO_OUTPUT        (4 << GPIO_FUNC_SHIFT) /* 100 GPIO outpout pin */
#  define GPIO_ALT1          (5 << GPIO_FUNC_SHIFT) /* 101 Alternate function 1 */
#  define GPIO_ALT2          (6 << GPIO_FUNC_SHIFT) /* 110 Alternate function 2 */
#  define GPIO_ALT3          (7 << GPIO_FUNC_SHIFT) /* 111 Alternate function 3 */

#define GPIO_EDGE_SHIFT      (13)       /* Bits 13-14: Interrupt edge bits */
#define GPIO_EDGE_MASK       (3 << GPIO_EDGE_SHIFT)

#define GPIO_INOUT_MASK      GPIO_OUTPUT
#define GPIO_FE_MASK         GPIO_INTFE
#define GPIO_RE_MASK         GPIO_INTRE

#define GPIO_ISGPIO(ps)      ((uint16_t(ps) & GPIO_FUNC_MASK) <= GPIO_OUTPUT)
#define GPIO_ISALT(ps)       ((uint16_t(ps) & GPIO_FUNC_MASK) > GPIO_OUTPUT)
#define GPIO_ISINPUT(ps)     (((ps) & GPIO_FUNC_MASK) == GPIO_INPUT)
#define GPIO_ISOUTPUT(ps)    (((ps) & GPIO_FUNC_MASK) == GPIO_OUTPUT)
#define GPIO_ISINORINT(ps)   (((ps) & GPIO_INOUT_MASK) == 0)
#define GPIO_ISOUTORALT(ps)  (((ps) & GPIO_INOUT_MASK) != 0)
#define GPIO_ISINTERRUPT(ps) (GPIO_ISOUTPUT(ps) && !GPIO_ISINPUT(ps))
#define GPIO_ISFE(ps)        (((ps) & GPIO_FE_MASK) != 0)
#define GPIO_ISRE(ps)        (((ps) & GPIO_RE_MASK) != 0)

/* Pin Mode: MM */

#define GPIO_PUMODE_SHIFT    (10)      /* Bits 10-11: Pin pull-up mode */
#define GPIO_PUMODE_MASK     (3 << GPIO_PUMODE_SHIFT)
#  define GPIO_PULLUP        (0 << GPIO_PUMODE_SHIFT) /* Pull-up resistor enabled */
#  define GPIO_REPEATER      (1 << GPIO_PUMODE_SHIFT) /* Repeater mode enabled */
#  define GPIO_FLOAT         (2 << GPIO_PUMODE_SHIFT) /* Neither pull-up nor -down */
#  define GPIO_PULLDN        (3 << GPIO_PUMODE_SHIFT) /* Pull-down resistor enabled */

/* Open drain: O */

#define GPIO_OPEN_DRAIN      (1 << 9)  /* Bit 9:  Open drain mode */

/* Initial value: V */

#define GPIO_VALUE           (1 << 8)  /* Bit 8:  Initial GPIO output value */
#define GPIO_VALUE_ONE       GPIO_VALUE
#define GPIO_VALUE_ZERO      (0)

/* Port number:    PPP (0-4) */

#define GPIO_PORT_SHIFT      (5)         /* Bit 5-7:  Port number */
#define GPIO_PORT_MASK       (7 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0         (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1         (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2         (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3         (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT4         (4 << GPIO_PORT_SHIFT)

#define GPIO_NPORTS          5

/* Pin number:     NNNNN (0-31) */

#define GPIO_PIN_SHIFT       0        /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK        (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN0            (0  << GPIO_PIN_SHIFT)
#define GPIO_PIN1            (1  << GPIO_PIN_SHIFT)
#define GPIO_PIN2            (2  << GPIO_PIN_SHIFT)
#define GPIO_PIN3            (3  << GPIO_PIN_SHIFT)
#define GPIO_PIN4            (4  << GPIO_PIN_SHIFT)
#define GPIO_PIN5            (5  << GPIO_PIN_SHIFT)
#define GPIO_PIN6            (6  << GPIO_PIN_SHIFT)
#define GPIO_PIN7            (7  << GPIO_PIN_SHIFT)
#define GPIO_PIN8            (8  << GPIO_PIN_SHIFT)
#define GPIO_PIN9            (9  << GPIO_PIN_SHIFT)
#define GPIO_PIN10           (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11           (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12           (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13           (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14           (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15           (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16           (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17           (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18           (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19           (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20           (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21           (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22           (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23           (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24           (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25           (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26           (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27           (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28           (28 << GPIO_PIN_SHIFT)
#define GPIO_PIN29           (29 << GPIO_PIN_SHIFT)
#define GPIO_PIN30           (30 << GPIO_PIN_SHIFT)
#define GPIO_PIN31           (31 << GPIO_PIN_SHIFT)

/* GPIO pin definitions *************************************************************/
/* NOTE that functions have a alternate pins that can be selected.  These alternates
 * are identified with a numerica suffix like _1, _2, or _3.  Your board.h file
 * should select the correct alternative for your board by including definitions
 * such as:
 *
 * #define GPIO_UART1_RXD GPIO_UART1_RXD_1
 *
 * (without the suffix)
 */

#define GPIO_CAN1_RD_1     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_UART3_TXD_1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_I2C1_SDA_1    (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_CAN1_TD_1     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_UART3_RXD_1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_I2C1_SCL_1    (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_UART0_TXD     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_AD0p7         (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_UART0_RXD     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN3)
#define GPIO_AD0p6         (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN3)
#define GPIO_I2S_RXCLK_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN4)
#define GPIO_CAN2_RD       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN4)
#define GPIO_CAP2p0        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN4)
#define GPIO_I2S_RXWS_1    (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5)
#define GPIO_CAN2_TD       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5)
#define GPIO_CAP2p1        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5)
#define GPIO_I2S_RXSDA_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_SSP1_SSEL     (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_MAT2p0_1      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_I2S_TXCLK_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN7)
#define GPIO_SSP1_SCK_1    (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN7)
#define GPIO_MAT2p1_1      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN7)
#define GPIO_I2S_TXWS_1    (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN8)
#define GPIO_SSP1_MISO     (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN8)
#define GPIO_MAT2p2        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN8)
#define GPIO_I2S_TXSDA_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN9)
#define GPIO_SSP1_MOSI     (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN9)
#define GPIO_MAT2p3        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN9)
#define GPIO_UART2_TXD_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_I2C2_SDA      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_MAT3p0        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_UART2_RXD_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN11)
#define GPIO_I2C2_SCL      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN11)
#define GPIO_MAT3p1        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN11)
#define GPIO_UART1_TXD_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN15)
#define GPIO_SSP0_SCK_1    (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN15)
#define GPIO_SPI_SCK       (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN15)
#define GPIO_UART1_RXD_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN16)
#define GPIO_SSP0_SSEL_1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN16)
#define GPIO_SPI_SSEL      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN16)
#define GPIO_UART1_CTS_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN17)
#define GPIO_SSP0_MISO_1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN17)
#define GPIO_SPI_MISO      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN17)
#define GPIO_UART1_DCD_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN18)
#define GPIO_SSP0_MOSI_1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN18)
#define GPIO_SPI_MOSI      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN18)
#define GPIO_UART1_DSR_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN19)
#define GPIO_I2C1_SDA_2    (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN19)
#define GPIO_UART1_DTR_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN20)
#define GPIO_I2C1_SCL_2    (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN20)
#define GPIO_UART1_RI_1    (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN21)
#define GPIO_CAN1_RD_2     (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN21)
#define GPIO_UART1_RTS_1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN22)
#define GPIO_CAN1_TD_2     (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN22)
#define GPIO_AD0p0         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN23)
#define GPIO_I2S_RXCLK_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN23)
#define GPIO_CAP3p0        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN23)
#define GPIO_AD0p1         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN24)
#define GPIO_I2S_RXWS_2    (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN24)
#define GPIO_CAP3p1        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN24)
#define GPIO_AD0p2         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN25)
#define GPIO_I2S_RXSDA_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN25)
#define GPIO_UART3_TXD_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN25)
#define GPIO_AD0p3         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN26)
#define GPIO_AOUT          (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN26)
#define GPIO_UART3_RXD_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN26)
#define GPIO_I2C0_SDA      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN27)
#define GPIO_USB_SDA       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN27)
#define GPIO_I2C0_SCL      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN28)
#define GPIO_USB_SCL       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN28)
#define GPIO_USB_DP        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN29)
#define GPIO_USB_DM        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN30)
#define GPIO_ENET_TXD0     (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN0)
#define GPIO_ENET_TXD1     (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN1)
#define GPIO_ENET_TXEN     (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN4)
#define GPIO_ENET_CRS      (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN8)
#define GPIO_ENET_RXD0     (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN9)
#define GPIO_ENET_RXD1     (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN10)
#define GPIO_ENET_RXER     (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN14)
#define GPIO_ENET_REFCLK   (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN15)
#define GPIO_ENET_MDC_1    (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN16)
#define GPIO_ENET_MDIO_1   (GPIO_ALT1 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN17)
#define GPIO_USB_UPLED     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN18)
#define GPIO_PWM1p1_1      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN18)
#define GPIO_CAP1p0        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN18)
#define GPIO_MCPWM_MCOA0   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN19)
#define GPIO_USB_PPWR      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN19)
#define GPIO_CAP1p1        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN19)
#define GPIO_MCPWM_MCI0    (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN20)
#define GPIO_PWM1p2_1      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN20)
#define GPIO_SSP0_SCK_2    (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN20)
#define GPIO_MCPWM_MCABORT (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN21)
#define GPIO_PWM1p3_1      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN21)
#define GPIO_SSP0_SSEL_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN21)
#define GPIO_MCPWM_MCOB0   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN22)
#define GPIO_USB_PWRD      (GPIO_ALT2 | GPIO_PULLDN | GPIO_PORT1 | GPIO_PIN22)
#define GPIO_MAT1p0        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN22)
#define GPIO_MCPWM_MCI1    (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN23)
#define GPIO_PWM1p4_1      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN23)
#define GPIO_SSP0_MISO_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN23)
#define GPIO_MCPWM_MCI2    (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN24)
#define GPIO_PWM1p5_1      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN24)
#define GPIO_SSP0_MOSI_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN24)
#define GPIO_MCPWM_MCOA1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN25)
#define GPIO_MAT1p1        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN25)
#define GPIO_MCPWM_MCOB1   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN26)
#define GPIO_PWM1p6_1      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN26)
#define GPIO_CAP0p0        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN26)
#define GPIO_CLKOUT        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN27)
#define GPIO_USB_OVRCR     (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN27)
#define GPIO_CAP0p1        (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN27)
#define GPIO_MCPWM_MCOA2   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN28)
#define GPIO_PCAP1p0_1     (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN28)
#define GPIO_MAT0p0_1      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN28)
#define GPIO_MCPWM_MCOB2   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN29)
#define GPIO_PCAP1p1       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN29)
#define GPIO_MAT0p1_1      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN29)
#define GPIO_USB_VBUS      (GPIO_ALT2 | GPIO_FLOAT  | GPIO_PORT1 | GPIO_PIN30)
#define GPIO_AD0p4         (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN30)
#define GPIO_SSP1_SCK_2    (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN31)
#define GPIO_AD0p5         (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN31)
#define GPIO_PWM1p1_2      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN0)
#define GPIO_UART1_TXD_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN0)
#define GPIO_PWM1p2_2      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN1)
#define GPIO_UART1_RXD_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN1)
#define GPIO_PWM1p3_2      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN2)
#define GPIO_UART1_CTS_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN2)
#define GPIO_PWM1p4_2      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN3)
#define GPIO_UART1_DCD_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN3)
#define GPIO_PWM1p5_2      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN4)
#define GPIO_UART1_DSR_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN4)
#define GPIO_PWM1p6_2      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN5)
#define GPIO_UART1_DTR_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN5)
#define GPIO_PCAP1p0_2     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN6)
#define GPIO_UART1_RI_2    (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN6)
#define GPIO_CAN2_RD_2     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN7)
#define GPIO_UART1_RTS_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN7)
#define GPIO_CAN2_TD_2     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN8)
#define GPIO_UART2_TXD_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN8)
#define GPIO_ENET_MDC_2    (GPIO_ALT3 | GPIO_FLOAT  | GPIO_PORT2 | GPIO_PIN8)
#define GPIO_USB_CONNECT   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN9)
#define GPIO_UART2_RXD_2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN9)
#define GPIO_ENET_MDIO_2   (GPIO_ALT3 | GPIO_FLOAT  | GPIO_PORT2 | GPIO_PIN9)
#define GPIO_EINT0         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN10)
#define GPIO_NMI           (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN10)
#define GPIO_EINT1         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#define GPIO_I2S_TXCLK_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#define GPIO_PEINT2        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN12)
#define GPIO_I2S_TXWS_2    (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN12)
#define GPIO_EINT3         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN13)
#define GPIO_I2S_TXSDA_2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN13)
#define GPIO_MAT0p0_2      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN25)
#define GPIO_PWM1p2_3      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN25)
#define GPIO_STCLK         (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN26)
#define GPIO_MAT0p1_2      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN26)
#define GPIO_PWM1p3_3      (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN26)
#define GPIO_RXMCLK        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN28)
#define GPIO_MAT2p0_2      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN28)
#define GPIO_UART3_TXD_3   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN28)
#define GPIO_TXMCLK        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN29)
#define GPIO_MAT2p1_2      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN29)
#define GPIO_UART3_RXD_3   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN29)

/************************************************************************************
 * Public Types
 ************************************************************************************/

typedef FAR void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, void *arg, int result);

/* The following is used for sampling DMA registers when CONFIG DEBUG_DMA is selected */

#ifdef CONFIG_DEBUG_DMA
struct lpc17_dmaglobalregs_s
{
  /* Global Registers */

  uint32_t intst;       /* DMA Interrupt Status Register */
  uint32_t inttcst;     /* DMA Interrupt Terminal Count Request Status Register */
  uint32_t interrst;    /* DMA Interrupt Error Status Register */
  uint32_t rawinttcst;  /* DMA Raw Interrupt Terminal Count Status Register */
  uint32_t rawinterrst; /* DMA Raw Error Interrupt Status Register */
  uint32_t enbldchns;   /* DMA Enabled Channel Register */
  uint32_t softbreq;    /* DMA Software Burst Request Register */
  uint32_t softsreq;    /* DMA Software Single Request Register */
  uint32_t softlbreq;   /* DMA Software Last Burst Request Register */
  uint32_t softlsreq;   /* DMA Software Last Single Request Register */
  uint32_t config;      /* DMA Configuration Register */
  uint32_t sync;        /* DMA Synchronization Register */
};

struct lpc17_dmachanregs_s
{
  /* Channel Registers */

  uint32_t srcaddr;  /* DMA Channel Source Address Register */
  uint32_t destaddr; /* DMA Channel Destination Address Register */
  uint32_t lli;      /* DMA Channel Linked List Item Register */
  uint32_t control;  /* DMA Channel Control Register */
  uint32_t config;   /* DMA Channel Configuration Register */
};

struct lpc17_dmaregs_s
{
  /* Global Registers */

  struct lpc17_dmaglobalregs_s gbl;

  /* Channel Registers */

  struct lpc17_dmachanregs_s   ch;
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

/* These tables have global scope only because they are shared between lpc17_gpio.c,
 * lpc17_gpioint.c, and lpc17_gpiodbg.c
 */

#ifdef CONFIG_GPIO_IRQ
EXTERN uint64_t g_intedge0;
EXTERN uint64_t g_intedge2;
#endif

EXTERN const uint32_t g_fiobase[GPIO_NPORTS];
EXTERN const uint32_t g_intbase[GPIO_NPORTS];
EXTERN const uint32_t g_lopinsel[GPIO_NPORTS];
EXTERN const uint32_t g_hipinsel[GPIO_NPORTS];
EXTERN const uint32_t g_lopinmode[GPIO_NPORTS];
EXTERN const uint32_t g_hipinmode[GPIO_NPORTS];
EXTERN const uint32_t g_odmode[GPIO_NPORTS];

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_clockconfig
 *
 * Description:
 *   Called to initialize the LPC17XX.  This does whatever setup is needed to put the
 *   MCU in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.
 *
 ************************************************************************************/

EXTERN void lpc17_clockconfig(void);

/************************************************************************************
 * Name: lpc17_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console UART.  This UART done early so that the serial
 *   console is available for debugging very early in the boot sequence.
 *
 ************************************************************************************/

EXTERN void lpc17_lowsetup(void);

/************************************************************************************
 * Name: lpc17_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirqinitialize(void);
#else
#  define lpc17_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: lpc17_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

EXTERN int lpc17_configgpio(uint16_t cfgset);

/************************************************************************************
 * Name: lpc17_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

EXTERN void lpc17_gpiowrite(uint16_t pinset, bool value);

/************************************************************************************
 * Name: lpc17_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

EXTERN bool lpc17_gpioread(uint16_t pinset);

/************************************************************************************
 * Name: lpc17_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirqenable(int irq);
#else
#  define lpc17_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: lpc17_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_GPIO_IRQ
EXTERN void lpc17_gpioirqdisable(int irq);
#else
#  define lpc17_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  lpc17_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
EXTERN int lpc17_dumpgpio(uint16_t pinset, const char *msg);
#else
#  define lpc17_dumpgpio(p,m)
#endif

/************************************************************************************
 * Name: lpc17_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be required
 *   for most interrupts.  Don't know why... but the LPC1766 Ethernet EMAC
 *   interrupt definitely needs it!
 *
 ************************************************************************************/

EXTERN void lpc17_clrpend(int irq);

/************************************************************************************
 * Name:  lpc17_spi/ssp0/ssp1select, lpc17_spi/ssp0/ssp1status, and
 *        lpc17_spi/ssp0/ssp1cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods 
 *   including up_spiinitialize()) are provided by common LPC17xx logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in lpc17_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide lpc17_spi/ssp0/ssp1select() and lpc17_spi/ssp0/ssp1status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      lpc17_spi/ssp0/ssp1cmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_SPI
EXTERN void  lpc17_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lpc17_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lpc17_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC17_SSP0
EXTERN void  lpc17_ssp0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lpc17_ssp0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lpc17_ssp0cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC17_SSP1
EXTERN void  lpc17_ssp1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lpc17_ssp1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lpc17_ssp1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from ssp0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_SPI
EXTERN void spi_flush(FAR struct spi_dev_s *dev);
#endif

#if defined(CONFIG_LPC17_SSP0) || defined(CONFIG_LPC17_SSP1)
EXTERN void ssp_flush(FAR struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: lpc17_spi/ssp0/1register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD drvier when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s) must
 *   must be implemented.  These functiosn implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_LPC17_SPI
EXTERN int lpc17_spiregister(FAR struct spi_dev_s *dev,
                             spi_mediachange_t callback, void *arg);
#endif

#ifdef CONFIG_LPC17_SSP0
EXTERN int lpc17_ssp0register(FAR struct spi_dev_s *dev,
                              spi_mediachange_t callback, void *arg);
#endif

#ifdef CONFIG_LPC17_SSP1
EXTERN int lpc17_ssp1register(FAR struct spi_dev_s *dev,
                              spi_mediachange_t callback, void *arg);
#endif
#endif

/****************************************************************************
 * Name: lpc17_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
EXTERN void lpc17_dmainitilaize(void);
#endif

/****************************************************************************
 * Name: lpc17_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
EXTERN DMA_HANDLE lpc17_dmachannel(void);
#endif

/****************************************************************************
 * Name: lpc17_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc17_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
EXTERN void lpc17_dmafree(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: lpc17_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
EXTERN int lpc17_dmarxsetup(DMA_HANDLE handle,
                            uint32_t control, uint32_t config,
                            uint32_t srcaddr, uint32_t destaddr,
                            size_t nbytes);
#endif

/****************************************************************************
 * Name: lpc17_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
EXTERN int lpc17_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
#endif

/****************************************************************************
 * Name: lpc17_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc17_dmastop() is called, the DMA channel is
 *   reset and lpc17_dmasetup() must be called before lpc17_dmastart() can be
 *   called again
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
EXTERN void lpc17_dmastop(DMA_HANDLE handle);
#endif

/****************************************************************************
 * Name: lpc17_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void lpc17_dmasample(DMA_HANDLE handle, struct lpc17_dmaregs_s *regs);
#else
#  define lpc17_dmasample(handle,regs)
#endif
#endif

/****************************************************************************
 * Name: lpc17_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_GPDMA
#ifdef CONFIG_DEBUG_DMA
EXTERN void lpc17_dmadump(DMA_HANDLE handle, const struct lpc17_dmaregs_s *regs,
                          const char *msg);
#else
#  define lpc17_dmadump(handle,regs,msg)
#endif
#endif

/****************************************************************************
 * Name: stm32_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_ADC
FAR struct adc_dev_s *lpc17_adcinitialize(void);
#endif

/****************************************************************************
 * Name: lpc17_dacinitialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Returned Value:
 *   Valid dac device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_DAC
EXTERN FAR struct dac_dev_s *lpc17_dacinitialize(void);
#endif

/****************************************************************************
 * Name: lpc17_caninitialize
 *
 * Description:
 *   Initialize the selected can port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple can interfaces)
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

#if defined(CONFIG_CAN) && (defined(CONFIG_LPC17_CAN1) || defined(CONFIG_LPC17_CAN2))
struct can_dev_s;
EXTERN FAR struct can_dev_s *lpc17_caninitialize(int port);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_INTERNAL_H */
