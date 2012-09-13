/************************************************************************************
 * arch/arm/src/lm3s/lm3s_internal.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM3S_LM3S_INTERNAL_H
#define __ARCH_ARM_SRC_LM3S_LM3S_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* The LM3S69xx only supports 8 priority levels.  The hardware priority mechanism
 * will only look at the upper N bits of the 8-bit priority level (where N is 3 for
 * the Stellaris family), so any prioritization must be performed in those bits.
 * The default priority level is set to the middle value
 */

#define NVIC_SYSH_PRIORITY_MIN     0xe0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

/* Bit-encoded input to lm3s_configgpio() *******************************************/

/* Encoding:
 * FFFS SPPP IIIn nnnn nnnn nnnn VPPP PBBB
 *
 * These bits set the primary function of the pin:
 * FFFn nnnn nnnn nnnn nnnn nnnn nnnn nnnn
 */

#define GPIO_FUNC_SHIFT               29                         /* Bit 31-29: GPIO function */
#define GPIO_FUNC_MASK                (7 << GPIO_FUNC_SHIFT)     /* (See table 9-1 in data sheet) */
#define GPIO_FUNC_INPUT               (0 << GPIO_FUNC_SHIFT)     /*   Digital GPIO input */
#define GPIO_FUNC_OUTPUT              (1 << GPIO_FUNC_SHIFT)     /*   Digital GPIO output */
#define GPIO_FUNC_ODINPUT             (2 << GPIO_FUNC_SHIFT)     /*   Open-drain GPIO input */
#define GPIO_FUNC_ODOUTPUT            (3 << GPIO_FUNC_SHIFT)     /*   Open-drain GPIO output */
#define GPIO_FUNC_PFODIO              (4 << GPIO_FUNC_SHIFT)     /*   Open-drain input/output (I2C) */
#define GPIO_FUNC_PFINPUT             (5 << GPIO_FUNC_SHIFT)     /*   Digital input (Timer, CCP) */
#define GPIO_FUNC_PFOUTPUT            (5 << GPIO_FUNC_SHIFT)     /*   Digital output (Timer, PWM, Comparator) */
#define GPIO_FUNC_PFIO                (5 << GPIO_FUNC_SHIFT)     /*   Digital input/output (SSI, UART) */
#define GPIO_FUNC_ANINPUT             (6 << GPIO_FUNC_SHIFT)     /*   Analog input (Comparator) */
#define GPIO_FUNC_INTERRUPT           (7 << GPIO_FUNC_SHIFT)     /*   Interrupt function */
#define GPIO_FUNC_MAX                 GPIO_FUNC_INTERRUPT

/* That primary may be modified by the following options
 * nnnS SPPP nnnn nnnn nnnn nnnn nnnn nnnn
 */

#define GPIO_STRENGTH_SHIFT           27                         /* Bits 28-27: Pad drive strength */
#define GPIO_STRENGTH_MASK            (3 << GPIO_STRENGTH_SHIFT)
#define GPIO_STRENGTH_2MA             (0 << GPIO_STRENGTH_SHIFT) /*   2mA pad drive strength */
#define GPIO_STRENGTH_4MA             (1 << GPIO_STRENGTH_SHIFT) /*   4mA pad drive strength */
#define GPIO_STRENGTH_8MA             (2 << GPIO_STRENGTH_SHIFT) /*   8mA pad drive strength */
#define GPIO_STRENGTH_8MASC           (3 << GPIO_STRENGTH_SHIFT) /*   8mA Pad drive with slew rate control */
#define GPIO_STRENGTH_MAX             GPIO_STRENGTH_8MASC

#define GPIO_PADTYPE_SHIFT            24                         /* Bits 26-24: Pad type */
#define GPIO_PADTYPE_MASK             (7 << GPIO_PADTYPE_SHIFT)
#define GPIO_PADTYPE_STD              (0 << GPIO_PADTYPE_SHIFT)  /*   Push-pull */
#define GPIO_PADTYPE_STDWPU           (1 << GPIO_PADTYPE_SHIFT)  /*   Push-pull with weak pull-up */
#define GPIO_PADTYPE_STDWPD           (2 << GPIO_PADTYPE_SHIFT)  /*   Push-pull with weak pull-down */
#define GPIO_PADTYPE_OD               (3 << GPIO_PADTYPE_SHIFT)  /*   Open-drain */
#define GPIO_PADTYPE_ODWPU            (4 << GPIO_PADTYPE_SHIFT)  /*   Open-drain with weak pull-up */
#define GPIO_PADTYPE_ODWPD            (5 << GPIO_PADTYPE_SHIFT)  /*   Open-drain with weak pull-down */
#define GPIO_PADTYPE_ANALOG           (6 << GPIO_PADTYPE_SHIFT)  /*   Analog comparator */

/* If the pin is an interrupt, then the following options apply
 * nnnn nnnn IIIn nnnn nnnn nnnn nnnn nnnn
 */

#define GPIO_INT_SHIFT                21                         /* Bits 23-21: Interrupt type */
#define GPIO_INT_MASK                 (7 << GPIO_INT_SHIFT)
#define GPIO_INT_FALLINGEDGE          (0 << GPIO_INT_SHIFT)      /*   Interrupt on falling edge */
#define GPIO_INT_RISINGEDGE           (1 << GPIO_INT_SHIFT)      /*   Interrupt on rising edge */
#define GPIO_INT_BOTHEDGES            (2 << GPIO_INT_SHIFT)      /*   Interrupt on both edges */
#define GPIO_INT_LOWLEVEL             (3 << GPIO_INT_SHIFT)      /*   Interrupt on low level */
#define GPIO_INT_HIGHLEVEL            (4 << GPIO_INT_SHIFT)      /*   Interrupt on high level */

/* If the pin is an GPIO digital output, then this identifies the initial output value:
 * nnnn nnnn nnnn nnnn nnnn nnnn Vnnn nnnn
 */

#define GPIO_VALUE_SHIFT              7                          /* Bit 7: If output, inital value of output */
#define GPIO_VALUE_MASK               (1 << GPIO_VALUE_SHIFT)
#define GPIO_VALUE_ZERO               (0 << GPIO_VALUE_SHIFT)    /*   Initial value is zero */
#define GPIO_VALUE_ONE                (1 << GPIO_VALUE_SHIFT)    /*   Initial value is one */

/* This identifies the GPIO port
 * nnnn nnnn nnnn nnnn nnnn nnnn nPPP Pnnn
 */

#define GPIO_PORT_SHIFT               3                          /* Bit 3-6:  Port number */
#define GPIO_PORT_MASK                (15 << GPIO_PORT_SHIFT)
#define GPIO_PORTA                    (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#define GPIO_PORTB                    (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#define GPIO_PORTC                    (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#define GPIO_PORTD                    (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#define GPIO_PORTE                    (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#define GPIO_PORTF                    (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#define GPIO_PORTG                    (6 << GPIO_PORT_SHIFT)     /*   GPIOG */
#define GPIO_PORTH                    (7 << GPIO_PORT_SHIFT)     /*   GPIOH */
#define GPIO_PORTJ                    (8 << GPIO_PORT_SHIFT)     /*   GPIOJ */

/* This identifies the bit in the port:
 * nnnn nnnn nnnn nnnn nnnn nnnn nnnn nBBB
 */

#define GPIO_NUMBER_SHIFT             0                           /* Bits 0-2: GPIO number: 0-7 */
#define GPIO_NUMBER_MASK              (7 << GPIO_NUMBER_SHIFT)

/* The following lists the input value to lm3s_configgpio to setup the alternate,
 * hardware function for each pin.
 */

#if defined(CONFIG_ARCH_CHIP_LM3S6918)
#  define GPIO_UART0_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTA | 0)  /* PA0: UART 0 receive (U0Rx) */
#  define GPIO_UART0_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 1) /* PA1: UART 0 transmit (U0Tx) */
#  define GPIO_SSI0_CLK    (GPIO_FUNC_PFIO | GPIO_PORTA | 2)     /* PA2: SSI0 clock (SSI0Clk) */
#  define GPIO_SSI0_FSS    (GPIO_FUNC_PFIO | GPIO_PORTA | 3)     /* PA3: SSI0 frame (SSI0Fss) */
#  define GPIO_SSI0_RX     (GPIO_FUNC_PFINPUT | GPIO_PORTA | 4)  /* PA4: SSI0 receive (SSI0Rx) */
#  define GPIO_SSI0_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 5) /* PA5: SSI0 transmit (SSI0Tx) */
#  define GPIO_TMR1_CCP    (GPIO_FUNC_PFIO | GPIO_PORTA | 6)     /* PA6: Capture/Compare/PWM1 (CCP1) */
#  define GPIO_I2C1_SDA    (GPIO_FUNC_PFODIO | GPIO_PORTA | 7)   /* PA7: I2C1 data (I2C1SDA) */
#  define GPIO_TMR0_CCP    (GPIO_FUNC_PFIO | GPIO_PORTB | 0)     /* PB0: Capture/Compare/PWM0 (CCP0) */
#  define GPIO_TMR2_CCP    (GPIO_FUNC_PFIO | GPIO_PORTB | 1)     /* PB1: Capture/Compare/PWM2 (CCP2) */
#  define GPIO_I2C0_SCL    (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 2) /* PB2: I2C0 clock (I2C0SCL) */
#  define GPIO_I2C0_SDA    (GPIO_FUNC_PFODIO | GPIO_PORTB | 3)   /* PB3: I2C0 data (I2C0SDA) */
#  define GPIO_CMP0_NIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 4)  /* PB4: Analog comparator 0 negative input (C0-) */
#  define GPIO_CMP1_NIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 5)  /* PB5: Analog comparator 1 negative input (C1-) */
#  define GPIO_CMP0_PIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 6)  /* PB6: Analog comparator 0 positive input (C0+) */
#  define GPIO_JTAG_TRST   (GPIO_FUNC_PFINPUT | GPIO_PORTB | 7)  /* PB7: JTAG ~TRST */
#  define GPIO_JTAG_TCK    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 0)  /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_SWCLK  (GPIO_FUNC_PFINPUT | GPIO_PORTC | 0)  /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_TMS    (GPIO_FUNC_PFIO | GPIO_PORTC | 1)     /* PC1: JTAG TMS */
#  define GPIO_JTAG_SWDIO  (GPIO_FUNC_PFIO | GPIO_PORTC | 1)     /* PC1: JTAG SWDIO */
#  define GPIO_JTAG_TDI    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 2)  /* PC2: JTAG TDI */
#  define GPIO_JTAG_TDO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG TDO */
#  define GPIO_JTAG_SWO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG SWO */
#  define GPIO_TMR5_CCP    (GPIO_FUNC_PFIO | GPIO_PORTC | 4)     /* PC4: Capture/Compare/PWM5 (CCP5) */
#  define GPIO_CMP1_PIN    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 5)  /* PC5: Analog comparator 1 positive input (C1+) */
#  define GPIO_CMP0_OUT    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 5) /* PC5: Analog comparator 0 output (C0o) */
#  define GPIO_TMR3_CCP    (GPIO_FUNC_PFIO | GPIO_PORTC | 6)     /* PC6: Capture/Compare/PWM3 (CCP3) */
#  define GPIO_TMR4_CCP    (GPIO_FUNC_PFIO | GPIO_PORTC | 7)     /* PC7: Capture/Compare/PWM4 (CCP4) */
#  define GPIO_UART1_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTD | 2)  /* PD2: UART 1 receive (U1Rx) */
#  define GPIO_UART1_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 3) /* PD3: UART 1 transmit (U1Tx) */
#  define GPIO_SSI1_CLK    (GPIO_FUNC_PFIO | GPIO_PORTE | 0)     /* PE0: SSI1 clock (SSI1Clk) */
#  define GPIO_SSI1_FSS    (GPIO_FUNC_PFIO | GPIO_PORTE | 1)     /* PE1: SSI1 frame (SSI1Fss) */
#  define GPIO_SSI1_RX     (GPIO_FUNC_PFINPUT | GPIO_PORTE | 2)  /* PE2: SSI1 receive (SSI1Rx) */
#  define GPIO_SSI1_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 3) /* PE3: SSI1 transmit (SSI1Tx) */
#  define GPIO_ETHPHY_LED1 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 2) /* PF2: LED1 */
#  define GPIO_ETHPHY_LED0 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 3) /* PF3: LED0 */
#  define GPIO_I2C1_SCL    (GPIO_FUNC_PFOUTPUT | GPIO_PORTG | 0) /* PG0: I2C1 clock (I2C1SCL) */
#elif defined(CONFIG_ARCH_CHIP_LM3S6432)
#  define GPIO_UART0_RX    (GPIO_FUNC_PFINPUT  | GPIO_PORTA | 0) /* PA0: UART 0 receive (U0Rx) */
#  define GPIO_UART0_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 1) /* PA1: UART 0 transmit (U0Tx) */
#  define GPIO_SSI0_CLK    (GPIO_FUNC_PFIO     | GPIO_PORTA | 2) /* PA2: SSI0 clock (SSI0Clk) */
#  define GPIO_SSI0_FSS    (GPIO_FUNC_PFIO     | GPIO_PORTA | 3) /* PA3: SSI0 frame (SSI0Fss) */
#  define GPIO_SSI0_RX     (GPIO_FUNC_PFINPUT  | GPIO_PORTA | 4) /* PA4: SSI0 receive (SSI0Rx) */
#  define GPIO_SSI0_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 5) /* PA5: SSI0 transmit (SSI0Tx) */
#  define GPIO_I2C0_SCL    (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 2) /* PB2: I2C0 clock (I2C0SCL) */
#  define GPIO_I2C0_SDA    (GPIO_FUNC_PFODIO   | GPIO_PORTB | 3) /* PB3: I2C0 data (I2C0SDA) */
#  define GPIO_CMP0_NIN    (GPIO_FUNC_PFINPUT  | GPIO_PORTB | 4) /* PB4: Analog comparator 0 negative input (C0-) */
#  define GPIO_CMP1_NIN    (GPIO_FUNC_PFINPUT  | GPIO_PORTB | 5) /* PB5: Analog comparator 1 negative input (C1-) */
#  define GPIO_CMP0_PIN    (GPIO_FUNC_PFINPUT  | GPIO_PORTB | 6) /* PB6: Analog comparator 0 positive input (C0+) */
#  define GPIO_JTAG_TRST   (GPIO_FUNC_PFINPUT  | GPIO_PORTB | 7) /* PB7: JTAG ~TRST */
#  define GPIO_JTAG_TCK    (GPIO_FUNC_PFINPUT  | GPIO_PORTC | 0) /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_SWCLK  (GPIO_FUNC_PFINPUT  | GPIO_PORTC | 0) /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_TMS    (GPIO_FUNC_PFIO     | GPIO_PORTC | 1) /* PC1: JTAG TMS */
#  define GPIO_JTAG_SWDIO  (GPIO_FUNC_PFIO     | GPIO_PORTC | 1) /* PC1: JTAG SWDIO */
#  define GPIO_JTAG_TDI    (GPIO_FUNC_PFINPUT  | GPIO_PORTC | 2) /* PC2: JTAG TDI */
#  define GPIO_JTAG_TDO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG TDO */
#  define GPIO_JTAG_SWO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG SWO */
#  define GPIO_CMP1_PIN    (GPIO_FUNC_PFINPUT  | GPIO_PORTC | 5) /* PC5: Analog comparator 1 positive input (C1+) */
#  define GPIO_CMP0_OUT    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 5) /* PC5: Analog comparator 0 output (C0o) */
#  define GPIO_PWM0_0      (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 0) /* PD0: PWM Generator 0, PWM0 */
#  define GPIO_PWM0_1      (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 1) /* PD1: PWM Generator 0, PWM1 */
#  define GPIO_UART1_RX    (GPIO_FUNC_PFINPUT  | GPIO_PORTD | 2) /* PD2: UART 1 receive (U1Rx) */
#  define GPIO_UART1_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 3) /* PD3: UART 1 transmit (U1Tx) */
#  define GPIO_PWM_FAULT   (GPIO_FUNC_PFINPUT  | GPIO_PORTD | 6) /* PD6: PWM Fault */
#  define GPIO_TMR1_CCP    (GPIO_FUNC_PFIO     | GPIO_PORTD | 7) /* PD7: Capture/Compare/TMR1 (CCP1) */
#  define GPIO_ETHPHY_LED1 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 2) /* PF2: LED1 */
#  define GPIO_ETHPHY_LED0 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 3) /* PF3: LED0 */
#elif defined(CONFIG_ARCH_CHIP_LM3S6965)
#  define GPIO_UART0_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTA | 0)  /* PA0: UART 0 receive (U0Rx) */
#  define GPIO_UART0_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 1) /* PA1: UART 0 transmit (U0Tx) */
#  define GPIO_SSI0_CLK    (GPIO_FUNC_PFIO | GPIO_PORTA | 2)     /* PA2: SSI0 clock (SSI0Clk) */
#  define GPIO_SSI0_FSS    (GPIO_FUNC_PFIO | GPIO_PORTA | 3)     /* PA3: SSI0 frame (SSI0Fss) */
#  define GPIO_SSI0_RX     (GPIO_FUNC_PFINPUT | GPIO_PORTA | 4)  /* PA4: SSI0 receive (SSI0Rx) */
#  define GPIO_SSI0_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 5) /* PA5: SSI0 transmit (SSI0Tx) */
#  define GPIO_I2C1_SCL    (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 6) /* PA6: I2C1 clock (I2C1SCL) */
#  define GPIO_I2C1_SDA    (GPIO_FUNC_PFODIO | GPIO_PORTA | 7)   /* PA7: I2C1 data (I2C1SDA) */
#  define GPIO_PWM1_2      (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 0) /* PB0: PWM Generator 1, PWM2 */
#  define GPIO_PWM1_3      (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 1) /* PB1: PWM Generator 1, PWM3 */
#  define GPIO_I2C0_SCL    (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 2) /* PB2: I2C0 clock (I2C0SCL) */
#  define GPIO_I2C0_SDA    (GPIO_FUNC_PFODIO | GPIO_PORTB | 3)   /* PB3: I2C0 data (I2C0SDA) */
#  define GPIO_CMP0_NIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 4)  /* PB4: Analog comparator 0 negative input (C0-) */
#  define GPIO_CMP1_NIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 5)  /* PB5: Analog comparator 1 negative input (C1-) */
#  define GPIO_CMP0_PIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 6)  /* PB6: Analog comparator 0 positive input (C0+) */
#  define GPIO_JTAG_TRST   (GPIO_FUNC_PFINPUT | GPIO_PORTB | 7)  /* PB7: JTAG ~TRST */
#  define GPIO_JTAG_TCK    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 0)  /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_SWCLK  (GPIO_FUNC_PFINPUT | GPIO_PORTC | 0)  /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_TMS    (GPIO_FUNC_PFIO | GPIO_PORTC | 1)     /* PC1: JTAG TMS */
#  define GPIO_JTAG_SWDIO  (GPIO_FUNC_PFIO | GPIO_PORTC | 1)     /* PC1: JTAG SWDIO */
#  define GPIO_JTAG_TDI    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 2)  /* PC2: JTAG TDI */
#  define GPIO_JTAG_TDO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG TDO */
#  define GPIO_JTAG_SWO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG SWO */
#  define GPIO_QEI0_PHA    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 4)  /* PC4: QEI module 0 phase A. */
#  define GPIO_CMP1_PIN    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 5)  /* PC5: Analog comparator 1 positive input (C1+) */
#  define GPIO_CMP0_OUT    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 5) /* PC5: Analog comparator 0 output (C0o) */
#  define GPIO_TMR3_CCP    (GPIO_FUNC_PFIO | GPIO_PORTC | 6)     /* PC6: Capture/Compare/PWM3 (CCP3) */
#  define GPIO_QEI0_PHB    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 7)  /* PC7: QEI module 0 phase B. */
#  define GPIO_QEI0_IDX    (GPIO_FUNC_PFINPUT | GPIO_PORTD | 1)  /* PD0: QEI module 0 index. ) */
#  define GPIO_PWM0_1      (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 1) /* PD1: PWM Generator 0, PWM1 */
#  define GPIO_UART1_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTD | 2)  /* PD2: UART 1 receive (U1Rx) */
#  define GPIO_UART1_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 3) /* PD3: UART 1 transmit (U1Tx) */
#  define GPIO_TMR0_CCP    (GPIO_FUNC_PFIO | GPIO_PORTD | 4)     /* PC4: Capture/Compare/PWM0 (CCP0) */
#  define GPIO_TMR2_CCP    (GPIO_FUNC_PFIO | GPIO_PORTD | 5)     /* PC5: Capture/Compare/PWM2 (CCP2) */
#  define GPIO_PWM_FAULT   (GPIO_FUNC_PFINPUT | GPIO_PORTD | 6)  /* PC5: PWM Fault */
#  define GPIO_TMR1_CCP    (GPIO_FUNC_PFIO | GPIO_PORTD | 7)     /* PC5: Capture/Compare/TMR1 (CCP1) */
#  define GPIO_SSI1_CLK    (GPIO_FUNC_PFIO | GPIO_PORTE | 0)     /* PE0: SSI1 clock (SSI1Clk) */
#  define GPIO_SSI1_FSS    (GPIO_FUNC_PFIO | GPIO_PORTE | 1)     /* PE1: SSI1 frame (SSI1Fss) */
#  define GPIO_SSI1_RX     (GPIO_FUNC_PFINPUT | GPIO_PORTE | 2)  /* PE2: SSI1 receive (SSI1Rx) */
#  define GPIO_SSI1_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 3) /* PE3: SSI1 transmit (SSI1Tx) */
#  define GPIO_PWM2_4      (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 0) /* PE0: PWM Generator 2, PWM4 */
#  define GPIO_PWM2_5      (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 1) /* PE1: PWM Generator 1, PWM5 */
#  define GPIO_QEI1_PHB    (GPIO_FUNC_PFINPUT | GPIO_PORTE | 2)  /* PE2: QEI module 1 phase B. */
#  define GPIO_QEI1_PHA    (GPIO_FUNC_PFINPUT | GPIO_PORTE | 3)  /* PE3: QEI module 1 phase A. */
#  define GPIO_PWM0_0      (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 0) /* PE4: PWM Generator 0, PWM0 */
#  define GPIO_QEI1_IDX    (GPIO_FUNC_PFINPUT | GPIO_PORTE | 1)  /* PD0: QEI module 1 index. ) */
#  define GPIO_ETHPHY_LED1 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 2) /* PF2: LED1 */
#  define GPIO_ETHPHY_LED0 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 3) /* PF3: LED0 */
#  define GPIO_UART2_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTG | 0)  /* PA0: UART 0 receive (UGRx) */
#  define GPIO_UART2_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTG | 1) /* PA1: UART 0 transmit (UGTx) */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96) 
#  define GPIO_UART0_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTA | 0)  /* PA0: UART 0 receive (U0Rx) */
#  define GPIO_UART0_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 1) /* PA1: UART 0 transmit (U0Tx) */
#  define GPIO_SSI0_CLK    (GPIO_FUNC_PFIO | GPIO_PORTA | 2)     /* PA2: SSI0 clock (SSI0Clk) */
#  define GPIO_SSI0_FSS    (GPIO_FUNC_PFIO | GPIO_PORTA | 3)     /* PA3: SSI0 frame (SSI0Fss) */
#  define GPIO_SSI0_RX     (GPIO_FUNC_PFINPUT | GPIO_PORTA | 4)  /* PA4: SSI0 receive (SSI0Rx) */
#  define GPIO_SSI0_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTA | 5) /* PA5: SSI0 transmit (SSI0Tx) */
#  define GPIO_I2C1_SCL    (GPIO_FUNC_PFODIO | GPIO_PORTA | 6)   /* PA6: I2C1 clock (I2C1SCL) */
#  define GPIO_I2C1_SDA    (GPIO_FUNC_PFODIO | GPIO_PORTA | 7)   /* PA7: I2C1 data (I2C1SDA) */
#  define GPIO_PWM1_2      (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 0) /* PB0: PWM Generator 1, PWM2 */
#  define GPIO_PWM1_3      (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | 1) /* PB1: PWM Generator 1, PWM3 */
#  define GPIO_I2C0_SCL    (GPIO_FUNC_PFODIO  | GPIO_PORTB | 2)  /* PB2: I2C0 clock (I2C0SCL) */
#  define GPIO_I2C0_SDA    (GPIO_FUNC_PFODIO | GPIO_PORTB | 3)   /* PB3: I2C0 data (I2C0SDA) */
#  define GPIO_CMP0_NIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 4)  /* PB4: Analog comparator 0 negative input (C0-) */
#  define GPIO_CMP1_NIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 5)  /* PB5: Analog comparator 1 negative input (C1-) */
#  define GPIO_CMP0_PIN    (GPIO_FUNC_PFINPUT | GPIO_PORTB | 6)  /* PB6: Analog comparator 0 positive input (C0+) */
#  define GPIO_JTAG_TRST   (GPIO_FUNC_PFINPUT | GPIO_PORTB | 7)  /* PB7: JTAG ~TRST */
#  define GPIO_JTAG_TCK    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 0)  /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_SWCLK  (GPIO_FUNC_PFINPUT | GPIO_PORTC | 0)  /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_TMS    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 1)  /* PC1: JTAG TMS */
#  define GPIO_JTAG_SWDIO  (GPIO_FUNC_PFIO | GPIO_PORTC | 1)     /* PC1: JTAG SWDIO */
#  define GPIO_JTAG_TDI    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 2)  /* PC2: JTAG TDI */
#  define GPIO_JTAG_TDO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG TDO */
#  define GPIO_JTAG_SWO    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 3) /* PC3: JTAG SWO */
#  define GPIO_QEI0_PHA    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 4)  /* PC4: QEI module 0 phase A. */
#  define GPIO_CMP1_PIN    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 5)  /* PC5: Analog comparator 1 positive input (C1+) */
#  define GPIO_CMP0_OUT    (GPIO_FUNC_PFOUTPUT | GPIO_PORTC | 5) /* PC5: Analog comparator 0 output (C0o) */
#  define GPIO_TMR3_CCP    (GPIO_FUNC_PFIO | GPIO_PORTC | 6)     /* PC6: Capture/Compare/PWM3 (CCP3) */
#  define GPIO_QEI0_PHB    (GPIO_FUNC_PFINPUT | GPIO_PORTC | 7)  /* PC7: QEI module 0 phase B. */
#  define GPIO_QEI0_IDX    (GPIO_FUNC_PFINPUT | GPIO_PORTD | 1)  /* PD0: QEI module 0 index. ) */
#  define GPIO_PWM0_1      (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 1) /* PD1: PWM Generator 0, PWM1 */
#  define GPIO_UART1_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTD | 2)  /* PD2: UART 1 receive (U1Rx) */
#  define GPIO_UART1_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | 3) /* PD3: UART 1 transmit (U1Tx) */
#  define GPIO_TMR0_CCP    (GPIO_FUNC_PFIO | GPIO_PORTD | 4)     /* PC4: Capture/Compare/PWM0 (CCP0) */
#  define GPIO_TMR2_CCP    (GPIO_FUNC_PFIO | GPIO_PORTD | 5)     /* PC5: Capture/Compare/PWM2 (CCP2) */
#  define GPIO_PWM_FAULT   (GPIO_FUNC_PFINPUT | GPIO_PORTD | 6)  /* PC5: PWM Fault */
#  define GPIO_TMR1_CCP    (GPIO_FUNC_PFIO | GPIO_PORTD | 7)     /* PC5: Capture/Compare/TMR1 (CCP1) */
#  define GPIO_SSI1_CLK    (GPIO_FUNC_PFIO | GPIO_PORTE | 0)     /* PE0: SSI1 clock (SSI1Clk) */
#  define GPIO_SSI1_FSS    (GPIO_FUNC_PFIO | GPIO_PORTE | 1)     /* PE1: SSI1 frame (SSI1Fss) */
#  define GPIO_SSI1_RX     (GPIO_FUNC_PFINPUT | GPIO_PORTE | 2)  /* PE2: SSI1 receive (SSI1Rx) */
#  define GPIO_SSI1_TX     (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 3) /* PE3: SSI1 transmit (SSI1Tx) */
#  define GPIO_PWM2_4      (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 0) /* PE0: PWM Generator 2, PWM4 */
#  define GPIO_PWM2_5      (GPIO_FUNC_PFOUTPUT | GPIO_PORTE | 1) /* PE1: PWM Generator 1, PWM5 */
#  define GPIO_QEI1_PHB    (GPIO_FUNC_PFINPUT | GPIO_PORTE | 2)  /* PE2: QEI module 1 phase B. */
#  define GPIO_QEI1_PHA    (GPIO_FUNC_PFINPUT | GPIO_PORTE | 3)  /* PE3: QEI module 1 phase A. */
#  define GPIO_PWM0_0      (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 0) /* PE4: PWM Generator 0, PWM0 */
#  define GPIO_QEI1_IDX    (GPIO_FUNC_PFINPUT | GPIO_PORTE | 1)  /* PD0: QEI module 1 index. ) */
#  define GPIO_ETHPHY_LED1 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 2) /* PF2: LED1 */
#  define GPIO_ETHPHY_LED0 (GPIO_FUNC_PFOUTPUT | GPIO_PORTF | 3) /* PF3: LED0 */
#  define GPIO_UART2_RX    (GPIO_FUNC_PFINPUT | GPIO_PORTG | 0)  /* PA0: UART 0 receive (UGRx) */
#  define GPIO_UART2_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTG | 1) /* PA1: UART 0 transmit (UGTx) */


#elif defined(CONFIG_ARCH_CHIP_LM3S8962)
#  define GPIO_UART0_RX    (GPIO_FUNC_PFINPUT   | GPIO_PORTA | 0)       /* PA0: UART 0 receive (U0Rx) */
#  define GPIO_UART0_TX    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTA | 1)       /* PA1: UART 0 transmit (U0Tx) */
#  define GPIO_SSI0_CLK    (GPIO_FUNC_PFIO      | GPIO_PORTA | 2)       /* PA2: SSI0 clock (SSI0Clk) */
#  define GPIO_SSI0_FSS    (GPIO_FUNC_PFIO      | GPIO_PORTA | 3)       /* PA3: SSI0 frame (SSI0Fss) */
#  define GPIO_SSI0_RX     (GPIO_FUNC_PFINPUT   | GPIO_PORTA | 4)       /* PA4: SSI0 receive (SSI0Rx) */
#  define GPIO_SSI0_TX     (GPIO_FUNC_PFOUTPUT  | GPIO_PORTA | 5)       /* PA5: SSI0 transmit (SSI0Tx) */
#  define GPIO_TMR1_CCP    (GPIO_FUNC_PFIO      | GPIO_PORTA | 6)       /* PA6: Capture/Compare/PWM0 (CCP1) */
#  define GPIO_PWM1_2      (GPIO_FUNC_PFOUTPUT  | GPIO_PORTB | 0)       /* PB0: PWM Generator 1, PWM2 */
#  define GPIO_PWM1_3      (GPIO_FUNC_PFOUTPUT  | GPIO_PORTB | 1)       /* PB1: PWM Generator 1, PWM3 */
#  define GPIO_I2C0_SCL    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTB | 2)       /* PB2: I2C0 clock (I2C0SCL) */
#  define GPIO_I2C0_SDA    (GPIO_FUNC_PFODIO    | GPIO_PORTB | 3)       /* PB3: I2C0 data (I2C0SDA) */
#  define GPIO_CMP0_NIN    (GPIO_FUNC_PFINPUT   | GPIO_PORTB | 4)       /* PB4: Analog comparator 0 negative input (C0-) */
#  define GPIO_CMP0_OUT    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTB | 5)       /* PB5: Analog comparator 0 output (C0o) (differs) */
#  define GPIO_CMP0_PIN    (GPIO_FUNC_PFINPUT   | GPIO_PORTB | 6)       /* PB6: Analog comparator 0 positive input (C0+) */
#  define GPIO_JTAG_TRST   (GPIO_FUNC_PFINPUT   | GPIO_PORTB | 7)       /* PB7: JTAG ~TRST */
#  define GPIO_JTAG_TCK    (GPIO_FUNC_PFINPUT   | GPIO_PORTC | 0)       /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_SWCLK  (GPIO_FUNC_PFINPUT   | GPIO_PORTC | 0)       /* PC0: JTAG/SWD CLK */
#  define GPIO_JTAG_TMS    (GPIO_FUNC_PFIO      | GPIO_PORTC | 1)       /* PC1: JTAG TMS */
#  define GPIO_JTAG_SWDIO  (GPIO_FUNC_PFIO      | GPIO_PORTC | 1)       /* PC1: JTAG SWDIO */
#  define GPIO_JTAG_TDI    (GPIO_FUNC_PFINPUT   | GPIO_PORTC | 2)       /* PC2: JTAG TDI */
#  define GPIO_JTAG_TDO    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTC | 3)       /* PC3: JTAG TDO */
#  define GPIO_JTAG_SWO    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTC | 3)       /* PC3: JTAG SWO */
#  define GPIO_QEI0_PHA    (GPIO_FUNC_PFINPUT   | GPIO_PORTC | 4)       /* PC4: QEI module 0 phase A. */
#  define GPIO_QEI0_PHB    (GPIO_FUNC_PFINPUT   | GPIO_PORTC | 6)       /* PC6: QEI module 0 phase B. */
#  define GPIO_CAN0_RX     (GPIO_FUNC_PFINPUT   | GPIO_PORTD | 0)       /* PD0: CAN module  RX */
#  define GPIO_CAN0_TX     (GPIO_FUNC_PFOUTPUT  | GPIO_PORTD | 1)       /* PD1: CAN module TX */
#  define GPIO_UART1_RX    (GPIO_FUNC_PFINPUT   | GPIO_PORTD | 2)       /* PD2: UART 1 receive (U1Rx) */
#  define GPIO_UART1_TX    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTD | 3)       /* PD3: UART 1 transmit (U1Tx) */
#  define GPIO_TMR0_CCP    (GPIO_FUNC_PFIO      | GPIO_PORTD | 4)       /* PD4: Capture/Compare/PWM0 (CCP0) */
#  define GPIO_PWM_FAULT   (GPIO_FUNC_PFINPUT   | GPIO_PORTD | 6)       /* PD6: PWM Fault */
#  define GPIO_QEI0_IDX    (GPIO_FUNC_PFIO      | GPIO_PORTD | 7)       /* PC7: QEI module 0 index */
#  define GPIO_PWM2_4      (GPIO_FUNC_PFOUTPUT  | GPIO_PORTE | 0)       /* PE0: PWM Generator 2, PWM4 */
#  define GPIO_PWM2_5      (GPIO_FUNC_PFOUTPUT  | GPIO_PORTE | 1)       /* PE1: PWM Generator 1, PWM5 */
#  define GPIO_QEI1_PHB    (GPIO_FUNC_PFINPUT   | GPIO_PORTE | 2)       /* PE2: QEI module 1 phase B. */
#  define GPIO_QEI1_PHA    (GPIO_FUNC_PFINPUT   | GPIO_PORTE | 3)       /* PE3: QEI module 1 phase A. */
#  define GPIO_PWM0_0      (GPIO_FUNC_PFOUTPUT  | GPIO_PORTF | 0)       /* PF0: PWM Generator 0, PWM0 */
#  define GPIO_QEI1_IDX    (GPIO_FUNC_PFINPUT   | GPIO_PORTE | 1)       /* PF1: QEI module 1 index. ) */
#  define GPIO_ETHPHY_LED1 (GPIO_FUNC_PFOUTPUT  | GPIO_PORTF | 2)       /* PF2: LED1 */
#  define GPIO_ETHPHY_LED0 (GPIO_FUNC_PFOUTPUT  | GPIO_PORTF | 3)       /* PF3: LED0 */
#  define GPIO_PWM0_1     (GPIO_FUNC_PFOUTPUT  | GPIO_PORTG | 1)       /* PG1:PWM Generator 0, PWM1 */
#else
#  error "Unknown LM3S chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization.
 *
 ****************************************************************************/

EXTERN void up_lowsetup(void);

/****************************************************************************
 * Name: lm3s_clockconfig
 *
 * Description:
 *   Called to change to new clock based on desired rcc and rcc2 settings.
 *   This is use to set up the initial clocking but can be used later to
 *   support slow clocked, low power consumption modes.
 *
 ****************************************************************************/

EXTERN void lm3s_clockconfig(uint32_t newrcc, uint32_t newrcc2);

/****************************************************************************
 * Name: up_clockconfig
 *
 * Description:
 *   Called early in the bootsequence (before .data and .bss are available)
 *   in order to configure initial clocking.
 *
 ****************************************************************************/

EXTERN void up_clockconfig(void);

/****************************************************************************
 * Name: lm3s_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

EXTERN int lm3s_configgpio(uint32_t cfgset);

/****************************************************************************
 * Name: lm3s_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

EXTERN void lm3s_gpiowrite(uint32_t pinset, bool value);

/****************************************************************************
 * Name: lm3s_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

EXTERN bool lm3s_gpioread(uint32_t pinset, bool value);

/****************************************************************************
 * Function:  lm3s_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

EXTERN int lm3s_dumpgpio(uint32_t pinset, const char *msg);

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

EXTERN int weak_function gpio_irqinitialize(void);

/****************************************************************************
 * Function: lm3s_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the LM3S chip
 *   supports multiple Ethernet controllers, then bould specific logic
 *   must implement up_netinitialize() and call this function to initialize
 *   the desiresed interfaces.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if LM3S_NETHCONTROLLERS > 1
EXTERN int lm3s_ethinitialize(int intf);
#endif

/****************************************************************************
 * The external functions, lm3s_spiselect, lm3s_spistatus, and
 * lm3s_spicmddata must be provided by board-specific logic.  These are
 * implementations of the select, status, and cmddata methods of the SPI
 * interface defined by struct spi_ops_s (see include/nuttx/spi.h).
 * All other methods (including up_spiinitialize()) are provided by common
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in lm3s_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide lm3s_spiselect() and lm3s_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide
 *      the lm3s_spicmddata() function in your board-specific logic.  This
 *      functions will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   4. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   5. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

struct spi_dev_s;
enum spi_dev_e;
EXTERN void lm3s_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lm3s_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lm3s_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LM3S_LM3S_INTERNAL_H */
