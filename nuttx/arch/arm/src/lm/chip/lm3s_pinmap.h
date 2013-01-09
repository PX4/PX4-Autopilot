/************************************************************************************
 * arch/arm/src/lm/chip/lm3s_pinmap.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM3S_PINMAP_H
#define __ARCH_ARM_SRC_LM_CHIP_LM3S_PINMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* The following lists the input value to lm_configgpio to setup the alternate,
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
#  error "Unknown Stellaris chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM3S_PINMAP_H */
