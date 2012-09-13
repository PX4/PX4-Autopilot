/****************************************************************************
 * arch/avr/include/avr/atmega/irq.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_ATMEGA_IRQ_H
#define __ARCH_AVR_INCLUDE_ATMEGA_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr/avr.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* The ATmega128 has 35 interrupt vectors including vector 0, the reset
 * vector.  The remaining 34 are assigned IRQ numbers here:
 */

#define ATMEGA_IRQ_INT0     0  /* 0x0002 External Interrupt Request 0 */
#define ATMEGA_IRQ_INT1     1  /* 0x0004 External Interrupt Request 1 */
#define ATMEGA_IRQ_INT2     2  /* 0x0006 External Interrupt Request 2 */
#define ATMEGA_IRQ_INT3     3  /* 0x0008 External Interrupt Request 3 */
#define ATMEGA_IRQ_INT4     4  /* 0x000a External Interrupt Request 4 */
#define ATMEGA_IRQ_INT5     5  /* 0x000c External Interrupt Request 5 */
#define ATMEGA_IRQ_INT6     6  /* 0x000e External Interrupt Request 6 */
#define ATMEGA_IRQ_INT7     7  /* 0x0010 External Interrupt Request 7 */
#define ATMEGA_IRQ_T2COMP   8  /* 0x0012 TIMER2 COMP Timer/Counter2 Compare Match */
#define ATMEGA_IRQ_T2OVF    9  /* 0x0014 TIMER2 OVF Timer/Counter2 Overflow */
#define ATMEGA_IRQ_T1CAPT  10  /* 0x0016 TIMER1 CAPT Timer/Counter1 Capture Event */
#define ATMEGA_IRQ_T1COMPA 11  /* 0x0018 TIMER1 COMPA Timer/Counter1 Compare Match A */
#define ATMEGA_IRQ_T1COMPB 12  /* 0x001a TIMER1 COMPB Timer/Counter1 Compare Match B */
#define ATMEGA_IRQ_T1OVF   13  /* 0x001c TIMER1 OVF Timer/Counter1 Overflow */
#define ATMEGA_IRQ_T0COMP  14  /* 0x001e TIMER0 COMP Timer/Counter0 Compare Match */
#define ATMEGA_IRQ_T0OVF   15  /* 0x0020 TIMER0 OVF Timer/Counter0 Overflow */
#define ATMEGA_IRQ_SPI     16  /* 0x0022 STC SPI Serial Transfer Complete */
#define ATMEGA_IRQ_U0RX    17  /* 0x0024 USART0 Rx Complete */
#define ATMEGA_IRQ_U0DRE   18  /* 0x0026 USART0 Data Register Empty */
#define ATMEGA_IRQ_U0TX    19  /* 0x0028 USART0 Tx Complete */
#define ATMEGA_IRQ_ADC     20  /* 0x002a ADC Conversion Complete */
#define ATMEGA_IRQ_EE      21  /* 0x002c EEPROM Ready */
#define ATMEGA_IRQ_ANACOMP 22  /* 0x002e ANALOG COMP Analog Comparator */
#define ATMEGA_IRQ_T1COMPC 23  /* 0x0030 TIMER1 COMPC Timer/Countre1 Compare Match C */
#define ATMEGA_IRQ_T3CAPT  24  /* 0x0032 TIMER3 CAPT Timer/Counter3 Capture Event */
#define ATMEGA_IRQ_T3COMPA 25  /* 0x0034 TIMER3 COMPA Timer/Counter3 Compare Match A */
#define ATMEGA_IRQ_T3COMPB 26  /* 0x0036 TIMER3 COMPB Timer/Counter3 Compare Match B */
#define ATMEGA_IRQ_T3COMPC 27  /* 0x0038 TIMER3 COMPC Timer/Counter3 Compare Match C */
#define ATMEGA_IRQ_T3OVF   28  /* 0x003a TIMER3 OVF Timer/Counter3 Overflow */
#define ATMEGA_IRQ_U1RX    29  /* 0x003c USART1 Rx Complete */
#define ATMEGA_IRQ_U1DRE   30  /* 0x003e USART1 Data Register Empty */
#define ATMEGA_IRQ_U1TX    31  /* 0x0040 USART1 Tx Complete */
#define ATMEGA_IRQ_TWI     32  /* 0x0042 TWI Two-wire Serial Interface */
#define ATMEGA_IRQ_SPMRDY  33  /* 0x0044 Store Program Memory Ready */

#define NR_IRQS            34

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_AVR_INCLUDE_ATMEGA_IRQ_H */

