/************************************************************************************
 * arch/arm/include/lm/irq.h
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LM_IRQ_H
#define __ARCH_ARM_INCLUDE_LM_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define LM3S_IRQ_RESERVED     (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                  /* Vector  0: Reset stack pointer value */
                                  /* Vector  1: Reset (not handler as an IRQ) */
#define LM3S_IRQ_NMI          (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define LM3S_IRQ_HARDFAULT    (3) /* Vector  3: Hard fault */
#define LM3S_IRQ_MEMFAULT     (4) /* Vector  4: Memory management (MPU) */
#define LM3S_IRQ_BUSFAULT     (5) /* Vector  5: Bus fault */
#define LM3S_IRQ_USAGEFAULT   (6) /* Vector  6: Usage fault */
#define LM3S_IRQ_SVCALL      (11) /* Vector 11: SVC call */
#define LM3S_IRQ_DBGMONITOR  (12) /* Vector 12: Debug Monitor */
                                  /* Vector 13: Reserved */
#define LM3S_IRQ_PENDSV      (14) /* Vector 14: Pendable system service request */
#define LM3S_IRQ_SYSTICK     (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define LM3S_IRQ_INTERRUPTS  (16) /* Vector number of the first external interrupt */
#if defined(CONFIG_ARCH_CHIP_LM3S6918)

#  define LM3S_IRQ_GPIOA     (16) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB     (17) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC     (18) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD     (19) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE     (20) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0     (21) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1     (22) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0      (23) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0      (24) /* Vector 24: I2C 0 */
                                  /* Vector 25-29: Reserved */
#  define LM3S_IRQ_ADC0      (30) /* Vector 30: ADC Sequence 0 */
#  define LM3S_IRQ_ADC1      (31) /* Vector 31: ADC Sequence 1 */
#  define LM3S_IRQ_ADC2      (32) /* Vector 32: ADC Sequence 2 */
#  define LM3S_IRQ_ADC3      (33) /* Vector 33: ADC Sequence 3 */
#  define LM3S_IRQ_WDOG      (34) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (35) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (36) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (37) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (38) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (39) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (40) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (41) /* Vector 41: Analog Comparator 0 */
#  define LM3S_IRQ_COMPARE1  (42) /* Vector 42: Analog Comparator 1 */
                                  /* Vector 43: Reserved */
#  define LM3S_IRQ_SYSCON    (44) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (45) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (46) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (47) /* Vector 47: GPIO Port G */
#  define LM3S_IRQ_GPIOH     (48) /* Vector 48: GPIO Port H */
                                  /* Vector 49: Reserved */
#  define LM3S_IRQ_SSI1      (50) /* Vector 50: SSI 1 */
#  define LM3S_IRQ_TIMER3A   (51) /* Vector 51: Timer 3 A */
#  define LM3S_IRQ_TIMER3B   (52) /* Vector 52: Timer 3 B */
#  define LM3S_IRQ_I2C1      (53) /* Vector 53: I2C 1 */
                                  /* Vectors 54-57: Reserved */
#  define LM3S_IRQ_ETHCON    (58) /* Vector 58: Ethernet Controller */
#  define LM3S_IRQ_HIBERNATE (59) /* Vector 59: Hibernation Module */
                                  /* Vectors 60-70: Reserved */
#  define NR_IRQS            (60) /* (Really less because of reserved vectors) */
#elif defined(CONFIG_ARCH_CHIP_LM3S6432)
#  define LM3S_IRQ_GPIOA     (16) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB     (17) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC     (18) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD     (19) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE     (20) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0     (21) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1     (22) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0      (23) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0      (24) /* Vector 24: I2C 0 */
                                  /* Vector 25: Reserved */
#  define LM3S_IRQ_PWM0      (26) /* Vector 26: PWM Generator 0 */
                                  /* Vectors 27-29: Reserved */
#  define LM3S_IRQ_ADC0      (30) /* Vector 30: ADC Sequence 0 */
#  define LM3S_IRQ_ADC1      (31) /* Vector 31: ADC Sequence 1 */
#  define LM3S_IRQ_ADC2      (32) /* Vector 32: ADC Sequence 2 */
#  define LM3S_IRQ_ADC3      (33) /* Vector 33: ADC Sequence 3 */
#  define LM3S_IRQ_WDOG      (34) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (35) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (36) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (37) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (38) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (39) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (40) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (41) /* Vector 41: Analog Comparator 0 */
#  define LM3S_IRQ_COMPARE1  (42) /* Vector 42: Analog Comparator 1 */
                                  /* Vector 43: Reserved */
#  define LM3S_IRQ_SYSCON    (44) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (45) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (46) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (47) /* Vector 47: GPIO Port G */
                                  /* Vectors 48-57: Reserved */
#  define LM3S_IRQ_ETHCON    (58) /* Vector 58: Ethernet Controller */
                                  /* Vectors 59-70: Reserved */
#  define NR_IRQS            (60) /* (Really less because of reserved vectors) */
#elif defined(CONFIG_ARCH_CHIP_LM3S6965)
#  define LM3S_IRQ_GPIOA     (16) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB     (17) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC     (18) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD     (19) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE     (20) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0     (21) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1     (22) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0      (23) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0      (24) /* Vector 24: I2C 0 */
#  define LM3S_IRQ_PWMFAULT  (25) /* Vector 25: PWM Fault */
#  define LM3S_IRQ_PWM0      (26) /* Vector 26: PWM Generator 0 */
#  define LM3S_IRQ_PWM1      (27) /* Vector 27: PWM Generator 1 */
#  define LM3S_IRQ_PWM2      (28) /* Vector 28: PWM Generator 2 */
#  define LM3S_IRQ_QEI0      (29) /* Vector 29: QEI0 */
#  define LM3S_IRQ_ADC0      (30) /* Vector 30: ADC Sequence 0 */
#  define LM3S_IRQ_ADC1      (31) /* Vector 31: ADC Sequence 1 */
#  define LM3S_IRQ_ADC2      (32) /* Vector 32: ADC Sequence 2 */
#  define LM3S_IRQ_ADC3      (33) /* Vector 33: ADC Sequence 3 */
#  define LM3S_IRQ_WDOG      (34) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (35) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (36) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (37) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (38) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (39) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (40) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (41) /* Vector 41: Analog Comparator 0 */
#  define LM3S_IRQ_COMPARE1  (42) /* Vector 42: Analog Comparator 1 */
                                  /* Vector 43: Reserved */
#  define LM3S_IRQ_SYSCON    (44) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (45) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (46) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (47) /* Vector 47: GPIO Port G */
                                  /* Vector 48: Reserved */
#  define LM3S_IRQ_UART2     (49) /* Vector 49: UART 2 */
                                  /* Vector 50: Reserved */
#  define LM3S_IRQ_TIMER3A   (51) /* Vector 51: Timer 3 A */
#  define LM3S_IRQ_TIMER3B   (52) /* Vector 52: Timer 3 B */
#  define LM3S_IRQ_I2C1      (53) /* Vector 53: I2C 1 */
#  define LM3S_IRQ_QEI1      (54) /* Vector 54: QEI1 */
                                  /* Vectors 55-57: Reserved */
#  define LM3S_IRQ_ETHCON    (58) /* Vector 58: Ethernet Controller */
#  define LM3S_IRQ_HIBERNATE (59) /* Vector 59: Hibernation Module */
                                  /* Vectors 60-70: Reserved */
#  define NR_IRQS            (60) /* (Really less because of reserved vectors) */
#elif defined(CONFIG_ARCH_CHIP_LM3S9B96)
#  define LM3S_IRQ_GPIOA     (16) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB     (17) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC     (18) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD     (19) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE     (20) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0     (21) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1     (22) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0      (23) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0      (24) /* Vector 24: I2C 0 */
#  define LM3S_IRQ_PWMFAULT  (25) /* Vector 25: PWM Fault */
#  define LM3S_IRQ_PWM0      (26) /* Vector 26: PWM Generator 0 */
#  define LM3S_IRQ_PWM1      (27) /* Vector 27: PWM Generator 1 */
#  define LM3S_IRQ_PWM2      (28) /* Vector 28: PWM Generator 2 */
#  define LM3S_IRQ_QEI0      (29) /* Vector 29: QEI0 */
#  define LM3S_IRQ_ADC0      (30) /* Vector 30: ADC0 Sequence 0 */
#  define LM3S_IRQ_ADC1      (31) /* Vector 31: ADC0 Sequence 1 */
#  define LM3S_IRQ_ADC2      (32) /* Vector 32: ADC0 Sequence 2 */
#  define LM3S_IRQ_ADC3      (33) /* Vector 33: ADC0 Sequence 3 */
#  define LM3S_IRQ_WDOG      (34) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (35) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (36) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (37) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (38) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (39) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (40) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (41) /* Vector 41: Analog Comparator 0 */
#  define LM3S_IRQ_COMPARE1  (42) /* Vector 42: Analog Comparator 1 */
#  define LM3S_IRQ_COMPARE2  (43) /* Vector 43: Analog Comparator 3 */
#  define LM3S_IRQ_SYSCON    (44) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (45) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (46) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (47) /* Vector 47: GPIO Port G */
#  define LM3S_IRQ_GPIOH     (48) /* Vector 48: GPIO Port H */
#  define LM3S_IRQ_UART2     (49) /* Vector 49: UART 2 */
#  define LM3S_IRQ_SSI1      (50) /* Vector 50: SSI 1  */
#  define LM3S_IRQ_TIMER3A   (51) /* Vector 51: Timer 3 A */
#  define LM3S_IRQ_TIMER3B   (52) /* Vector 52: Timer 3 B */
#  define LM3S_IRQ_I2C1      (53) /* Vector 53: I2C 1 */
#  define LM3S_IRQ_QEI1      (54) /* Vector 54: QEI1 */
#  define LM3S_IRQ_CAN0      (55) /* Vector 55: CAN 1 */
#  define LM3S_IRQ_CAN1      (56) /* Vector 56: CAN 2 */
                                  /* Vector 57: Reserved */
#  define LM3S_IRQ_ETHCON    (58) /* Vector 58: Ethernet Controller */
                                  /* Vector 59: Reserved */
#  define LM3S_IRQ_USB       (60) /* Vector 60: USB */
#  define LM3S_IRQ_PWM3      (61) /* Vector 61: PWM Generator 3 */
#  define LM3S_IRQ_UDMASOFT  (62) /* Vector 62: uDMA Software */
#  define LM3S_IRQ_UDMAERROR (63) /* Vector 63: uDMA Error */
#  define LM3S_IRQ_ADC1_0    (64) /* Vector 64: ADC1 Sequence 0 */
#  define LM3S_IRQ_ADC1_1    (65) /* Vector 65: ADC1 Sequence 1 */
#  define LM3S_IRQ_ADC1_2    (66) /* Vector 66: ADC1 Sequence 2 */
#  define LM3S_IRQ_ADC1_3    (67) /* Vector 67: ADC1 Sequence 3 */
#  define LM3S_IRQ_I2S0      (68) /* Vector 68: I2S0 */
#  define LM3S_IRQ_EPI       (69) /* Vector 69: EPI */
#  define LM3S_IRQ_GPIOJ     (70) /* Vector 70: GPIO Port J */
                                  /* Vector 71: Reserved */
#  define NR_IRQS            (71) /* (Really less because of reserved vectors) */
#elif defined(CONFIG_ARCH_CHIP_LM3S8962)
#  define LM3S_IRQ_GPIOA     (16) /* Vector 16: GPIO Port A */
#  define LM3S_IRQ_GPIOB     (17) /* Vector 17: GPIO Port B */
#  define LM3S_IRQ_GPIOC     (18) /* Vector 18: GPIO Port C */
#  define LM3S_IRQ_GPIOD     (19) /* Vector 19: GPIO Port D */
#  define LM3S_IRQ_GPIOE     (20) /* Vector 20: GPIO Port E */
#  define LM3S_IRQ_UART0     (21) /* Vector 21: UART 0 */
#  define LM3S_IRQ_UART1     (22) /* Vector 22: UART 1 */
#  define LM3S_IRQ_SSI0      (23) /* Vector 23: SSI 0 */
#  define LM3S_IRQ_I2C0      (24) /* Vector 24: I2C 0 */
#  define LM3S_IRQ_PWMFAULT  (25) /* Vector 25: PWM Fault */
#  define LM3S_IRQ_PWM0      (26) /* Vector 26: PWM Generator 0 */
#  define LM3S_IRQ_PWM1      (27) /* Vector 27: PWM Generator 1 */
#  define LM3S_IRQ_PWM2      (28) /* Vector 28: PWM Generator 2 */
#  define LM3S_IRQ_QEI0      (29) /* Vector 29: QEI0 */
#  define LM3S_IRQ_ADC0      (30) /* Vector 30: ADC Sequence 0 */
#  define LM3S_IRQ_ADC1      (31) /* Vector 31: ADC Sequence 1 */
#  define LM3S_IRQ_ADC2      (32) /* Vector 32: ADC Sequence 2 */
#  define LM3S_IRQ_ADC3      (33) /* Vector 33: ADC Sequence 3 */
#  define LM3S_IRQ_WDOG      (34) /* Vector 34: Watchdog Timer */
#  define LM3S_IRQ_TIMER0A   (35) /* Vector 35: Timer 0 A */
#  define LM3S_IRQ_TIMER0B   (36) /* Vector 36: Timer 0 B */
#  define LM3S_IRQ_TIMER1A   (37) /* Vector 37: Timer 1 A */
#  define LM3S_IRQ_TIMER1B   (38) /* Vector 38: Timer 1 B */
#  define LM3S_IRQ_TIMER2A   (39) /* Vector 39: Timer 2 A */
#  define LM3S_IRQ_TIMER2B   (40) /* Vector 40: Timer 3 B */
#  define LM3S_IRQ_COMPARE0  (41) /* Vector 41: Analog Comparator 0 */
                                  /* Vector 42: Reserved */
                                  /* Vector 43: Reserved */
#  define LM3S_IRQ_SYSCON    (44) /* Vector 44: System Control */
#  define LM3S_IRQ_FLASHCON  (45) /* Vector 45: FLASH Control */
#  define LM3S_IRQ_GPIOF     (46) /* Vector 46: GPIO Port F */
#  define LM3S_IRQ_GPIOG     (47) /* Vector 47: GPIO Port G */
                                  /* Vector 48: Reserved */
                                  /* Vector 49: Reserved */
                                  /* Vector 50: Reserved */
#  define LM3S_IRQ_TIMER3A   (51) /* Vector 51: Timer 3 A */
#  define LM3S_IRQ_TIMER3B   (52) /* Vector 52: Timer 3 B */
#  define LM3S_IRQ_I2C1      (53) /* Vector 53: I2C 1 */
#  define LM3S_IRQ_QEI1      (54) /* Vector 54: QEI1 */
#  define LM3S_IRQ_CAN0      (54) /* Vector 55: CAN0 */
                                  /* Vectors 56-57: Reserved */
#  define LM3S_IRQ_ETHCON    (58) /* Vector 58: Ethernet Controller */
#  define LM3S_IRQ_HIBERNATE (59) /* Vector 59: Hibernation Module */
                                  /* Vectors 60-70: Reserved */
#  define NR_IRQS            (60) /* (Really less because of reserved vectors) */
#else
#  error "IRQ Numbers not specified for this LM3S chip"
#endif

/* GPIO IRQs -- Note that support for individual GPIO ports can
 * be disabled in order to reduce the size of the implemenation.
 */

#ifndef CONFIG_LM3S_DISABLE_GPIOA_IRQS
#  define LM3S_IRQ_GPIOA_0 (NR_IRQS + 0)
#  define LM3S_IRQ_GPIOA_1 (NR_IRQS + 1)
#  define LM3S_IRQ_GPIOA_2 (NR_IRQS + 2)
#  define LM3S_IRQ_GPIOA_3 (NR_IRQS + 3)
#  define LM3S_IRQ_GPIOA_4 (NR_IRQS + 4)
#  define LM3S_IRQ_GPIOA_5 (NR_IRQS + 5)
#  define LM3S_IRQ_GPIOA_6 (NR_IRQS + 6)
#  define LM3S_IRQ_GPIOA_7 (NR_IRQS + 7)
#  define _NGPIOAIRQS      (NR_IRQS + 8)
#else
#  define _NGPIOAIRQS      NR_IRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOB_IRQS
#  define LM3S_IRQ_GPIOB_0 (_NGPIOAIRQS + 0)
#  define LM3S_IRQ_GPIOB_1 (_NGPIOAIRQS + 1)
#  define LM3S_IRQ_GPIOB_2 (_NGPIOAIRQS + 2)
#  define LM3S_IRQ_GPIOB_3 (_NGPIOAIRQS + 3)
#  define LM3S_IRQ_GPIOB_4 (_NGPIOAIRQS + 4)
#  define LM3S_IRQ_GPIOB_5 (_NGPIOAIRQS + 5)
#  define LM3S_IRQ_GPIOB_6 (_NGPIOAIRQS + 6)
#  define LM3S_IRQ_GPIOB_7 (_NGPIOAIRQS + 7)
#  define _NGPIOBIRQS      (_NGPIOAIRQS + 8)
#else
#  define _NGPIOBIRQS      _NGPIOAIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOC_IRQS
#  define LM3S_IRQ_GPIOC_0 (_NGPIOBIRQS + 0)
#  define LM3S_IRQ_GPIOC_1 (_NGPIOBIRQS + 1)
#  define LM3S_IRQ_GPIOC_2 (_NGPIOBIRQS + 2)
#  define LM3S_IRQ_GPIOC_3 (_NGPIOBIRQS + 3)
#  define LM3S_IRQ_GPIOC_4 (_NGPIOBIRQS + 4)
#  define LM3S_IRQ_GPIOC_5 (_NGPIOBIRQS + 5)
#  define LM3S_IRQ_GPIOC_6 (_NGPIOBIRQS + 6)
#  define LM3S_IRQ_GPIOC_7 (_NGPIOBIRQS + 7)
#  define _NGPIOCIRQS      (_NGPIOBIRQS + 8)
#else
#  define _NGPIOCIRQS      _NGPIOBIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOD_IRQS
#  define LM3S_IRQ_GPIOD_0 (_NGPIOCIRQS + 0)
#  define LM3S_IRQ_GPIOD_1 (_NGPIOCIRQS + 1)
#  define LM3S_IRQ_GPIOD_2 (_NGPIOCIRQS + 2)
#  define LM3S_IRQ_GPIOD_3 (_NGPIOCIRQS + 3)
#  define LM3S_IRQ_GPIOD_4 (_NGPIOCIRQS + 4)
#  define LM3S_IRQ_GPIOD_5 (_NGPIOCIRQS + 5)
#  define LM3S_IRQ_GPIOD_6 (_NGPIOCIRQS + 6)
#  define LM3S_IRQ_GPIOD_7 (_NGPIOCIRQS + 7)
#  define _NGPIODIRQS      (_NGPIOCIRQS + 8)
#else
#  define _NGPIODIRQS      _NGPIOCIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOE_IRQS
#  define LM3S_IRQ_GPIOE_0 (_NGPIODIRQS + 0)
#  define LM3S_IRQ_GPIOE_1 (_NGPIODIRQS + 1)
#  define LM3S_IRQ_GPIOE_2 (_NGPIODIRQS + 2)
#  define LM3S_IRQ_GPIOE_3 (_NGPIODIRQS + 3)
#  define LM3S_IRQ_GPIOE_4 (_NGPIODIRQS + 4)
#  define LM3S_IRQ_GPIOE_5 (_NGPIODIRQS + 5)
#  define LM3S_IRQ_GPIOE_6 (_NGPIODIRQS + 6)
#  define LM3S_IRQ_GPIOE_7 (_NGPIODIRQS + 7)
#  define _NGPIOEIRQS      (_NGPIODIRQS + 8)
#else
#  define _NGPIOEIRQS      _NGPIODIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOF_IRQS
#  define LM3S_IRQ_GPIOF_0 (_NGPIOEIRQS + 0)
#  define LM3S_IRQ_GPIOF_1 (_NGPIOEIRQS + 1)
#  define LM3S_IRQ_GPIOF_2 (_NGPIOEIRQS + 2)
#  define LM3S_IRQ_GPIOF_3 (_NGPIOEIRQS + 3)
#  define LM3S_IRQ_GPIOF_4 (_NGPIOEIRQS + 4)
#  define LM3S_IRQ_GPIOF_5 (_NGPIOEIRQS + 5)
#  define LM3S_IRQ_GPIOF_6 (_NGPIOEIRQS + 6)
#  define LM3S_IRQ_GPIOF_7 (_NGPIOEIRQS + 7)
#  define _NGPIOFIRQS      (_NGPIOEIRQS + 8)
#else
#  define _NGPIOFIRQS      _NGPIOEIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOG_IRQS
#  define LM3S_IRQ_GPIOG_0 (_NGPIOFIRQS + 0)
#  define LM3S_IRQ_GPIOG_1 (_NGPIOFIRQS + 1)
#  define LM3S_IRQ_GPIOG_2 (_NGPIOFIRQS + 2)
#  define LM3S_IRQ_GPIOG_3 (_NGPIOFIRQS + 3)
#  define LM3S_IRQ_GPIOG_4 (_NGPIOFIRQS + 4)
#  define LM3S_IRQ_GPIOG_5 (_NGPIOFIRQS + 5)
#  define LM3S_IRQ_GPIOG_6 (_NGPIOFIRQS + 6)
#  define LM3S_IRQ_GPIOG_7 (_NGPIOFIRQS + 7)
#  define _NGPIOGIRQS      (_NGPIOFIRQS + 8)
#else
#  define _NGPIOGIRQS      _NGPIOFIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOH_IRQS
#  define LM3S_IRQ_GPIOH_0 (_NGPIOGIRQS + 0)
#  define LM3S_IRQ_GPIOH_1 (_NGPIOGIRQS + 1)
#  define LM3S_IRQ_GPIOH_2 (_NGPIOGIRQS + 2)
#  define LM3S_IRQ_GPIOH_3 (_NGPIOGIRQS + 3)
#  define LM3S_IRQ_GPIOH_4 (_NGPIOGIRQS + 4)
#  define LM3S_IRQ_GPIOH_5 (_NGPIOGIRQS + 5)
#  define LM3S_IRQ_GPIOH_6 (_NGPIOGIRQS + 6)
#  define LM3S_IRQ_GPIOH_7 (_NGPIOGIRQS + 7)
#  define _NGPIOHIRQS      (_NGPIOGIRQS + 8)
#else
#  define _NGPIOHIRQS      _NGPIOGIRQS
#endif

#ifndef CONFIG_LM3S_DISABLE_GPIOJ_IRQS
#  define LM3S_IRQ_GPIOJ_0 (_NGPIOHIRQS + 0)
#  define LM3S_IRQ_GPIOJ_1 (_NGPIOHIRQS + 1)
#  define LM3S_IRQ_GPIOJ_2 (_NGPIOHIRQS + 2)
#  define LM3S_IRQ_GPIOJ_3 (_NGPIOHIRQS + 3)
#  define LM3S_IRQ_GPIOJ_4 (_NGPIOHIRQS + 4)
#  define LM3S_IRQ_GPIOJ_5 (_NGPIOHIRQS + 5)
#  define LM3S_IRQ_GPIOJ_6 (_NGPIOHIRQS + 6)
#  define LM3S_IRQ_GPIOJ_7 (_NGPIOHIRQS + 7)
#  define _NGPIOJIRQS      (_NGPIOHIRQS + 8)
#else
#  define _NGPIOJIRQS      _NGPIOHIRQS
#endif

#define NR_GPIO_IRQS       (_NGPIOJIRQS - NR_IRQS)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach the interrupt handler 'isr' to the GPIO IRQ 'irq'
 *
 ****************************************************************************/

EXTERN int gpio_irqattach(int irq, xcpt_t isr);
#define gpio_irqdetach(isr) gpio_irqattach(isr, NULL)

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

EXTERN void gpio_irqenable(int irq);

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

EXTERN void gpio_irqdisable(int irq);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LM_IRQ_H */

