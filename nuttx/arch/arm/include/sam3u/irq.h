/****************************************************************************************
 * arch/arm/include/sam3u/irq.h
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
 ****************************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_SAM3U_IRQ_H
#define __ARCH_ARM_INCLUDE_SAM3U_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Definitions
 ****************************************************************************************/

/* SAM3U Peripheral Identifiers */

#define SAM3U_PID_SUPC           (0)  /* Supply Controller */
#define SAM3U_PID_RSTC           (1)  /* Reset Controller */
#define SAM3U_PID_RTC            (2)  /* Real Time Clock */
#define SAM3U_PID_RTT            (3)  /* Real Time Timer */
#define SAM3U_PID_WDT            (4)  /* Watchdog Timer */
#define SAM3U_PID_PMC            (5)  /* Power Management Controller */
#define SAM3U_PID_EEFC0          (6)  /* Enhanced Embedded Flash Controller 0 */
#define SAM3U_PID_EEFC1          (7)  /* Enhanced Embedded Flash Controller 1 */
#define SAM3U_PID_UART           (8)  /* Universal Asynchronous Receiver Transmitter */
#define SAM3U_PID_SMC            (9)  /* Static Memory Controller */
#define SAM3U_PID_PIOA          (10)  /* Parallel I/O Controller A */
#define SAM3U_PID_PIOB          (11)  /* Parallel I/O Controller B */
#define SAM3U_PID_PIOC          (12)  /* Parallel I/O Controller C */
#define SAM3U_PID_USART0        (13)  /* USART 0 */
#define SAM3U_PID_USART1        (14)  /* USART 1 */
#define SAM3U_PID_USART2        (15)  /* USART 2 */
#define SAM3U_PID_USART3        (16)  /* USART 3 */
#define SAM3U_PID_HSMCI         (17)  /* High Speed Multimedia Card Interface */
#define SAM3U_PID_TWI0          (18)  /* Two-Wire Interface 0 */
#define SAM3U_PID_TWI1          (19)  /* Two-Wire Interface 1 */
#define SAM3U_PID_SPI           (20)  /* Serial Peripheral Interface */
#define SAM3U_PID_SSC           (21)  /* Synchronous Serial Controller */
#define SAM3U_PID_TC0           (22)  /* Timer Counter 0 */
#define SAM3U_PID_TC1           (23)  /* Timer Counter 1 */
#define SAM3U_PID_TC2           (24)  /* Timer Counter 2 */
#define SAM3U_PID_PWM           (25)  /* Pulse Width Modulation Controller */
#define SAM3U_PID_ADC12B        (26)  /* 12-bit ADC Controller */
#define SAM3U_PID_ADC           (27)  /* 10-bit ADC Controller */
#define SAM3U_PID_DMAC          (28)  /* DMA Controller */
#define SAM3U_PID_UDPHS         (29)  /* USB Device High Speed */
#define NR_PIDS                 (30)  /* Number of peripheral identifiers */

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define SAM3U_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define SAM3U_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define SAM3U_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define SAM3U_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define SAM3U_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define SAM3U_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define SAM3U_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define SAM3U_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define SAM3U_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define SAM3U_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define SAM3U_IRQ_EXTINT        (16) /* Vector number of the first external interrupt */
#define SAM3U_IRQ_SUPC          (SAM3U_IRQ_EXTINT+SAM3U_PID_SUPC)   /* Supply Controller */
#define SAM3U_IRQ_RSTC          (SAM3U_IRQ_EXTINT+SAM3U_PID_RSTC)   /* Reset Controller */
#define SAM3U_IRQ_RTC           (SAM3U_IRQ_EXTINT+SAM3U_PID_RTC)    /* Real Time Clock */
#define SAM3U_IRQ_RTT           (SAM3U_IRQ_EXTINT+SAM3U_PID_RTT)    /* Real Time Timer */
#define SAM3U_IRQ_WDT           (SAM3U_IRQ_EXTINT+SAM3U_PID_WDT)    /* Watchdog Timer */
#define SAM3U_IRQ_PMC           (SAM3U_IRQ_EXTINT+SAM3U_PID_PMC)    /* Power Management Controller */
#define SAM3U_IRQ_EEFC0         (SAM3U_IRQ_EXTINT+SAM3U_PID_EEFC0)  /* Enhanced Embedded Flash Controller 0 */
#define SAM3U_IRQ_EEFC1         (SAM3U_IRQ_EXTINT+SAM3U_PID_EEFC1)  /* Enhanced Embedded Flash Controller 1 */
#define SAM3U_IRQ_UART          (SAM3U_IRQ_EXTINT+SAM3U_PID_UART)   /* Universal Asynchronous Receiver Transmitter */
#define SAM3U_IRQ_SMC           (SAM3U_IRQ_EXTINT+SAM3U_PID_SMC)    /* Static Memory Controller */
#define SAM3U_IRQ_PIOA          (SAM3U_IRQ_EXTINT+SAM3U_PID_PIOA)   /* Parallel I/O Controller A */
#define SAM3U_IRQ_PIOB          (SAM3U_IRQ_EXTINT+SAM3U_PID_PIOB)   /* Parallel I/O Controller B */
#define SAM3U_IRQ_PIOC          (SAM3U_IRQ_EXTINT+SAM3U_PID_PIOC)   /* Parallel I/O Controller C */
#define SAM3U_IRQ_USART0        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART0) /* USART 0 */
#define SAM3U_IRQ_USART1        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART1) /* USART 1 */
#define SAM3U_IRQ_USART2        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART2) /* USART 2 */
#define SAM3U_IRQ_USART3        (SAM3U_IRQ_EXTINT+SAM3U_PID_USART3) /* USART 3 */
#define SAM3U_IRQ_HSMCI         (SAM3U_IRQ_EXTINT+SAM3U_PID_HSMCI)  /* High Speed Multimedia Card Interface */
#define SAM3U_IRQ_TWI0          (SAM3U_IRQ_EXTINT+SAM3U_PID_TWI0)   /* Two-Wire Interface 0 */
#define SAM3U_IRQ_TWI1          (SAM3U_IRQ_EXTINT+SAM3U_PID_TWI1)   /* Two-Wire Interface 1 */
#define SAM3U_IRQ_SPI           (SAM3U_IRQ_EXTINT+SAM3U_PID_SPI)    /* Serial Peripheral Interface */
#define SAM3U_IRQ_SSC           (SAM3U_IRQ_EXTINT+SAM3U_PID_SSC)    /* Synchronous Serial Controller */
#define SAM3U_IRQ_TC0           (SAM3U_IRQ_EXTINT+SAM3U_PID_TC0)    /* Timer Counter 0 */
#define SAM3U_IRQ_TC1           (SAM3U_IRQ_EXTINT+SAM3U_PID_TC1)    /* Timer Counter 1 */
#define SAM3U_IRQ_TC2           (SAM3U_IRQ_EXTINT+SAM3U_PID_TC2)    /* Timer Counter 2 */
#define SAM3U_IRQ_PWM           (SAM3U_IRQ_EXTINT+SAM3U_PID_PWM)    /* Pulse Width Modulation Controller */
#define SAM3U_IRQ_ADC12B        (SAM3U_IRQ_EXTINT+SAM3U_PID_ADC12B) /* 12-bit ADC Controller */
#define SAM3U_IRQ_ADC           (SAM3U_IRQ_EXTINT+SAM3U_PID_ADC)    /* 10-bit ADC Controller */
#define SAM3U_IRQ_DMAC          (SAM3U_IRQ_EXTINT+SAM3U_PID_DMAC)   /* DMA Controller */
#define SAM3U_IRQ_UDPHS         (SAM3U_IRQ_EXTINT+SAM3U_PID_UDPHS)  /* USB Device High Speed */
#define SAM3U_IRQ_NEXTINT       NR_PIDS                             /* Total number of external interrupt numbers */
#define SAM3U_IRQ_NIRQS         (SAM3U_IRQ_EXTINT+NR_PIDS)          /* The number of real IRQs */

/* GPIO interrupts (derived from SAM3U_IRQ_PIOA/B/C) */

#ifdef CONFIG_GPIOA_IRQ
#  define SAM3U_IRQ_GPIOA_PINS  (SAM3U_IRQ_EXTINT+SAM3U_IRQ_NEXTINT)
#  define SAM3U_IRQ_PA0         (SAM3U_IRQ_GPIOA_PINS+0)            /* GPIOA, PIN 0 */
#  define SAM3U_IRQ_PA1         (SAM3U_IRQ_GPIOA_PINS+1)            /* GPIOA, PIN 1 */
#  define SAM3U_IRQ_PA2         (SAM3U_IRQ_GPIOA_PINS+2)            /* GPIOA, PIN 2 */
#  define SAM3U_IRQ_PA3         (SAM3U_IRQ_GPIOA_PINS+3)            /* GPIOA, PIN 3 */
#  define SAM3U_IRQ_PA4         (SAM3U_IRQ_GPIOA_PINS+4)            /* GPIOA, PIN 4 */
#  define SAM3U_IRQ_PA5         (SAM3U_IRQ_GPIOA_PINS+5)            /* GPIOA, PIN 5 */
#  define SAM3U_IRQ_PA6         (SAM3U_IRQ_GPIOA_PINS+6)            /* GPIOA, PIN 6 */
#  define SAM3U_IRQ_PA7         (SAM3U_IRQ_GPIOA_PINS+7)            /* GPIOA, PIN 7 */
#  define SAM3U_IRQ_PA8         (SAM3U_IRQ_GPIOA_PINS+8)            /* GPIOA, PIN 8 */
#  define SAM3U_IRQ_PA9         (SAM3U_IRQ_GPIOA_PINS+9)            /* GPIOA, PIN 9 */
#  define SAM3U_IRQ_PA10        (SAM3U_IRQ_GPIOA_PINS+10)           /* GPIOA, PIN 10 */
#  define SAM3U_IRQ_PA11        (SAM3U_IRQ_GPIOA_PINS+11)           /* GPIOA, PIN 11 */
#  define SAM3U_IRQ_PA12        (SAM3U_IRQ_GPIOA_PINS+12)           /* GPIOA, PIN 12 */
#  define SAM3U_IRQ_PA13        (SAM3U_IRQ_GPIOA_PINS+13)           /* GPIOA, PIN 13 */
#  define SAM3U_IRQ_PA14        (SAM3U_IRQ_GPIOA_PINS+14)           /* GPIOA, PIN 14 */
#  define SAM3U_IRQ_PA15        (SAM3U_IRQ_GPIOA_PINS+15)           /* GPIOA, PIN 15 */
#  define SAM3U_IRQ_PA16        (SAM3U_IRQ_GPIOA_PINS+16)           /* GPIOA, PIN 16 */
#  define SAM3U_IRQ_PA17        (SAM3U_IRQ_GPIOA_PINS+17)           /* GPIOA, PIN 17 */
#  define SAM3U_IRQ_PA18        (SAM3U_IRQ_GPIOA_PINS+18)           /* GPIOA, PIN 18 */
#  define SAM3U_IRQ_PA19        (SAM3U_IRQ_GPIOA_PINS+19)           /* GPIOA, PIN 19 */
#  define SAM3U_IRQ_PA20        (SAM3U_IRQ_GPIOA_PINS+20)           /* GPIOA, PIN 20 */
#  define SAM3U_IRQ_PA21        (SAM3U_IRQ_GPIOA_PINS+21)           /* GPIOA, PIN 21 */
#  define SAM3U_IRQ_PA22        (SAM3U_IRQ_GPIOA_PINS+22)           /* GPIOA, PIN 22 */
#  define SAM3U_IRQ_PA23        (SAM3U_IRQ_GPIOA_PINS+23)           /* GPIOA, PIN 23 */
#  define SAM3U_IRQ_PA24        (SAM3U_IRQ_GPIOA_PINS+24)           /* GPIOA, PIN 24 */
#  define SAM3U_IRQ_PA25        (SAM3U_IRQ_GPIOA_PINS+25)           /* GPIOA, PIN 25 */
#  define SAM3U_IRQ_PA26        (SAM3U_IRQ_GPIOA_PINS+26)           /* GPIOA, PIN 26 */
#  define SAM3U_IRQ_PA27        (SAM3U_IRQ_GPIOA_PINS+27)           /* GPIOA, PIN 27 */
#  define SAM3U_IRQ_PA28        (SAM3U_IRQ_GPIOA_PINS+28)           /* GPIOA, PIN 28 */
#  define SAM3U_IRQ_PA29        (SAM3U_IRQ_GPIOA_PINS+29)           /* GPIOA, PIN 29 */
#  define SAM3U_IRQ_PA30        (SAM3U_IRQ_GPIOA_PINS+30)           /* GPIOA, PIN 30 */
#  define SAM3U_IRQ_PA31        (SAM3U_IRQ_GPIOA_PINS+31)           /* GPIOA, PIN 31 */
#  define SAM3U_NGPIOAIRQS      32
#else
#  define SAM3U_NGPIOAIRQS      0
#endif

#ifdef CONFIG_GPIOB_IRQ
#  define SAM3U_IRQ_GPIOB_PINS  (SAM3U_IRQ_EXTINT+SAM3U_IRQ_NEXTINT+SAM3U_IRQ_GPIOA_PINS)
#  define SAM3U_IRQ_PB0         (SAM3U_IRQ_GPIOB_PINS+0)            /* GPIOB, PIN 0 */
#  define SAM3U_IRQ_PB1         (SAM3U_IRQ_GPIOB_PINS+1)            /* GPIOB, PIN 1 */
#  define SAM3U_IRQ_PB2         (SAM3U_IRQ_GPIOB_PINS+2)            /* GPIOB, PIN 2 */
#  define SAM3U_IRQ_PB3         (SAM3U_IRQ_GPIOB_PINS+3)            /* GPIOB, PIN 3 */
#  define SAM3U_IRQ_PB4         (SAM3U_IRQ_GPIOB_PINS+4)            /* GPIOB, PIN 4 */
#  define SAM3U_IRQ_PB5         (SAM3U_IRQ_GPIOB_PINS+5)            /* GPIOB, PIN 5 */
#  define SAM3U_IRQ_PB6         (SAM3U_IRQ_GPIOB_PINS+6)            /* GPIOB, PIN 6 */
#  define SAM3U_IRQ_PB7         (SAM3U_IRQ_GPIOB_PINS+7)            /* GPIOB, PIN 7 */
#  define SAM3U_IRQ_PB8         (SAM3U_IRQ_GPIOB_PINS+8)            /* GPIOB, PIN 8 */
#  define SAM3U_IRQ_PB9         (SAM3U_IRQ_GPIOB_PINS+9)            /* GPIOB, PIN 9 */
#  define SAM3U_IRQ_PB10        (SAM3U_IRQ_GPIOB_PINS+10)           /* GPIOB, PIN 10 */
#  define SAM3U_IRQ_PB11        (SAM3U_IRQ_GPIOB_PINS+11)           /* GPIOB, PIN 11 */
#  define SAM3U_IRQ_PB12        (SAM3U_IRQ_GPIOB_PINS+12)           /* GPIOB, PIN 12 */
#  define SAM3U_IRQ_PB13        (SAM3U_IRQ_GPIOB_PINS+13)           /* GPIOB, PIN 13 */
#  define SAM3U_IRQ_PB14        (SAM3U_IRQ_GPIOB_PINS+14)           /* GPIOB, PIN 14 */
#  define SAM3U_IRQ_PB15        (SAM3U_IRQ_GPIOB_PINS+15)           /* GPIOB, PIN 15 */
#  define SAM3U_IRQ_PB16        (SAM3U_IRQ_GPIOB_PINS+16)           /* GPIOB, PIN 16 */
#  define SAM3U_IRQ_PB17        (SAM3U_IRQ_GPIOB_PINS+17)           /* GPIOB, PIN 17 */
#  define SAM3U_IRQ_PB18        (SAM3U_IRQ_GPIOB_PINS+18)           /* GPIOB, PIN 18 */
#  define SAM3U_IRQ_PB19        (SAM3U_IRQ_GPIOB_PINS+19)           /* GPIOB, PIN 19 */
#  define SAM3U_IRQ_PB20        (SAM3U_IRQ_GPIOB_PINS+20)           /* GPIOB, PIN 20 */
#  define SAM3U_IRQ_PB21        (SAM3U_IRQ_GPIOB_PINS+21)           /* GPIOB, PIN 21 */
#  define SAM3U_IRQ_PB22        (SAM3U_IRQ_GPIOB_PINS+22)           /* GPIOB, PIN 22 */
#  define SAM3U_IRQ_PB23        (SAM3U_IRQ_GPIOB_PINS+23)           /* GPIOB, PIN 23 */
#  define SAM3U_IRQ_PB24        (SAM3U_IRQ_GPIOB_PINS+24)           /* GPIOB, PIN 24 */
#  define SAM3U_IRQ_PB25        (SAM3U_IRQ_GPIOB_PINS+25)           /* GPIOB, PIN 25 */
#  define SAM3U_IRQ_PB26        (SAM3U_IRQ_GPIOB_PINS+26)           /* GPIOB, PIN 26 */
#  define SAM3U_IRQ_PB27        (SAM3U_IRQ_GPIOB_PINS+27)           /* GPIOB, PIN 27 */
#  define SAM3U_IRQ_PB28        (SAM3U_IRQ_GPIOB_PINS+28)           /* GPIOB, PIN 28 */
#  define SAM3U_IRQ_PB29        (SAM3U_IRQ_GPIOB_PINS+29)           /* GPIOB, PIN 29 */
#  define SAM3U_IRQ_PB30        (SAM3U_IRQ_GPIOB_PINS+30)           /* GPIOB, PIN 30 */
#  define SAM3U_IRQ_PB31        (SAM3U_IRQ_GPIOB_PINS+31)           /* GPIOB, PIN 31 */
#  define SAM3U_NGPIOAIRQS      32
#else
#  define SAM3U_NGPIOBIRQS      0
#endif

#ifdef CONFIG_GPIOC_IRQ
#  define SAM3U_IRQ_GPIOC_PINS  (SAM3U_IRQ_EXTINT+SAM3U_IRQ_NEXTINT+SAM3U_IRQ_GPIOA_PINS+SAM3U_IRQ_GPIOB_PINS)
#  define SAM3U_IRQ_PC0         (SAM3U_IRQ_GPIOC_PINS+0)            /* GPIOC, PIN 0 */
#  define SAM3U_IRQ_PC1         (SAM3U_IRQ_GPIOC_PINS+1)            /* GPIOC, PIN 1 */
#  define SAM3U_IRQ_PC2         (SAM3U_IRQ_GPIOC_PINS+2)            /* GPIOC, PIN 2 */
#  define SAM3U_IRQ_PC3         (SAM3U_IRQ_GPIOC_PINS+3)            /* GPIOC, PIN 3 */
#  define SAM3U_IRQ_PC4         (SAM3U_IRQ_GPIOC_PINS+4)            /* GPIOC, PIN 4 */
#  define SAM3U_IRQ_PC5         (SAM3U_IRQ_GPIOC_PINS+5)            /* GPIOC, PIN 5 */
#  define SAM3U_IRQ_PC6         (SAM3U_IRQ_GPIOC_PINS+6)            /* GPIOC, PIN 6 */
#  define SAM3U_IRQ_PC7         (SAM3U_IRQ_GPIOC_PINS+7)            /* GPIOC, PIN 7 */
#  define SAM3U_IRQ_PC8         (SAM3U_IRQ_GPIOC_PINS+8)            /* GPIOC, PIN 8 */
#  define SAM3U_IRQ_PC9         (SAM3U_IRQ_GPIOC_PINS+9)            /* GPIOC, PIN 9 */
#  define SAM3U_IRQ_PC10        (SAM3U_IRQ_GPIOC_PINS+10)           /* GPIOC, PIN 10 */
#  define SAM3U_IRQ_PC11        (SAM3U_IRQ_GPIOC_PINS+11)           /* GPIOC, PIN 11 */
#  define SAM3U_IRQ_PC12        (SAM3U_IRQ_GPIOC_PINS+12)           /* GPIOC, PIN 12 */
#  define SAM3U_IRQ_PC13        (SAM3U_IRQ_GPIOC_PINS+13)           /* GPIOC, PIN 13 */
#  define SAM3U_IRQ_PC14        (SAM3U_IRQ_GPIOC_PINS+14)           /* GPIOC, PIN 14 */
#  define SAM3U_IRQ_PC15        (SAM3U_IRQ_GPIOC_PINS+15)           /* GPIOC, PIN 15 */
#  define SAM3U_IRQ_PC16        (SAM3U_IRQ_GPIOC_PINS+16)           /* GPIOC, PIN 16 */
#  define SAM3U_IRQ_PC17        (SAM3U_IRQ_GPIOC_PINS+17)           /* GPIOC, PIN 17 */
#  define SAM3U_IRQ_PC18        (SAM3U_IRQ_GPIOC_PINS+18)           /* GPIOC, PIN 18 */
#  define SAM3U_IRQ_PC19        (SAM3U_IRQ_GPIOC_PINS+19)           /* GPIOC, PIN 19 */
#  define SAM3U_IRQ_PC20        (SAM3U_IRQ_GPIOC_PINS+20)           /* GPIOC, PIN 20 */
#  define SAM3U_IRQ_PC21        (SAM3U_IRQ_GPIOC_PINS+21)           /* GPIOC, PIN 21 */
#  define SAM3U_IRQ_PC22        (SAM3U_IRQ_GPIOC_PINS+22)           /* GPIOC, PIN 22 */
#  define SAM3U_IRQ_PC23        (SAM3U_IRQ_GPIOC_PINS+23)           /* GPIOC, PIN 23 */
#  define SAM3U_IRQ_PC24        (SAM3U_IRQ_GPIOC_PINS+24)           /* GPIOC, PIN 24 */
#  define SAM3U_IRQ_PC25        (SAM3U_IRQ_GPIOC_PINS+25)           /* GPIOC, PIN 25 */
#  define SAM3U_IRQ_PC26        (SAM3U_IRQ_GPIOC_PINS+26)           /* GPIOC, PIN 26 */
#  define SAM3U_IRQ_PC27        (SAM3U_IRQ_GPIOC_PINS+27)           /* GPIOC, PIN 27 */
#  define SAM3U_IRQ_PC28        (SAM3U_IRQ_GPIOC_PINS+28)           /* GPIOC, PIN 28 */
#  define SAM3U_IRQ_PC29        (SAM3U_IRQ_GPIOC_PINS+29)           /* GPIOC, PIN 29 */
#  define SAM3U_IRQ_PC30        (SAM3U_IRQ_GPIOC_PINS+30)           /* GPIOC, PIN 30 */
#  define SAM3U_IRQ_PC31        (SAM3U_IRQ_GPIOC_PINS+31)           /* GPIOC, PIN 31 */
#  define SAM3U_NGPIOAIRQS      32
#else
#  define SAM3U_NGPIOCIRQS      0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS                 (SAM3U_IRQ_EXTINT+SAM3U_IRQ_NEXTINT+\
                                 SAM3U_NGPIOAIRQS+SAM3U_NGPIOBIRQS+SAM3U_NGPIOCIRQS)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Inline functions
 ****************************************************************************************/

/****************************************************************************************
 * Public Variables
 ****************************************************************************************/

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

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

#endif /* __ARCH_ARM_INCLUDE_SAM3U_IRQ_H */

