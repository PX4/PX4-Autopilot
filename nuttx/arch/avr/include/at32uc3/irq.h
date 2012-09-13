/****************************************************************************
 * arch/avr/include/at32uc3/irq.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_AT32UC3_IRQ_H
#define __ARCH_AVR_INCLUDE_AT32UC3_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the overall
 * GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 * CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 * interrupts on.
 */

#ifndef CONFIG_AVR32_GPIOIRQ
#  undef CONFIG_AVR32_GPIOIRQSETA
#  undef CONFIG_AVR32_GPIOIRQSETB
#endif
 
/* IRQ numbers **************************************************************/
/* Events.  These exclude:
 *
 * - The Reset event which vectors directly either to 0x8000:0000 (uc3a) or
 *   to 0xa000:0000 (uc3b).
 * - The OCD stop from the OSD system
 * - Autovectored interrupt requests
 *
 * Others vector relative to the contents of the EVBA register.
 */

#define AVR32_IRQ_UNREC         0 /* EVBA+0x00 Unrecoverable exception */
#define AVR32_IRQ_TLBMULT       1 /* EVBA+0x04 TLB multiple hit */
#define AVR32_IRQ_BUSDATA       2 /* EVBA+0x08 Bus error data fetch */
#define AVR32_IRQ_BUSINST       3 /* EVBA+0x0c Bus error instruction fetch */
#define AVR32_IRQ_NMI           4 /* EVBA+0x10 NMI */
#define AVR32_IRQ_INSTADDR      5 /* EVBA+0x14 Instruction Address */
#define AVR32_IRQ_ITLBPROT      6 /* EVBA+0x18 ITLB Protection */
#define AVR32_IRQ_BP            7 /* EVBA+0x1c Breakpoint */
#define AVR32_IRQ_INVINST       8 /* EVBA+0x20 Illegal Opcode */
#define AVR32_IRQ_UNIMPINST     9 /* EVBA+0x24 Unimplemented instruction */
#define AVR32_IRQ_PRIV         10 /* EVBA+0x28 Privilege violation */
#define AVR32_IRQ_FP           11 /* EVBA+0x2c Floating-point */
#define AVR32_IRQ_COP          12 /* EVBA+0x30 Coprocessor absent */
#define AVR32_IRQ_RDDATA       13 /* EVBA+0x34 Data Address (Read) */
#define AVR32_IRQ_WRDATA       14 /* EVBA+0x38 Data Address (Write) */
#define AVR32_IRQ_RDDTLBPROT   15 /* EVBA+0x3c DTLB Protection (Read) */
#define AVR32_IRQ_WRDTLBPROT   16 /* EVBA+0x40 DTLB Protection (Write) */
#define AVR32_IRQ_DLTBMOD      17 /* EVBA+0x44 DTLB Modified */
#define AVR32_IRQ_ITLBMISS     18 /* EVBA+0x50 ITLB Miss */
#define AVR32_IRQ_RDDTLB       19 /* EVBA+0x60 DTLB Miss (Read) */
#define AVR32_IRQ_WRDTLB       20 /* EVBA+0x70 DTLB Miss (Write) */
#define AVR32_IRQ_SUPER        21 /* EVBA+0x100 Supervisor call */
#define AVR32_IRQ_NEVENTS      22

/* "The INTC collects interrupt requests from the peripherals, prioritizes
 *  them, and delivers an interrupt request and an autovector to the CPU. The
 *  AVR32 architecture supports 4 priority levels for regular, maskable
 *  interrupts, and a Non-Maskable Interrupt (NMI)."
 *
 * "The INTC supports up to 64 groups of interrupts. Each group can have up
 *  to 32 interrupt request lines, these lines are connected to the peripherals.
 *  Each group has an Interrupt Priority Register (IPR) and an Interrupt Request
 *  Register (IRR). The IPRs are used to assign a priority level and an autovector
 *  to each group, and the IRRs are used to identify the active interrupt request
 *  within each group. If a group has only one interrupt request line, an active
 *  interrupt group uniquely identifies the active interrupt request line, and
 *  the corresponding IRR is not needed. The INTC also provides one Interrupt
 *  Cause Register (ICR) per priority level. These registers identify the group
 *  that has a pending interrupt of the corresponding priority level. If several
 *  groups have a pending interrupt of the same level, the group with the lowest
 *  number takes priority."
 */

/* Only 19 groups (0-18) are used with the AT32UC3A/B: */

#define AVR32_IRQ_INTPRIOS      4 /* 4 interrupt priorities */
#define AVR32_IRQ_MAXGROUPS    64 /* Architecture supports up to 64 groups */
#define AVR32_IRQ_NGROUPS      19 /* UC3 A/B support only 19 */

/* Group 0 */

#define AVR32_IRQ_BASEIRQGRP0  22
#define AVR32_IRQ_NREQGRP0      1

#define AVR32_IRQ_UC           22 /* 0 AVR32 UC CPU */

/* Group 1 */

#define AVR32_IRQ_BASEIRQGRP1  23
#define AVR32_IRQ_NREQGRP1     10

#define AVR32_IRQ_EIC0         23 /* 0 External Interrupt Controller 0 */
#define AVR32_IRQ_EIC1         24 /* 1 External Interrupt Controller 1 */
#define AVR32_IRQ_EIC2         25 /* 2 External Interrupt Controller 2 */
#define AVR32_IRQ_EIC3         26 /* 3 External Interrupt Controller 3 */
#define AVR32_IRQ_EIC4         27 /* 4 External Interrupt Controller 4 */
#define AVR32_IRQ_EIC5         28 /* 5 External Interrupt Controller 5 */
#define AVR32_IRQ_EIC6         29 /* 6 External Interrupt Controller 6 */
#define AVR32_IRQ_EIC7         30 /* 7 External Interrupt Controller 7 */
#define AVR32_IRQ_RTC          31 /* 8 Real Time Counter RTC */
#define AVR32_IRQ_PM           32 /* 9 Power Manager PM */

/* Group 2 */

#define AVR32_IRQ_BASEIRQGRP2  33
#define AVR32_IRQ_NREQGRP2      6

#define AVR32_IRQ_GPIO0        33 /* 0 General Purpose Input/Output Controller 0 */
#define AVR32_IRQ_GPIO1        34 /* 1 General Purpose Input/Output Controller 1 */
#define AVR32_IRQ_GPIO2        35 /* 2 General Purpose Input/Output Controller 2 */
#define AVR32_IRQ_GPIO3        36 /* 3 General Purpose Input/Output Controller 3 */
#define AVR32_IRQ_GPIO4        37 /* 4 General Purpose Input/Output Controller 4 */
#define AVR32_IRQ_GPIO5        38 /* 5 General Purpose Input/Output Controller 5 */

/* Group 3 */

#define AVR32_IRQ_BASEIRQGRP3  39
#define AVR32_IRQ_NREQGRP3      7

#define AVR32_IRQ_PDCA0        39 /* 0 Peripheral DMA Controller 0 */
#define AVR32_IRQ_PDCA1        40 /* 1 Peripheral DMA Controller 1 */
#define AVR32_IRQ_PDCA2        41 /* 2 Peripheral DMA Controller 2 */
#define AVR32_IRQ_PDCA3        42 /* 3 Peripheral DMA Controller 3 */
#define AVR32_IRQ_PDCA4        43 /* 4 Peripheral DMA Controller 4 */
#define AVR32_IRQ_PDCA5        44 /* 5 Peripheral DMA Controller 5 */
#define AVR32_IRQ_PDCA6        45 /* 6 Peripheral DMA Controller 6 */

/* Group 4 */

#define AVR32_IRQ_BASEIRQGRP4  46
#define AVR32_IRQ_NREQGRP4      1

#define AVR32_IRQ_FLASHC       46 /* 0 Flash Controller */

/* Group 5 */

#define AVR32_IRQ_BASEIRQGRP5  47
#define AVR32_IRQ_NREQGRP5      1

#define AVR32_IRQ_USART0       47 /* 0 Universal Synchronous/Asynchronous
                                   *   Receiver/Transmitter 0 */
/* Group 6 */

#define AVR32_IRQ_BASEIRQGRP6  48
#define AVR32_IRQ_NREQGRP6      1

#define AVR32_IRQ_USART1       48 /* 0 Universal Synchronous/Asynchronous
                                   *   Receiver/Transmitter 1 */
/* Group 7 */

#define AVR32_IRQ_BASEIRQGRP7  49
#define AVR32_IRQ_NREQGRP7      1

#define AVR32_IRQ_USART2       49 /* 0 Universal Synchronous/Asynchronous
                                   *   Receiver/Transmitter 2 */

#define AVR32_IRQ_BASEIRQGRP8  50
#define AVR32_IRQ_NREQGRP8      0

/* Group 9 */

#define AVR32_IRQ_BASEIRQGRP9  50
#define AVR32_IRQ_NREQGRP9      1

#define AVR32_IRQ_SPI          50 /* 0 Serial Peripheral Interface */

#define AVR32_IRQ_BASEIRQGRP10 51
#define AVR32_IRQ_NREQGRP10     0

/* Group 11 */

#define AVR32_IRQ_BASEIRQGRP11 51
#define AVR32_IRQ_NREQGRP11     1

#define AVR32_IRQ_TWI          51 /* 0 Two-wire Interface TWI */

/* Group 12 */

#define AVR32_IRQ_BASEIRQGRP12 52
#define AVR32_IRQ_NREQGRP12     1

#define AVR32_IRQ_PWM          52 /* 0 Pulse Width Modulation Controller */

/* Group 13 */

#define AVR32_IRQ_BASEIRQGRP13 53
#define AVR32_IRQ_NREQGRP13     1

#define AVR32_IRQ_SSC          53 /* 0 Synchronous Serial Controller */

/* Group 14 */

#define AVR32_IRQ_BASEIRQGRP14 54
#define AVR32_IRQ_NREQGRP14     3

#define AVR32_IRQ_TC0          54 /* 0 Timer/Counter 0 */
#define AVR32_IRQ_TC1          55 /* 1 Timer/Counter 1 */
#define AVR32_IRQ_TC2          56 /* 2 Timer/Counter 2 */

/* Group 15 */

#define AVR32_IRQ_BASEIRQGRP15 57
#define AVR32_IRQ_NREQGRP15     1

#define AVR32_IRQ_ADC          57 /* 0 Analog to Digital Converter */

#define AVR32_IRQ_BASEIRQGRP16 58
#define AVR32_IRQ_NREQGRP16     0

/* Group 17 */

#define AVR32_IRQ_BASEIRQGRP17 58
#define AVR32_IRQ_NREQGRP17     1

#define AVR32_IRQ_USBB         58 /* 0 USB 2.0 Interface USBB */

/* Group 18 */

#define AVR32_IRQ_BASEIRQGRP18 59
#define AVR32_IRQ_NREQGRP18     1

#define AVR32_IRQ_ABDAC        59 /* 0 Audio Bitstream DAC */

/* Total number of IRQ numbers */

#define AVR32_IRQ_BADVECTOR    60 /* Not a real IRQ number */
#define NR_IRQS                60

/* GPIO IRQ Numbers *********************************************************/
/* These numbers correspond to GPIO port numbers that have interrupts
 * enabled.  These are all decoded by the AVR32_IRQ_GPIO interrupt handler.
 * A lot of effort was made here to keep the number of IRQs to a minimum
 * since it will correspond to various, internal table sizes.
 */

/* Up to 32 GPIO interrupts in PORTA0-31 */

#define __IRQ_GPPIO_PA0       0

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000001) != 0
#  define AVR32_IRQ_GPIO_PA0  __IRQ_GPPIO_PA0
#  define __IRQ_GPIO_PA1      (__IRQ_GPPIO_PA0+1)
#else 
#  define __IRQ_GPIO_PA1      __IRQ_GPPIO_PA0
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000002) != 0
#  define AVR32_IRQ_GPIO_PA1  __IRQ_GPIO_PA1
#  define __IRQ_GPIO_PA2      (__IRQ_GPIO_PA1+1)
#else
#  define __IRQ_GPIO_PA2      __IRQ_GPIO_PA1
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000004) != 0
#  define AVR32_IRQ_GPIO_PA2  __IRQ_GPIO_PA2
#  define __IRQ_GPIO_PA3      (__IRQ_GPIO_PA2+1)
#else
#  define __IRQ_GPIO_PA3      __IRQ_GPIO_PA2
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000008) != 0
#  define AVR32_IRQ_GPIO_PA3  __IRQ_GPIO_PA3
#  define __IRQ_GPIO_PA4      (__IRQ_GPIO_PA3+1)
#else
#  define __IRQ_GPIO_PA4      __IRQ_GPIO_PA3
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000010) != 0
#  define AVR32_IRQ_GPIO_PA4  __IRQ_GPIO_PA4
#  define __IRQ_GPIO_PA5      (__IRQ_GPIO_PA4+1)
#else
#  define __IRQ_GPIO_PA5      __IRQ_GPIO_PA4
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000020) != 0
#  define AVR32_IRQ_GPIO_PA5  __IRQ_GPIO_PA5
#  define __IRQ_GPIO_PA6      (__IRQ_GPIO_PA5+1)
#else
#  define __IRQ_GPIO_PA6      __IRQ_GPIO_PA5
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000040) != 0
#  define AVR32_IRQ_GPIO_PA6  __IRQ_GPIO_PA6
#  define __IRQ_GPIO_PA7      (__IRQ_GPIO_PA6+1)
#else
#  define __IRQ_GPIO_PA7      __IRQ_GPIO_PA6
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000080) != 0
#  define AVR32_IRQ_GPIO_PA7  __IRQ_GPIO_PA7
#  define __IRQ_GPIO_PA8      (__IRQ_GPIO_PA7+1)
#else
#  define __IRQ_GPIO_PA8      __IRQ_GPIO_PA7
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000100) != 0
#  define AVR32_IRQ_GPIO_PA8  __IRQ_GPIO_PA8
#  define __IRQ_GPIO_PA9      (__IRQ_GPIO_PA8+1)
#else
#  define __IRQ_GPIO_PA9      __IRQ_GPIO_PA8
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000200) != 0
#  define AVR32_IRQ_GPIO_PA9  __IRQ_GPIO_PA9
#  define __IRQ_GPIO_PA10     (__IRQ_GPIO_PA9+1)
#else
#  define __IRQ_GPIO_PA10     __IRQ_GPIO_PA9
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000400) != 0
#  define AVR32_IRQ_GPIO_PA10 __IRQ_GPIO_PA10
#  define __IRQ_GPIO_PA11     (__IRQ_GPIO_PA10+1)
#else
#  define __IRQ_GPIO_PA11     __IRQ_GPIO_PA10
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00000800) != 0
#  define AVR32_IRQ_GPIO_PA11 __IRQ_GPIO_PA11
#  define __IRQ_GPIO_PA12     (__IRQ_GPIO_PA11+1)
#else
#  define __IRQ_GPIO_PA12     __IRQ_GPIO_PA11
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00001000) != 0
#  define AVR32_IRQ_GPIO_PA12 __IRQ_GPIO_PA12
#  define __IRQ_GPIO_PA13     (__IRQ_GPIO_PA12+1)
#else
#  define __IRQ_GPIO_PA13     __IRQ_GPIO_PA12
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00002000) != 0
#  define AVR32_IRQ_GPIO_PA13 __IRQ_GPIO_PA13
#  define __IRQ_GPIO_PA14     (__IRQ_GPIO_PA13+1)
#else
#  define __IRQ_GPIO_PA14     __IRQ_GPIO_PA13
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00004000) != 0
#  define AVR32_IRQ_GPIO_PA14 __IRQ_GPIO_PA14
#  define __IRQ_GPIO_PA15     (__IRQ_GPIO_PA14+1)
#else
#  define __IRQ_GPIO_PA15     __IRQ_GPIO_PA14
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00008000) != 0
#  define AVR32_IRQ_GPIO_PA15 __IRQ_GPIO_PA15
#  define __IRQ_GPIO_PA16     (__IRQ_GPIO_PA15+1)
#else
#  define __IRQ_GPIO_PA16     __IRQ_GPIO_PA15
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00010000) != 0
#  define AVR32_IRQ_GPIO_PA16 __IRQ_GPIO_PA16
#  define __IRQ_GPIO_PA17     (__IRQ_GPIO_PA16+1)
#else
#  define __IRQ_GPIO_PA17     __IRQ_GPIO_PA16
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00020000) != 0
#  define AVR32_IRQ_GPIO_PA17 __IRQ_GPIO_PA17
#  define __IRQ_GPIO_PA18     (__IRQ_GPIO_PA17+1)
#else
#  define __IRQ_GPIO_PA18     __IRQ_GPIO_PA17
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00040000) != 0
#  define AVR32_IRQ_GPIO_PA18 __IRQ_GPIO_PA18
#  define __IRQ_GPIO_PA19     (__IRQ_GPIO_PA18+1)
#else
#  define __IRQ_GPIO_PA19     __IRQ_GPIO_PA18
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00080000) != 0
#  define AVR32_IRQ_GPIO_PA19 __IRQ_GPIO_PA19
#  define __IRQ_GPIO_PA20     (__IRQ_GPIO_PA19+1)
#else
#  define __IRQ_GPIO_PA20     __IRQ_GPIO_PA19
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00100000) != 0
#  define AVR32_IRQ_GPIO_PA20 __IRQ_GPIO_PA20
#  define __IRQ_GPIO_PA21     (__IRQ_GPIO_PA20+1)
#else
#  define __IRQ_GPIO_PA21     __IRQ_GPIO_PA20
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00200000) != 0
#  define AVR32_IRQ_GPIO_PA21 __IRQ_GPIO_PA21
#  define __IRQ_GPIO_PA22     (__IRQ_GPIO_PA21+1)
#else
#  define __IRQ_GPIO_PA22     __IRQ_GPIO_PA21
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00400000) != 0
#  define AVR32_IRQ_GPIO_PA22 __IRQ_GPIO_PA22
#  define __IRQ_GPIO_PA23     (__IRQ_GPIO_PA22+1)
#else
#  define __IRQ_GPIO_PA23     __IRQ_GPIO_PA22
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x00800000) != 0
#  define AVR32_IRQ_GPIO_PA23 __IRQ_GPIO_PA23
#  define __IRQ_GPIO_PA24     (__IRQ_GPIO_PA23+1)
#else
#  define __IRQ_GPIO_PA24     __IRQ_GPIO_PA23
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x01000000) != 0
#  define AVR32_IRQ_GPIO_PA24 __IRQ_GPIO_PA24
#  define __IRQ_GPIO_PA25     (__IRQ_GPIO_PA24+1)
#else
#  define __IRQ_GPIO_PA25     __IRQ_GPIO_PA24
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x02000000) != 0
#  define AVR32_IRQ_GPIO_PA25 __IRQ_GPIO_PA25
#  define __IRQ_GPIO_PA26     (__IRQ_GPIO_PA25+1)
#else
#  define __IRQ_GPIO_PA26     __IRQ_GPIO_PA25
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x04000000) != 0
#  define AVR32_IRQ_GPIO_PA26 __IRQ_GPIO_PA26
#  define __IRQ_GPIO_PA27     (__IRQ_GPIO_PA26+1)
#else
#  define __IRQ_GPIO_PA27     __IRQ_GPIO_PA26
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x08000000) != 0
#  define AVR32_IRQ_GPIO_PA27 __IRQ_GPIO_PA27
#  define __IRQ_GPIO_PA28     (__IRQ_GPIO_PA27+1)
#else
#  define __IRQ_GPIO_PA28     __IRQ_GPIO_PA27
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x10000000) != 0
#  define AVR32_IRQ_GPIO_PA28 __IRQ_GPIO_PA28
#  define __IRQ_GPIO_PA29     (__IRQ_GPIO_PA28+1)
#else
#  define __IRQ_GPIO_PA29     __IRQ_GPIO_PA28
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x20000000) != 0
#  define AVR32_IRQ_GPIO_PA29 __IRQ_GPIO_PA29
#  define __IRQ_GPIO_PA30     (__IRQ_GPIO_PA29+1)
#else
#  define __IRQ_GPIO_PA30     __IRQ_GPIO_PA29
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x40000000) != 0
#  define AVR32_IRQ_GPIO_PA30 __IRQ_GPIO_PA30
#  define __IRQ_GPIO_PA31     (__IRQ_GPIO_PA30+1)
#else
#  define __IRQ_GPIO_PA31     __IRQ_GPIO_PA30
#endif

#if (CONFIG_AVR32_GPIOIRQSETA & 0x80000000) != 0
#  define AVR32_IRQ_GPIO_PA31 __IRQ_GPIO_PA31
#  define __IRQ_GPIO_PB0      (__IRQ_GPIO_PA31+1)
#else
#  define __IRQ_GPIO_PB0      __IRQ_GPIO_PA31
#endif


/* Up to 12 GPIO interrupts in PORTB0-11 */

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000001) != 0
#  define AVR32_IRQ_GPIO_PB0  __IRQ_GPIO_pb0
#  define __IRQ_GPIO_PB1      (__IRQ_GPIO_PB0+1)
#else
#  define __IRQ_GPIO_PB1      __IRQ_GPIO_PB0
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000002) != 0
#  define AVR32_IRQ_GPIO_PB1  __IRQ_GPIO_PB1
#  define __IRQ_GPIO_PB2      (__IRQ_GPIO_PB1+1)
#else
#  define __IRQ_GPIO_PB2      __IRQ_GPIO_PB1
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000004) != 0
#  define AVR32_IRQ_GPIO_PB2  __IRQ_GPIO_PB2
#  define __IRQ_GPIO_PB3      (__IRQ_GPIO_PB2+1)
#else
#  define __IRQ_GPIO_PB3      __IRQ_GPIO_PB2
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000008) != 0
#  define AVR32_IRQ_GPIO_PB3  __IRQ_GPIO_PB3
#  define __IRQ_GPIO_PB4      (__IRQ_GPIO_PB3+1)
#else
#  define __IRQ_GPIO_PB4      __IRQ_GPIO_PB3
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000010) != 0
#  define AVR32_IRQ_GPIO_PB4  __IRQ_GPIO_PB4
#  define __IRQ_GPIO_PB5      (__IRQ_GPIO_PB4+1)
#else
#  define __IRQ_GPIO_PB5      __IRQ_GPIO_PB4
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000020) != 0
#  define AVR32_IRQ_GPIO_PB5  __IRQ_GPIO_PB5
#  define __IRQ_GPIO_PB6      (__IRQ_GPIO_PB5+1)
#else
#  define __IRQ_GPIO_PB6      __IRQ_GPIO_PB5
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000040) != 0
#  define AVR32_IRQ_GPIO_PB6  __IRQ_GPIO_PB6
#  define __IRQ_GPIO_PB7      (__IRQ_GPIO_PB6+1)
#else
#  define __IRQ_GPIO_PB7      __IRQ_GPIO_PB6
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000080) != 0
#  define AVR32_IRQ_GPIO_PB7  __IRQ_GPIO_PB7
#  define __IRQ_GPIO_PB8      (__IRQ_GPIO_PB7+1)
#else
#  define __IRQ_GPIO_PB8      __IRQ_GPIO_PB7
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000100) != 0
#  define AVR32_IRQ_GPIO_PB8  __IRQ_GPIO_PB8
#  define __IRQ_GPIO_PB9      (__IRQ_GPIO_PB8+1)
#else
#  define __IRQ_GPIO_PB9      __IRQ_GPIO_PB8
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000200) != 0
#  define AVR32_IRQ_GPIO_PB9  __IRQ_GPIO_PB9
#  define __IRQ_GPIO_PB10     (__IRQ_GPIO_PB9+1)
#else
#  define __IRQ_GPIO_PB10     __IRQ_GPIO_PB9
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000400) != 0
#  define AVR32_IRQ_GPIO_PB10 __IRQ_GPIO_PB10
#  define __IRQ_GPIO_PB11     (__IRQ_GPIO_PB10+1)
#else
#  define __IRQ_GPIO_PB11     __IRQ_GPIO_PB10
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 0x00000800) != 0
#  define AVR32_IRQ_GPIO_PB11 __IRQ_GPIO_PB11
#  define __IRQ_GPIO_PB12     (__IRQ_GPIO_PB11+1)
#else
#  define __IRQ_GPIO_PB12     __IRQ_GPIO_PB11
#endif

#ifdef CONFIG_AVR32_GPIOIRQ
#  define NR_GPIO_IRQS        __IRQ_GPIO_PB12
#else
#  define NR_GPIO_IRQS       0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

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

#endif /* __ARCH_AVR_INCLUDE_AT32UC3_IRQ_H */

