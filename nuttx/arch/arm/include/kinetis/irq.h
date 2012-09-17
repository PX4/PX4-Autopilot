/************************************************************************************
 * arch/arm/include/kinetis/irq.h
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
 ************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_KINETIS_IRQ_H
#define __ARCH_ARM_INCLUDE_KINETIS_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* IRQ numbers **********************************************************************/
/* The IRQ numbers corresponds directly to vector numbers and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define KINETIS_IRQ_RESERVED      (0)   /* Reserved vector (only used with CONFIG_DEBUG) */
                                        /* Vector  0: Reset stack pointer value */
                                        /* Vector  1: Reset (not handler as an IRQ) */
#define KINETIS_IRQ_NMI           (2)   /* Vector  2: Non-Maskable Interrupt (NMI) */
#define KINETIS_IRQ_HARDFAULT     (3)   /* Vector  3: Hard fault */
#define KINETIS_IRQ_MEMFAULT      (4)   /* Vector  4: Memory management (MPU) */
#define KINETIS_IRQ_BUSFAULT      (5)   /* Vector  5: Bus fault */
#define KINETIS_IRQ_USAGEFAULT    (6)   /* Vector  6: Usage fault */
                                        /* Vectors 7-10: Reserved */
#define KINETIS_IRQ_SVCALL        (11)  /* Vector 11: SVC call */
#define KINETIS_IRQ_DBGMONITOR    (12)  /* Vector 12: Debug Monitor */
                                        /* Vector 13: Reserved */
#define KINETIS_IRQ_PENDSV        (14)  /* Vector 14: Pendable system service request */
#define KINETIS_IRQ_SYSTICK       (15)  /* Vector 15: System tick */

/* External interrupts (vectors >= 16) */

#define KINETIS_IRQ_EXTINT        (16)

/* K40 Family ***********************************************************************
 *
 * The interrupt vectors  for the following parts is defined in Freescale document
 * K40P144M100SF2RM
 */

#if defined(CONFIG_ARCH_CHIP_MK40X128VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X128VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40X256VLQ100) || defined(CONFIG_ARCH_CHIP_MK40X256VMD100) || \
    defined(CONFIG_ARCH_CHIP_MK40N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK40N512VMD100)

#  define KINETIS_IRQ_DMACH0      (16)  /* Vector 16: DMA channel 0 transfer complete */
#  define KINETIS_IRQ_DMACH1      (17)  /* Vector 17: DMA channel 1 transfer complete */
#  define KINETIS_IRQ_DMACH2      (18)  /* Vector 18: DMA channel 2 transfer complete */
#  define KINETIS_IRQ_DMACH3      (19)  /* Vector 19: DMA channel 3 transfer complete */
#  define KINETIS_IRQ_DMACH4      (20)  /* Vector 20: DMA channel 4 transfer complete */
#  define KINETIS_IRQ_DMACH5      (21)  /* Vector 21: DMA channel 5 transfer complete */
#  define KINETIS_IRQ_DMACH6      (22)  /* Vector 22: DMA channel 6 transfer complete */
#  define KINETIS_IRQ_DMACH7      (23)  /* Vector 23: DMA channel 7 transfer complete */
#  define KINETIS_IRQ_DMACH8      (24)  /* Vector 24: DMA channel 8 transfer complete */
#  define KINETIS_IRQ_DMACH9      (25)  /* Vector 25: DMA channel 9 transfer complete */
#  define KINETIS_IRQ_DMACH10     (26)  /* Vector 26: DMA channel 10 transfer complete */
#  define KINETIS_IRQ_DMACH11     (27)  /* Vector 27: DMA channel 11 transfer complete */
#  define KINETIS_IRQ_DMACH12     (28)  /* Vector 28: DMA channel 12 transfer complete */
#  define KINETIS_IRQ_DMACH13     (29)  /* Vector 29: DMA channel 13 transfer complete */
#  define KINETIS_IRQ_DMACH14     (30)  /* Vector 30: DMA channel 14 transfer complete */
#  define KINETIS_IRQ_DMACH15     (31)  /* Vector 31: DMA channel 15 transfer complete */
#  define KINETIS_IRQ_DMAERR      (32)  /* Vector 32: DMA error interrupt channels 0-15 */
#  define KINETIS_IRQ_MCM         (33)  /* Vector 33: MCM Normal interrupt */
#  define KINETIS_IRQ_FLASHCC     (34)  /* Vector 34: Flash memory command complete */
#  define KINETIS_IRQ_FLASHRC     (35)  /* Vector 35: Flash memory read collision */
#  define KINETIS_IRQ_SMCLVD      (36)  /* Vector 36: Mode Controller low-voltage
                                         *            detect, low-voltage warning */
#  define KINETIS_IRQ_LLWU        (37)  /* Vector 37: LLWU Normal Low Leakage Wakeup */
#  define KINETIS_IRQ_WDOG        (38)  /* Vector 38: Watchdog */
                                        /* Vector 39: Reserved */
#  define KINETIS_IRQ_I2C0        (40)  /* Vector 40: I2C0 */
#  define KINETIS_IRQ_I2C1        (41)  /* Vector 41: I2C1 */
#  define KINETIS_IRQ_SPI0        (42)  /* Vector 42: SPI0 all sources */
#  define KINETIS_IRQ_SPI1        (43)  /* Vector 43: SPI1 all sources */
#  define KINETIS_IRQ_SPI2        (44)  /* Vector 44: SPI2 all sources */
#  define KINETIS_IRQ_CAN0MB      (45)  /* Vector 45: CAN0 OR'ed Message buffer (0-15) */
#  define KINETIS_IRQ_CAN0BO      (46)  /* Vector 46: CAN0 Bus Off */
#  define KINETIS_IRQ_CAN0ERR     (47)  /* Vector 47: CAN0 Error */
#  define KINETIS_IRQ_CAN0TW      (48)  /* Vector 48: CAN0 Transmit Warning */
#  define KINETIS_IRQ_CAN0RW      (49)  /* Vector 49: CAN0 Receive Warning */
#  define KINETIS_IRQ_CAN0WU      (50)  /* Vector 50: CAN0 Wake UP */
                                        /* Vectors 51-52: Reserved */
#  define KINETIS_IRQ_CAN1MB      (53)  /* Vector 53: CAN1 OR'ed Message buffer (0-15) */
#  define KINETIS_IRQ_CAN1BO      (54)  /* Vector 54: CAN1 Bus Off */
#  define KINETIS_IRQ_CAN1ERR     (55)  /* Vector 55: CAN1 Error */
#  define KINETIS_IRQ_CAN1TW      (56)  /* Vector 56: CAN1 Transmit Warning */
#  define KINETIS_IRQ_CAN1RW      (57)  /* Vector 57: CAN1 Receive Warning */
#  define KINETIS_IRQ_CAN1WU      (58)  /* Vector 58: CAN1 Wake UP */
                                        /* Vectors 59-60: Reserved */
#  define KINETIS_IRQ_UART0S      (61)  /* Vector 61: UART0 status */
#  define KINETIS_IRQ_UART0E      (62)  /* Vector 62: UART0 error */
#  define KINETIS_IRQ_UART1S      (63)  /* Vector 63: UART1 status */
#  define KINETIS_IRQ_UART1E      (64)  /* Vector 64: UART1 error */
#  define KINETIS_IRQ_UART2S      (65)  /* Vector 65: UART2 status */
#  define KINETIS_IRQ_UART2E      (66)  /* Vector 66: UART2 error */
#  define KINETIS_IRQ_UART3S      (67)  /* Vector 67: UART3 status */
#  define KINETIS_IRQ_UART3E      (68)  /* Vector 68: UART3 error */
#  define KINETIS_IRQ_UART4S      (69)  /* Vector 69: UART4 status */
#  define KINETIS_IRQ_UART4E      (70)  /* Vector 70: UART4 error */
#  define KINETIS_IRQ_UART5S      (71)  /* Vector 71: UART5 status */
#  define KINETIS_IRQ_UART5E      (72)  /* Vector 72: UART5 error */
#  define KINETIS_IRQ_ADC0        (73)  /* Vector 73: ADC0 */
#  define KINETIS_IRQ_ADC1        (74)  /* Vector 74: ADC1 */
#  define KINETIS_IRQ_CMP0        (75)  /* Vector 75: CMP0 */
#  define KINETIS_IRQ_CMP1        (76)  /* Vector 76: CMP1 */
#  define KINETIS_IRQ_CMP2        (77)  /* Vector 77: CMP2 */
#  define KINETIS_IRQ_FTM0        (78)  /* Vector 78: FTM0 all sources */
#  define KINETIS_IRQ_FTM1        (79)  /* Vector 79: FTM1 all sources */
#  define KINETIS_IRQ_FTM2        (80)  /* Vector 80: FTM2 all sources */
#  define KINETIS_IRQ_CMT         (81)  /* Vector 81: CMT */
#  define KINETIS_IRQ_RTC         (82)  /* Vector 82: RTC alarm interrupt */
                                        /* Vector 83: Reserved */
#  define KINETIS_IRQ_PITCH0      (84)  /* Vector 84: PIT channel 0 */
#  define KINETIS_IRQ_PITCH1      (85)  /* Vector 85: PIT channel 1 */
#  define KINETIS_IRQ_PITCH2      (86)  /* Vector 86: PIT channel 2 */
#  define KINETIS_IRQ_PITCH3      (87)  /* Vector 87: PIT channel 3 */
#  define KINETIS_IRQ_PDB         (88)  /* Vector 88: PDB */
#  define KINETIS_IRQ_USBOTG      (89)  /* Vector 88: USB OTG */
#  define KINETIS_IRQ_USBCD       (90)  /* Vector 90: USB charger detect */
                                        /* Vectors 91-94: Reserved */
#  define KINETIS_IRQ_I2S0        (95)  /* Vector 95: I2S0 */
#  define KINETIS_IRQ_SDHC        (96)  /* Vector 96: SDHC */
#  define KINETIS_IRQ_DAC0        (97)  /* Vector 97: DAC0 */
#  define KINETIS_IRQ_DAC1        (98)  /* Vector 98: DAC1 */
#  define KINETIS_IRQ_TSI         (99)  /* Vector 97: TSI all sources */
#  define KINETIS_IRQ_MCG         (100) /* Vector 100: MCG */
#  define KINETIS_IRQ_LPT         (101) /* Vector 101: Low power timer */
#  define KINETIS_IRQ_SLCD        (102) /* Vector 102: Segment LCD all sources */
#  define KINETIS_IRQ_PORTA       (103) /* Vector 103: Pin detect port A */
#  define KINETIS_IRQ_PORTB       (104) /* Vector 104: Pin detect port B */
#  define KINETIS_IRQ_PORTC       (105) /* Vector 105: Pin detect port C */
#  define KINETIS_IRQ_PORTD       (106) /* Vector 106: Pin detect port D */
#  define KINETIS_IRQ_PORTE       (107) /* Vector 107: Pin detect port E */
                                        /* Vectors 108-109: Reserved */
#  define KINETIS_IRQ_SWI         (110) /* Vector 110: Software interrupt */

/* Note that the total number of IRQ numbers supported is equal to the number of
 * valid interrupt vectors.  This is wasteful in that certain tables are sized by
 * this value.  There are only 94 valid interrupts so, potentially the numver of
 * IRQs to could be reduced to 94.  However, equating IRQ numbers with vector numbers
 * also simplifies operations on NVIC registers and (at least in my state of mind
 * now) seems to justify the waste.
 */

#  define NR_VECTORS              (111) /* 111 vectors */
#  define NR_IRQS                 (111) /* 94 interrupts but 111 IRQ numbers */

/* K60 Family ***********************************************************************
 *
 * The memory map for the following parts is defined in Freescale document
 * K60P144M100SF2RM
 */

#elif defined(CONFIG_ARCH_CHIP_MK60N256VLQ100) || defined(CONFIG_ARCH_CHIP_MK60X256VLQ100) || \
      defined(CONFIG_ARCH_CHIP_MK60N512VLQ100) || defined(CONFIG_ARCH_CHIP_MK60N256VMD100) || \
      defined(CONFIG_ARCH_CHIP_MK60X256VMD100) || defined(CONFIG_ARCH_CHIP_MK60N512VMD100)

#  define KINETIS_IRQ_DMACH0      (16)  /* Vector 16: DMA channel 0 transfer complete */
#  define KINETIS_IRQ_DMACH1      (17)  /* Vector 17: DMA channel 1 transfer complete */
#  define KINETIS_IRQ_DMACH2      (18)  /* Vector 18: DMA channel 2 transfer complete */
#  define KINETIS_IRQ_DMACH3      (19)  /* Vector 19: DMA channel 3 transfer complete */
#  define KINETIS_IRQ_DMACH4      (20)  /* Vector 20: DMA channel 4 transfer complete */
#  define KINETIS_IRQ_DMACH5      (21)  /* Vector 21: DMA channel 5 transfer complete */
#  define KINETIS_IRQ_DMACH6      (22)  /* Vector 22: DMA channel 6 transfer complete */
#  define KINETIS_IRQ_DMACH7      (23)  /* Vector 23: DMA channel 7 transfer complete */
#  define KINETIS_IRQ_DMACH8      (24)  /* Vector 24: DMA channel 8 transfer complete */
#  define KINETIS_IRQ_DMACH9      (25)  /* Vector 25: DMA channel 9 transfer complete */
#  define KINETIS_IRQ_DMACH10     (26)  /* Vector 26: DMA channel 10 transfer complete */
#  define KINETIS_IRQ_DMACH11     (27)  /* Vector 27: DMA channel 11 transfer complete */
#  define KINETIS_IRQ_DMACH12     (28)  /* Vector 28: DMA channel 12 transfer complete */
#  define KINETIS_IRQ_DMACH13     (29)  /* Vector 29: DMA channel 13 transfer complete */
#  define KINETIS_IRQ_DMACH14     (30)  /* Vector 30: DMA channel 14 transfer complete */
#  define KINETIS_IRQ_DMACH15     (31)  /* Vector 31: DMA channel 15 transfer complete */
#  define KINETIS_IRQ_DMAERR      (32)  /* Vector 32: DMA error interrupt channels 0-15 */
#  define KINETIS_IRQ_MCM         (33)  /* Vector 33: MCM Normal interrupt */
#  define KINETIS_IRQ_FLASHCC     (34)  /* Vector 34: Flash memory command complete */
#  define KINETIS_IRQ_FLASHRC     (35)  /* Vector 35: Flash memory read collision */
#  define KINETIS_IRQ_SMCLVD      (36)  /* Vector 36: Mode Controller low-voltage
                                         *            detect, low-voltage warning */
#  define KINETIS_IRQ_LLWU        (37)  /* Vector 37: LLWU Normal Low Leakage Wakeup */
#  define KINETIS_IRQ_WDOG        (38)  /* Vector 38: Watchdog */
#  define KINETIS_IRQ_RNGB        (39)  /* Vector 39: Random number generator */
#  define KINETIS_IRQ_I2C0        (40)  /* Vector 40: I2C0 */
#  define KINETIS_IRQ_I2C1        (41)  /* Vector 41: I2C1 */
#  define KINETIS_IRQ_SPI0        (42)  /* Vector 42: SPI0 all sources */
#  define KINETIS_IRQ_SPI1        (43)  /* Vector 43: SPI1 all sources */
#  define KINETIS_IRQ_SPI2        (44)  /* Vector 44: SPI2 all sources */
#  define KINETIS_IRQ_CAN0MB      (45)  /* Vector 45: CAN0 OR'ed Message buffer (0-15) */
#  define KINETIS_IRQ_CAN0BO      (46)  /* Vector 46: CAN0 Bus Off */
#  define KINETIS_IRQ_CAN0ERR     (47)  /* Vector 47: CAN0 Error */
#  define KINETIS_IRQ_CAN0TW      (48)  /* Vector 48: CAN0 Transmit Warning */
#  define KINETIS_IRQ_CAN0RW      (49)  /* Vector 49: CAN0 Receive Warning */
#  define KINETIS_IRQ_CAN0WU      (50)  /* Vector 50: CAN0 Wake UP */
                                        /* Vectors 51-52: Reserved */
#  define KINETIS_IRQ_CAN1MB      (53)  /* Vector 53: CAN1 OR'ed Message buffer (0-15) */
#  define KINETIS_IRQ_CAN1BO      (54)  /* Vector 54: CAN1 Bus Off */
#  define KINETIS_IRQ_CAN1ERR     (55)  /* Vector 55: CAN1 Error */
#  define KINETIS_IRQ_CAN1TW      (56)  /* Vector 56: CAN1 Transmit Warning */
#  define KINETIS_IRQ_CAN1RW      (57)  /* Vector 57: CAN1 Receive Warning */
#  define KINETIS_IRQ_CAN1WU      (58)  /* Vector 58: CAN1 Wake UP */
                                        /* Vectors 59-60: Reserved */
#  define KINETIS_IRQ_UART0S      (61)  /* Vector 61: UART0 status */
#  define KINETIS_IRQ_UART0E      (62)  /* Vector 62: UART0 error */
#  define KINETIS_IRQ_UART1S      (63)  /* Vector 63: UART1 status */
#  define KINETIS_IRQ_UART1E      (64)  /* Vector 64: UART1 error */
#  define KINETIS_IRQ_UART2S      (65)  /* Vector 65: UART2 status */
#  define KINETIS_IRQ_UART2E      (66)  /* Vector 66: UART2 error */
#  define KINETIS_IRQ_UART3S      (67)  /* Vector 67: UART3 status */
#  define KINETIS_IRQ_UART3E      (68)  /* Vector 68: UART3 error */
#  define KINETIS_IRQ_UART4S      (69)  /* Vector 69: UART4 status */
#  define KINETIS_IRQ_UART4E      (70)  /* Vector 70: UART4 error */
#  define KINETIS_IRQ_UART5S      (71)  /* Vector 71: UART5 status */
#  define KINETIS_IRQ_UART5E      (72)  /* Vector 72: UART5 error */
#  define KINETIS_IRQ_ADC0        (73)  /* Vector 73: ADC0 */
#  define KINETIS_IRQ_ADC1        (74)  /* Vector 74: ADC1 */
#  define KINETIS_IRQ_CMP0        (75)  /* Vector 75: CMP0 */
#  define KINETIS_IRQ_CMP1        (76)  /* Vector 76: CMP1 */
#  define KINETIS_IRQ_CMP2        (77)  /* Vector 77: CMP2 */
#  define KINETIS_IRQ_FTM0        (78)  /* Vector 78: FTM0 all sources */
#  define KINETIS_IRQ_FTM1        (79)  /* Vector 79: FTM1 all sources */
#  define KINETIS_IRQ_FTM2        (80)  /* Vector 80: FTM2 all sources */
#  define KINETIS_IRQ_CMT         (81)  /* Vector 81: CMT */
#  define KINETIS_IRQ_RTC         (82)  /* Vector 82: RTC alarm interrupt */
                                        /* Vector 83: Reserved */
#  define KINETIS_IRQ_PITCH0      (84)  /* Vector 84: PIT channel 0 */
#  define KINETIS_IRQ_PITCH1      (85)  /* Vector 85: PIT channel 1 */
#  define KINETIS_IRQ_PITCH2      (86)  /* Vector 86: PIT channel 2 */
#  define KINETIS_IRQ_PITCH3      (87)  /* Vector 87: PIT channel 3 */
#  define KINETIS_IRQ_PDB         (88)  /* Vector 88: PDB */
#  define KINETIS_IRQ_USBOTG      (89)  /* Vector 88: USB OTG */
#  define KINETIS_IRQ_USBCD       (90)  /* Vector 90: USB charger detect */
#  define KINETIS_IRQ_EMACTMR     (91)  /* Vector 91: Ethernet MAC IEEE 1588 timer interrupt */
#  define KINETIS_IRQ_EMACTX      (92)  /* Vector 92: Ethernet MAC transmit interrupt */
#  define KINETIS_IRQ_EMACRX      (93)  /* Vector 93: Ethernet MAC receive interrupt */
#  define KINETIS_IRQ_EMACMISC    (94)  /* Vector 94: Ethernet MAC error and misc interrupt */
#  define KINETIS_IRQ_I2S0        (95)  /* Vector 95: I2S0 */
#  define KINETIS_IRQ_SDHC        (96)  /* Vector 96: SDHC */
#  define KINETIS_IRQ_DAC0        (97)  /* Vector 97: DAC0 */
#  define KINETIS_IRQ_DAC1        (98)  /* Vector 98: DAC1 */
#  define KINETIS_IRQ_TSI         (99)  /* Vector 97: TSI all sources */
#  define KINETIS_IRQ_MCG         (100) /* Vector 100: MCG */
#  define KINETIS_IRQ_LPT         (101) /* Vector 101: Low power timer */
                                        /* Vector 102: Reserved */
#  define KINETIS_IRQ_PORTA       (103) /* Vector 103: Pin detect port A */
#  define KINETIS_IRQ_PORTB       (104) /* Vector 104: Pin detect port B */
#  define KINETIS_IRQ_PORTC       (105) /* Vector 105: Pin detect port C */
#  define KINETIS_IRQ_PORTD       (106) /* Vector 106: Pin detect port D */
#  define KINETIS_IRQ_PORTE       (107) /* Vector 107: Pin detect port E */
                                        /* Vectors 108-119: Reserved */

/* Note that the total number of IRQ numbers supported is equal to the number of
 * valid interrupt vectors.  This is wasteful in that certain tables are sized by
 * this value.  There are only 97 valid interrupts so, potentially the numver of
 * IRQs to could be reduced to 97.  However, equating IRQ numbers with vector numbers
 * also simplifies operations on NVIC registers and (at least in my state of mind
 * now) seems to justify the waste.
 */

#  define NR_VECTORS              (120) /* 120 vectors */
#  define NR_IRQS                 (108) /* 97 interrupts but 108 IRQ numbers */

#else
  /* The interrupt vectors for other parts are defined in other documents and may or
   * may not be the same as above (the family members are all very similar)  This
   * error just means that you have to look at the document and determine for yourself
   * if the memory map is the same.
   */

#  error "No IRQ numbers for this Kinetis part"
#endif

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

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_KINETIS_IRQ_H */

