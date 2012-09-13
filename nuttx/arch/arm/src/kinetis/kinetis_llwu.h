/************************************************************************************
 * arch/arm/src/kinetis/kinetis_llwu.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_LLWU_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_LLWU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_LLWU_PE1_OFFSET    0x0000 /* LLWU Pin Enable 1 Register */
#define KINETIS_LLWU_PE2_OFFSET    0x0001 /* LLWU Pin Enable 2 Register */
#define KINETIS_LLWU_PE3_OFFSET    0x0002 /* LLWU Pin Enable 3 Register */
#define KINETIS_LLWU_PE4_OFFSET    0x0003 /* LLWU Pin Enable 4 Register */
#define KINETIS_LLWU_ME_OFFSET     0x0004 /* LLWU Module Enable Register */
#define KINETIS_LLWU_F1_OFFSET     0x0005 /* LLWU Flag 1 Register */
#define KINETIS_LLWU_F2_OFFSET     0x0006 /* LLWU Flag 2 Register */
#define KINETIS_LLWU_F3_OFFSET     0x0007 /* LLWU Flag 3 Register */
#define KINETIS_LLWU_CS_OFFSET     0x0008 /* LLWU Control and Status Register */

/* Register Addresses ***************************************************************/

#define KINETIS_LLWU_PE1           (KINETIS_LLWU_BASE+KINETIS_LLWU_PE1_OFFSET)
#define KINETIS_LLWU_PE2           (KINETIS_LLWU_BASE+KINETIS_LLWU_PE2_OFFSET)
#define KINETIS_LLWU_PE3           (KINETIS_LLWU_BASE+KINETIS_LLWU_PE3_OFFSET)
#define KINETIS_LLWU_PE4           (KINETIS_LLWU_BASE+KINETIS_LLWU_PE4_OFFSET)
#define KINETIS_LLWU_ME            (KINETIS_LLWU_BASE+KINETIS_LLWU_ME_OFFSET)
#define KINETIS_LLWU_F1            (KINETIS_LLWU_BASE+KINETIS_LLWU_F1_OFFSET)
#define KINETIS_LLWU_F2            (KINETIS_LLWU_BASE+KINETIS_LLWU_F2_OFFSET)
#define KINETIS_LLWU_F3            (KINETIS_LLWU_BASE+KINETIS_LLWU_F3_OFFSET)
#define KINETIS_LLWU_CS            (KINETIS_LLWU_BASE+KINETIS_LLWU_CS_OFFSET)

/* Register Bit Definitions *********************************************************/

/* LLWU Pin Enable 1 Register */

#define LLWU_PE1_WUPE0_SHIFT       (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P0 */
#define LLWU_PE1_WUPE0_MASK        (3 << LLWU_PE1_WUPE0_SHIFT)
#  define LLWU_PE1_WUPE0_DISABLED  (0 << LLWU_PE1_WUPE0_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE0_RISING    (1 << LLWU_PE1_WUPE0_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE0_FALLING   (2 << LLWU_PE1_WUPE0_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE0_BOTH      (3 << LLWU_PE1_WUPE0_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE1_WUPE1_SHIFT       (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P1 */
#define LLWU_PE1_WUPE1_MASK        (3 << LLWU_PE1_WUPE1_SHIFT)
#  define LLWU_PE1_WUPE1_DISABLED  (0 << LLWU_PE1_WUPE1_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE1_RISING    (1 << LLWU_PE1_WUPE1_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE1_FALLING   (2 << LLWU_PE1_WUPE1_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE1_BOTH      (3 << LLWU_PE1_WUPE1_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE1_WUPE2_SHIFT       (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P2 */
#define LLWU_PE1_WUPE2_MASK        (3 << LLWU_PE1_WUPE2_SHIFT)
#  define LLWU_PE1_WUPE2_DISABLED  (0 << LLWU_PE1_WUPE2_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE2_RISING    (1 << LLWU_PE1_WUPE2_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE2_FALLING   (2 << LLWU_PE1_WUPE2_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE2_BOTH      (3 << LLWU_PE1_WUPE2_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE1_WUPE3_SHIFT       (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P3 */
#define LLWU_PE1_WUPE3_MASK        (3 << LLWU_PE1_WUPE3_SHIFT)
#  define LLWU_PE1_WUPE3_DISABLED  (0 << LLWU_PE1_WUPE3_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE1_WUPE3_RISING    (1 << LLWU_PE1_WUPE3_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE1_WUPE3_FALLING   (2 << LLWU_PE1_WUPE3_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE1_WUPE3_BOTH      (3 << LLWU_PE1_WUPE3_SHIFT) /* Ext input enabled for any change */

/* LLWU Pin Enable 2 Register */

#define LLWU_PE2_WUPE4_SHIFT       (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P4 */
#define LLWU_PE2_WUPE4_MASK        (3 << LLWU_PE2_WUPE4_SHIFT)
#  define LLWU_PE2_WUPE4_DISABLED  (0 << LLWU_PE2_WUPE4_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE4_RISING    (1 << LLWU_PE2_WUPE4_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE4_FALLING   (2 << LLWU_PE2_WUPE4_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE4_BOTH      (3 << LLWU_PE2_WUPE4_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE2_WUPE5_SHIFT       (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P5 */
#define LLWU_PE2_WUPE5_MASK        (3 << LLWU_PE2_WUPE5_SHIFT)
#  define LLWU_PE2_WUPE5_DISABLED  (0 << LLWU_PE2_WUPE5_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE5_RISING    (1 << LLWU_PE2_WUPE5_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE5_FALLING   (2 << LLWU_PE2_WUPE5_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE5_BOTH      (3 << LLWU_PE2_WUPE5_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE2_WUPE6_SHIFT       (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P6 */
#define LLWU_PE2_WUPE6_MASK        (3 << LLWU_PE2_WUPE6_SHIFT)
#  define LLWU_PE2_WUPE6_DISABLED  (0 << LLWU_PE2_WUPE6_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE6_RISING    (1 << LLWU_PE2_WUPE6_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE6_FALLING   (2 << LLWU_PE2_WUPE6_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE6_BOTH      (3 << LLWU_PE2_WUPE6_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE2_WUPE7_SHIFT       (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P7 */
#define LLWU_PE2_WUPE7_MASK        (3 << LLWU_PE2_WUPE7_SHIFT)
#  define LLWU_PE2_WUPE7_DISABLED  (0 << LLWU_PE2_WUPE7_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE2_WUPE7_RISING    (1 << LLWU_PE2_WUPE7_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE2_WUPE7_FALLING   (2 << LLWU_PE2_WUPE7_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE2_WUPE7_BOTH      (3 << LLWU_PE2_WUPE7_SHIFT) /* Ext input enabled for any change */

/* LLWU Pin Enable 3 Register */

#define LLWU_PE3_WUPE8_SHIFT       (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P8 */
#define LLWU_PE3_WUPE8_MASK        (3 << LLWU_PE3_WUPE8_SHIFT)
#  define LLWU_PE3_WUPE8_DISABLED  (0 << LLWU_PE3_WUPE8_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE8_RISING    (1 << LLWU_PE3_WUPE8_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE8_FALLING   (2 << LLWU_PE3_WUPE8_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE8_BOTH      (3 << LLWU_PE3_WUPE8_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE3_WUPE9_SHIFT       (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P9 */
#define LLWU_PE3_WUPE9_MASK        (3 << LLWU_PE3_WUPE9_SHIFT)
#  define LLWU_PE3_WUPE9_DISABLED  (0 << LLWU_PE3_WUPE9_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE9_RISING    (1 << LLWU_PE3_WUPE9_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE9_FALLING   (2 << LLWU_PE3_WUPE9_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE9_BOTH      (3 << LLWU_PE3_WUPE9_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE3_WUPE10_SHIFT      (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P10 */
#define LLWU_PE3_WUPE10_MASK       (3 << LLWU_PE3_WUPE10_SHIFT)
#  define LLWU_PE3_WUPE10_DISABLED (0 << LLWU_PE3_WUPE10_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE10_RISING   (1 << LLWU_PE3_WUPE10_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE10_FALLING  (2 << LLWU_PE3_WUPE10_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE10_BOTH     (3 << LLWU_PE3_WUPE10_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE3_WUPE11_SHIFT      (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P11 */
#define LLWU_PE3_WUPE11_MASK       (3 << LLWU_PE3_WUPE11_SHIFT)
#  define LLWU_PE3_WUPE11_DISABLED (0 << LLWU_PE3_WUPE11_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE3_WUPE11_RISING   (1 << LLWU_PE3_WUPE11_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE3_WUPE11_FALLING  (2 << LLWU_PE3_WUPE11_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE3_WUPE11_BOTH     (3 << LLWU_PE3_WUPE11_SHIFT) /* Ext input enabled for any change */

/* LLWU Pin Enable 4 Register */

#define LLWU_PE4_WUPE12_SHIFT      (0)       /* Bits 0-1: Wakeup Pin Enable for LLWU_P12 */
#define LLWU_PE4_WUPE12_MASK       (3 << LLWU_PE4_WUPE12_SHIFT)
#  define LLWU_PE4_WUPE12_DISABLED (0 << LLWU_PE4_WUPE12_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE12_RISING   (1 << LLWU_PE4_WUPE12_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE12_FALLING  (2 << LLWU_PE4_WUPE12_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE12_BOTH     (3 << LLWU_PE4_WUPE12_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE4_WUPE13_SHIFT      (2)       /* Bits 2-3: Wakeup Pin Enable for LLWU_P13 */
#define LLWU_PE4_WUPE13_MASK       (3 << LLWU_PE4_WUPE13_SHIFT)
#  define LLWU_PE4_WUPE13_DISABLED (0 << LLWU_PE4_WUPE13_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE13_RISING   (1 << LLWU_PE4_WUPE13_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE13_FALLING  (2 << LLWU_PE4_WUPE13_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE13_BOTH     (3 << LLWU_PE4_WUPE13_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE4_WUPE14_SHIFT      (4)       /* Bits 4-5: Wakeup Pin Enable for LLWU_P14 */
#define LLWU_PE4_WUPE14_MASK       (3 << LLWU_PE4_WUPE14_SHIFT)
#  define LLWU_PE4_WUPE14_DISABLED (0 << LLWU_PE4_WUPE14_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE14_RISING   (1 << LLWU_PE4_WUPE14_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE14_FALLING  (2 << LLWU_PE4_WUPE14_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE14_BOTH     (3 << LLWU_PE4_WUPE14_SHIFT) /* Ext input enabled for any change */
#define LLWU_PE4_WUPE15_SHIFT      (6)       /* Bits 6-7: Wakeup Pin Enable for LLWU_P15 */
#define LLWU_PE4_WUPE15_MASK       (3 << LLWU_PE4_WUPE15_SHIFT)
#  define LLWU_PE4_WUPE15_DISABLED (0 << LLWU_PE4_WUPE15_SHIFT) /* Ext input disabled as wakeup input */
#  define LLWU_PE4_WUPE15_RISING   (1 << LLWU_PE4_WUPE15_SHIFT) /* Ext input enabled for rising edge */
#  define LLWU_PE4_WUPE15_FALLING  (2 << LLWU_PE4_WUPE15_SHIFT) /* Ext input enabled for falling edge */
#  define LLWU_PE4_WUPE15_BOTH     (3 << LLWU_PE4_WUPE15_SHIFT) /* Ext input enabled for any change */

/* LLWU Module Enable Register */

#define LLWU_ME_WUME(n)            (1 << (n))
#define LLWU_ME_WUME0              (1 << 0)  /* Bit 0:  Wakeup Module Enable for Module 0 */
#define LLWU_ME_WUME1              (1 << 1)  /* Bit 1:  Wakeup Module Enable for Module 1 */
#define LLWU_ME_WUME2              (1 << 2)  /* Bit 2:  Wakeup Module Enable for Module 2 */
#define LLWU_ME_WUME3              (1 << 3)  /* Bit 3:  Wakeup Module Enable for Module 3 */
#define LLWU_ME_WUME4              (1 << 4)  /* Bit 4:  Wakeup Module Enable for Module 4 */
#define LLWU_ME_WUME5              (1 << 5)  /* Bit 5:  Wakeup Module Enable for Module 5 */
#define LLWU_ME_WUME6              (1 << 6)  /* Bit 6:  Wakeup Module Enable for Module 6 */
#define LLWU_ME_WUME7              (1 << 7)  /* Bit 7:  Wakeup Module Enable for Module 7 */

/* LLWU Flag 1 Register */

#define LLWU_F1_WUF(n)             (1 << (n))
#define LLWU_F1_WUF0               (1 << 0)  /* Bit 0:  Wakeup Flag for LLWU_P0 */
#define LLWU_F1_WUF1               (1 << 1)  /* Bit 1:  Wakeup Flag for LLWU_P1 */
#define LLWU_F1_WUF2               (1 << 2)  /* Bit 2:  Wakeup Flag for LLWU_P2 */
#define LLWU_F1_WUF3               (1 << 3)  /* Bit 3:  Wakeup Flag for LLWU_P3 */
#define LLWU_F1_WUF4               (1 << 4)  /* Bit 4:  Wakeup Flag for LLWU_P4 */
#define LLWU_F1_WUF5               (1 << 5)  /* Bit 5:  Wakeup Flag for LLWU_P5 */
#define LLWU_F1_WUF6               (1 << 6)  /* Bit 6:  Wakeup Flag for LLWU_P6 */
#define LLWU_F1_WUF7               (1 << 7)  /* Bit 7:  Wakeup Flag for LLWU_P7 */

/* LLWU Flag 2 Register */

#define LLWU_F2_WUF(n)             (1 << ((n)-8))
#define LLWU_F2_WUF8               (1 << 8)  /* Bit 0:  Wakeup Flag for LLWU_P8 */
#define LLWU_F2_WUF9               (1 << 9)  /* Bit 1:  Wakeup Flag for LLWU_P9 */
#define LLWU_F2_WUF10              (1 << 10) /* Bit 2:  Wakeup Flag for LLWU_P10 */
#define LLWU_F2_WUF11              (1 << 11) /* Bit 3:  Wakeup Flag for LLWU_P11 */
#define LLWU_F2_WUF12              (1 << 12) /* Bit 4:  Wakeup Flag for LLWU_P12 */
#define LLWU_F2_WUF13              (1 << 13) /* Bit 5:  Wakeup Flag for LLWU_P13 */
#define LLWU_F2_WUF14              (1 << 14) /* Bit 6:  Wakeup Flag for LLWU_P14 */
#define LLWU_F2_WUF15              (1 << 15) /* Bit 7:  Wakeup Flag for LLWU_P15 */

/* LLWU Flag 3 Register */

#define LLWU_F3_MWUF(n)            (1 << (n))
#define LLWU_F3_MWUF0              (1 << 0)  /* Bit 0:  Wakeup flag for module 0 */
#define LLWU_F3_MWUF1              (1 << 1)  /* Bit 1:  Wakeup flag for module 1 */
#define LLWU_F3_MWUF2              (1 << 2)  /* Bit 2:  Wakeup flag for module 2 */
#define LLWU_F3_MWUF3              (1 << 3)  /* Bit 3:  Wakeup flag for module 3 */
#define LLWU_F3_MWUF4              (1 << 4)  /* Bit 4:  Wakeup flag for module 4 */
#define LLWU_F3_MWUF5              (1 << 5)  /* Bit 5:  Wakeup flag for module 5 */
#define LLWU_F3_MWUF6              (1 << 6)  /* Bit 6:  Wakeup flag for module 6 */
#define LLWU_F3_MWUF7              (1 << 7)  /* Bit 7:  Wakeup flag for module 7 */

/* LLWU Control and Status Register */

#define LLWU_CS_ACKISO             (1 << 7)  /* Bit 7:  Acknowledge Isolation */
                                             /* Bits 2-6: Reserved */
#define LLWU_CS_FLTEP              (1 << 1)  /* Bit 1:  Digital Filter on External Pin */
#define LLWU_CS_FLTR               (1 << 0)  /* Bit 0:  Digital Filter on RESET Pin */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_LLWU_H */
