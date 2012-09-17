/************************************************************************************
 * arch/arm/src/kinetis/kinetis_rtc.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_RTC_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(KINETIS_NRTC) && KINETIS_NRTC > 0

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_RTC_TSR_OFFSET   0x0000 /* RTC Time Seconds Register */
#define KINETIS_RTC_TPR_OFFSET   0x0004 /* RTC Time Prescaler Register */
#define KINETIS_RTC_TAR_OFFSET   0x0008 /* RTC Time Alarm Register */
#define KINETIS_RTC_TCR_OFFSET   0x000c /* RTC Time Compensation Register */
#define KINETIS_RTC_CR_OFFSET    0x0010 /* RTC Control Register */
#define KINETIS_RTC_SR_OFFSET    0x0014 /* RTC Status Register */
#define KINETIS_RTC_LR_OFFSET    0x0018 /* RTC Lock Register */
#ifdef KINETIS_K40
#  define KINETIS_RTC_IER_OFFSET 0x001c /* RTC Interrupt Enable Register (K40) */
#endif
#ifdef KINETIS_K60
#  define KINETIS_RTC_CCR_OFFSET 0x001c /* RTC Chip Configuration Register (K60) */
#endif
#define KINETIS_RTC_WAR_OFFSET   0x0800 /* RTC Write Access Register */
#define KINETIS_RTC_RAR_OFFSET   0x0804 /* RTC Read Access Register */

/* Register Addresses ***************************************************************/

#define KINETIS_RTC_TSR          (KINETIS_RTC_BASE+KINETIS_RTC_TSR_OFFSET)
#define KINETIS_RTC_TPR          (KINETIS_RTC_BASE+KINETIS_RTC_TPR_OFFSET)
#define KINETIS_RTC_TAR          (KINETIS_RTC_BASE+KINETIS_RTC_TAR_OFFSET)
#define KINETIS_RTC_TCR          (KINETIS_RTC_BASE+KINETIS_RTC_TCR_OFFSET)
#define KINETIS_RTC_CR           (KINETIS_RTC_BASE+KINETIS_RTC_CR_OFFSET)
#define KINETIS_RTC_SR           (KINETIS_RTC_BASE+KINETIS_RTC_SR_OFFSET)
#define KINETIS_RTC_LR           (KINETIS_RTC_BASE+KINETIS_RTC_LR_OFFSET)
#ifdef KINETIS_K40
#  define KINETIS_RTC_IER        (KINETIS_RTC_BASE+KINETIS_RTC_IER_OFFSET)
#endif
#ifdef KINETIS_K60
#  define KINETIS_CCR_IER        (KINETIS_RTC_BASE+KINETIS_RTC_CCR_OFFSET)
#endif
#define KINETIS_RTC_WAR          (KINETIS_RTC_BASE+KINETIS_RTC_WAR_OFFSET)
#define KINETIS_RTC_RAR          (KINETIS_RTC_BASE+KINETIS_RTC_RAR_OFFSET)

/* Register Bit Definitions *********************************************************/

/* RTC Time Seconds Register (32-bits of time in seconds) */

/* RTC Time Prescaler Register */

#define RTC_TPR_SHIFT            (0)       /* Bits 0-15: Time Prescaler Register */
#define RTC_TPR_MASK             (0xffff << RTC_TPR_SHIFT)
                                           /* Bits 16-31: Reserved */
/* RTC Time Alarm Register (32-bits of time alarm) */

/* RTC Time Compensation Register (32-bits) */

#define RTC_TCR_TCR_SHIFT        (0)       /* Bits 0-7: Time Compensation Register */
#define RTC_TCR_TCR_MASK         (0xff << RTC_TCR_CIR_MASK)
#define RTC_TCR_CIR_SHIFT        (8)       /* Bits 8-15: Compensation Interval Register */
#define RTC_TCR_CIR_MASK         (0xff << RTC_TCR_CIR_SHIFT)
#define RTC_TCR_TCV_SHIFT        (16)      /* Bits 16-23: Time Compensation Value */
#define RTC_TCR_TCV_MASK         (0xff << RTC_TCR_TCV_SHIFT)
#define RTC_TCR_CIC_SHIFT        (24)      /* Bits 24-31: Compensation Interval Counter */
#define RTC_TCR_CIC_MASK         (0xff << RTC_TCR_CIC_SHIFT)

/* RTC Control Register (32-bits) */

#define RTC_CR_SWR               (1 << 0)  /* Bit 0:  Software Reset */
#define RTC_CR_WPE               (1 << 1)  /* Bit 1:  Wakeup Pin Enable */
#define RTC_CR_SUP               (1 << 2)  /* Bit 2:  Supervisor Access */
#define RTC_CR_UM                (1 << 3)  /* Bit 3:  Update Mode */
                                           /* Bits 4-7: Reserved */
#define RTC_CR_OSCE              (1 << 8)  /* Bit 8:  Oscillator Enable */
#define RTC_CR_CLKO              (1 << 9)  /* Bit 9:  Clock Output */
#define RTC_CR_SC16P             (1 << 10) /* Bit 10: Oscillator 16pF load configure */
#define RTC_CR_SC8P              (1 << 11) /* Bit 11: Oscillator 8pF load configure */
#define RTC_CR_SC4P              (1 << 12) /* Bit 12: Oscillator 4pF load configure */
#define RTC_CR_SC2P              (1 << 13) /* Bit 13: Oscillator 2pF load configure */
                                           /* Bits 14-31: Reserved */
/* RTC Status Register (32-bits) */

#define RTC_SR_TIF               (1 << 0)  /* Bit 0:  Time Invalid Flag */
#define RTC_SR_TOF               (1 << 1)  /* Bit 1:  Time Overflow Flag */
                                           /* Bit 3: Reserved */
#define RTC_SR_TAF               (1 << 2)  /* Bit 2:  Time Alarm Flag */
#define RTC_SR_TCE               (1 << 4)  /* Bit 4:  Time Counter Enable */
                                           /* Bits 5-31: Reserved */
/* RTC Lock Register (32-bits) */
                                           /* Bits 0-2: Reserved */
#define RTC_LR_TCL               (1 << 3)  /* Bit 3:  Time Compensation Lock */
#define RTC_LR_CRL               (1 << 4)  /* Bit 4:  Control Register Lock */
#define RTC_LR_SRL               (1 << 5)  /* Bit 5:  Status Register Lock */
#ifdef KINETIS_K40
#  define RTC_LR_LRL             (1 << 6)  /* Bit 6:  Lock Register Lock (K40) */
#endif
                                           /* Bits 7-31: Reserved */
/* RTC Interrupt Enable Register (32-bits, K40) */

#ifdef KINETIS_K40
#  define RTC_IER_TIIE           (1 << 0)  /* Bit 0:  Time Invalid Interrupt Enable */
#  define RTC_IER_TOIE           (1 << 1)  /* Bit 1:  Time Overflow Interrupt Enable */
#  define RTC_IER_TAIE           (1 << 2)  /* Bit 2:  Time Alarm Interrupt Enable */
                                           /* Bit 3: Reserved */
#  define RTC_IER_TSIE           (1 << 4)  /* Bit 4:  Time Seconds Interrupt Enable */
                                           /* Bits 5-31: Reserved */
#endif

/* RTC Chip Configuration Register (32-bits,K60) */

#ifdef KINETIS_K60
#  define RTC_CCR_CONFIG_SHIFT   (0)       /* Bits 0-7: Chip Configuration */
#  define RTC_CCR_CONFIG_MASK    (0xff << RTC_CCR_CONFIG_SHIFT)
                                           /* Bits 8-31: Reserved */
#endif

/* RTC Write Access Register (32-bits) */

#define RTC_WAR_TSRW             (1 << 0)  /* Bit 0:  Time Seconds Register Write */
#define RTC_WAR_TPRW             (1 << 1)  /* Bit 1:  Time Prescaler Register Write */
#define RTC_WAR_TARW             (1 << 2)  /* Bit 2:  Time Alarm Register Write */
#define RTC_WAR_TCRW             (1 << 3)  /* Bit 3:  Time Compensation Register Write */
#define RTC_WAR_CRW              (1 << 4)  /* Bit 4:  Control Register Write */
#define RTC_WAR_SRW              (1 << 5)  /* Bit 5:  Status Register Write */
#define RTC_WAR_LRW              (1 << 6)  /* Bit 6:  Lock Register Write */
#ifdef KINETIS_K40
#  define RTC_WAR_IERW           (1 << 7)  /* Bit 7:  Interrupt Enable Register Write */
#endif
#ifdef KINETIS_K60
#  define RTC_WAR_CCRW           (1 << 7)  /* Bit 7:  Chip Config Register Write */
#endif
                                           /* Bits 8-31: Reserved */
/* RTC Read Access Register */

#define RTC_RAR_TSRR             (1 << 0)  /* Bit 0:  Time Seconds Register Read */
#define RTC_RAR_TPRR             (1 << 1)  /* Bit 1:  Time Prescaler Register Read */
#define RTC_RAR_TARR             (1 << 2)  /* Bit 2:  Time Alarm Register Read */
#define RTC_RAR_TCRR             (1 << 3)  /* Bit 3:  Time Compensation Register Read */
#define RTC_RAR_CRR              (1 << 4)  /* Bit 4:  Control Register Read */
#define RTC_RAR_SRR              (1 << 5)  /* Bit 5:  Status Register Read */
#define RTC_RAR_LRR              (1 << 6)  /* Bit 6:  Lock Register Read */
#ifdef KINETIS_K40
#  define RTC_RAR_IERR           (1 << 7)  /* Bit 7:  Interrupt Enable Register Read */
#endif
#ifdef KINETIS_K60
#  define RTC_RAR_CCRR           (1 << 7)  /* Bit 7:  Chip Config Register Read */
#endif
                                           /* Bits 8-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* KINETIS_NRTC && KINETIS_NRTC > 0 */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_RTC_H */
