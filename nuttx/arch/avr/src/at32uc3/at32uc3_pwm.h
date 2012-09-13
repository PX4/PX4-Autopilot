/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_pwm.h
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
 ************************************************************************************/

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_PWM_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_PWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* PWM Channel Offsets **************************************************************/

#define AVR32_PWM_CHAN_OFFSET(n)   (0x200+((n)<<5))
#define AVR32_PWM_CHAN0_OFFSET     (0x200)
#define AVR32_PWM_CHAN1_OFFSET     (0x220)

/* Register offsets *****************************************************************/

#define AVR32_PWM_MR_OFFSET        0x000 /* PWM Mode Register */
#define AVR32_PWM_ENA_OFFSET       0x004 /* PWM Enable Register */
#define AVR32_PWM_DIS_OFFSET       0x008 /* PWM Disable Register */
#define AVR32_PWM_SR_OFFSET        0x00c /* PWM Status Register */
#define AVR32_PWM_IER_OFFSET       0x010 /* PWM Interrupt Enable Register */
#define AVR32_PWM_IDR_OFFSET       0x014 /* PWM Interrupt Disable Register */
#define AVR32_PWM_IMR_OFFSET       0x018 /* PWM Interrupt Mask Register */
#define AVR32_PWM_ISR_OFFSET       0x01c /* PWM Interrupt Status Register */

#define AVR32_PWM_CMR_OFFSET       0x000 /* Channel Mode Register */
#define AVR32_PWM_CDTY_OFFSET      0x004 /* Channel Duty Cycle Register */
#define AVR32_PWM_CPRD_OFFSET      0x008 /* Channel Period Register */
#define AVR32_PWM_CCNT_OFFSET      0x00c /* Channel Counter Register */
#define AVR32_PWM_CUPD_OFFSET      0x010 /* Channel Update Register */

#define AVR32_PWMCH_CMR_OFFSET(n)  (AVR32_PWM_CHAN_OFFSET(n)+PWM_CMR_OFFSET)
#define AVR32_PWMCH_CDTY_OFFSET(n) (AVR32_PWM_CHAN_OFFSET(n)+PWM_CDTY_OFFSET)
#define AVR32_PWMCH_CPRD_OFFSET(n) (AVR32_PWM_CHAN_OFFSET(n)+PWM_CPRD_OFFSET)
#define AVR32_PWMCH_CCNT_OFFSET(n) (AVR32_PWM_CHAN_OFFSET(n)+PWM_CCNT_OFFSET)
#define AVR32_PWMCH_CUPD_OFFSET(n) (AVR32_PWM_CHAN_OFFSET(n)+PWM_CUPD_OFFSET)

#define AVR32_PWMCH0_CMR_OFFSET    (AVR32_PWM_CHAN0_OFFSET+PWM_CMR_OFFSET)
#define AVR32_PWMCH0_CDTY_OFFSET   (AVR32_PWM_CHAN0_OFFSET+PWM_CDTY_OFFSET)
#define AVR32_PWMCH0_CPRD_OFFSET   (AVR32_PWM_CHAN0_OFFSET+PWM_CPRD_OFFSET)
#define AVR32_PWMCH0_CCNT_OFFSET   (AVR32_PWM_CHAN0_OFFSET+PWM_CCNT_OFFSET)
#define AVR32_PWMCH0_CUPD_OFFSET   (AVR32_PWM_CHAN0_OFFSET+PWM_CUPD_OFFSET)

#define AVR32_PWMCH1_CMR_OFFSET    (AVR32_PWM_CHAN1_OFFSET+PWM_CMR_OFFSET)
#define AVR32_PWMCH1_CDTY_OFFSET   (AVR32_PWM_CHAN1_OFFSET+PWM_CDTY_OFFSET)
#define AVR32_PWMCH1_CPRD_OFFSET   (AVR32_PWM_CHAN1_OFFSET+PWM_CPRD_OFFSET)
#define AVR32_PWMCH1_CCNT_OFFSET   (AVR32_PWM_CHAN1_OFFSET+PWM_CCNT_OFFSET)
#define AVR32_PWMCH1_CUPD_OFFSET   (AVR32_PWM_CHAN1_OFFSET+PWM_CUPD_OFFSET)

/* PWM Channel Base Addresses *******************************************************/

#define AVR32_PWM_CHAN_BASE(n)     (AVR32_PWM_BASE+PWM_CHAN_OFFSET(n))
#define AVR32_PWM_CHAN0_BASE       (AVR32_PWM_BASE+PWM_CHAN0_OFFSET)
#define AVR32_PWM_CHAN1_BASE       (AVR32_PWM_BASE+PWM_CHAN1_OFFSET)

/* Register Addresses ***************************************************************/

#define AVR32_PWM_MR               (AVR32_PWM_BASE+AVR32_PWM_MR_OFFSET)
#define AVR32_PWM_ENA              (AVR32_PWM_BASE+AVR32_PWM_ENA_OFFSET)
#define AVR32_PWM_DIS              (AVR32_PWM_BASE+AVR32_PWM_DIS_OFFSET)
#define AVR32_PWM_SR               (AVR32_PWM_BASE+AVR32_PWM_SR_OFFSET)
#define AVR32_PWM_IER              (AVR32_PWM_BASE+AVR32_PWM_IER_OFFSET)
#define AVR32_PWM_IDR              (AVR32_PWM_BASE+AVR32_PWM_IDR_OFFSET)
#define AVR32_PWM_IMR              (AVR32_PWM_BASE+AVR32_PWM_IMR_OFFSET)
#define AVR32_PWM_ISR              (AVR32_PWM_BASE+AVR32_PWM_ISR_OFFSET)

#define AVR32_PWMCH_CMR(n)         (AVR32_PWM_CHAN_BASE(n)+PWM_CMR_OFFSET)
#define AVR32_PWMCH_CDTY(n)        (AVR32_PWM_CHAN_BASE(n)+PWM_CDTY_OFFSET)
#define AVR32_PWMCH_CPRD(n)        (AVR32_PWM_CHAN_BASE(n)+PWM_CPRD_OFFSET)
#define AVR32_PWMCH_CCNT(n)        (AVR32_PWM_CHAN_BASE(n)+PWM_CCNT_OFFSET)
#define AVR32_PWMCH_CUPD(n)        (AVR32_PWM_CHAN_BASE(n)+PWM_CUPD_OFFSET)

#define AVR32_PWMCH0_CMR           (AVR32_PWM_CHAN0_BASE+PWM_CMR_OFFSET)
#define AVR32_PWMCH0_CDTY          (AVR32_PWM_CHAN0_BASE+PWM_CDTY_OFFSET)
#define AVR32_PWMCH0_CPRD          (AVR32_PWM_CHAN0_BASE+PWM_CPRD_OFFSET)
#define AVR32_PWMCH0_CCNT          (AVR32_PWM_CHAN0_BASE+PWM_CCNT_OFFSET)
#define AVR32_PWMCH0_CUPD          (AVR32_PWM_CHAN0_BASE+PWM_CUPD_OFFSET)

#define AVR32_PWMCH1_CMR           (AVR32_PWM_CHAN1_BASE+PWM_CMR_OFFSET)
#define AVR32_PWMCH1_CDTY          (AVR32_PWM_CHAN1_BASE+PWM_CDTY_OFFSET)
#define AVR32_PWMCH1_CPRD          (AVR32_PWM_CHAN1_BASE+PWM_CPRD_OFFSET)
#define AVR32_PWMCH1_CCNT          (AVR32_PWM_CHAN1_BASE+PWM_CCNT_OFFSET)
#define AVR32_PWMCH1_CUPD          (AVR32_PWM_CHAN1_BASE+PWM_CUPD_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* PWM Mode Register Bit-field Definitions */

#define PWM_MR_DIVA_SHIFT          (0)       /* Bits 0-7: CLKA Divide Factor */
#define PWM_MR_DIVA_MASK           (0xff << PWM_MR_DIVA_SHIFT)
#  define PWM_MR_DIVA_OFF          (0 << PWM_MR_DIVA_SHIFT)
#  define PWM_MR_DIVA(n)           ((n) << PWM_MR_DIVA_SHIFT)
#define PWM_MR_PREA_SHIFT          (8)       /* Bits 8-11 */
#define PWM_MR_PREA_MASK           (15 << PWM_MR_PREA_SHIFT)
#  define PWM_MR_PREA_MCK          (0 << PWM_MR_PREA_SHIFT)  /* MCK */
#  define PWM_MR_PREA_MCKDIV2      (1 << PWM_MR_PREA_SHIFT)  /* MCK/2 */
#  define PWM_MR_PREA_MCKDIV4      (2 << PWM_MR_PREA_SHIFT)  /* MCK/4 */
#  define PWM_MR_PREA_MCKDIV8      (3 << PWM_MR_PREA_SHIFT)  /* MCK/8 */
#  define PWM_MR_PREA_MCKDIV16     (4 << PWM_MR_PREA_SHIFT)  /* MCK/16 */
#  define PWM_MR_PREA_MCKDIV32     (5 << PWM_MR_PREA_SHIFT)  /* MCK/32 */
#  define PWM_MR_PREA_MCKDIV64     (6 << PWM_MR_PREA_SHIFT)  /* MCK/64 */
#  define PWM_MR_PREA_MCKDIV128    (7 << PWM_MR_PREA_SHIFT)  /* MCK/128 */
#  define PWM_MR_PREA_MCKDIV256    (8 << PWM_MR_PREA_SHIFT)  /* MCK/256 */
#  define PWM_MR_PREA_MCKDIV512    (9 << PWM_MR_PREA_SHIFT)  /* MCK/512 */
#  define PWM_MR_PREA_MCKDIV1024   (10 << PWM_MR_PREA_SHIFT) /* MCK/1024 */
#define PWM_MR_DIVB_SHIFT          (16)      /* Bits 16-23: CLKB Divide Factor */
#define PWM_MR_DIVB_MASK           (0xff << PWM_MR_DIVB_SHIFT)
#  define PWM_MR_DIVB_OFF          (0 << PWM_MR_DIVB_SHIFT)
#  define PWM_MR_DIVB(n)           ((n) << PWM_MR_DIVB_SHIFT)
#define PWM_MR_PREB_SHIFT          (24)      /* Bits 24-27  */
#define PWM_MR_PREB_MASK           (15 << PWM_MR_PREB_SHIFT)
#  define PWM_MR_PREB_MCK          (0 << PWM_MR_PREB_SHIFT)  /* MCK */
#  define PWM_MR_PREB_MCKDIV2      (1 << PWM_MR_PREB_SHIFT)  /* MCK/2 */
#  define PWM_MR_PREB_MCKDIV4      (2 << PWM_MR_PREB_SHIFT)  /* MCK/4 */
#  define PWM_MR_PREB_MCKDIV8      (3 << PWM_MR_PREB_SHIFT)  /* MCK/8 */
#  define PWM_MR_PREB_MCKDIV16     (4 << PWM_MR_PREB_SHIFT)  /* MCK/16 */
#  define PWM_MR_PREB_MCKDIV32     (5 << PWM_MR_PREB_SHIFT)  /* MCK/32 */
#  define PWM_MR_PREB_MCKDIV64     (6 << PWM_MR_PREB_SHIFT)  /* MCK/64 */
#  define PWM_MR_PREB_MCKDIV128    (7 << PWM_MR_PREB_SHIFT)  /* MCK/128 */
#  define PWM_MR_PREB_MCKDIV256    (8 << PWM_MR_PREB_SHIFT)  /* MCK/256 */
#  define PWM_MR_PREB_MCKDIV512    (9 << PWM_MR_PREB_SHIFT)  /* MCK/512 */
#  define PWM_MR_PREB_MCKDIV1024   (10 << PWM_MR_PREB_SHIFT) /* MCK/1024 */

/* PWM Enable Register Bit-field Definitions */
/* PWM Disable Register Bit-field Definitions */
/* PWM Status Register Bit-field Definitions */
/* PWM Interrupt Enable Register Bit-field Definitions */
/* PWM Interrupt Disable Register Bit-field Definitions */
/* PWM Interrupt Mask Register Bit-field Definitions */
/* PWM Interrupt Status Register Bit-field Definitions */

#define PWM_CHID(n)                (1 << (n)) /* Bit n: Channel ID n */
#define PWM_CHID0                  (1 << 0)  /* Bit 0:  Channel ID 0 */
#define PWM_CHID1                  (1 << 1)  /* Bit 1:  Channel ID 1 */
#define PWM_CHID2                  (1 << 2)  /* Bit 2:  Channel ID 2 */
#define PWM_CHID3                  (1 << 3)  /* Bit 3:  Channel ID 3 */
#define PWM_CHID4                  (1 << 4)  /* Bit 4:  Channel ID 4 */
#define PWM_CHID5                  (1 << 5)  /* Bit 5:  Channel ID 5 */
#define PWM_CHID6                  (1 << 6)  /* Bit 6:  Channel ID 6 */

/* Channel Mode Register Bit-field Definitions */

#define PWM_CMR_CPRE_SHIFT         (0)       /* Bits 0-3: CLKA Divide FactorChannel Pre-scaler */
#define PWM_CMR_CPRE_MASK          (15 << PWM_CMR_CPRE_SHIFT)
#  define PWM_CMR_CPRE_MCK         (0 << PWM_CMR_CPRE_SHIFT)  /* MCK */
#  define PWM_CMR_CPRE_MCKDIV2     (1 << PWM_CMR_CPRE_SHIFT)  /* MCK/2 */
#  define PWM_CMR_CPRE_MCKDIV4     (2 << PWM_CMR_CPRE_SHIFT)  /* MCK/4 */
#  define PWM_CMR_CPRE_MCKDIV8     (3 << PWM_CMR_CPRE_SHIFT)  /* MCK/8 */
#  define PWM_CMR_CPRE_MCKDIV16    (4 << PWM_CMR_CPRE_SHIFT)  /* MCK/16 */
#  define PWM_CMR_CPRE_MCKDIV32    (5 << PWM_CMR_CPRE_SHIFT)  /* MCK/32 */
#  define PWM_CMR_CPRE_MCKDIV64    (6 << PWM_CMR_CPRE_SHIFT)  /* MCK/64 */
#  define PWM_CMR_CPRE_MCKDIV128   (7 << PWM_CMR_CPRE_SHIFT)  /* MCK/128 */
#  define PWM_CMR_CPRE_MCKDIV256   (8 << PWM_CMR_CPRE_SHIFT)  /* MCK/256 */
#  define PWM_CMR_CPRE_MCKDIV512   (9 << PWM_CMR_CPRE_SHIFT)  /* MCK/512 */
#  define PWM_CMR_CPRE_MCKDIV1024  (10 << PWM_CMR_CPRE_SHIFT) /* MCK/1024 */
#  define PWM_CMR_CPRE_CLKA        (11 << PWM_CMR_CPRE_SHIFT) /* CLKA */
#  define PWM_CMR_CPRE_CLKB        (12 << PWM_CMR_CPRE_SHIFT) /* CLB */
#define PWM_CMR_CALG               (1 << 8)  /* Bit 8:  Channel Alignment */
#define PWM_CMR_CPOL               (1 << 9)  /* Bit 9:  Channel Polarity */
#define PWM_CMR_CPD                (1 << 10) /* Bit 10: Channel Update Period */

/* Channel Duty Cycle Register Bit-field Definitions */
/* Channel Period Register Bit-field Definitions */
/* Channel Counter Register Bit-field Definitions */
/* Channel Update Register Bit-field Definitions */

/* These registers hold a 32-bit value with bit-fiels */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_PWM_H */

