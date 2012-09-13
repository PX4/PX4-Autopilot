/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_tc.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_TC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_TC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Timer Channel Offset *************************************************************/

#define AVR32_TC_CHAN_OFFSET(n)   ((n) << 6)
#define AVR32_TC_CHAN0_OFFSET     (0x000)
#define AVR32_TC_CHAN1_OFFSET     (0x040)
#define AVR32_TC_CHAN2_OFFSET     (0x080)

/* Register offsets *****************************************************************/

#define AVR32_TC_CCR_OFFSET       0x000 /* Channel Control Register */
#define AVR32_TC_CMR_OFFSET       0x004 /* Channel Mode Register */
#define AVR32_TC_CV_OFFSET        0x010 /* Channel Counter Value */
#define AVR32_TC_RA_OFFSET        0x014 /* Channel Register A */
#define AVR32_TC_RB_OFFSET        0x018 /* Channel Register B */
#define AVR32_TC_RC_OFFSET        0x01c /* Channel Register C */
#define AVR32_TC_SR_OFFSET        0x020 /* Channel Status Register */
#define AVR32_TC_IER_OFFSET       0x024 /* Interrupt Enable Register */
#define AVR32_TC_IDR_OFFSET       0x028 /* Channel Interrupt Disable Register */
#define AVR32_TC_IMR_OFFSET       0x02c /* Channel Interrupt Mask Register */

#define AVR32_TC_CCRn_OFFSET(n)   (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMRn_OFFSET(n)   (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CVn_OFFSET(n)    (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RAn_OFFSET(n)    (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RBn_OFFSET(n)    (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RCn_OFFSET(n)    (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SRn_OFFSET(n)    (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IERn_OFFSET(n)   (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDRn_OFFSET(n)   (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMRn_OFFSET(n)   (AVR32_TC_CHAN0_OFFSET(n)+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_CCR0_OFFSET      (AVR32_TC_CHAN0_OFFSET+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMR0_OFFSET      (AVR32_TC_CHAN0_OFFSET+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV0_OFFSET       (AVR32_TC_CHAN0_OFFSET+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA0_OFFSET       (AVR32_TC_CHAN0_OFFSET+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB0_OFFSET       (AVR32_TC_CHAN0_OFFSET+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC0_OFFSET       (AVR32_TC_CHAN0_OFFSET+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR0_OFFSET       (AVR32_TC_CHAN0_OFFSET+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER0_OFFSET      (AVR32_TC_CHAN0_OFFSET+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR0_OFFSET      (AVR32_TC_CHAN0_OFFSET+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR0_OFFSET      (AVR32_TC_CHAN0_OFFSET+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_CCR1_OFFSET      (AVR32_TC_CHAN1_OFFSET+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMR1_OFFSET      (AVR32_TC_CHAN1_OFFSET+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV1_OFFSET       (AVR32_TC_CHAN1_OFFSET+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA1_OFFSET       (AVR32_TC_CHAN1_OFFSET+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB1_OFFSET       (AVR32_TC_CHAN1_OFFSET+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC1_OFFSET       (AVR32_TC_CHAN1_OFFSET+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR1_OFFSET       (AVR32_TC_CHAN1_OFFSET+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER1_OFFSET      (AVR32_TC_CHAN1_OFFSET+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR1_OFFSET      (AVR32_TC_CHAN1_OFFSET+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR1_OFFSET      (AVR32_TC_CHAN1_OFFSET+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_CCR2_OFFSET      (AVR32_TC_CHAN2_OFFSET+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMR2_OFFSET      (AVR32_TC_CHAN2_OFFSET+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV2_OFFSET       (AVR32_TC_CHAN2_OFFSET+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA2_OFFSET       (AVR32_TC_CHAN2_OFFSET+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB2_OFFSET       (AVR32_TC_CHAN2_OFFSET+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC2_OFFSET       (AVR32_TC_CHAN2_OFFSET+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR2_OFFSET       (AVR32_TC_CHAN2_OFFSET+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER2_OFFSET      (AVR32_TC_CHAN2_OFFSET+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR2_OFFSET      (AVR32_TC_CHAN2_OFFSET+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR2_OFFSET      (AVR32_TC_CHAN2_OFFSET+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_BCR_OFFSET       0x0c0 /* Block Control Register */
#define AVR32_TC_BMR_OFFSET       0x0c4 /* Block Mode Register */

/* Timer Channel Base Addresses *****************************************************/

#define AVR32_TC_CHAN_BASE(n)     (AVR32_TC_BASE+AVR32_TC_CHAN_OFFSET(n))
#define AVR32_TC_CHAN0_BASE       (AVR32_TC_BASE+AVR32_TC_CHAN0_OFFSET)
#define AVR32_TC_CHAN1_BASE       (AVR32_TC_BASE+AVR32_TC_CHAN1_OFFSET)
#define AVR32_TC_CHAN2_BASE       (AVR32_TC_BASE+AVR32_TC_CHAN2_OFFSET)

/* Register Addresses ***************************************************************/

#define AVR32_TC_CCR(n)           (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMRn(n)          (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV(n)            (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA(n)            (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB(n)            (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC(n)            (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR(n)            (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER(n)           (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR(n)           (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR(n)           (AVR32_TC_CHAN0_BASE(n)+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_CCR0             (AVR32_TC_CHAN0_BASE+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMR0             (AVR32_TC_CHAN0_BASE+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV0              (AVR32_TC_CHAN0_BASE+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA0              (AVR32_TC_CHAN0_BASE+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB0              (AVR32_TC_CHAN0_BASE+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC0              (AVR32_TC_CHAN0_BASE+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR0              (AVR32_TC_CHAN0_BASE+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER0             (AVR32_TC_CHAN0_BASE+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR0             (AVR32_TC_CHAN0_BASE+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR0             (AVR32_TC_CHAN0_BASE+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_CCR1             (AVR32_TC_CHAN1_BASE+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMR1             (AVR32_TC_CHAN1_BASE+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV1              (AVR32_TC_CHAN1_BASE+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA1              (AVR32_TC_CHAN1_BASE+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB1              (AVR32_TC_CHAN1_BASE+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC1              (AVR32_TC_CHAN1_BASE+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR1              (AVR32_TC_CHAN1_BASE+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER1             (AVR32_TC_CHAN1_BASE+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR1             (AVR32_TC_CHAN1_BASE+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR1             (AVR32_TC_CHAN1_BASE+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_CCR2             (AVR32_TC_CHAN2_BASE+AVR32_TC_CCR_OFFSET)
#define AVR32_TC_CMR2             (AVR32_TC_CHAN2_BASE+AVR32_TC_CMR_OFFSET)
#define AVR32_TC_CV2              (AVR32_TC_CHAN2_BASE+AVR32_TC_CV_OFFSET)
#define AVR32_TC_RA2              (AVR32_TC_CHAN2_BASE+AVR32_TC_RA_OFFSET)
#define AVR32_TC_RB2              (AVR32_TC_CHAN2_BASE+AVR32_TC_RB_OFFSET)
#define AVR32_TC_RC2              (AVR32_TC_CHAN2_BASE+AVR32_TC_RC_OFFSET)
#define AVR32_TC_SR2              (AVR32_TC_CHAN2_BASE+AVR32_TC_SR_OFFSET)
#define AVR32_TC_IER2             (AVR32_TC_CHAN2_BASE+AVR32_TC_IER_OFFSET)
#define AVR32_TC_IDR2             (AVR32_TC_CHAN2_BASE+AVR32_TC_IDR_OFFSET)
#define AVR32_TC_IMR2             (AVR32_TC_CHAN2_BASE+AVR32_TC_IMR_OFFSET)

#define AVR32_TC_BCR              (AVR32_TC_BASE+AVR32_TC_BCR_OFFSET)
#define AVR32_TC_BMR              (AVR32_TC_BASE+AVR32_TC_BMR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Channel Control Register */

#define TC_CCR_CLKEN              (1 << 0)  /* Bit 0:  Counter Clock Enable Command */
#define TC_CCR_CLKDIS             (1 << 1)  /* Bit 3:  Counter Clock Disable Command */
#define TC_CCR_SWTRG              (1 << 2)  /* Bit 2:  Software Trigger Command */

/* Channel Mode Register -- All Modes */

#define TC_CMR_TCCLKS_SHIFT       (0)       /* Bits 0-2:  Clock Selection */
#define TC_CMR_TCCLKS_MASK        (7 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TMRCLK1   (0 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK1 */
#  define TC_CMR_TCCLKS_TMRCLK2   (1 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK2 */
#  define TC_CMR_TCCLKS_TMRCLK3   (2 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK3 */
#  define TC_CMR_TCCLKS_TMRCLK4   (3 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK4 */
#  define TC_CMR_TCCLKS_TMRCLK5   (4 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK5 */
#  define TC_CMR_TCCLKS_XC0       (5 << TC_CMR_TCCLKS_SHIFT) /* XC0 */
#  define TC_CMR_TCCLKS_XC1       (6 << TC_CMR_TCCLKS_SHIFT) /* XC1 */
#  define TC_CMR_TCCLKS_XC2       (7 << TC_CMR_TCCLKS_SHIFT) /* XC2 */
#define TC_CMR_CLKI               (1 << 3)  /* Bit 3:  Clock Invert */
#define TC_CMR_BURST_SHIFT        (4)       /* Bits 4-5:  Burst Signal Selection */
#define TC_CMR_BURST_MASK         (3 << TC_CMR_BURST_SHIFT)
#  define TC_CMR_BURST_NONE       (0 << TC_CMR_BURST_SHIFT) /* Clock not gated by External signal */
#  define TC_CMR_BURST_XC0        (1 << TC_CMR_BURST_SHIFT) /* XC0 ANDed with selected clock */
#  define TC_CMR_BURST_XC1        (2 << TC_CMR_BURST_SHIFT) /* XC1 ANDed with selected clock */
#  define TC_CMR_BURST_XC2        (3 << TC_CMR_BURST_SHIFT) /* XC2 ANDed with selected clock */

#define TC_CMR_WAVE               (1 << 15) /* Bit 15: Enable waveform mode */

/* Channel Mode Register -- Capture Mode */

#define TC_CMR_LDBSTOP            (1 << 6)  /* Bit 6:  Counter Clock Stopped with RB Loading */
#define TC_CMR_LDBDIS             (1 << 7)  /* Bit 7:  Counter Clock Disable with RB Loading */
#define TC_CMR_ETRGEDG_SHIFT      (8)       /* Bits 8-9:  External Trigger Edge Selection */
#define TC_CMR_ETRGEDG_MASK       (3 << TC_CMR_ETRGEDG_SHIFT)
#  define TC_CMR_ETRGEDG_NONE     (0 << TC_CMR_ETRGEDG_SHIFT) /* None */
#  define TC_CMR_ETRGEDG_RISING   (1 << TC_CMR_ETRGEDG_SHIFT) /* Rising edge */
#  define TC_CMR_ETRGEDG_FALLING  (2 << TC_CMR_ETRGEDG_SHIFT) /* Falling edge */
#  define TC_CMR_ETRGEDG_BOTH     (3 << TC_CMR_ETRGEDG_SHIFT) /* Each edge */
#define TC_CMR_ABETRG             (1 << 10) /* Bit 10: TIOA or TIOB External Trigger Selection */
#define TC_CMR_CPCTRG             (1 << 14) /* Bit 14: RC Compare Trigger Enable */
#define TC_CMR_LDRA_SHIFT         (16)      /* Bits 16-17: A Loading Selection */
#define TC_CMR_LDRA_MASK          (3 << TC_CMR_LDRA_SHIFT)
#  define TC_CMR_LDRA_NONE        (0 << TC_CMR_LDRA_SHIFT) /* None */
#  define TC_CMR_LDRA_RISING      (1 << TC_CMR_LDRA_SHIFT) /* Rising edge of TIOA */
#  define TC_CMR_LDRA_FALLING     (2 << TC_CMR_LDRA_SHIFT) /* Falling edge of TIOA */
#  define TC_CMR_LDRA_BOTH        (3 << TC_CMR_LDRA_SHIFT) /* Each edge of TIOA */
#define TC_CMR_LDRB_SHIFT         (18)      /* Bits 18-19: RB Loading Selection */
#define TC_CMR_LDRB_MASK          (3 << TC_CMR_LDRB_SHIFT)
#  define TC_CMR_LDRB_NONE        (0 << TC_CMR_LDRB_SHIFT) /* None */
#  define TC_CMR_LDRB_RISING      (1 << TC_CMR_LDRB_SHIFT) /* Rising edge of TIOA */
#  define TC_CMR_LDRB_FALLING     (2 << TC_CMR_LDRB_SHIFT) /* Falling edge of TIOA */
#  define TC_CMR_LDRB_BOTH        (3 << TC_CMR_LDRB_SHIFT) /* Each edge of TIOA */

/* Channel Mode Register -- Waveform Mode */

#define TC_CMR_CPCSTOP            (1 << 6)  /* Bit 6:  Counter Clock Stopped with RC Compare */
#define TC_CMR_CPCDIS             (1 << 7)  /* Bit 7:  Counter Clock Disable with RC Compare */
#define TC_CMR_EEVTEDG_SHIFT      (8)       /* Bits 8-9:  External Event Edge Selection */
#define TC_CMR_EEVTEDG_MASK       (3 << TC_CMR_EEVTEDG_SHIFT)
#  define TC_CMR_EEVTEDG_NONE     (0 << TC_CMR_EEVTEDG_SHIFT) /* None */
#  define TC_CMR_EEVTEDG_RISING   (1 << TC_CMR_EEVTEDG_SHIFT) /* Rising edge */
#  define TC_CMR_EEVTEDG_FALLING  (2 << TC_CMR_EEVTEDG_SHIFT) /* Falling edge */
#  define TC_CMR_EEVTEDG_BOTH     (3 << TC_CMR_EEVTEDG_SHIFT) /* Each edge */
#define TC_CMR_EEVT_SHIFT         (10)       /* Bits 10-11:  External Event Selection */
#define TC_CMR_EEVT_MASK          (3 << TC_CMR_EEVT_SHIFT)
#  define TC_CMR_EEVT_TIOB        (0 << TC_CMR_EEVT_SHIFT) /* TIOB Input */
#  define TC_CMR_EEVT_XC0         (1 << TC_CMR_EEVT_SHIFT) /* XC0 Output */
#  define TC_CMR_EEVT_XC1         (2 << TC_CMR_EEVT_SHIFT) /* XC1 Output */
#  define TC_CMR_EEVT_XC2         (3 << TC_CMR_EEVT_SHIFT) /* XC2 Output */
#define TC_CMR_ENETRG             (1 << 12)  /* Bit 12: External Event Trigger Enable */
#define TC_CMR_WAVSEL_SHIFT       (13)       /* Bits 13-14:  Waveform Selection */
#define TC_CMR_WAVSEL_MASK        (3 << TC_CMR_WAVSEL_SHIFT)
#  define TC_CMR_WAVSEL_UPNOT     (0 << TC_CMR_WAVSEL_SHIFT) /* UP mode no trigger on RC compare */
#  define TC_CMR_WAVSEL_UPDWNNOT  (1 << TC_CMR_WAVSEL_SHIFT) /* UPDOWN mode no trigger on RC compare */
#  define TC_CMR_WAVSEL_UPT       (2 << TC_CMR_WAVSEL_SHIFT) /* UP mode with trigger on RC compare */
#  define TC_CMR_WAVSEL_UPDWNT    (3 << TC_CMR_WAVSEL_SHIFT) /* UPDOWN mode with trigger on RC compare */
#define TC_CMR_ACPA_SHIFT         (16)       /* Bits 16-17: RA Compare Effect on TIOA */
#define TC_CMR_ACPA_MASK          (3 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_NONE        (0 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_SET         (1 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_CLEAR       (2 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_TOGGLE      (3 << TC_CMR_ACPA_SHIFT)
#define TC_CMR_ACPC_SHIFT         (18)       /* Bits 18-19: RC Compare Effect on TIOA */
#define TC_CMR_ACPC_MASK          (3 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_NONE        (0 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_SET         (1 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_CLEAR       (2 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_TOGGLE      (3 << TC_CMR_ACPC_SHIFT)
#define TC_CMR_AEEVT_SHIFT        (20)       /* Bits 20-21: External Event Effect on TIOA */
#define TC_CMR_AEEVT_MASK         (3 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_NONE       (0 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_SET        (1 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_CLEAR      (2 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_TOGGLE     (3 << TC_CMR_AEEVT_SHIFT)
#define TC_CMR_ASWTRG_SHIFT       (22)       /* Bits 22-23: Software Trigger Effect on TIOA */
#define TC_CMR_ASWTRG_MASK        (3 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_NONE      (0 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_SET       (1 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_CLEAR     (2 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_TOGGLE    (3 << TC_CMR_ASWTRG_SHIFT)
#define TC_CMR_BCPB_SHIFT         (24)       /* Bits 24-25: RB Compare Effect on TIOB */
#define TC_CMR_BCPB_MASK          (3 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_NONE        (0 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_SET         (1 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_CLEAR       (2 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_TOGGLE      (3 << TC_CMR_BCPB_SHIFT)
#define TC_CMR_BCPC_SHIFT         (26)       /* Bits 26-27: RC Compare Effect on TIOB */
#define TC_CMR_BCPC_MASK          (3 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_NONE        (0 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_SET         (1 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_CLEAR       (2 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_TOGGLE      (3 << TC_CMR_BCPC_SHIFT)
#define TC_CMR_BEEVT_SHIFT        (28)       /* Bits 28-29: External Event Effect on TIOB */
#define TC_CMR_BEEVT_MASK         (3 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_NONE       (0 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_SET        (1 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_CLEAR      (2 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_TOGGLE     (3 << TC_CMR_BEEVT_SHIFT)
#define TC_CMR_BSWTRG_SHIFT       (30)       /* Bits 30-31: Software Trigger Effect on TIOB */
#define TC_CMR_BSWTRG_MASK        (3 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_NONE      (0 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_SET       (1 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_CLEAR     (2 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_TOGGLE    (3 << TC_CMR_BSWTRG_SHIFT)

/* Channel Counter Value */

#define TC_CV_MASK                (0xffff)

/* Channel Register A */

#define TC_RA_MASK                (0xffff)

/* Channel Register B */

#define TC_RB_MASK                (0xffff)

/* Channel Register C */

#define TC_RC_MASK                (0xffff)

/* Channel Status Register (common) */
/* Interrupt Enable Register */
/* Channel Interrupt Disable Register */
/* Channel Interrupt Mask Register */

#define TC_INT_COVFS              (1 << 0)  /* Bit 0:  Counter Overflow Status/Int */
#define TC_INT_LOVRS              (1 << 1)  /* Bit 1:  Load Overrun Status/Int */
#define TC_INT_CPAS               (1 << 2)  /* Bit 2:  RA Compare Status/Int */
#define TC_INT_CPBS               (1 << 3)  /* Bit 3:  RB Compare Status/Int */
#define TC_INT_CPCS               (1 << 4)  /* Bit 4:  RC Compare Status/Int */
#define TC_INT_LDRAS              (1 << 5)  /* Bit 5:  RA Loading Status/Int */
#define TC_INT_LDRBS              (1 << 6)  /* Bit 6:  RB Loading Status/Int */
#define TC_INT_ETRGS              (1 << 7)  /* Bit 7:  External Trigger Status/Int */

/* Channel Status Register (only) */

#define TC_SR_CLKSTA              (1 << 16) /* Bit 16: Clock Enabling Status */
#define TC_SR_MTIOA               (1 << 17) /* Bit 17: TIOA Mirror */
#define TC_SR_MTIOB               (1 << 18) /* Bit 18: TIOB Mirror */

/* Block Control Register */

#define TC_BCR_SYNC               (1 << 0)  /* Bit 0:  Synchro Command */

/* Block Mode Register */

#define TC_BMR_TC0XC0S_SHIFT      (0)       /* Bits 0-1: External Clock Signal 0 Selection */
#define TC_BMR_TC0XC0S_MASK       (3 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC2XC0S_TCLK0    (0 << TC_BMR_TC2XC0S_SHIFT) /* TCLK0 */
#  define TC_BMR_TC2XC0S_NONE     (1 << TC_BMR_TC2XC0S_SHIFT) /* None */
#  define TC_BMR_TC2XC0S_TIOA1    (2 << TC_BMR_TC2XC0S_SHIFT) /* TIOA1 */
#  define TC_BMR_TC2XC0S_TIOA2    (3 << TC_BMR_TC2XC0S_SHIFT) /* TIOA2 */
#define TC_BMR_TC1XC1S_SHIFT      (2)       /* Bits 2-3: External Clock Signal 1 Selection */
#define TC_BMR_TC1XC1S_MASK       (3 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC2XC1S_TCLK1    (0 << TC_BMR_TC2XC1S_SHIFT) /* TCLK1 */
#  define TC_BMR_TC2XC1S_NONE     (1 << TC_BMR_TC2XC1S_SHIFT) /* None */
#  define TC_BMR_TC2XC1S_TIOA0    (2 << TC_BMR_TC2XC1S_SHIFT) /* TIOA0 */
#  define TC_BMR_TC2XC1S_TIOA2    (3 << TC_BMR_TC2XC1S_SHIFT) /* TIOA2 */
#define TC_BMR_TC2XC2S_SHIFT      (3)       /* Bits 4-5: External Clock Signal 2 Selection */
#define TC_BMR_TC2XC2S_MASK       (3 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TCLK2    (0 << TC_BMR_TC2XC2S_SHIFT) /* TCLK2 */
#  define TC_BMR_TC2XC2S_NONE     (1 << TC_BMR_TC2XC2S_SHIFT) /* None */
#  define TC_BMR_TC2XC2S_TIOA0    (2 << TC_BMR_TC2XC2S_SHIFT) /* TIOA0 */
#  define TC_BMR_TC2XC2S_TIOA1    (3 << TC_BMR_TC2XC2S_SHIFT) /* TIOA1 */
				 
/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_TC_H */

