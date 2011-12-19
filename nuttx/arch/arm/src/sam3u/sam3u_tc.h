/************************************************************************************************
 * arch/arm/src/sam3u/sam3u_tc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_TC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_TC_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* TC register offsets **************************************************************************/

/* Timer channel offsets (with respect to timer base offset 0f 0x00, 0x40, or 0x80 */

#define SAM3U_TCN_OFFSET(n)            (0x00 + ((n)<<6)) /* 0x00, 0x40, 0x80 */
#define SAM3U_TCN_CCR_OFFSET           0x00 /* Channel Control Register */
#define SAM3U_TCN_CMR_OFFSET           0x04 /* Channel Mode Register */
                                            /* 0x08 Reserved */
                                            /* 0x0c Reserved */
#define SAM3U_TCN_CV_OFFSET            0x10 /* Counter Value */
#define SAM3U_TCN_RA_OFFSET            0x14 /* Register A */
#define SAM3U_TCN_RB_OFFSET            0x18 /* Register B */
#define SAM3U_TCN_RC_OFFSET            0x1c /* Register C */
#define SAM3U_TCN_SR_OFFSET            0x20 /* Status Register */
#define SAM3U_TCN_IER_OFFSET           0x24 /* Interrupt Enable Register */
#define SAM3U_TCN_IDR_OFFSET           0x28 /* Interrupt Disable Register */
#define SAM3U_TCN_IMR_OFFSET           0x2c /* Interrupt Mask Register */

/* Timer common registers */

#define SAM3U_TC_BCR_OFFSET            0xc0 /* Block Control Register */
#define SAM3U_TC_BMR_OFFSET            0xc4 /* Block Mode Register */
#define SAM3U_TC_QIER_OFFSET           0xc8 /* QDEC Interrupt Enable Register */
#define SAM3U_TC_QIDR_OFFSET           0xcc /* QDEC Interrupt Disable Register */
#define SAM3U_TC_QIMR_OFFSET           0xd0 /* QDEC Interrupt Mask Register */
#define SAM3U_TC_QISR_OFFSET           0xd4 /* QDEC Interrupt Status Register */
                                            /* 0xd8 Reserved */
                                            /* 0xe4 Reserved */

/* TC register adresses *************************************************************************/

/* Timer channel offsets (with respect to timer base offset 0f 0x00, 0x40, or 0x80 */

#define SAM3U_TC_CCR(n)                (SAM3U_TCN_BASE(n)+SAM3U_TCN_CCR_OFFSET)
#define SAM3U_TC_CMR(n)                (SAM3U_TCN_BASE(n)+SAM3U_TCN_CMR_OFFSET)
#define SAM3U_TC_CV(n)                 (SAM3U_TCN_BASE(n)+SAM3U_TCN_CV_OFFSET)
#define SAM3U_TC_RA(n)                 (SAM3U_TCN_BASE(n)+SAM3U_TCN_RA_OFFSET)
#define SAM3U_TC_RB(n)                 (SAM3U_TCN_BASE(n)+SAM3U_TCN_RB_OFFSET)
#define SAM3U_TC_RC(n)                 (SAM3U_TCN_BASE(n)+SAM3U_TCN_RC_OFFSET)
#define SAM3U_TC_SR(n)                 (SAM3U_TCN_BASE(n)+SAM3U_TCN_SR_OFFSET)
#define SAM3U_TC_IER(n)                (SAM3U_TCN_BASE(n)+SAM3U_TCN_IER_OFFSET)
#define SAM3U_TC_IDR(n)                (SAM3U_TCN_BASE(n)+SAM3U_TCN_IDR_OFFSET)
#define SAM3U_TC_IMR(n)                (SAM3U_TCN_BASE(n)+SAM3U_TCN_IMR_OFFSET)

#define SAM3U_TC0_CCR                  (SAM3U_TC0_BASE+SAM3U_TCN_CCR_OFFSET)
#define SAM3U_TC0_CMR                  (SAM3U_TC0_BASE+SAM3U_TCN_CMR_OFFSET)
#define SAM3U_TC0_CV                   (SAM3U_TC0_BASE+SAM3U_TCN_CV_OFFSET)
#define SAM3U_TC0_RA                   (SAM3U_TC0_BASE+SAM3U_TCN_RA_OFFSET)
#define SAM3U_TC0_RB                   (SAM3U_TC0_BASE+SAM3U_TCN_RB_OFFSET)
#define SAM3U_TC0_RC                   (SAM3U_TC0_BASE+SAM3U_TCN_RC_OFFSET)
#define SAM3U_TC0_SR                   (SAM3U_TC0_BASE+SAM3U_TCN_SR_OFFSET)
#define SAM3U_TC0_IER                  (SAM3U_TC0_BASE+SAM3U_TCN_IER_OFFSET)
#define SAM3U_TC0_IDR                  (SAM3U_TC0_BASE+SAM3U_TCN_IDR_OFFSET)
#define SAM3U_TC0_IMR                  (SAM3U_TC0_BASE+SAM3U_TCN_IMR_OFFSET)

#define SAM3U_TC1_CCR                  (SAM3U_TC1_BASE+SAM3U_TCN_CCR_OFFSET)
#define SAM3U_TC1_CMR                  (SAM3U_TC1_BASE+SAM3U_TCN_CMR_OFFSET)
#define SAM3U_TC1_CV                   (SAM3U_TC1_BASE+SAM3U_TCN_CV_OFFSET)
#define SAM3U_TC1_RA                   (SAM3U_TC1_BASE+SAM3U_TCN_RA_OFFSET)
#define SAM3U_TC1_RB                   (SAM3U_TC1_BASE+SAM3U_TCN_RB_OFFSET)
#define SAM3U_TC1_RC                   (SAM3U_TC1_BASE+SAM3U_TCN_RC_OFFSET)
#define SAM3U_TC1_SR                   (SAM3U_TC1_BASE+SAM3U_TCN_SR_OFFSET)
#define SAM3U_TC1_IER                  (SAM3U_TC1_BASE+SAM3U_TCN_IER_OFFSET)
#define SAM3U_TC1_IDR                  (SAM3U_TC1_BASE+SAM3U_TCN_IDR_OFFSET)
#define SAM3U_TC1_IMR                  (SAM3U_TC1_BASE+SAM3U_TCN_IMR_OFFSET)

#define SAM3U_TC2_CCR                  (SAM3U_TC2_BASE+SAM3U_TCN_CCR_OFFSET)
#define SAM3U_TC2_CMR                  (SAM3U_TC2_BASE+SAM3U_TCN_CMR_OFFSET)
#define SAM3U_TC2_CV                   (SAM3U_TC2_BASE+SAM3U_TCN_CV_OFFSET)
#define SAM3U_TC2_RA                   (SAM3U_TC2_BASE+SAM3U_TCN_RA_OFFSET)
#define SAM3U_TC2_RB                   (SAM3U_TC2_BASE+SAM3U_TCN_RB_OFFSET)
#define SAM3U_TC2_RC                   (SAM3U_TC2_BASE+SAM3U_TCN_RC_OFFSET)
#define SAM3U_TC2_SR                   (SAM3U_TC2_BASE+SAM3U_TCN_SR_OFFSET)
#define SAM3U_TC2_IER                  (SAM3U_TC2_BASE+SAM3U_TCN_IER_OFFSET)
#define SAM3U_TC2_IDR                  (SAM3U_TC2_BASE+SAM3U_TCN_IDR_OFFSET)
#define SAM3U_TC2_IMR                  (SAM3U_TC2_BASE+SAM3U_TCN_IMR_OFFSET)

/* Timer common registers */

#define SAM3U_TC_BCR                   (SAM3U_TC_BASE+SAM3U_TC_BCR_OFFSET)
#define SAM3U_TC_BMR                   (SAM3U_TC_BASE+SAM3U_TC_BMR_OFFSET)
#define SAM3U_TC_QIER                  (SAM3U_TC_BASE+SAM3U_TC_QIER_OFFSET)
#define SAM3U_TC_QIDR                  (SAM3U_TC_BASE+SAM3U_TC_QIDR_OFFSET)
#define SAM3U_TC_QIMR                  (SAM3U_TC_BASE+SAM3U_TC_QIMR_OFFSET)
#define SAM3U_TC_QISR                  (SAM3U_TC_BASE+SAM3U_TC_QISR_OFFSET)

/* TC register bit definitions ******************************************************************/

/* Timer common registers */
/* TC Block Control Register */

#define TC_BCR_SYNC                    (1 << 0)  /* Bit 0: Synchro Command

/* TC Block Mode Register */

#define TC_BMR_TC0XC0S_SHIFT           (0)       /* Bits 0-1: External Clock Signal 0 Selection */
#define TC_BMR_TC0XC0S_MASK            (3 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TCLK0         (0 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_NONE          (1 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TIOA1         (2 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TIOA2         (3 << TC_BMR_TC0XC0S_SHIFT)
#define TC_BMR_TC1XC1S_SHIFT           (2)       /* Bits 2-3: External Clock Signal 1 Selection */
#define TC_BMR_TC1XC1S_MASK            (3 << TC_BMR_TC1XC1S_MASK)
#  define TC_BMR_TC1XC1S_TCLK1         (0 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_NONE          (1 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_TIOA0         (2 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_TIOA2         (3 << TC_BMR_TC1XC1S_SHIFT)
#define TC_BMR_TC2XC2S_SHIFT           (4)       /* Bits 4-5: External Clock Signal 2 Selection */
#define TC_BMR_TC2XC2S_MASK            (3 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TCLK2         (0 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_NONE          (1 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TIOA0         (2 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TIOA1         (3 << TC_BMR_TC2XC2S_SHIFT)
#define TC_BMR_QDEN                    (1 << 8)  /* Bit 8:  Quadrature Decoder Enabled */
#define TC_BMR_POSEN                   (1 << 9)  /* Bit 9:  Position Enabled */
#define TC_BMR_SPEEDEN                 (1 << 10) /* Bit 10: Speed Enabled */
#define TC_BMR_QDTRANS                 (1 << 11) /* Bit 11: Quadrature Decoding Transparent */
#define TC_BMR_EDGPHA                  (1 << 12) /* Bit 12: Edge on PHA count mode */
#define TC_BMR_INVA                    (1 << 13) /* Bit 13: Inverted PHA */
#define TC_BMR_INVB                    (1 << 14) /* Bit 14: Inverted PHB */
#define TC_BMR_SWAP                    (1 << 15) /* Bit 15: Swap PHA and PHB */
#define TC_BMR_INVIDX                  (1 << 16) /* Bit 16: Inverted Index */
#define TC_BMR_IDXPHB                  (1 << 17) /* Bit 17: Index pin is PHB pin */
#define TC_BMR_FILTER                  (1 << 19) /* Bit 19 */
#define TC_BMR_MAXFILT_SHIFT           (20)      /* Bits 20-25: Maximum Filter */
#define TC_BMR_MAXFILT_MASK            (63 << TC_BMR_MAXFILT_SHIFT)

/* TC QDEC Interrupt Enable Register, TC QDEC Interrupt Disable Register,
 * TC QDEC Interrupt Mask Register, TC QDEC Interrupt Status Register common
 * bit field definitions
 */
 
#define TC_QINT_IDX                    (1 << 0)  /* Bit 0: Index (Common) */
#define TC_QINT_DIRCHG                 (1 << 1)  /* Bit 1: Direction Change (Common) */
#define TC_QINT_QERR                   (1 << 2)  /* Bit 2: Quadrature Error (Common) */
#define TC_QISR_DIR                    (1 << 8)  /* Bit 8: Direction (QISR only) */

/* Timer Channel Registers */
/* TC Channel Control Register */

#define TCN_CCR_CLKEN                  (1 << 0)  /* Bit 0: Counter Clock Enable Command */
#define TCN_CCR_CLKDIS                 (1 << 1)  /* Bit 1: Counter Clock Disable Command */
#define TCN_CCR_SWTRG                  (1 << 2)  /* Bit 2: Software Trigger Command */

/* TC Channel Mode Register */

#define TCN_CMR_TCCLKS_SHIFT           (0)       /* Bits 0-2: Clock Selection (Common) */
#define TCN_CMR_TCCLKS_MASK            (7 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_TIMERCLOCK1   (0 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_TIMERCLOCK2   (1 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_TIMERCLOCK3   (2 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_TIMERCLOCK4   (3 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_TIMERCLOCK5   (4 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_XC0           (5 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_XC1           (6 << TCN_CMR_TCCLKS_SHIFT)
#  define TCN_CMR_TCCLKS_XC2           (7 << TCN_CMR_TCCLKS_SHIFT)
#define TCN_CMR_CLKI                   (1 << 3)  /* Bit 3: Clock Invert (Common) */
#define TCN_CMR_BURST_SHIFT            (4)       /* Bits 4-5: Burst Signal Selection (Common) */
#define TCN_CMR_BURST_MASK             (3 << TCN_CMR_BURST_MASK)
#define TCN_CMR_BURST_MASK             (3 << TCN_CMR_BURST_MASK)
#  define TCN_CMR_BURST_NOTGATED       (0 << TCN_CMR_BURST_MASK) /* Nott gated by external signal */
#  define TCN_CMR_BURST_XC0            (1 << TCN_CMR_BURST_MASK) /* XC0 ANDed with selected clock */
#  define TCN_CMR_BURST_XC1            (2 << TCN_CMR_BURST_MASK) /* XC1 ANDed with selected clock */
#  define TCN_CMR_BURST_XC2            (3 << TCN_CMR_BURST_MASK) /* XC2 ANDed with selected clock */
#define TCN_CMR_WAVE                   (1 << 15) /* Bit 15: (Common) */

#define TCN_CMR_LDBSTOP                (1 << 6)  /* Bit 6: Counter stopped with RB Loading (Capture mode) */
#define TCN_CMR_LDBDIS                 (1 << 7)  /* Bit 7: Counter disable with RB Loading (Capture mode) */
#define TCN_CMR_ETRGEDG_SHIFT          (8)       /* Bits 8-9: External Trigger Edge Selection (Capture mode) */
#define TCN_CMR_ETRGEDG_MASK           (3 << TCN_CMR_ETRGEDG_SHIFT)
#  define TCN_CMR_ETRGEDG_NONE         (0 << TCN_CMR_ETRGEDG_SHIFT) /* None */
#  define TCN_CMR_ETRGEDG_REDGE        (1 << TCN_CMR_ETRGEDG_SHIFT) /* Rising edge */
#  define TCN_CMR_ETRGEDG_FEDGE        (2 << TCN_CMR_ETRGEDG_SHIFT) /* Falling edge */
#  define TCN_CMR_ETRGEDG_EACH         (3 << TCN_CMR_ETRGEDG_SHIFT) /* Each */
#define TCN_CMR_ABETRG                 (1 << 10) /* Bit 10: TIOA or TIOB External Trigger Selection (Capture mode) */
#define TCN_CMR_CPCTRG                 (1 << 14) /* Bit 14: RC Compare Trigger Enable (Capture mode) */
#define TCN_CMR_LDRA_SHIFT             (16)      /* Bits 16-17: RA Loading Selection (Capture mode) */
#define TCN_CMR_LDRA_MASK              (3 << TCN_CMR_LDRA_SHIFT)
#  define TCN_CMR_LDRA_NONE            (0 << TCN_CMR_LDRA_SHIFT) /* None */
#  define TCN_CMR_LDRA_REDGE           (1 << TCN_CMR_LDRA_SHIFT) /* Rising edge of TIOA */
#  define TCN_CMR_LDRA_FEDGE           (2 << TCN_CMR_LDRA_SHIFT) /* Falling edge of TIOA */
#  define TCN_CMR_LDRA_EACH            (3 << TCN_CMR_LDRA_SHIFT) /* Each  edge of TIOA */
#define TCN_CMR_LDRB_SHIFT             (18)      /* Bits 18-19: RB Loading Selection (Capture mode) */
#define TCN_CMR_LDRB_MASK              (3 << TCN_CMR_LDRB_SHIFT)
#  define TCN_CMR_LDRB_NONE            (0 << TCN_CMR_LDRB_SHIFT) /* None */
#  define TCN_CMR_LDRB_REDGE           (1 << TCN_CMR_LDRB_SHIFT) /* Rising edge of TIOB */
#  define TCN_CMR_LDRB_FEDGE           (2 << TCN_CMR_LDRB_SHIFT) /* Falling edge of TIOB */
#  define TCN_CMR_LDRB_EACH            (3 << TCN_CMR_LDRB_SHIFT) /* Each  edge of TIOB */

#define TCN_CMR_CPCSTOP                (1 << 6)  /* Bit 6: Counter Clock Stopped with RC Compare (Waveform mode) */
#define TCN_CMR_CPCDIS                 (1 << 7)  /* Bit 7: Counter Clock Disable with RC Compare (Waveform mode) */
#define TCN_CMR_EEVTEDG_SHIFT          (8)       /* Bits 8-9: External Event Edge Selection (Waveform mode) */
#define TCN_CMR_EEVTEDG_MASK           (3 << TCN_CMR_EEVTEDG_SHIFT)
#  define TCN_CMR_EEVTEDG_NONE         (0 << TCN_CMR_EEVTEDG_SHIFT) /* None */
#  define TCN_CMR_EEVTEDG_REDGE        (1 << TCN_CMR_EEVTEDG_SHIFT) /* Rising edge */
#  define TCN_CMR_EEVTEDG_FEDGE        (2 << TCN_CMR_EEVTEDG_SHIFT) /* Falling edge */
#  define TCN_CMR_EEVTEDG_EACH         (3 << TCN_CMR_EEVTEDG_SHIFT) /* Each edge */
#define TCN_CMR_EEVT_SHIFT             (10)      /* Bits 10-11: External Event Selection (Waveform mode) */
#define TCN_CMR_EEVT_MASK              (3 << TCN_CMR_EEVT_SHIFT)
#  define TCN_CMR_EEVT_TIOB            (0 << TCN_CMR_EEVT_SHIFT) /* TIOB input */
#  define TCN_CMR_EEVT_XC0             (1 << TCN_CMR_EEVT_SHIFT) /* XC0 output */
#  define TCN_CMR_EEVT_XC1             (2 << TCN_CMR_EEVT_SHIFT) /* XC1 output */
#  define TCN_CMR_EEVT_XC2             (3 << TCN_CMR_EEVT_SHIFT) /* XC2 output */
#define TCN_CMR_ENETRG                 (1 << 12) /* Bit 12: External Event Trigger Enable (Waveform mode) */
#define TCN_CMR_WAVSEL_SHIFT           (13)      /* Bits 13-14: Waveform Selection (Waveform mode) */
#define TCN_CMR_WAVSEL_MASK            (3 << TCN_CMR_WAVSEL_SHIFT)
#  define TCN_CMR_WAVSEL_UP            (0 << TCN_CMR_WAVSEL_SHIFT) /* UP mode w/o auto trigger (Waveform mode) */
#  define TCN_CMR_WAVSEL_UPAUTO        (1 << TCN_CMR_WAVSEL_SHIFT) /* UP mode with auto trigger (Waveform mode) */
#  define TCN_CMR_WAVSEL_UPDWN         (2 << TCN_CMR_WAVSEL_SHIFT) /* UPDOWN mode w/o  auto trigger (Waveform mode) */
#  define TCN_CMR_WAVSEL_UPDWNAUTO     (3 << TCN_CMR_WAVSEL_SHIFT) /* UPDOWN mode with auto trigger (Waveform mode) */
#define TCN_CMR_ACPA_SHIFT             (16)      /* Bits 16-17: RA Compare Effect on TIOA (Waveform mode) */
#define TCN_CMR_ACPA_MASK              (3 << TCN_CMR_ACPA_SHIFT)
#  define TCN_CMR_ACPA_NONE            (0 << TCN_CMR_ACPA_SHIFT)
#  define TCN_CMR_ACPA_SET             (1 << TCN_CMR_ACPA_SHIFT)
#  define TCN_CMR_ACPA_CLEAR           (2 << TCN_CMR_ACPA_SHIFT)
#  define TCN_CMR_ACPA_TOGGLE          (3 << TCN_CMR_ACPA_SHIFT)
#define TCN_CMR_ACPC_SHIFT             (18)      /* Bits 18-19: RC Compare Effect on TIOA (Waveform mode) */
#define TCN_CMR_ACPC_MASK              (3 << TCN_CMR_ACPC_SHIFT)
#  define TCN_CMR_ACPC_NONE            (0 << TCN_CMR_ACPC_SHIFT)
#  define TCN_CMR_ACPC_SET             (1 << TCN_CMR_ACPC_SHIFT)
#  define TCN_CMR_ACPC_CLEAR           (2 << TCN_CMR_ACPC_SHIFT)
#  define TCN_CMR_ACPC_TOGGLE          (3 << TCN_CMR_ACPC_SHIFT)
#define TCN_CMR_AEEVT_SHIFT            (20)      /* Bits 20-21: External Event Effect on TIOA (Waveform mode) */
#define TCN_CMR_AEEVT_MASK             (3 << TCN_CMR_AEEVT_SHIFT)
#  define TCN_CMR_AEEVT_NONE           (0 << TCN_CMR_AEEVT_SHIFT)
#  define TCN_CMR_AEEVT_SET            (1 << TCN_CMR_AEEVT_SHIFT)
#  define TCN_CMR_AEEVT_CLEAR          (2 << TCN_CMR_AEEVT_SHIFT)
#  define TCN_CMR_AEEVT_TOGGLE         (3 << TCN_CMR_AEEVT_SHIFT)
#define TCN_CMR_ASWTRG_SHIFT           (22)      /* Bits 22-23: Software Trigger Effect on TIOA (Waveform mode) */
#define TCN_CMR_ASWTRG_MASK            (3 << TCN_CMR_ASWTRG_SHIFT)
#  define TCN_CMR_ASWTRG_NONE          (0 << TCN_CMR_ASWTRG_SHIFT)
#  define TCN_CMR_ASWTRG_SET           (1 << TCN_CMR_ASWTRG_SHIFT)
#  define TCN_CMR_ASWTRG_CLEAR         (2 << TCN_CMR_ASWTRG_SHIFT)
#  define TCN_CMR_ASWTRG_TOGGLE        (3 << TCN_CMR_ASWTRG_SHIFT)
#define TCN_CMR_BCPB_SHIFT             (24)      /* Bits 24-25: RB Compare Effect on TIOB (Waveform mode) */
#define TCN_CMR_BCPB_MASK              (3 << TCN_CMR_BCPB_SHIFT)
#  define TCN_CMR_BCPB_NONE            (0 << TCN_CMR_BCPB_SHIFT)
#  define TCN_CMR_BCPB_SET             (1 << TCN_CMR_BCPB_SHIFT)
#  define TCN_CMR_BCPB_CLEAR           (2 << TCN_CMR_BCPB_SHIFT)
#  define TCN_CMR_BCPB_TOGGLE          (3 << TCN_CMR_BCPB_SHIFT)
#define TCN_CMR_BCPC_SHIFT             (26)      /* Bits 26-27: RC Compare Effect on TIOB (Waveform mode) */
#define TCN_CMR_BCPC_MASK              (3 << TCN_CMR_BCPC_SHIFT)
#  define TCN_CMR_BCPC_NONE            (0 << TCN_CMR_BCPC_SHIFT)
#  define TCN_CMR_BCPC_SET             (1 << TCN_CMR_BCPC_SHIFT)
#  define TCN_CMR_BCPC_CLEAR           (2 << TCN_CMR_BCPC_SHIFT)
#  define TCN_CMR_BCPC_TOGGLE          (3 << TCN_CMR_BCPC_SHIFT)
#define TCN_CMR_BEEVT_SHIFT            (28)      /* Bits 28-29: External Event Effect on TIOB (Waveform mode) */
#define TCN_CMR_BEEVT_MASK             (3 << TCN_CMR_BEEVT_SHIFT)
#  define TCN_CMR_BEEVT_NONE           (0 << TCN_CMR_BEEVT_SHIFT)
#  define TCN_CMR_BEEVT_SET            (1 << TCN_CMR_BEEVT_SHIFT)
#  define TCN_CMR_BEEVT_CLEAR          (2 << TCN_CMR_BEEVT_SHIFT)
#  define TCN_CMR_BEEVT_TOGGLE         (3 << TCN_CMR_BEEVT_SHIFT)
#define TCN_CMR_BSWTRG_SHIFT           (30)      /* Bits 30-31: Software Trigger Effect on TIOB (Waveform mode) */
#define TCN_CMR_BSWTRG_MASK            (3 << TCN_CMR_BSWTRG_SHIFT)
#  define TCN_CMR_BSWTRG_NONE          (0 << TCN_CMR_BSWTRG_SHIFT)
#  define TCN_CMR_BSWTRG_SET           (1 << TCN_CMR_BSWTRG_SHIFT)
#  define TCN_CMR_BSWTRG_CLEAR         (2 << TCN_CMR_BSWTRG_SHIFT)
#  define TCN_CMR_BSWTRG_TOGGLE        (3 << TCN_CMR_BSWTRG_SHIFT)

/* TC Counter Value Register */

#define TCN_CV_SHIFT                   (0)       /* Bits 0-15: Counter Value */
#define TCN_CV_MASK                    (0xffff << TCN_CV_SHIFT)

/* TC Register A, B, C */

#define TCN_RVALUE_SHIFT               (0)       /* Bits 0-15: Register A, B, or C value */
#define TCN_RVALUE_MASK                (0xffff << TCN_RVALUE_SHIFT)

/* TC Status Register, TC Interrupt Enable Register, TC Interrupt Disable Register, and  TC Interrupt Mask Register common bit-field definitions */

#define TCN_INT_COVFS                  (1 << 0)  /* Bit 0:  Counter Overflow */
#define TCN_INT_LOVRS                  (1 << 1)  /* Bit 1:  Load Overrun */
#define TCN_INT_CPAS                   (1 << 2)  /* Bit 2:  RA Compare */
#define TCN_INT_CPBS                   (1 << 3)  /* Bit 3:  RB Compare */
#define TCN_INT_CPCS                   (1 << 4)  /* Bit 4:  RC Compare */
#define TCN_INT_LDRAS                  (1 << 5)  /* Bit 5:  RA Loading */
#define TCN_INT_LDRBS                  (1 << 6)  /* Bit 6:  RB Loading */
#define TCN_INT_ETRGS                  (1 << 7)  /* Bit 7:  External Trigger */
#define TCN_INT_CLKSTA                 (1 << 16) /* Bit 16: Clock Enabling (SR only) */
#define TCN_SR_MTIOA                   (1 << 17) /* Bit 17: TIOA Mirror (SR only) */
#define TCN_SR_MTIOB                   (1 << 18) /* Bit 18: TIOB Mirror (SR only)*/

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_TC_H */
