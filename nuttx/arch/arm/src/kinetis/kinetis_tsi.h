/************************************************************************************
 * arch/arm/src/kinetis/kinetis_tsi.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_TSI_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_TSI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_TSI_GENCS_OFFSET       0x0000 /* General Control and Status Register */
#define KINETIS_TSI_SCANC_OFFSET       0x0004 /* SCAN control register */
#define KINETIS_TSI_PEN_OFFSET         0x0008 /* Pin enable register */
#define KINETIS_TSI_STATUS_OFFSET      0x000c /* Status Register */

#define KINETIS_TSI_CNTR_OFFSET(n)     (0x0100+(((n)-1)<<1) /* Counter Register n */
#define KINETIS_TSI_CNTR1_OFFSET       0x0100 /* Counter Register 1 */
#define KINETIS_TSI_CNTR3_OFFSET       0x0104 /* Counter Register 3 */
#define KINETIS_TSI_CNTR5_OFFSET       0x0108 /* Counter Register 5 */
#define KINETIS_TSI_CNTR7_OFFSET       0x010c /* Counter Register 7 */
#define KINETIS_TSI_CNTR9_OFFSET       0x0110 /* Counter Register 9 */
#define KINETIS_TSI_CNTR11_OFFSET      0x0114 /* Counter Register 11 */
#define KINETIS_TSI_CNTR13_OFFSET      0x0118 /* Counter Register 13 */
#define KINETIS_TSI_CNTR15_OFFSET      0x011c /* Counter Register 15 */

#define KINETIS_TSI_THRESHLD_OFFSET(n) (0x0120+((n)<<2)) /* Channel n threshold register */
#define KINETIS_TSI_THRESHLD0_OFFSET   0x0120 /* Channel 0 threshold register */
#define KINETIS_TSI_THRESHLD1_OFFSET   0x0124 /* Channel 1 threshold register */
#define KINETIS_TSI_THRESHLD2_OFFSET   0x0128 /* Channel 2 threshold register */
#define KINETIS_TSI_THRESHLD3_OFFSET   0x012c /* Channel 3 threshold register */
#define KINETIS_TSI_THRESHLD4_OFFSET   0x0130 /* Channel 4 threshold register */
#define KINETIS_TSI_THRESHLD5_OFFSET   0x0134 /* Channel 5 threshold register */
#define KINETIS_TSI_THRESHLD6_OFFSET   0x0138 /* Channel 6 threshold register */
#define KINETIS_TSI_THRESHLD7_OFFSET   0x013c /* Channel 7 threshold register */
#define KINETIS_TSI_THRESHLD8_OFFSET   0x0140 /* Channel 8 threshold register */
#define KINETIS_TSI_THRESHLD9_OFFSET   0x0144 /* Channel 9 threshold register */
#define KINETIS_TSI_THRESHLD10_OFFSET  0x0148 /* Channel 10 threshold register */
#define KINETIS_TSI_THRESHLD11_OFFSET  0x014c /* Channel 11 threshold register */
#define KINETIS_TSI_THRESHLD12_OFFSET  0x0150 /* Channel 12 threshold register */
#define KINETIS_TSI_THRESHLD13_OFFSET  0x0154 /* Channel 13 threshold register */
#define KINETIS_TSI_THRESHLD14_OFFSET  0x0158 /* Channel 14 threshold register */
#define KINETIS_TSI_THRESHLD15_OFFSET  0x015c /* Channel 15 threshold register */

/* Register Addresses ***************************************************************/

#define KINETIS_TSI0_GENCS             (KINETIS_TSI0_BASE+KINETIS_TSI_GENCS_OFFSET)
#define KINETIS_TSI0_SCANC             (KINETIS_TSI0_BASE+KINETIS_TSI_SCANC_OFFSET)
#define KINETIS_TSI0_PEN               (KINETIS_TSI0_BASE+KINETIS_TSI_PEN_OFFSET)
#define KINETIS_TSI0_STATUS            (KINETIS_TSI0_BASE+KINETIS_TSI_STATUS_OFFSET)

#define KINETIS_TSI0_CNTR              (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR_OFFSET(n))
#define KINETIS_TSI0_CNTR1             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR1_OFFSET)
#define KINETIS_TSI0_CNTR3             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR3_OFFSET)
#define KINETIS_TSI0_CNTR5             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR5_OFFSET)
#define KINETIS_TSI0_CNTR7             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR7_OFFSET)
#define KINETIS_TSI0_CNTR9             (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR9_OFFSET)
#define KINETIS_TSI0_CNTR11            (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR11_OFFSET)
#define KINETIS_TSI0_CNTR13            (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR13_OFFSET)
#define KINETIS_TSI0_CNTR15            (KINETIS_TSI0_BASE+KINETIS_TSI_CNTR15_OFFSET)

#define KINETIS_TSI0_THRESHLD(n)       (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD_OFFSET(n))
#define KINETIS_TSI0_THRESHLD0         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD0_OFFSET)
#define KINETIS_TSI0_THRESHLD1         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD1_OFFSET)
#define KINETIS_TSI0_THRESHLD2         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD2_OFFSET)
#define KINETIS_TSI0_THRESHLD3         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD3_OFFSET)
#define KINETIS_TSI0_THRESHLD4         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD4_OFFSET)
#define KINETIS_TSI0_THRESHLD5         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD5_OFFSET)
#define KINETIS_TSI0_THRESHLD6         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD6_OFFSET)
#define KINETIS_TSI0_THRESHLD7         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD7_OFFSET)
#define KINETIS_TSI0_THRESHLD8         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD8_OFFSET)
#define KINETIS_TSI0_THRESHLD9         (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD9_OFFSET)
#define KINETIS_TSI0_THRESHLD10        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD10_OFFSET)
#define KINETIS_TSI0_THRESHLD11        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD11_OFFSET)
#define KINETIS_TSI0_THRESHLD12        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD12_OFFSET)
#define KINETIS_TSI0_THRESHLD13        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD13_OFFSET)
#define KINETIS_TSI0_THRESHLD14        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD14_OFFSET)
#define KINETIS_TSI0_THRESHLD15        (KINETIS_TSI0_BASE+KINETIS_TSI_THRESHLD15_OFFSET)

/* Register Bit Definitions *********************************************************/

/* General Control and Status Register */

#define TSI_GENCS_STPE                 (1 << 0)  /* Bit 0:  TSI stop enable while in low-power modes */
#define TSI_GENCS_STM                  (1 << 1)  /* Bit 1:  Scan trigger mode */
                                                 /* Bits 2-3: Reserved */
#define TSI_GENCS_ESOR                 (1 << 4)  /* Bit 4:  End-of-scan or out-of-range interrupt select */
#define TSI_GENCS_ERIE                 (1 << 5)  /* Bit 5:  TSI error interrupt enable */
#define TSI_GENCS_TSIIE                (1 << 6)  /* Bit 6:  TSI interrupt enable */
#define TSI_GENCS_TSIEN                (1 << 7)  /* Bit 7:  TSI module enable */
#define TSI_GENCS_SWTS                 (1 << 8)  /* Bit 8:  Software trigger start */
#define TSI_GENCS_SCNIP                (1 << 9)  /* Bit 9:  Scan-in-progress status */
                                                 /* Bits 10-11: Reserved */
#define TSI_GENCS_OVRF                 (1 << 12) /* Bit 12: Overrun error flag
#define TSI_GENCS_EXTERF               (1 << 13) /* Bit 13: External electrode error occurred */
#define TSI_GENCS_OUTRGF               (1 << 14) /* Bit 14: Out of Range Flag */
#define TSI_GENCS_EOSF                 (1 << 15) /* Bit 15: End of scan flag */
#define TSI_GENCS_PS_SHIFT             (16)      /* Bits 16-18: Electrode oscillator prescaler */
#define TSI_GENCS_PS_MASK              (3 << TSI_GENCS_PS_SHIFT)
#  define TSI_GENCS_PS_DIV1            (0 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 1 */
#  define TSI_GENCS_PS_DIV2            (1 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 2 */
#  define TSI_GENCS_PS_DIV4            (2 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 4 */
#  define TSI_GENCS_PS_DIV8            (3 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 8 */
#  define TSI_GENCS_PS_DIV16           (4 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 16 */
#  define TSI_GENCS_PS_DIV32           (5 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 32 */
#  define TSI_GENCS_PS_DIV64           (6 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 64 */
#  define TSI_GENCS_PS_DIV128          (7 << TSI_GENCS_PS_SHIFT) /* Electrode oscillator / 128 */
#define TSI_GENCS_NSCN_SHIFT           (19)      /* Bits 19-23: Number of Consecutive Scans per Electrode */
#define TSI_GENCS_NSCN_MASK            (31 << TSI_GENCS_NSCN_SHIFT)
#  define TSI_GENCS_NSCN_TIMES(n)      (((n)-1) << TSI_GENCS_NSCN_MASK) /* n times per electrode, n=1..32 */
#define TSI_GENCS_LPSCNITV_SHIFT       (24)      /* Bits 24-27: TSI Low Power Mode Scan Interval */
#define TSI_GENCS_LPSCNITV_MASK        (15 << TSI_GENCS_LPSCNITV_SHIFT)
#  define TSI_GENCS_LPSCNITV_1MS       (0 << TSI_GENCS_LPSCNITV_SHIFT)  /* 1 ms scan interval */
#  define TSI_GENCS_LPSCNITV_5MS       (1 << TSI_GENCS_LPSCNITV_SHIFT)  /* 5 ms scan interval */
#  define TSI_GENCS_LPSCNITV_10MS      (2 << TSI_GENCS_LPSCNITV_SHIFT)  /* 10 ms scan interval */
#  define TSI_GENCS_LPSCNITV_15MS      (3 << TSI_GENCS_LPSCNITV_SHIFT)  /* 15 ms scan interval */
#  define TSI_GENCS_LPSCNITV_20MS      (4 << TSI_GENCS_LPSCNITV_SHIFT)  /* 20 ms scan interval */
#  define TSI_GENCS_LPSCNITV_30MS      (5 << TSI_GENCS_LPSCNITV_SHIFT)  /* 30 ms scan interval */
#  define TSI_GENCS_LPSCNITV_40MS      (6 << TSI_GENCS_LPSCNITV_SHIFT)  /* 40 ms scan interval */
#  define TSI_GENCS_LPSCNITV_50MS      (7 << TSI_GENCS_LPSCNITV_SHIFT)  /* 50 ms scan interval */
#  define TSI_GENCS_LPSCNITV_60MS      (8 << TSI_GENCS_LPSCNITV_SHIFT)  /* 75 ms scan interval */
#  define TSI_GENCS_LPSCNITV_75MS      (9 << TSI_GENCS_LPSCNITV_SHIFT)  /* 100 ms scan interval */
#  define TSI_GENCS_LPSCNITV_100MS     (10 << TSI_GENCS_LPSCNITV_SHIFT) /* 125 ms scan interval */
#  define TSI_GENCS_LPSCNITV_150MS     (11 << TSI_GENCS_LPSCNITV_SHIFT) /* 150 ms scan interval */
#  define TSI_GENCS_LPSCNITV_200MS     (12 << TSI_GENCS_LPSCNITV_SHIFT) /* 200 ms scan interval */
#  define TSI_GENCS_LPSCNITV_300MS     (13 << TSI_GENCS_LPSCNITV_SHIFT) /* 300 ms scan interval */
#  define TSI_GENCS_LPSCNITV_400MS     (14 << TSI_GENCS_LPSCNITV_SHIFT) /* 400 ms scan interval */
#  define TSI_GENCS_LPSCNITV_500MS     (15 << TSI_GENCS_LPSCNITV_SHIFT) /* 500 ms scan interval */
#define TSI_GENCS_LPCLKS               (1 << 28) /* Bit 28: Low Power Mode Clock Source Selection */
                                                 /* Bits 29-31: Reserved */
/* SCAN control register */

#define TSI_SCANC_AMPSC_SHIFT          (0)       /* Bits 0-2: Active mode prescaler */
#define TSI_SCANC_AMPSC_MASK           (7 << TSI_SCANC_AMPSC_SHIFT)
#  define TSI_SCANC_AMPSC_DIV1         (0 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 1 */
#  define TSI_SCANC_AMPSC_DIV2         (1 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 2 */
#  define TSI_SCANC_AMPSC_DIV4         (2 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 4 */
#  define TSI_SCANC_AMPSC_DIV8         (3 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 8 */
#  define TSI_SCANC_AMPSC_DIV16        (4 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 16 */
#  define TSI_SCANC_AMPSC_DIV32        (5 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 32 */
#  define TSI_SCANC_AMPSC_DIV64        (6 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 64 */
#  define TSI_SCANC_AMPSC_DIV128       (7 << TSI_SCANC_AMPSC_SHIFT) /* Input clock source / 128 */
#define TSI_SCANC_AMCLKS_SHIFT         (3)       /* Bits 3-4: Active mode clock source */
#define TSI_SCANC_AMCLKS_MASK          (3 << TSI_SCANC_AMCLKS_SHIFT)
#  define TSI_SCANC_AMCLKS_BUSCLK      (0 << TSI_SCANC_AMCLKS_SHIFT) /* Bus Clock */
#  define TSI_SCANC_AMCLKS_MCGIRCLK    (1 << TSI_SCANC_AMCLKS_SHIFT) /* MCGIRCLK */
#  define TSI_SCANC_AMCLKS_OSCERCLK    (2 << TSI_SCANC_AMCLKS_SHIFT) /* OSCERCLK */
#define TSI_SCANC_AMCLKDIV             (1 << 5)  /* Bit 5:  Active mode clock divider */
                                                 /* Bits 6-7: Reserved */
#define TSI_SCANC_SMOD_SHIFT           (8)       /* Bits 8-15: Scan modulo */
#define TSI_SCANC_SMOD_MASK            (0xff << TSI_SCANC_SMOD_SHIFT)
#  define TSI_SCANC_SMOD_CONTINUOUS    (0 << TSI_SCANC_SMOD_SHIFT)
#  define TSI_SCANC_SMOD(n)            ((n) << TSI_SCANC_SMOD_SHIFT)
#define TSI_SCANC_DELVOL_SHIFT         (16)      /* Bits 16-18: Delta voltage select applied to analog oscillators */
#define TSI_SCANC_DELVOL_MASK          (7 << TSI_SCANC_DELVOL_SHIFT)
#  define TSI_SCANC_DELVOL_100MV       (0 << TSI_SCANC_DELVOL_SHIFT) /* 100 mV delta voltage */
#  define TSI_SCANC_DELVOL_150MV       (1 << TSI_SCANC_DELVOL_SHIFT) /* 150 mV delta voltage */
#  define TSI_SCANC_DELVOL_200MV       (2 << TSI_SCANC_DELVOL_SHIFT) /* 200 mV delta voltage */
#  define TSI_SCANC_DELVOL_250MV       (3 << TSI_SCANC_DELVOL_SHIFT) /* 250 mV delta voltage */
#  define TSI_SCANC_DELVOL_300MV       (4 << TSI_SCANC_DELVOL_SHIFT) /* 300 mV delta voltage */
#  define TSI_SCANC_DELVOL_400MV       (5 << TSI_SCANC_DELVOL_SHIFT) /* 400 mV delta voltage */
#  define TSI_SCANC_DELVOL_500MV       (6 << TSI_SCANC_DELVOL_SHIFT) /* 500 mV delta voltage */
#  define TSI_SCANC_DELVOL_600MV       (7 << TSI_SCANC_DELVOL_SHIFT) /* 600 mV delta voltage */
#define TSI_SCANC_EXTCHRG_SHIFT        (19)      /* Bits 19-23: External oscillator charge current select */
#define TSI_SCANC_EXTCHRG_MASK         (31 << TSI_SCANC_EXTCHRG_SHIFT)
#  define TSI_SCANC_EXTCHRG_UA(n)      (((n)-1) << TSI_SCANC_EXTCHRG_SHIFT) /* n µA charge current, n=1..32 */
#define TSI_SCANC_CAPTRM_SHIFT         (24)      /* Bits 24-26: Internal capacitance trim value */
#define TSI_SCANC_CAPTRM_MASK          (7 << TSI_SCANC_CAPTRM_SHIFT)
#define TSI_SCANC_CAPTRM_0p5PF         (0 << TSI_SCANC_CAPTRM_SHIFT) /* 0.5 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_0p6PF         (1 << TSI_SCANC_CAPTRM_SHIFT) /* 0.6 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_0p7PF         (2 << TSI_SCANC_CAPTRM_SHIFT) /* 0.7 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_0p8PF         (3 << TSI_SCANC_CAPTRM_SHIFT) /* 0.8 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_0p9PF         (4 << TSI_SCANC_CAPTRM_SHIFT) /* 0.9 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_1p0PF         (5 << TSI_SCANC_CAPTRM_SHIFT) /* 1.0 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_1p1PF         (6 << TSI_SCANC_CAPTRM_SHIFT) /* 1.1 pF internal reference capacitance */
#define TSI_SCANC_CAPTRM_1p2PF         (7 << TSI_SCANC_CAPTRM_SHIFT) /* 1.2 pF internal reference capacitance */
#define TSI_SCANC_REFCHRG_SHIFT        (27)      /* Bits 27-31: Reference oscillator charge current select */
#define TSI_SCANC_REFCHRG_MASK         (31 << TSI_SCANC_REFCHRG_SHIFT)
#  define TSI_SCANC_REFCHRG_UA(n)      (((n)-1) << TSI_SCANC_REFCHRG_SHIFT) /* n µA charge current, n=1..32 */

/* Pin enable register */

#define TSI_PEN0                       (1 << 0)  /* Bit 0:  TSI pin 0 enable */
#define TSI_PEN1                       (1 << 1)  /* Bit 1:  TSI pin 1 enable */
#define TSI_PEN2                       (1 << 2)  /* Bit 2:  TSI pin 2 enable */
#define TSI_PEN3                       (1 << 3)  /* Bit 3:  TSI pin 3 enable */
#define TSI_PEN4                       (1 << 4)  /* Bit 4:  TSI pin 4 enable */
#define TSI_PEN5                       (1 << 5)  /* Bit 5:  TSI pin 5 enable */
#define TSI_PEN6                       (1 << 6)  /* Bit 6:  TSI pin 6 enable */
#define TSI_PEN7                       (1 << 7)  /* Bit 7:  TSI pin 7 enable */
#define TSI_PEN8                       (1 << 8)  /* Bit 8:  TSI pin 8 enable */
#define TSI_PEN9                       (1 << 9)  /* Bit 9:  TSI pin 9 enable */
#define TSI_PEN10                      (1 << 10) /* Bit 10: TSI pin 10 enable */
#define TSI_PEN11                      (1 << 11) /* Bit 11: TSI pin 11 enable */
#define TSI_PEN12                      (1 << 12) /* Bit 12: TSI pin 11 enable */
#define TSI_PEN13                      (1 << 13) /* Bit 13: TSI pin 13 enable */
#define TSI_PEN14                      (1 << 14) /* Bit 14: TSI pin 14 enable */
#define TSI_PEN15                      (1 << 15) /* Bit 15: TSI pin 15 enable */
#define TSI_PEN(n)                     (1 << (n)) /* Bit n: TSI pin n enable, n=0..15 */
#define TSI_PEN_LPSP_SHIFT             (16)      /* Bits 16-19: Low-power scan pin */
#define TSI_PEN_LPSP_MASK              (15 << TSI_PEN_LPSP_SHIFT)
#  define TSI_PEN_LPSP(n)              ((n) << TSI_PEN_LPSP_SHIFT) /* TSI_IN[n] active in low power mode */
                                                 /* Bits 20-31: Reserved */
/* Status Register */

#define TSI_STATUS_ORNGF0              (1 << 0)  /* Bit 0:  Touch Sensing Electrode Out-of-Range Flag 0 */
#define TSI_STATUS_ORNGF1              (1 << 1)  /* Bit 1:  Touch Sensing Electrode Out-of-Range Flag 1 */
#define TSI_STATUS_ORNGF2              (1 << 2)  /* Bit 2:  Touch Sensing Electrode Out-of-Range Flag 2 */
#define TSI_STATUS_ORNGF3              (1 << 3)  /* Bit 3:  Touch Sensing Electrode Out-of-Range Flag 3 */
#define TSI_STATUS_ORNGF4              (1 << 4)  /* Bit 4:  Touch Sensing Electrode Out-of-Range Flag 4 */
#define TSI_STATUS_ORNGF5              (1 << 5)  /* Bit 5:  Touch Sensing Electrode Out-of-Range Flag 5 */
#define TSI_STATUS_ORNGF6              (1 << 6)  /* Bit 6:  Touch Sensing Electrode Out-of-Range Flag 6 */
#define TSI_STATUS_ORNGF7              (1 << 7)  /* Bit 7:  Touch Sensing Electrode Out-of-Range Flag 7 */
#define TSI_STATUS_ORNGF8              (1 << 8)  /* Bit 8:  Touch Sensing Electrode Out-of-Range Flag 8 */
#define TSI_STATUS_ORNGF9              (1 << 9)  /* Bit 9:  Touch Sensing Electrode Out-of-Range Flag 9 */
#define TSI_STATUS_ORNGF10             (1 << 10) /* Bit 10: Touch Sensing Electrode Out-of-Range Flag 10 */
#define TSI_STATUS_ORNGF11             (1 << 11) /* Bit 11: Touch Sensing Electrode Out-of-Range Flag 11 */
#define TSI_STATUS_ORNGF12             (1 << 12) /* Bit 12: Touch Sensing Electrode Out-of-Range Flag 12 */
#define TSI_STATUS_ORNGF13             (1 << 13) /* Bit 13: Touch Sensing Electrode Out-of-Range Flag 13 */
#define TSI_STATUS_ORNGF14             (1 << 14) /* Bit 14: Touch Sensing Electrode Out-of-Range Flag 14 */
#define TSI_STATUS_ORNGF15             (1 << 15) /* Bit 15: Touch Sensing Electrode Out-of-Range Flag 15 */
#define TSI_STATUS_ORNGF(n)            (1 << (n)) /* Bits 0-15:  Touch Sensing Electrode Out-of-Range Flag n, n=0..15 */
#define TSI_STATUS_ERROF(n)            (1 << ((n)+16)) /* Bits 16-31:  TouchSensing Error Flag n, n=0..15 */
#define TSI_STATUS_ERROF0              (1 << 16) /* Bit 16: TouchSensing Error Flag 0 */
#define TSI_STATUS_ERROF1              (1 << 17) /* Bit 17: TouchSensing Error Flag 1 */
#define TSI_STATUS_ERROF2              (1 << 18) /* Bit 18: TouchSensing Error Flag 2 */
#define TSI_STATUS_ERROF3              (1 << 19) /* Bit 19: TouchSensing Error Flag 3 */
#define TSI_STATUS_ERROF4              (1 << 20) /* Bit 20: TouchSensing Error Flag 4 */
#define TSI_STATUS_ERROF5              (1 << 21) /* Bit 21: TouchSensing Error Flag 5 */
#define TSI_STATUS_ERROF6              (1 << 22) /* Bit 22: TouchSensing Error Flag 6 */
#define TSI_STATUS_ERROF7              (1 << 23) /* Bit 23: TouchSensing Error Flag 7 */
#define TSI_STATUS_ERROF8              (1 << 24) /* Bit 24: TouchSensing Error Flag 8 */
#define TSI_STATUS_ERROF9              (1 << 25) /* Bit 25: TouchSensing Error Flag 9 */
#define TSI_STATUS_ERROF10             (1 << 26) /* Bit 26: TouchSensing Error Flag 10 */
#define TSI_STATUS_ERROF11             (1 << 27) /* Bit 27: TouchSensing Error Flag 11 */
#define TSI_STATUS_ERROF12             (1 << 28) /* Bit 28: TouchSensing Error Flag 12 */
#define TSI_STATUS_ERROF13             (1 << 29) /* Bit 29: TouchSensing Error Flag 13 */
#define TSI_STATUS_ERROF14             (1 << 30) /* Bit 30: TouchSensing Error Flag 14 */
#define TSI_STATUS_ERROF15             (1 << 31) /* Bit 31: TouchSensing Error Flag 15 */

/* Counter Register n.  Note:  These values are reversed in the K40 and K60
 * documentation.  In the K40/K60 header files, however, CNTN1 is always the
 * the field in the most significant bits.  Let's go with that.
 */

#define TSI_CNTR_CNTN_SHIFT            (0)      /* Bits 0-15: TouchSensing channel n 16-bit counter value */
#define TSI_CNTR_CNTN_MASK             (0xffff << TSI_CNTR_CNTN_SHIFT)
#define TSI_CNTR_CNTN1_SHIFT           (16)     /* Bits 16-31: TouchSensing channel n-1 16-bit counter value */
#define TSI_CNTR_CNTN1_MASK            (0xffff << TSI_CNTR_CNTN1_SHIFT)

/* Channel n threshold register */

#define TSI_THRESHLD_HTHH_SHIFT        (0)       /* Bits 0-15: High threshold value */
#define TSI_THRESHLD_HTHH_MASK         (0xffff << TSI_THRESHLD_HTHH_SHIFT)
#define TSI_THRESHLD_LTHH_SHIFT        (16)      /* Bits 16-31: Low threshold value */
#define TSI_THRESHLD_LTHH_MASK         (0xffff << TSI_THRESHLD_LTHH_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_TSI_H */
