/************************************************************************************
 * arch/hc/src/m9s12/m9s12_atd.h
 * Defintions for ATD10b8c v3
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_ATD_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_ATD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_ATD_CTL0_OFFSET           0x0000 /* ATD control register 0 */
#define HCS12_ATD_CTL1_OFFSET           0x0001 /* ATD control register 1 */
#define HCS12_ATD_CTL2_OFFSET           0x0002 /* ATD control register 2 */
#define HCS12_ATD_CTL3_OFFSET           0x0003 /* ATD control register 3 */
#define HCS12_ATD_CTL4_OFFSET           0x0004 /* ATD control register 4 */
#define HCS12_ATD_CTL5_OFFSET           0x0005 /* ATD control register 5 */
#define HCS12_ATD_STAT0_OFFSET          0x0006 /* ATD status register 0 */
#define HCS12_ATD_TEST0_OFFSET          0x0008 /* ATD test register 0 */
#define HCS12_ATD_TEST1_OFFSET          0x0009 /* ATD test register 1 */
#define HCS12_ATD_STAT1_OFFSET          0x000b /* ATD status register 1 */
#define HCS12_ATD_IEN_OFFSET            0x000d /* ATD Input enable register */
#define HCS12_ATD_PORTAD_OFFSET         0x000f /* Port data register */
                                               /* Left justified data */
#define HCS12_ATD_DLH_OFFSET(n)         (0x0010+(n))
#define HCS12_ATD_DLL_OFFSET(n)         (0x0011+(n))
#define HCS12_ATD_DL0H_OFFSET           0x0010 /* ATD conversion result register 0 (high) */
#define HCS12_ATD_DL0L_OFFSET           0x0011 /* ATD conversion result register 0 (low) */
#define HCS12_ATD_DL1H_OFFSET           0x0012 /* ATD conversion result register 1 (high) */
#define HCS12_ATD_DL1L_OFFSET           0x0013 /* ATD conversion result register 1 (low) */
#define HCS12_ATD_DL2H_OFFSET           0x0014 /* ATD conversion result register 2 (high) */
#define HCS12_ATD_DL2L_OFFSET           0x0015 /* ATD conversion result register 2 (low) */
#define HCS12_ATD_DL3H_OFFSET           0x0016 /* ATD conversion result register 3 (high) */
#define HCS12_ATD_DL3L_OFFSET           0x0017 /* ATD conversion result register 3 (low) */
#define HCS12_ATD_DL4H_OFFSET           0x0018 /* ATD conversion result register 4 (high) */
#define HCS12_ATD_DL4L_OFFSET           0x0019 /* ATD conversion result register 4 (low) */
#define HCS12_ATD_DL5H_OFFSET           0x001a /* ATD conversion result register 5 (high) */
#define HCS12_ATD_DL5L_OFFSET           0x001b /* ATD conversion result register 5 (low) */
#define HCS12_ATD_DL6H_OFFSET           0x001c /* ATD conversion result register 6 (high) */
#define HCS12_ATD_DL6L_OFFSET           0x001d /* ATD conversion result register 6 (low) */
#define HCS12_ATD_DL7H_OFFSET           0x001e /* ATD conversion result register 7 (high) */
#define HCS12_ATD_DL7L_OFFSET           0x001f /* ATD conversion result register 7 (low) */
                                               /* Right justified data */
#define HCS12_ATD_DRH_OFFSET(n)         (0x0020+(n))
#define HCS12_ATD_DRL_OFFSET(n)         (0x0021+(n))
#define HCS12_ATD_DR0H_OFFSET           0x0020 /* ATD conversion result register 0 (high) */
#define HCS12_ATD_DR0L_OFFSET           0x0021 /* ATD conversion result register 0 (low) */
#define HCS12_ATD_DR1H_OFFSET           0x0022 /* ATD conversion result register 1 (high) */
#define HCS12_ATD_DR1L_OFFSET           0x0023 /* ATD conversion result register 1 (low) */
#define HCS12_ATD_DR2H_OFFSET           0x0024 /* ATD conversion result register 2 (high) */
#define HCS12_ATD_DR2L_OFFSET           0x0025 /* ATD conversion result register 2 (low) */
#define HCS12_ATD_DR3H_OFFSET           0x0026 /* ATD conversion result register 3 (high) */
#define HCS12_ATD_DR3L_OFFSET           0x0027 /* ATD conversion result register 3 (low) */
#define HCS12_ATD_DR4H_OFFSET           0x0028 /* ATD conversion result register 4 (high) */
#define HCS12_ATD_DR4L_OFFSET           0x0029 /* ATD conversion result register 4 (low) */
#define HCS12_ATD_DR5H_OFFSET           0x002a /* ATD conversion result register 5 (high) */
#define HCS12_ATD_DR5L_OFFSET           0x002b /* ATD conversion result register 5 (low) */
#define HCS12_ATD_DR6H_OFFSET           0x002c /* ATD conversion result register 6 (high) */
#define HCS12_ATD_DR6L_OFFSET           0x002d /* ATD conversion result register 6 (low) */
#define HCS12_ATD_DR7H_OFFSET           0x002e /* ATD conversion result register 7 (high) */
#define HCS12_ATD_DR7L_OFFSET           0x002f /* ATD conversion result register 7 (low) */

/* Register Addresses ***************************************************************/

#define HCS12_ATD_CTL0                  (HCS12_ATD_BASE+HCS12_ATD_CTL0_OFFSET)
#define HCS12_ATD_CTL1                  (HCS12_ATD_BASE+HCS12_ATD_CTL1_OFFSET)
#define HCS12_ATD_CTL2                  (HCS12_ATD_BASE+HCS12_ATD_CTL2_OFFSET)
#define HCS12_ATD_CTL3                  (HCS12_ATD_BASE+HCS12_ATD_CTL3_OFFSET)
#define HCS12_ATD_CTL4                  (HCS12_ATD_BASE+HCS12_ATD_CTL4_OFFSET)
#define HCS12_ATD_CTL5                  (HCS12_ATD_BASE+HCS12_ATD_CTL5_OFFSET)
#define HCS12_ATD_STAT0                 (HCS12_ATD_BASE+HCS12_ATD_STAT0_OFFSET)
#define HCS12_ATD_TEST0                 (HCS12_ATD_BASE+HCS12_ATD_TEST0_OFFSET)
#define HCS12_ATD_TEST1                 (HCS12_ATD_BASE+HCS12_ATD_TEST1_OFFSET)
#define HCS12_ATD_STAT1                 (HCS12_ATD_BASE+HCS12_ATD_STAT1_OFFSET)
#define HCS12_ATD_IEN                   (HCS12_ATD_BASE+HCS12_ATD_IEN_OFFSET)
#define HCS12_ATD_PORTAD                (HCS12_ATD_BASE+HCS12_ATD_PORTAD_OFFSET)
#define HCS12_ATD_DLH(n)                (HCS12_ATD_BASE+HCS12_ATD_DLH_OFFSET(n))
#define HCS12_ATD_DLL(n)                (HCS12_ATD_BASE+HCS12_ATD_DLL_OFFSET(n))
#define HCS12_ATD_DL0H                  (HCS12_ATD_BASE+HCS12_ATD_DL0H_OFFSET)
#define HCS12_ATD_DL0L                  (HCS12_ATD_BASE+HCS12_ATD_DL0L_OFFSET)
#define HCS12_ATD_DL1H                  (HCS12_ATD_BASE+HCS12_ATD_DL1H_OFFSET)
#define HCS12_ATD_DL1L                  (HCS12_ATD_BASE+HCS12_ATD_DL1L_OFFSET)
#define HCS12_ATD_DL2H                  (HCS12_ATD_BASE+HCS12_ATD_DL2H_OFFSET)
#define HCS12_ATD_DL2L                  (HCS12_ATD_BASE+HCS12_ATD_DL2L_OFFSET)
#define HCS12_ATD_DL3H                  (HCS12_ATD_BASE+HCS12_ATD_DL3H_OFFSET)
#define HCS12_ATD_DL3L                  (HCS12_ATD_BASE+HCS12_ATD_DL3L_OFFSET)
#define HCS12_ATD_DL4H                  (HCS12_ATD_BASE+HCS12_ATD_DL4H_OFFSET)
#define HCS12_ATD_DL4L                  (HCS12_ATD_BASE+HCS12_ATD_DL4L_OFFSET)
#define HCS12_ATD_DL5H                  (HCS12_ATD_BASE+HCS12_ATD_DL5H_OFFSET)
#define HCS12_ATD_DL5L                  (HCS12_ATD_BASE+HCS12_ATD_DL5L_OFFSET)
#define HCS12_ATD_DL6H                  (HCS12_ATD_BASE+HCS12_ATD_DL6H_OFFSET)
#define HCS12_ATD_DL6L                  (HCS12_ATD_BASE+HCS12_ATD_DL6L_OFFSET)
#define HCS12_ATD_DL7H                  (HCS12_ATD_BASE+HCS12_ATD_DL7H_OFFSET)
#define HCS12_ATD_DL7L                  (HCS12_ATD_BASE+HCS12_ATD_DL7L_OFFSET)
#define HCS12_ATD_DRH(n)                (HCS12_ATD_BASE+HCS12_ATD_DRH_OFFSET(n))
#define HCS12_ATD_DRL(n)                (HCS12_ATD_BASE+HCS12_ATD_DRL_OFFSET(n))
#define HCS12_ATD_DR0H                  (HCS12_ATD_BASE+HCS12_ATD_DR0H_OFFSET)
#define HCS12_ATD_DR0L                  (HCS12_ATD_BASE+HCS12_ATD_DR0L_OFFSET)
#define HCS12_ATD_DR1H                  (HCS12_ATD_BASE+HCS12_ATD_DR1H_OFFSET)
#define HCS12_ATD_DR1L                  (HCS12_ATD_BASE+HCS12_ATD_DR1L_OFFSET)
#define HCS12_ATD_DR2H                  (HCS12_ATD_BASE+HCS12_ATD_DR2H_OFFSET)
#define HCS12_ATD_DR2L                  (HCS12_ATD_BASE+HCS12_ATD_DR2L_OFFSET)
#define HCS12_ATD_DR3H                  (HCS12_ATD_BASE+HCS12_ATD_DR3H_OFFSET)
#define HCS12_ATD_DR3L                  (HCS12_ATD_BASE+HCS12_ATD_DR3L_OFFSET)
#define HCS12_ATD_DR4H                  (HCS12_ATD_BASE+HCS12_ATD_DR4H_OFFSET)
#define HCS12_ATD_DR4L                  (HCS12_ATD_BASE+HCS12_ATD_DR4L_OFFSET)
#define HCS12_ATD_DR5H                  (HCS12_ATD_BASE+HCS12_ATD_DR5H_OFFSET)
#define HCS12_ATD_DR5L                  (HCS12_ATD_BASE+HCS12_ATD_DR5L_OFFSET)
#define HCS12_ATD_DR6H                  (HCS12_ATD_BASE+HCS12_ATD_DR6H_OFFSET)
#define HCS12_ATD_DR6L                  (HCS12_ATD_BASE+HCS12_ATD_DR6L_OFFSET)
#define HCS12_ATD_DR7H                  (HCS12_ATD_BASE+HCS12_ATD_DR7H_OFFSET)
#define HCS12_ATD_DR7L                  (HCS12_ATD_BASE+HCS12_ATD_DR7L_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* ATD control register 0 */

#define ATD_CTL0_WRAP_SHIFT             (0)       /* Bits 0-2: Wrap around channel select */
#define ATD_CTL0_WRAP_MASK              (7)
#  define ATD_CTL0_WRAP_AN(n)           (n)
#  define ATD_CTL0_WRAP_AN1             (1)
#  define ATD_CTL0_WRAP_AN2             (2)
#  define ATD_CTL0_WRAP_AN3             (3)
#  define ATD_CTL0_WRAP_AN4             (4)
#  define ATD_CTL0_WRAP_AN5             (5)
#  define ATD_CTL0_WRAP_AN6             (6)
#  define ATD_CTL0_WRAP_AN7             (7)

/* ATD control register 1 */

#define ATD_CTL1_ETRIGCH_SHIFT          (0)       /* Bits 0-2: External channel select */
#define ATD_CTL1_ETRIGCH_MASK           (7)
#  define ATD_CTL2_ETRIGCH_AN(n)        (n)
#  define ATD_CTL2_ETRIGCH_AN0          (0)
#  define ATD_CTL2_ETRIGCH_AN1          (1)
#  define ATD_CTL2_ETRIGCH_AN2          (2)
#  define ATD_CTL2_ETRIGCH_AN3          (3)
#  define ATD_CTL2_ETRIGCH_AN4          (4)
#  define ATD_CTL2_ETRIGCH_AN5          (5)
#  define ATD_CTL2_ETRIGCH_AN6          (6)
#  define ATD_CTL2_ETRIGCH_AN7          (7)
#  define ATD_CTL2_ETRIGCH_ETRIG(n)     (n)
#  define ATD_CTL2_ETRIGCH_ETRIG0       (0)
#  define ATD_CTL2_ETRIGCH_ETRIG1       (1)
#  define ATD_CTL2_ETRIGCH_ETRIG2       (2)
#  define ATD_CTL2_ETRIGCH_ETRIG3       (3)
#define ATD_CTL1_ETRIGSEL               (1 << 7)  /* Bit 7: External trigger source select */

/* ATD control register 2 */

#define ATD_CTL2_ASCIF                  (1 << 0)  /* Bit 0: ATD Sequence Complete Interrup */
#define ATD_CTL2_ASCIE                  (1 << 1)  /* Bit 1: ATD Sequence Complete Interrupt Enable */
#define ATD_CTL2_ETRIG                  (1 << 1)  /* Bit 2: External Trigger Mode Enable */
#define ATD_CTL2_ETRIG_SHIFT            (3)       /* Bits 3-4: External Trigger Mode */
#define ATD_CTL2_ETRIG_MASK             (0 << ATD_CTL2_ETRIG_SHIFT)
#  define ATD_CTL2_ETRIG_FALLING        (0 << ATD_CTL2_ETRIG_SHIFT)
#  define ATD_CTL2_ETRIG_RISING         (1 << ATD_CTL2_ETRIG_SHIFT)
#  define ATD_CTL2_ETRIG_LOW            (2 << ATD_CTL2_ETRIG_SHIFT)
#  define ATD_CTL2_ETRIG_HIGH           (3 << ATD_CTL2_ETRIG_SHIFT)
#define ATD_CTL2_AWAI                   (1 << 5)  /* Bit 5: ATD Power Down inWait Mode */
#define ATD_CTL2_AFFC                   (1 << 6)  /* Bit 6: ATD Fast Flag Clear All */
#define ATD_CTL2_ADPU                   (1 << 7)  /* Bit 7: ATD Power Up */

/* ATD control register 3 */

#define ATD_CTL3_FRZ_SHIFT              (0)       /* Bits 0-1: Background Debug Freeze Enable */
#define ATD_CTL3_FRZ_MASK               (3)
#  define ATD_CTL3_FRZ_CONTINUE         (0)       /* Continue conversion */
#  define ATD_CTL3_FRZ_FINISH           (2)       /* Finish current conversion, then freeze */
#  define ATD_CTL3_FRZ_IMMEDIATE        (3)       /* Freeze Immediately */
#define ATD_CTL3_FIFO                   (1 << 2)  /* Bit 2: Result Register FIFO Mode */
#define ATD_CTL3_SC_SHIFT               (3)       /* Bits 3-6: Conversion Sequence Length */
#define ATD_CTL3_SC_MASK                (15 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC(n)                ((n) << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC1                  (1 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC2                  (2 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC3                  (3 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC4                  (4 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC5                  (5 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC6                  (6 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC7                  (7 << ATD_CTL3_SC_SHIFT)
#  define ATD_CTL3_SC8                  (8 << ATD_CTL3_SC_SHIFT)

/* ATD control register 4 */

#define ATD_CTL4_PRS_SHIFT              (0)       /* Bits 0-4: ATD Clock Prescaler */
#define ATD_CTL4_PRS_MASK               (31 << ATD_CTL4_PRS_SHIFT)
#  define ATD_CTL4_PRS_DIV(n)           ((((n)-2) >> 1) << ATD_CTL4_PRS_SHIFT) /* Divide by n={2,4,6,...,64} */
#define ATD_CTL4_SMP_SHIFT              (5)       /* Bits 5-6: Sample Time Select */
#define ATD_CTL4_SMP_MASK               (3 << ATD_CTL4_SMP_SHIFT)
#  define ATD_CTL4_SMP2                 (0 << ATD_CTL4_SMP_SHIFT) /* 2 A/D conversion clock periods */
#  define ATD_CTL4_SMP4                 (1 << ATD_CTL4_SMP_SHIFT) /* 4 A/D conversion clock periods */
#  define ATD_CTL4_SMP8                 (2 << ATD_CTL4_SMP_SHIFT) /* 8 A/D conversion clock periods */
#  define ATD_CTL4_SMP16                (3 << ATD_CTL4_SMP_SHIFT) /* 16 A/D conversion clock periods */
#define ATD_CTL4_SRES8                  (1 << 7)  /* Bit 7: A/D Resolution Select */

/* ATD control register 5 */

#define ATD_CTL5_C_SHIFT                (0)      /* Bits 0-2: Analog Input Channel Select Code */
#define ATD_CTL5_C_MASK                 (7 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN(n)              ((n) << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN0                (0 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN1                (1 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN2                (2 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN3                (3 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN4                (4 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN5                (5 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN6                (6 << ATD_CTL5_C_SHIFT)
#  define ATD_CTL5_C_AN7                (7 << ATD_CTL5_C_SHIFT)
#define ATD_CTL5_MULT                   (1 << 4)  /* Bit 4: Multi-Channel Sample Mode */
#define ATD_CTL5_SCAN                   (1 << 5)  /* Bit 5: Continuous Conversion Sequence Mode */
#define ATD_CTL5_DSGN                   (1 << 6)  /* Bit 6: Result Register Data Signed or Unsigned Representation */
#define ATD_CTL5_DJM                    (1 << 7)  /* Bit 7: Result Register Data Justification */

/* ATD status register 0 */

#define ATD_STAT0_CC_SHIFT              (0)       /* Bits 0-2: Conversion counter */
#define ATD_STAT0_CC_MASK               (7)
#define ATD_STAT0_FIFOR                 (1 << 4)  /* Bit 4: FIFO Over Run Flag */
#define ATD_STAT0_ETORF                 (1 << 5)  /* Bit 5: External Trigger Overrun Flag */
#define ATD_STAT0_SCF                   (1 << 7)  /* Bit 7: Sequence Complete Flag */

/* ATD test register 0 -- 8 MS bits of data written in special mode */
/* ATD test register 1 */

#define ATD_TEST1_SC                    (1 << 0)  /* Bit 0: Enable special conversions */
#define ATD_TEST1_LSU_MASK              0xc0      /* Bits 6-7: 2 LS bits of special mode data */

/* ATD status register 1 */

#define ATD_STAT1_CCF(n)                (1 << (n)) /* Bit n: Conversion complete flag channel n */
#define ATD_STAT1_CCF0                  (1 << 0)  /* Bit 0: Conversion complete flag channel 0 */
#define ATD_STAT1_CCF1                  (1 << 1)  /* Bit 1: Conversion complete flag channel 1 */
#define ATD_STAT1_CCF2                  (1 << 2)  /* Bit 2: Conversion complete flag channel 2 */
#define ATD_STAT1_CCF3                  (1 << 3)  /* Bit 3: Conversion complete flag channel 3 */
#define ATD_STAT1_CCF4                  (1 << 4)  /* Bit 4: Conversion complete flag channel 4 */
#define ATD_STAT1_CCF5                  (1 << 5)  /* Bit 5: Conversion complete flag channel 5 */
#define ATD_STAT1_CCF6                  (1 << 6)  /* Bit 6: Conversion complete flag channel 6 */
#define ATD_STAT1_CCF7                  (1 << 7)  /* Bit 7: Conversion complete flag channel 7 */

/* ATD Input enable register */

#define ATD_IEN(n)                      (1 << (n)) /* Bit n: ATD Digital Input Enable on channel n */
#define ATD_IEN0                        (1 << 0)  /* Bit 0: ATD Digital Input Enable on channel 0 */
#define ATD_IEN1                        (1 << 1)  /* Bit 1: ATD Digital Input Enable on channel 1 */
#define ATD_IEN2                        (1 << 2)  /* Bit 2: ATD Digital Input Enable on channel 2 */
#define ATD_IEN3                        (1 << 3)  /* Bit 3: ATD Digital Input Enable on channel 3 */
#define ATD_IEN4                        (1 << 4)  /* Bit 4: ATD Digital Input Enable on channel 4 */
#define ATD_IEN5                        (1 << 5)  /* Bit 5: ATD Digital Input Enable on channel 5 */
#define ATD_IEN6                        (1 << 6)  /* Bit 6: ATD Digital Input Enable on channel 6 */
#define ATD_IEN7                        (1 << 7)  /* Bit 7: ATD Digital Input Enable on channel 7 */

/* Port data register */

#define ATD_PORTAD(n)                   (1 << (n)) /* Bit n: A/D Channel n (ANn) Digital Input */
#define ATD_PORTAD0                     (1 << 0)  /* Bit 0: A/D Channel 0 (AN0) Digital Input */
#define ATD_PORTAD1                     (1 << 1)  /* Bit 1: A/D Channel 1 (AN1) Digital Input */
#define ATD_PORTAD2                     (1 << 2)  /* Bit 2: A/D Channel 2 (AN2) Digital Input */
#define ATD_PORTAD3                     (1 << 3)  /* Bit 3: A/D Channel 3 (AN3) Digital Input */
#define ATD_PORTAD4                     (1 << 4)  /* Bit 4: A/D Channel 4 (AN4) Digital Input */
#define ATD_PORTAD5                     (1 << 5)  /* Bit 5: A/D Channel 5 (AN5) Digital Input */
#define ATD_PORTAD6                     (1 << 6)  /* Bit 6: A/D Channel 6 (AN6) Digital Input */
#define ATD_PORTAD7                     (1 << 7)  /* Bit 7: A/D Channel 7 (AN7) Digital Input */

/* ATD conversion result register 0-7 (high/l;ow) */

#define ATD_DLL_MASK                    0xc0      /* Bits 6-7: LS bits of left justified data */
#define ATD_DRH_MASK                    0x03      /* Bits 0-1: MS bits of right justified data */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_ATD_H */
