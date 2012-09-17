/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_pdb.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_PDB_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_PDB_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_PDB_SC_OFFSET        0x0000 /* Status and Control Register */
#define KINETIS_PDB_MOD_OFFSET       0x0004 /* Modulus Register */
#define KINETIS_PDB_CNT_OFFSET       0x0008 /* Counter Register */
#define KINETIS_PDB_IDLY_OFFSET      0x000c /* Interrupt Delay Register */

#define KINETIS_PDB_CH_OFFSET(n)     (0x0010+(0x28*(n)) /* Channel n */
#define KINETIS_PDB_CHC1_OFFSET      0x0000 /* Channel n Control Register 1 */
#define KINETIS_PDB_CHS_OFFSET       0x0004 /* Channel n Status Register */
#define KINETIS_PDB_CHDLY0_OFFSET    0x0008 /* Channel n Delay 0 Register */
#define KINETIS_PDB_CHDLY1_OFFSET    0x000c /* Channel n Delay 1 Register */

#define KINETIS_PDB_CH0C1_OFFSET     0x0010 /* Channel 0 Control Register 1 */
#define KINETIS_PDB_CH0S_OFFSET      0x0014 /* Channel 0 Status Register */
#define KINETIS_PDB_CH0DLY0_OFFSET   0x0018 /* Channel 0 Delay 0 Register */
#define KINETIS_PDB_CH0DLY1_OFFSET   0x001c /* Channel 0 Delay 1 Register */

#define KINETIS_PDB_CH1C1_OFFSET     0x0038 /* Channel 1 Control Register 1 */
#define KINETIS_PDB_CH1S_OFFSET      0x003c /* Channel 1 Status Register */
#define KINETIS_PDB_CH1DLY0_OFFSET   0x0040 /* Channel 1 Delay 0 Register */
#define KINETIS_PDB_CH1DLY1_OFFSET   0x0044 /* Channel 1 Delay 1 Register */

#define KINETIS_PDB_INT_OFFSET(n)    (0x0150+((n)<<3) /* DAC Interval n offset */
#define KINETIS_PDB_DACINTC_OFFSET   0x0000 /* DAC Interval Trigger n Control Register */
#define KINETIS_PDB_DACINT_OFFSET    0x0004 /* DAC Interval n Register */

#define KINETIS_PDB_DACINTC0_OFFSET  0x0150 /* DAC Interval Trigger 0 Control Register */
#define KINETIS_PDB_DACINT0_OFFSET   0x0154 /* DAC Interval 0 Register */

#define KINETIS_PDB_DACINTC1_OFFSET  0x0158 /* DAC Interval Trigger 1 Control Register */
#define KINETIS_PDB_DACINT1_OFFSET   0x015c /* DAC Interval 1 Register */

#define KINETIS_PDB_PO0EN_OFFSET     0x0190 /* Pulse-Out 0 Enable Register */
#define KINETIS_PDB_PO0DLY_OFFSET    0x0194 /* Pulse-Out 0 Delay Register */

/* Register Addresses ***********************************************************************/

#define KINETIS_PDB0_SC              (KINETIS_PDB0_BASE+KINETIS_PDB_SC_OFFSET)
#define KINETIS_PDB0_MOD             (KINETIS_PDB0_BASE+KINETIS_PDB_MOD_OFFSET)
#define KINETIS_PDB0_CNT             (KINETIS_PDB0_BASE+KINETIS_PDB_CNT_OFFSET)
#define KINETIS_PDB0_IDLY            (KINETIS_PDB0_BASE+KINETIS_PDB_IDLY_OFFSET)

#define KINETIS_PDB0_CH_BASE(n)      (KINETIS_PDB0_BASE+KINETIS_PDB_CH_OFFSET(n))
#define KINETIS_PDB0_CHC1(n)         (KINETIS_PDB_CH_BASE(n)+KINETIS_PDB_CHC1_OFFSET)
#define KINETIS_PDB0_CHS(n)          (KINETIS_PDB_CH_BASE(n)+KINETIS_PDB_CHS_OFFSET)
#define KINETIS_PDB0_CHDLY0(n)       (KINETIS_PDB_CH_BASE(n)+KINETIS_PDB_CHDLY0_OFFSET)
#define KINETIS_PDB0_CHDLY1(n)       (KINETIS_PDB_CH_BASE(n)+KINETIS_PDB_CHDLY1_OFFSET)

#define KINETIS_PDB0_CH0C1           (KINETIS_PDB0_BASE+KINETIS_PDB_CH0C1_OFFSET)
#define KINETIS_PDB0_CH0S            (KINETIS_PDB0_BASE+KINETIS_PDB_CH0S_OFFSET)
#define KINETIS_PDB0_CH0DLY0         (KINETIS_PDB0_BASE+KINETIS_PDB_CH0DLY0_OFFSET)
#define KINETIS_PDB0_CH0DLY1         (KINETIS_PDB0_BASE+KINETIS_PDB_CH0DLY1_OFFSET)

#define KINETIS_PDB0_CH1C1           (KINETIS_PDB0_BASE+KINETIS_PDB_CH1C1_OFFSET)
#define KINETIS_PDB0_CH1S            (KINETIS_PDB0_BASE+KINETIS_PDB_CH1S_OFFSET)
#define KINETIS_PDB0_CH1DLY0         (KINETIS_PDB0_BASE+KINETIS_PDB_CH1DLY0_OFFSET)
#define KINETIS_PDB0_CH1DLY1         (KINETIS_PDB0_BASE+KINETIS_PDB_CH1DLY1_OFFSET)

#define KINETIS_PDB0_INT_BASE(n)     (KINETIS_PDB0_BASE+KINETIS_PDB_INT_OFFSET(n))
#define KINETIS_PDB0_DACINTC(n)      (KINETIS_PDB_INT_BASE(n)+KINETIS_PDB_DACINTC_OFFSET)
#define KINETIS_PDB0_DACINT(n)       (KINETIS_PDB_INT_BASE(n)+KINETIS_PDB_DACINT_OFFSET)

#define KINETIS_PDB0_DACINTC0        (KINETIS_PDB0_BASE+KINETIS_PDB_DACINTC0_OFFSET)
#define KINETIS_PDB0_DACINT0         (KINETIS_PDB0_BASE+KINETIS_PDB_DACINT0_OFFSET)

#define KINETIS_PDB0_DACINTC1        (KINETIS_PDB0_BASE+KINETIS_PDB_DACINTC1_OFFSET)
#define KINETIS_PDB0_DACINT1         (KINETIS_PDB0_BASE+KINETIS_PDB_DACINT1_OFFSET)

#define KINETIS_PDB0_PO0EN           (KINETIS_PDB0_BASE+KINETIS_PDB_PO0EN_OFFSET)
#define KINETIS_PDB0_PO0DLY          (KINETIS_PDB0_BASE+KINETIS_PDB_PO0DLY_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* Status and Control Register */

#define PDB_SC_LDOK                  (1 << 0)  /* Bit 0:  Load OK */
#define PDB_SC_CONT                  (1 << 1)  /* Bit 1:  Continuous Mode Enable */
#define PDB_SC_MULT_SHIFT            (2)       /* Bits 2-3: Multiplication Factor Select for Prescaler */
#define PDB_SC_MULT_MASK             (3 << PDB_SC_MULT_SHIFT)
#  define PDB_SC_MULT_1              (0 << PDB_SC_MULT_SHIFT)
#  define PDB_SC_MULT_10             (1 << PDB_SC_MULT_SHIFT)
#  define PDB_SC_MULT_20             (2 << PDB_SC_MULT_SHIFT)
#  define PDB_SC_MULT_40             (3 << PDB_SC_MULT_SHIFT)
                                               /* Bit 4: Reserved */
#define PDB_SC_PDBIE                 (1 << 5)  /* Bit 5:  PDB Interrupt Enable */
#define PDB_SC_PDBIF                 (1 << 6)  /* Bit 6:  PDB Interrupt Flag */
#define PDB_SC_PDBEN                 (1 << 7)  /* Bit 7:  PDB Enable */
#define PDB_SC_TRGSEL_SHIFT          (8)       /* Bits 8-11: Trigger Input Source Select */
#define PDB_SC_TRGSEL_MASK           (15 << PDB_SC_TRGSEL_SHIFT)
#  define PDB_SC_TRGSEL_TRGIN0       (0 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 0 */
#  define PDB_SC_TRGSEL_TRGIN1       (1 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 1 */
#  define PDB_SC_TRGSEL_TRGIN2       (2 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 2 */
#  define PDB_SC_TRGSEL_TRGIN3       (3 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 3 */
#  define PDB_SC_TRGSEL_TRGIN4       (4 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 4 */
#  define PDB_SC_TRGSEL_TRGIN5       (5 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 5 */
#  define PDB_SC_TRGSEL_TRGIN6       (6 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 6 */
#  define PDB_SC_TRGSEL_TRGIN7       (7 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 7 */
#  define PDB_SC_TRGSEL_TRGIN8       (8 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 8 */
#  define PDB_SC_TRGSEL_TRGIN9       (9 << PDB_SC_TRGSEL_SHIFT)  /* Trigger-In 9 */
#  define PDB_SC_TRGSEL_TRGIN10      (10 << PDB_SC_TRGSEL_SHIFT) /* Trigger-In 0 */
#  define PDB_SC_TRGSEL_TRGIN11      (11 << PDB_SC_TRGSEL_SHIFT) /* Trigger-In 1 */
#  define PDB_SC_TRGSEL_TRGIN12      (12 << PDB_SC_TRGSEL_SHIFT) /* Trigger-In 2 */
#  define PDB_SC_TRGSEL_TRGIN13      (13 << PDB_SC_TRGSEL_SHIFT) /* Trigger-In 3 */
#  define PDB_SC_TRGSEL_TRGIN14      (14 << PDB_SC_TRGSEL_SHIFT) /* Trigger-In 4 */
#  define PDB_SC_TRGSEL_TRGSW        (15 << PDB_SC_TRGSEL_SHIFT) /* Software trigger */
#define PDB_SC_PRESCALER_SHIFT       (12)       /* Bits 12-14: Prescaler Divider Select */
#define PDB_SC_PRESCALER_MASK        (7 << PDB_SC_PRESCALER_SHIFT)
#  define PDB_SC_PRESCALER_DIVM      (0 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / MULT */
#  define PDB_SC_PRESCALER_DIV2M     (1 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 2*MULT */
#  define PDB_SC_PRESCALER_DIV4M     (2 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 4*MULT */
#  define PDB_SC_PRESCALER_DIV8M     (3 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 8*MULT */
#  define PDB_SC_PRESCALER_DIV16M    (4 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 16*MULT */
#  define PDB_SC_PRESCALER_DIV32M    (5 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 32*MULT */
#  define PDB_SC_PRESCALER_DIV64M    (6 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 64*MULT */
#  define PDB_SC_PRESCALER_DIV128M   (7 << PDB_SC_PRESCALER_SHIFT) /* Peripheral clock / 128*MULT */
#define PDB_SC_DMAEN                 (1 << 15)  /* Bit 15: DMA Enable */
#define PDB_SC_SWTRIG                (1 << 16)  /* Bit 16: Software Trigger */
#define PDB_SC_PDBEIE                (1 << 17)  /* Bit 17: PDB Sequence Error Interrupt Enable */
#define PDB_SC_LDMOD_SHIFT           (18)       /* Bits 18-19: Load Mode Select */
#define PDB_SC_LDMOD_MASK            (3 << PDB_SC_LDMOD_SHIFT)
#  define PDB_SC_LDMOD_LDOK          (0 << PDB_SC_LDMOD_SHIFT) /* Load after 1 written to LDOK */
#  define PDB_SC_LDMOD_PDBCNT        (1 << PDB_SC_LDMOD_SHIFT) /* Load when the PDB counter = MOD */
#  define PDB_SC_LDMOD_TRIGGER       (2 << PDB_SC_LDMOD_SHIFT) /* Load when trigger input event */
#  define PDB_SC_LDMOD_EITHER        (3 << PDB_SC_LDMOD_SHIFT) /* Load when either occurs */
                                                /* Bits 20-31: Reserved */
/* Modulus Register */

                                                /* Bits 16-31: Reserved */
#define PDB_MOD_MASK                 (0xffff)   /* Bits 0-15: PDB Modulus */

/* Counter Register */

                                                /* Bits 16-31: Reserved */
#define PDB_CNT_MASK                 (0xffff)   /* Bits 0-15: PDB Counter */

/* Interrupt Delay Register */

                                                /* Bits 16-31: Reserved */
#define PDB_IDLY_MASK                 (0xffff)  /* Bits 0-15: PDB Interrupt Delay */

/* Channel n Control Register 1 */

#define PDB_CHC1_EN_SHIFT             (0)       /* Bits 0-7: Pre-Trigger Enable */
#define PDB_CHC1_EN_MASK              (0xff << PDB_CHC1_EN_SHIFT)
#  define PDB_CHC1_EN_CHAN(n)         ((1 << (n)) << PDB_CHC1_EN_SHIFT)
#define PDB_CHC1_TOS_SHIFT            (8)       /* Bits 8-15: Pre-Trigger Output Select */
#define PDB_CHC1_TOS_MASK             (0xff << PDB_CHC1_TOS_SHIFT)
#  define PDB_CHC1_TOS_CHAN(n)        ((1 << (n)) << PDB_CHC1_TOS_SHIFT)
#define PDB_CHC1_BB_SHIFT             (16)      /* Bits 16-23: Pre-Trigger Back-to-Back Operation Enable */
#define PDB_CHC1_BB_MASK              (0xff << PDB_CHC1_BB_SHIFT)
#  define PDB_CHC1_BB_CHAN(n)         ((1 << (n)) << PDB_CHC1_BB_SHIFT)
                                                /* Bits 24-31: Reserved */
/* Channel n Status Register */

#define PDB_CHS_ERR_SHIFT             (0)       /* Bits 0-7: PDB Channel Sequence Error Flags */
#define PDB_CHS_ERR_MASK              (0xff << PDB_CHS_ERR_SHIFT)
#  define PDB_CHS1_ERR_CHAN(n)        ((1 << (n)) << PDB_CHS_ERR_SHIFT)
                                                /* Bits 8-15: Reserved */
#define PDB_CHS_CF_SHIFT              (16)      /* Bits 16-23: PDB Channel Flags */
#define PDB_CHS_CF_MASK               (0xff << PDB_CHS_CF_SHIFT)
#  define PDB_CHS_CF_CHAN(n)          ((1 << (n)) << PDB_CHS_CF_SHIFT)
                                                /* Bits 24-31: Reserved */
/* Channel n Delay 0 Register */
                                                /* Bits 16-31: Reserved */
#define PDB_CHDLY0_MASK               (0xffff)  /* Bits 0-15: PDB Channel Delay */

/* Channel n Delay 1 Register */
                                                /* Bits 16-31: Reserved */
#define PDB_CHDLY1_MASK               (0xffff)  /* Bits 0-15: PDB Channel Delay */

/* DAC Interval Trigger n Control Register */

#define PDB_DACINTC_TOE               (1 << 0)  /* Bit 0:  DAC Interval Trigger Enable */
#define PDB_DACINTC_EXT               (1 << 1)  /* Bit 1:  DAC External Trigger Input Enable */
                                                /* Bits 2-31: Reserved */
/* DAC Interval n Register */
                                                /* Bits 16-31: Reserved */
#define PDB_DACINT_MASK               (0xffff)  /* Bits 0-15: DAC Interval */

/* Pulse-Out 0 Enable Register */
#define PDB__
                                                /* Bits 6-31: Reserved */
#define PDB_PO0EN_MASK                (0xff)    /* Bits 0-7: PDB Pulse-Out Enable */

/* Pulse-Out 0 Delay Register */

#define PDB_PO0DLY_DLY1_SHIFT         (16)      /* Bits 16-31: PDB Pulse-Out Delay 1 */
#define PDB_PO0DLY_DLY1_MASK          (0xffff << PDB_PO0DLY_DLY1_SHIFT)
#define PDB_PO0DLY_DLY2_SHIFT         (0)       /* Bits 0-15: PDB Pulse-Out Delay 2 */
#define PDB_PO0DLY_DLY2_MASK          (0xffff << PDB_PO0DLY_DLY2_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_PDB_H */
