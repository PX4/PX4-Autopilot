/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_matric.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_MATRIX_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_MATRIX_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* MATRIX register offsets **************************************************************/

#define SAM3U_MATRIX_MCFG_OFFSET(n)      ((n)<<2)
#define SAM3U_MATRIX_MCFG0_OFFSET        0x0000 /* Master Configuration Register 0 */
#define SAM3U_MATRIX_MCFG1_OFFSET        0x0004 /* Master Configuration Register 1 */
#define SAM3U_MATRIX_MCFG2_OFFSET        0x0008 /* Master Configuration Register 2 */
#define SAM3U_MATRIX_MCFG3_OFFSET        0x000c /* Master Configuration Register 3 */
#define SAM3U_MATRIX_MCFG4_OFFSET        0x0010 /* Master Configuration Register 4 */
                                                /* 0x0014-0x003c: Reserved */
#define SAM3U_MATRIX_SCFG_OFFSET(n)      (0x0040+((n)<<2))
#define SAM3U_MATRIX_SCFG0_OFFSET        0x0040 /* Slave Configuration Register 0 */
#define SAM3U_MATRIX_SCFG1_OFFSET        0x0044 /* Slave Configuration Register 1 */
#define SAM3U_MATRIX_SCFG2_OFFSET        0x0048 /* Slave Configuration Register 2 */
#define SAM3U_MATRIX_SCFG3_OFFSET        0x004c /* Slave Configuration Register 3 */
#define SAM3U_MATRIX_SCFG4_OFFSET        0x0050 /* Slave Configuration Register 4 */
#define SAM3U_MATRIX_SCFG5_OFFSET        0x0054 /* Slave Configuration Register 5 */
#define SAM3U_MATRIX_SCFG6_OFFSET        0x0058 /* Slave Configuration Register 6 */
#define SAM3U_MATRIX_SCFG7_OFFSET        0x005c /* Slave Configuration Register 7 */
#define SAM3U_MATRIX_SCFG8_OFFSET        0x0060 /* Slave Configuration Register 8 */
#define SAM3U_MATRIX_SCFG9_OFFSET        0x0064 /* Slave Configuration Register 9 */
                                                /* 0x0068-0x007c: Reserved */
#define SAM3U_MATRIX_PRAS_OFFSET(n)     (0x0080+((n)<<3))
#define SAM3U_MATRIX_PRAS0_OFFSET        0x0080 /* Priority Register A for Slave 0 */
                                                /* 0x0084: Reserved */
#define SAM3U_MATRIX_PRAS1_OFFSET        0x0088 /* Priority Register A for Slave 1 */
                                                /* 0x008c: Reserved */
#define SAM3U_MATRIX_PRAS2_OFFSET        0x0090 /* Priority Register A for Slave 2 */
                                                /* 0x0094: Reserved */
#define SAM3U_MATRIX_PRAS3_OFFSET        0x0098 /* Priority Register A for Slave 3 */
                                                /* 0x009c: Reserved */
#define SAM3U_MATRIX_PRAS4_OFFSET        0x00a0 /* Priority Register A for Slave 4 */
                                                /* 0x00a4: Reserved */
#define SAM3U_MATRIX_PRAS5_OFFSET        0x00a8 /* Priority Register A for Slave 5 */
                                                /* 0x00ac: Reserved */
#define SAM3U_MATRIX_PRAS6_OFFSET        0x00b0 /* Priority Register A for Slave 6 */
                                                /* 0x00b4: Reserved */
#define SAM3U_MATRIX_PRAS7_OFFSET        0x00b8 /* Priority Register A for Slave 7 */
                                                /* 0x00bc: Reserved */
#define SAM3U_MATRIX_PRAS8_OFFSET        0x00c0 /* Priority Register A for Slave 8 */
                                                /* 0x00c4: Reserved */
#define SAM3U_MATRIX_PRAS9_OFFSET        0x00c8 /* Priority Register A for Slave 9 */
                                                /* 0x00cc-0x00fc: Reserved */
#define SAM3U_MATRIX_MRCR_OFFSET         0x0100 /* Master Remap Control Register */
                                                /* 0x0104-0x010c: Reserved */
#define SAM3U_MATRIX_WPMR_OFFSET         0x01e4 /* Write Protect Mode Register */
#define SAM3U_MATRIX_WPSR_OFFSET         0x01e8 /* Write Protect Status Register */
                                                /* 0x0110 - 0x01fc: Reserved */

/* MATRIX register adresses *************************************************************/

#define SAM3U_MATRIX_MCFG(n))            (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MCFG_OFFSET(n))
#define SAM3U_MATRIX_MCFG0               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MCFG0_OFFSET)
#define SAM3U_MATRIX_MCFG1               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MCFG1_OFFSET)
#define SAM3U_MATRIX_MCFG2               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MCFG2_OFFSET)
#define SAM3U_MATRIX_MCFG3               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MCFG3_OFFSET)
#define SAM3U_MATRIX_MCFG4               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MCFG4_OFFSET)
 
#define SAM3U_MATRIX_SCFG(n)             (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG_OFFSET(n))
#define SAM3U_MATRIX_SCFG0               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG0_OFFSET)
#define SAM3U_MATRIX_SCFG1               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG1_OFFSET)
#define SAM3U_MATRIX_SCFG2               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG2_OFFSET)
#define SAM3U_MATRIX_SCFG3               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG3_OFFSET)
#define SAM3U_MATRIX_SCFG4               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG4_OFFSET)
#define SAM3U_MATRIX_SCFG5               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG5_OFFSET)
#define SAM3U_MATRIX_SCFG6               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG6_OFFSET)
#define SAM3U_MATRIX_SCFG7               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG7_OFFSET)
#define SAM3U_MATRIX_SCFG8               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG8_OFFSET)
#define SAM3U_MATRIX_SCFG9               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_SCFG9_OFFSET)

#define SAM3U_MATRIX_PRAS(n)             (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS_OFFSET(n))
#define SAM3U_MATRIX_PRAS0               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS0_OFFSET)
#define SAM3U_MATRIX_PRAS1               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS1_OFFSET)
#define SAM3U_MATRIX_PRAS2               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS2_OFFSET)
#define SAM3U_MATRIX_PRAS3               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS3_OFFSET)
#define SAM3U_MATRIX_PRAS4               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS4_OFFSET)
#define SAM3U_MATRIX_PRAS5               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS5_OFFSET)
#define SAM3U_MATRIX_PRAS6               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS6_OFFSET)
#define SAM3U_MATRIX_PRAS7               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS7_OFFSET)
#define SAM3U_MATRIX_PRAS8               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS8_OFFSET)
#define SAM3U_MATRIX_PRAS9               (SAM3U_MATRIX_BASE+SAM3U_MATRIX_PRAS9_OFFSET)

#define SAM3U_MATRIX_MRCR                (SAM3U_MATRIX_BASE+SAM3U_MATRIX_MRCR_OFFSET)
#define SAM3U_MATRIX_WPMR                (SAM3U_MATRIX_BASE+SAM3U_MATRIX_WPMR_OFFSET)
#define SAM3U_MATRIX_WPSR                (SAM3U_MATRIX_BASE+SAM3U_MATRIX_WPSR_OFFSET)

/* MATRIX register bit definitions ******************************************************/

#define MATRIX_MCFG_ULBT_SHIFT           (0)       /* Bits 0-2:  Undefined Length Burst Type */
#define MATRIX_MCFG_ULBT_MASK            (7 << MATRIX_MCFG_ULBT_SHIFT)
#  define MATRIX_MCFG_ULBT_INF           (0 << MATRIX_MCFG_ULBT_SHIFT) /* Infinite Length Burst */
#  define MATRIX_MCFG_ULBT_SINGLE        (1 << MATRIX_MCFG_ULBT_SHIFT) /* Single Access */
#  define MATRIX_MCFG_ULBT_4BEAT         (2 << MATRIX_MCFG_ULBT_SHIFT) /* Four Beat Burst */
#  define MATRIX_MCFG_ULBT_8BEAT         (3 << MATRIX_MCFG_ULBT_SHIFT) /* Eight Beat Burst */
#  define MATRIX_MCFG_ULBT_16BEAT        (4 << MATRIX_MCFG_ULBT_SHIFT) /* Sixteen Beat Burst */

#define MATRIX_SCFG_SLOTCYCLE_SHIFT      (0)       /* Bits 0-7:  Maximum Number of Allowed Cycles for a Burst */
#define MATRIX_SCFG_SLOTCYCLE_MASK       (0xff << MATRIX_SCFG_SLOTCYCLE_SHIFT)
#define MATRIX_SCFG_DEFMSTRTYPE_SHIFT    (16)      /* Bits 16-17:  Default Master Type */
#define MATRIX_SCFG_DEFMSTRTYPE_MASK     (3 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_NONE   (0 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_LAST   (1 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#  define MATRIX_SCFG_DEFMSTRTYPE_FIXED  (2 << MATRIX_SCFG_DEFMSTRTYPE_SHIFT)
#define MATRIX_SCFG_FIXEDDEFMSTR_SHIFT   (18)      /* Bits 18-20:   Fixed Default Master */
#define MATRIX_SCFG_FIXEDDEFMSTR_MASK    (7 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG0_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG1_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG2_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG3_FIXEDDEFMSTR_ARMC (0 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG4_FIXEDDEFMSTR_ARMC (0 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG5_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG6_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG7_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG8_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG8_FIXEDDEFMSTR_HDMA (4 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG9_FIXEDDEFMSTR_ARMS (1 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)
#  define MATRIX_SCFG9_FIXEDDEFMSTR_HDMA (4 << MATRIX_SCFG_FIXEDDEFMSTR_SHIFT)

#define MATRIX_SCFG_ARBT_SHIFT          (24)      /* Bits 24-25:   Arbitration Type */
#define MATRIX_SCFG_ARBT_MASK           (3 << MATRIX_SCFG_ARBT_SHIFT)
#  define MATRIX_SCFG_ARBT_RR           (0 << MATRIX_SCFG_ARBT_SHIFT) /* Round-Robin Arbitration */
#  define MATRIX_SCFG_ARBT_FIXED        (1 << MATRIX_SCFG_ARBT_SHIFT) /* Fixed Priority Arbitration */

#define MATRIX_PRAS_MPR_SHIFT(x)        ((n)<<2)
#define MATRIX_PRAS_MPR_MASK(x)         (3 << MATRIX_PRAS_MPR_SHIFT(x))
#define MATRIX_PRAS_M0PR_SHIFT          (0)       /* Bits 0-1:  Master 0 Priority */
#define MATRIX_PRAS_M0PR_MASK           (3 << MATRIX_PRAS_M0PR_SHIFT)
#define MATRIX_PRAS_M1PR_SHIFT          (4)       /* Bits 4-5:   Master 1 Priority */
#define MATRIX_PRAS_M1PR_MASK           (3 << MATRIX_PRAS_M1PR_SHIFT)
#define MATRIX_PRAS_M2PR_SHIFT          (8)       /* Bits 8-9:  Master 2 Priority */
#define MATRIX_PRAS_M2PR_MASK           (3 << MATRIX_PRAS_M2PR_SHIFT)
#define MATRIX_PRAS_M3PR_SHIFT          (12)      /* Bits 12-13:  Master 3 Priority */
#define MATRIX_PRAS_M3PR_MASK           (3 << MATRIX_PRAS_M3PR_SHIFT)
#define MATRIX_PRAS_M4PR_SHIFT          (16)      /* Bits 16-17  Master 4 Priority */
#define MATRIX_PRAS_M4PR_MASK           (3 << MATRIX_PRAS_M4PR_SHIFT)

#define MATRIX_MRCR_RCB(x)              (1 << (x))
#define MATRIX_MRCR_RCB0                (1 << 0)  /* Bit 0:  Remap Command Bit for AHB Master 0 */
#define MATRIX_MRCR_RCB1                (1 << 1)  /* Bit 1:  Remap Command Bit for AHB Master 1 */
#define MATRIX_MRCR_RCB2                (1 << 2)  /* Bit 2:  Remap Command Bit for AHB Master 2 */
#define MATRIX_MRCR_RCB3                (1 << 3)  /* Bit 3:  Remap Command Bit for AHB Master 3 */
#define MATRIX_MRCR_RCB4                (1 << 4)  /* Bit 4:  Remap Command Bit for AHB Master 4 */

#define MATRIX_WPMR_WPEN                (1 << 0)  /* Bit 0:  Write Protect Enable */
#define MATRIX_WPMR_WPKEY_SHIFT         (8)       /* Bits 8-31:   Write Protect KEY (Write-only) */
#define MATRIX_WPMR_WPKEY_MASK          (0x00ffffff << MATRIX_WPMR_WPKEY_SHIFT)

#define MATRIX_WPSR_WPVS                (1 << 0)  /* Bit 0:  Enable Write Protect */
#define MATRIX_WPSR_WPVSRC_SHIFT        (8)       /* Bits 8-23:  Write Protect Violation Source */
#define MATRIX_WPSR_WPVSRC_MASK         (0xffff << MATRIX_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_MATRIX_H */
