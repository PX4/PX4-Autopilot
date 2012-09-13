/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_qei.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_QEI_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_QEI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* Control registers */

#define LPC43_QEI_CON_OFFSET     0x0000 /* Control register */
#define LPC43_QEI_STAT_OFFSET    0x0004 /* Encoder status register */
#define LPC43_QEI_CONF_OFFSET    0x0008 /* Configuration register */

/* Position, index, and timer registers */

#define LPC43_QEI_POS_OFFSET     0x000c /* Position register */
#define LPC43_QEI_MAXPOS_OFFSET  0x0010 /* Maximum position register */
#define LPC43_QEI_CMPOS0_OFFSET  0x0014 /* Position compare register */
#define LPC43_QEI_CMPOS1_OFFSET  0x0018 /* Position compare register */
#define LPC43_QEI_CMPOS2_OFFSET  0x001c /* Position compare register */
#define LPC43_QEI_INXCNT_OFFSET  0x0020 /* Index count register */
#define LPC43_QEI_INXCMP0_OFFSET 0x0024 /* Index compare register 0 */
#define LPC43_QEI_LOAD_OFFSET    0x0028 /* Velocity timer reload register */
#define LPC43_QEI_TIME_OFFSET    0x002c /* Velocity timer register */
#define LPC43_QEI_VEL_OFFSET     0x0030 /* Velocity counter register */
#define LPC43_QEI_CAP_OFFSET     0x0034 /* Velocity capture register */
#define LPC43_QEI_VELCOMP_OFFSET 0x0038 /* Velocity compare register */
#define LPC43_QEI_FLTRPHA_OFFSET 0x003c /* Input digital filter register phase A */
#define LPC43_QEI_FLTRPHB_OFFSET 0x0040 /* Input digital filter register phase B */
#define LPC43_QEI_FLTRINX_OFFSET 0x0044 /* Input digital filter register index */
#define LPC43_QEI_WINDOW_OFFSET  0x0048 /* Index acceptance window register */
#define LPC43_QEI_INXCMP1_OFFSET 0x004c /* Index compare register 1 */
#define LPC43_QEI_INXCMP2_OFFSET 0x0050 /* Index compare register 2 */

/* Interrupt registers */

#define LPC43_QEI_IEC_OFFSET     0x0fd8 /* Interrupt enable clear register */
#define LPC43_QEI_IES_OFFSET     0x0fdc /* Interrupt enable set register */
#define LPC43_QEI_INTSTAT_OFFSET 0x0fe0 /* Interrupt status register */
#define LPC43_QEI_IE_OFFSET      0x0fe4 /* Interrupt enable register */
#define LPC43_QEI_CLR_OFFSET     0x0fe8 /* Interrupt status clear register */
#define LPC43_QEI_SET_OFFSET     0x0fec /* Interrupt status set register */

/* Register addresses ***************************************************************/
/* Control registers */

#define LPC43_QEI_CON            (LPC43_QEI_BASE+LPC43_QEI_CON_OFFSET)
#define LPC43_QEI_STAT           (LPC43_QEI_BASE+LPC43_QEI_STAT_OFFSET)
#define LPC43_QEI_CONF           (LPC43_QEI_BASE+LPC43_QEI_CONF_OFFSET)

/* Position, index, and timer registers */

#define LPC43_QEI_POS            (LPC43_QEI_BASE+LPC43_QEI_POS_OFFSET)
#define LPC43_QEI_MAXPOS         (LPC43_QEI_BASE+LPC43_QEI_MAXPOS_OFFSET)
#define LPC43_QEI_CMPOS0         (LPC43_QEI_BASE+LPC43_QEI_CMPOS0_OFFSET)
#define LPC43_QEI_CMPOS1         (LPC43_QEI_BASE+LPC43_QEI_CMPOS1_OFFSET)
#define LPC43_QEI_CMPOS2         (LPC43_QEI_BASE+LPC43_QEI_CMPOS2_OFFSET)
#define LPC43_QEI_INXCNT         (LPC43_QEI_BASE+LPC43_QEI_INXCNT_OFFSET)
#define LPC43_QEI_INXCMP0        (LPC43_QEI_BASE+LPC43_QEI_INXCMP0_OFFSET)
#define LPC43_QEI_LOAD           (LPC43_QEI_BASE+LPC43_QEI_LOAD_OFFSET)
#define LPC43_QEI_TIME           (LPC43_QEI_BASE+LPC43_QEI_TIME_OFFSET)
#define LPC43_QEI_VEL            (LPC43_QEI_BASE+LPC43_QEI_VEL_OFFSET)
#define LPC43_QEI_CAP            (LPC43_QEI_BASE+LPC43_QEI_CAP_OFFSET)
#define LPC43_QEI_VELCOMP        (LPC43_QEI_BASE+LPC43_QEI_VELCOMP_OFFSET)
#define LPC43_QEI_FLTRPHA        (LPC43_QEI_BASE+LPC43_QEI_FLTRPHA_OFFSET)
#define LPC43_QEI_FLTRPHB        (LPC43_QEI_BASE+LPC43_QEI_FLTRPHB_OFFSET)
#define LPC43_QEI_FLTRINX        (LPC43_QEI_BASE+LPC43_QEI_FLTRINX_OFFSET)
#define LPC43_QEI_WINDOW         (LPC43_QEI_BASE+LPC43_QEI_WINDOW_OFFSET)
#define LPC43_QEI_INXCMP1        (LPC43_QEI_BASE+LPC43_QEI_INXCMP1_OFFSET)
#define LPC43_QEI_INXCMP2        (LPC43_QEI_BASE+LPC43_QEI_INXCMP2_OFFSET)

/* Interrupt registers */

#define LPC43_QEI_IEC            (LPC43_QEI_BASE+LPC43_QEI_IEC_OFFSET)
#define LPC43_QEI_IES            (LPC43_QEI_BASE+LPC43_QEI_IES_OFFSET)
#define LPC43_QEI_INTSTAT        (LPC43_QEI_BASE+LPC43_QEI_INTSTAT_OFFSET)
#define LPC43_QEI_IE             (LPC43_QEI_BASE+LPC43_QEI_IE_OFFSET)
#define LPC43_QEI_CLR            (LPC43_QEI_BASE+LPC43_QEI_CLR_OFFSET)
#define LPC43_QEI_SET            (LPC43_QEI_BASE+LPC43_QEI_SET_OFFSET)

/* Register bit definitions *********************************************************/
/* The following registers hold 32-bit integer values and have no bit fields defined
 * in this section:
 *
 *   Position register (POS)
 *   Maximum position register (MAXPOS)
 *   Position compare register 0 (CMPOS0)
 *   Position compare register 1 (CMPOS1)
 *   Position compare register 2 (CMPOS2)
 *   Index count register (INXCNT)
 *   Index compare register 0 (INXCMP0)
 *   Index compare register 1 (INXCMP1)
 *   Index compare register 2 (INXCMP2)
 *   Velocity timer reload register (LOAD)
 *   Velocity timer register (TIME)
 *   Velocity counter register (VEL)
 *   Velocity capture register (CAP)
 *   Velocity compare register (VELCOMP)
 *   Digital filter registers (FLTRPHA, FLTRPHB)
 *   Digital filter index register (FLTINX)
 *   Index acceptance window register (WINDOW)
 */

/* Control registers */
/* Control register */

#define QEI_CON_RESP             (1 << 0)  /* Bit 0:  Reset position counter */
#define QEI_CON_RESPI            (1 << 1)  /* Bit 1:  Reset position counter on index */
#define QEI_CON_RESV             (1 << 2)  /* Bit 2:  Reset velocity */
#define QEI_CON_RESI             (1 << 3)  /* Bit 3:  Reset index counter */
                                           /* Bits 4-31: reserved */
/* Encoder status register */

#define QEI_STAT_DIR             (1 << 0)  /* Bit 0:  Direction bit */
                                           /* Bits 1-31: reserved */
/* Configuration register */

#define QEI_CONF_DIRINV          (1 << 0)  /* Bit 0:  Direction invert */
#define QEI_CONF_SIGMODE         (1 << 1)  /* Bit 1:  Signal Mode */
#define QEI_CONF_CAPMODE         (1 << 2)  /* Bit 2:  Capture Mode */
#define QEI_CONF_INVINX          (1 << 3)  /* Bit 3:  Invert Index */
#define QEI_CONF_CRESPI          (1 << 4)  /* Bit 4:  Reset position counter on index */
                                           /* Bits 1-15: reserved */
#define QEI_CONF_INXGATE_SHIFT   (16)      /* Bits 16-19: Index gating configuration */
#define QEI_CONF_INXGATE_MASK    (15 << QEI_CONF_INXGATE_SHIFT)
#  define QEI_CONF_INXGATE_A1B0  (1 << QEI_CONF_INXGATE_SHIFT) /* Pass index on Pha=1 Phb=0 */
#  define QEI_CONF_INXGATE_A1B1  (2 << QEI_CONF_INXGATE_SHIFT) /* Pass index on Pha=1 Phb=1 */
#  define QEI_CONF_INXGATE_A0B1  (4 << QEI_CONF_INXGATE_SHIFT) /* Pass index on Pha=0 Phb=1 */
#  define QEI_CONF_INXGATE_A0B0  (8 << QEI_CONF_INXGATE_SHIFT) /* Pass index on Pha=0 Phb=0 */
                                           /* Bits 4-31: reserved */

/* Interrupt registers */
/* Interrupt enable clear register (IEC), Interrupt enable set register (IES),
 * Interrupt status register (INTSTAT), Interrupt enable register (IE), Interrupt
 * status clear register (CLR), and Interrupt status set register (SET) common
 * bit definitions.
 */

#define QEI_INT_INX              (1 << 0)  /* Bit 0:  Index pulse detected */
#define QEI_INT_TIM              (1 << 1)  /* Bit 1:  Velocity timer overflow occurred */
#define QEI_INT_VELC             (1 << 2)  /* Bit 2:  Captured velocity less than compare velocity */
#define QEI_INT_DIR              (1 << 3)  /* Bit 3:  Change of direction detected */
#define QEI_INT_ERR              (1 << 4)  /* Bit 4:  Encoder phase error detected */
#define QEI_INT_ENCLK            (1 << 5)  /* Bit 5:  Eencoder clock pulse detected */
#define QEI_INT_POS0             (1 << 6)  /* Bit 6:  Position 0 compare equal to current position */
#define QEI_INT_POS1             (1 << 7)  /* Bit 7:  Position 1 compare equal to current position */
#define QEI_INT_POS2             (1 << 8)  /* Bit 8:  Position 2 compare equal to current position */
#define QEI_INT_REV0             (1 << 9)  /* Bit 9:  Index 0 compare equal to current index count */
#define QEI_INT_POS0REV          (1 << 10) /* Bit 10: Position 0 and revolution count interrupt */
#define QEI_INT_POS1REV          (1 << 11) /* Bit 11: Position 1 and revolution count interrupt */
#define QEI_INT_POS2REV          (1 << 12) /* Bit 12: Position 2 and revolution count interrupt */
#define QEI_INT_REV1             (1 << 13) /* Bit 13: Index 1 compare equal to current index count */
#define QEI_INT_REV2             (1 << 14) /* Bit 14: Index 2 compare equal to current index count */
#define QEI_INT_MAXPOS           (1 << 15) /* Bit 15: Current position count goes through MAXPOS */
                                           /* Bits 16-31: reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_QEI_H */
