/********************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-che.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CHE_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CHE_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Register Offsets *************************************************************************/

#define PIC32MX_CHE_CON_OFFSET    0x0000 /* Pre-fetch cache control register */
#define PIC32MX_CHE_CONCLR_OFFSET 0x0004 /* Pre-fetch cache control clear register */
#define PIC32MX_CHE_CONSET_OFFSET 0x0008 /* Pre-fetch cache control set register */
#define PIC32MX_CHE_CONINV_OFFSET 0x000c /* Pre-fetch cache control invert register */
#define PIC32MX_CHE_ACC_OFFSET    0x0010 /* Pre-fetch cache access register */
#define PIC32MX_CHE_ACCCLR_OFFSET 0x0014 /* Pre-fetch cache access clear register */
#define PIC32MX_CHE_ACCSET_OFFSET 0x0018 /* Pre-fetch cache access set register */
#define PIC32MX_CHE_ACCINV_OFFSET 0x001c /* Pre-fetch cache access invert register */
#define PIC32MX_CHE_TAG_OFFSET    0x0020 /* Pre-fetch cache tag register */
#define PIC32MX_CHE_TAGCLR_OFFSET 0x0024 /* Pre-fetch cache tag clear register */
#define PIC32MX_CHE_TAGSET_OFFSET 0x0028 /* Pre-fetch cache tag set register */
#define PIC32MX_CHE_TAGINV_OFFSET 0x002c /* Pre-fetch cache tag invert register */
#define PIC32MX_CHE_MSK_OFFSET    0x0030 /* Pre-fetch cache tag mask register */
#define PIC32MX_CHE_MSKCLR_OFFSET 0x0034 /* Pre-fetch cache tag mask clear register */
#define PIC32MX_CHE_MSKSET_OFFSET 0x0038 /* Pre-fetch cache tag mask set register */
#define PIC32MX_CHE_MSKINV_OFFSET 0x003c /* Pre-fetch cache tag mask invert register */
#define PIC32MX_CHE_W0_OFFSET     0x0040 /* Cache word 0 register */
#define PIC32MX_CHE_W1_OFFSET     0x0050 /* Cache word 1 register */
#define PIC32MX_CHE_W2_OFFSET     0x0060 /* Cache word 2 register */
#define PIC32MX_CHE_W3_OFFSET     0x0070 /* Cache word 3 register */
#define PIC32MX_CHE_LRU_OFFSET    0x0080 /* Cache LRU register */
#define PIC32MX_CHE_HIT_OFFSET    0x0090 /* Cache hit statistics register */
#define PIC32MX_CHE_MIS_OFFSET    0x00a0 /* Cache miss statistics register */
#define PIC32MX_CHE_PFABT_OFFSET  0x00c0 /* Pre-fetch cache abort statistics register */

/* Register Addresses ***********************************************************************/

#define PIC32MX_CHE_CON           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_CON_OFFSET)
#define PIC32MX_CHE_CONCLR        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_CONCLR_OFFSET)
#define PIC32MX_CHE_CONSET        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_CONSET_OFFSET)
#define PIC32MX_CHE_CONINV        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_CONINV_OFFSET)
#define PIC32MX_CHE_ACC           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_ACC_OFFSET)
#define PIC32MX_CHE_ACCCLR        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_ACCCLR_OFFSET)
#define PIC32MX_CHE_ACCSET        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_ACCSET_OFFSET)
#define PIC32MX_CHE_ACCINV        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_ACCINV_OFFSET)
#define PIC32MX_CHE_TAG           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_TAG_OFFSET)
#define PIC32MX_CHE_TAGCLR        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_TAGCLR_OFFSET)
#define PIC32MX_CHE_TAGSET        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_TAGSET_OFFSET)
#define PIC32MX_CHE_TAGINV        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_TAGINV_OFFSET)
#define PIC32MX_CHE_MSK           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_MSK_OFFSET)
#define PIC32MX_CHE_MSKCLR        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_MSKCLR_OFFSET)
#define PIC32MX_CHE_MSKSET        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_MSKSET_OFFSET)
#define PIC32MX_CHE_MSKINV        (PIC32MX_CHE_K1BASE+PIC32MX_CHE_MSKINV_OFFSET)
#define PIC32MX_CHE_W0            (PIC32MX_CHE_K1BASE+PIC32MX_CHE_W0_OFFSET)
#define PIC32MX_CHE_W1            (PIC32MX_CHE_K1BASE+PIC32MX_CHE_W1_OFFSET)
#define PIC32MX_CHE_W2            (PIC32MX_CHE_K1BASE+PIC32MX_CHE_W2_OFFSET)
#define PIC32MX_CHE_W3            (PIC32MX_CHE_K1BASE+PIC32MX_CHE_W3_OFFSET)
#define PIC32MX_CHE_LRU           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_LRU_OFFSET)
#define PIC32MX_CHE_HIT           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_HIT_OFFSET)
#define PIC32MX_CHE_MIS           (PIC32MX_CHE_K1BASE+PIC32MX_CHE_MIS_OFFSET)
#define PIC32MX_CHE_PFABT         (PIC32MX_CHE_K1BASE+PIC32MX_CHE_PFABT_OFFSET)

/* Register Bit-Field Definitions ***********************************************************/

/* Pre-fetch cache control register */


#define CHE_CON_PFMWS_SHIFT       (0)       /* Bits 0-2: PFM access time (SYSCLK wait states) */
#define CHE_CON_PFMWS_MASK        (7 << CHE_CON_PFMWS_SHIFT)
#  define CHE_CON_PFMWS(n)        ((n) << CHE_CON_PFMWS_SHIFT) /* n wait states, n=0-7 */
#define CHE_CON_PREFEN_SHIFT      (4)       /* Bits 4-5: Predictive pre-fetch cache enable */
#define CHE_CON_PREFEN_MASK       (3 << CHE_CON_PREFEN_SHIFT)
#  define CHE_CON_PREFEN_DISABLE  (0 << CHE_CON_PREFEN_SHIFT) /* Disable predictive pre-fetch cache */
#  define CHE_CON_PREFEN_CACHE    (1 << CHE_CON_PREFEN_SHIFT) /* Enable for cacheable regions only */
#  define CHE_CON_PREFEN_NONCACHE (2 << CHE_CON_PREFEN_SHIFT) /* Enable for non-cacheable regions only */
#  define CHE_CON_PREFEN_ALL      (3 << CHE_CON_PREFEN_SHIFT) /* Enable for both regions */
#define CHE_CON_DCSZ_SHIFT        (8)       /* Bits 8-9: Data cache size (lines) */
#define CHE_CON_DCSZ_MASK         (3 << CHE_CON_DCSZ_SHIFT)
#  define CHE_CON_DCSZ_DISABLE    (0 << CHE_CON_DCSZ_SHIFT) /* Disable data caching */
#  define CHE_CON_DCSZ_1LINE      (1 << CHE_CON_DCSZ_SHIFT) /* Enable with size of 1 line */
#  define CHE_CON_DCSZ_2LINES     (2 << CHE_CON_DCSZ_SHIFT) /* Enable with size of 2 lines */
#  define CHE_CON_DCSZ_4LINES     (3 << CHE_CON_DCSZ_SHIFT) /* Enable with size of 4 lines */
#define CHE_CON_CHECOH            (1 << 16) /* Bit 16: Cache coherency setting */

/* Pre-fetch cache access register */

#define CHE_ACC_CHEIDX_SHIFT      (0)       /* Bits 0-3: Cache line index */
#define CHE_ACC_CHEIDX_MASK       (15 << CHE_ACC_CHEIDX_SHIFT)
#define CHE_ACC_CHEWEN            (1 << 31) /* Bit 31: Cache access enable */

/* Pre-fetch cache tag register */

#define CHE_TAG_LTYPE             (1 << 1)  /* Bit 1:  Line type */
#define CHE_TAG_LLOCK             (1 << 2)  /* Bit 2:  Line lock */
#define CHE_TAG_LVALID            (1 << 3)  /* Bit 3:  Line valid */
#define CHE_TAG_LTAG_SHIFT        (4)       /* Bits 4-23: Line tag address */
#define CHE_TAG_LTAG_MASK         (0x000fffff << CHE_TAG_LTAG_SHIFT)
#define CHE_TAG_LTAGBOOT          (1 << 31) /* Bit 31: Line tag address boot */

/* Pre-fetch cache tag mask register */

#define CHE_MSK_SHIFT             (5)       /* Bits 5-15: Line mask */
#define CHE_MSK_MASK              (0x7ff << CHE_MSK_SHIFT)

/* Cache word 0-3 register  -- 32-bit cache line data */

/* Cache LRU register */

#define CHE_LRU_MASK              0x01ffffff  /* Bits 0-24 */

/* Cache hit statistics register -- 32 bit counter value */

/* Cache miss statistics register -- 32 bit counter value */

/* Pre-fetch cache abort statistics register -- 32 bit counter value */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifndef __ASSEMBLY__

/********************************************************************************************
 * Inline Functions
 ********************************************************************************************/

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CHE_H */
