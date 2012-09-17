/************************************************************************************
 * arch/arm/src/kinetis/kinetis_mmcau.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_MMCAU_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_MMCAU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(KINETIS_NMMCAU) && KINETIS_NMMCAU > 0

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define KINETIS_CAU_CASR_OFFSET  0x0000  /* Status Register */
#define KINETIS_CAU_CAA_OFFSET   0x0001  /* Accumulator */

#define KINETIS_CAU_CA_OFFSET(n) ((n)+2) /* General Purpose Register n */
#define KINETIS_CAU_CA0_OFFSET   0x0002  /* General Purpose Register 0 */
#define KINETIS_CAU_CA1_OFFSET   0x0003  /* General Purpose Register 1 */
#define KINETIS_CAU_CA2_OFFSET   0x0004  /* General Purpose Register 2 */
#define KINETIS_CAU_CA3_OFFSET   0x0005  /* General Purpose Register 3 */
#define KINETIS_CAU_CA4_OFFSET   0x0006  /* General Purpose Register 4 */
#define KINETIS_CAU_CA5_OFFSET   0x0007  /* General Purpose Register 5 */
#define KINETIS_CAU_CA6_OFFSET   0x0008  /* General Purpose Register 6 */
#define KINETIS_CAU_CA7_OFFSET   0x0009  /* General Purpose Register 7 */
#define KINETIS_CAU_CA8_OFFSET   0x000a  /* General Purpose Register 8 */

/* Register Addresses ***************************************************************/

#define KINETIS_CAU_CASR         (KINETIS_MMCAU_BASE+KINETIS_CAU_CASR_OFFSET)
#define KINETIS_CAU_CAA          (KINETIS_MMCAU_BASE+KINETIS_CAU_CAA_OFFSET)

#define KINETIS_CAU_CA(n)        (KINETIS_MMCAU_BASE+KINETIS_CAU_CA_OFFSET(n))
#define KINETIS_CAU_CA0          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA0_OFFSET)
#define KINETIS_CAU_CA1          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA1_OFFSET)
#define KINETIS_CAU_CA2          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA2_OFFSET)
#define KINETIS_CAU_CA3          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA3_OFFSET)
#define KINETIS_CAU_CA4          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA4_OFFSET)
#define KINETIS_CAU_CA5          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA5_OFFSET)
#define KINETIS_CAU_CA6          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA6_OFFSET)
#define KINETIS_CAU_CA7          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA7_OFFSET)
#define KINETIS_CAU_CA8          (KINETIS_MMCAU_BASE+KINETIS_CAU_CA8_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Status Register */

#define CAU_CASR_IC              (1 << 0)  /* Bit 0:  Illegal command */
#define CAU_CASR_DPE             (1 << 1)  /* Bit 1:  DES parity error */
                                           /* Bits 2-27: Reserved */
#define CAU_CASR_VER_SHIFT       (28)      /* Bits 28-31: CAU version */
#define CAU_CASR_VER_MASK        (15 << CAU_CASR_VER_SHIFT)

/* Accumulator (32-bit accumulated value)*/
/* General Purpose Register n (32-bit value used by CAU commands) */

/* CAU Commands *********************************************************************/

/* Bits 4-8 of 9-bit commands (bits 0-3 may be arguments of the command) */

#define CAU_CMD_CNOP  0x000 /* No Operation */
#define CAU_CMD_LDR   0x010 /* Load Reg */
#define CAU_CMD_STR   0x020 /* Store Reg */
#define CAU_CMD_ADR   0x030 /* Add */
#define CAU_CMD_RADR  0x040 /* Reverse and Add */
#define CAU_CMD_ADRA  0x050 /* Add Reg to Acc */
#define CAU_CMD_XOR   0x060 /* Exclusive Or */
#define CAU_CMD_ROTL  0x070 /* Rotate Left */
#define CAU_CMD_MVRA  0x080 /* Move Reg to Acc */
#define CAU_CMD_MVAR  0x090 /* Move Acc to Reg */
#define CAU_CMD_AESS  0x0a0 /* AES Sub Bytes */
#define CAU_CMD_AESIS 0x0b0 /* AES Inv Sub Bytes */
#define CAU_CMD_AESC  0x0c0 /* AES Column Op */
#define CAU_CMD_AESIC 0x0d0 /* AES Inv Column Op */
#define CAU_CMD_AESR  0x0e0 /* AES Shift Rows */
#define CAU_CMD_AESIR 0x0f0 /* AES Inv Shift Rows */
#define CAU_CMD_DESR  0x100 /* DES Round */
#define CAU_CMD_DESK  0x110 /* DES Key Setup */
#define CAU_CMD_HASH  0x120 /* Hash Function */
#define CAU_CMD_SHS   0x130 /* Secure Hash Shift */
#define CAU_CMD_MDS   0x140 /* Message Digest Shift */
#define CAU_CMD_SHS2  0x150 /* Secure Hash Shift 2 */
#define CAU_CMD_ILL   0x1f0 /* Illegal Command */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* KINETIS_NMMCAU && KINETIS_NMMCAU > 0 */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_MMCAU_H */
