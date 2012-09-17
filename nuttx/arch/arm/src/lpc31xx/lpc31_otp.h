/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_otp.h
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_OTP_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_OTP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Virtual and physical base address of the OTG register group **********************************/

#define LPC31_OTG_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_OTP_OFFSET)
#define LPC31_OTG_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_OTP_OFFSET)

/* OTP register offsets (with respect to the RNG base) ******************************************/

#define LPC31_OTP_CON_OFFSET     0x000 /* Control Register */
#define LPC31_OTP_RPROT_OFFSET   0x004 /* Read-protect Register */
#define LPC31_OTP_WPROT_OFFSET   0x008 /* Write-protect Register */
#define LPC31_OTP_DATA_OFFSET(n) (0x00c + ((n) << 2)
#define LPC31_OTP_DATA0_OFFSET   0x00c /* Fuse-output data register 0 */
#define LPC31_OTP_DATA1_OFFSET   0x010 /* Fuse-output data register 1 */
#define LPC31_OTP_DATA2_OFFSET   0x014 /* Fuse-output data register 2 */
#define LPC31_OTP_DATA3_OFFSET   0x018 /* Fuse-output data register 3 */
#define LPC31_OTP_DATA4_OFFSET   0x01c /* Fuse-output data register 4 */
#define LPC31_OTP_DATA5_OFFSET   0x020 /* Fuse-output data register 5 */
#define LPC31_OTP_DATA6_OFFSET   0x024 /* Fuse-output data register 6 */
#define LPC31_OTP_DATA7_OFFSET   0x028 /* Fuse-output data register 7 */
#define LPC31_OTP_DATA8_OFFSET   0x02c /* Fuse-output data register 8 */
#define LPC31_OTP_DATA9_OFFSET   0x030 /* Fuse-output data register 9 */
#define LPC31_OTP_DATA10_OFFSET  0x034 /* Fuse-output data register 10 */
#define LPC31_OTP_DATA11_OFFSET  0x038 /* Fuse-output data register 11 */
#define LPC31_OTP_DATA12_OFFSET  0x03c /* Fuse-output data register 12 */
#define LPC31_OTP_DATA13_OFFSET  0x040 /* Fuse-output data register 13 */
#define LPC31_OTP_DATA14_OFFSET  0x044 /* Fuse-output data register 14 */
#define LPC31_OTP_DATA15_OFFSET  0x048 /* Fuse-output data register 15 */

/* OTP register (virtual) addresses *************************************************************/

#define LPC31_OTP_CON            (LPC31_OTG_VBASE+LPC31_OTP_CON_OFFSET)
#define LPC31_OTP_RPROT          (LPC31_OTG_VBASE+LPC31_OTP_RPROT_OFFSET)
#define LPC31_OTP_WPROT          (LPC31_OTG_VBASE+LPC31_OTP_WPROT_OFFSET)
#define LPC31_OTP_DATA(n)        (LPC31_OTG_VBASE+LPC31_OTP_DATA_OFFSET(n))
#define LPC31_OTP_DATA0          (LPC31_OTG_VBASE+LPC31_OTP_DATA0_OFFSET)
#define LPC31_OTP_DATA1          (LPC31_OTG_VBASE+LPC31_OTP_DATA1_OFFSET)
#define LPC31_OTP_DATA2          (LPC31_OTG_VBASE+LPC31_OTP_DATA2_OFFSET)
#define LPC31_OTP_DATA3          (LPC31_OTG_VBASE+LPC31_OTP_DATA3_OFFSET)
#define LPC31_OTP_DATA4          (LPC31_OTG_VBASE+LPC31_OTP_DATA4_OFFSET)
#define LPC31_OTP_DATA5          (LPC31_OTG_VBASE+LPC31_OTP_DATA5_OFFSET)
#define LPC31_OTP_DATA6          (LPC31_OTG_VBASE+LPC31_OTP_DATA6_OFFSET)
#define LPC31_OTP_DATA7          (LPC31_OTG_VBASE+LPC31_OTP_DATA7_OFFSET)
#define LPC31_OTP_DATA8          (LPC31_OTG_VBASE+LPC31_OTP_DATA8_OFFSET)
#define LPC31_OTP_DATA9          (LPC31_OTG_VBASE+LPC31_OTP_DATA9_OFFSET)
#define LPC31_OTP_DATA10         (LPC31_OTG_VBASE+LPC31_OTP_DATA10_OFFSET)
#define LPC31_OTP_DATA11         (LPC31_OTG_VBASE+LPC31_OTP_DATA11_OFFSET)
#define LPC31_OTP_DATA12         (LPC31_OTG_VBASE+LPC31_OTP_DATA12_OFFSET)
#define LPC31_OTP_DATA13         (LPC31_OTG_VBASE+LPC31_OTP_DATA13_OFFSET)
#define LPC31_OTP_DATA14         (LPC31_OTG_VBASE+LPC31_OTP_DATA14_OFFSET)
#define LPC31_OTP_DATA15         (LPC31_OTG_VBASE+LPC31_OTP_DATA15_OFFSET)

/* RNG register bit definitions *****************************************************************/

/* Control Register */

#define OTP_CON_ADRS_SHIFT       (0)       /* Bits 0-8: Address bits for accessing fuse data */
#define OTP_CON_ADRS_MASK        (0x1ff << OTP_CON_ADRS_SHIFT)
#define OTP_CON_MODE_SHIFT       (16)      /* Bits 16-17: Selects: Idle, Copy and Write mode */
#define OTP_CON_MODE_MASK        (3 << OTP_CON_MODE_SHIFT)
#define OTP_CON_JTAGEN           (1 << 31) /* Bit 31: Enable ARM_JTAG clock */

/* Read-protect Register */

#define OTP_RPROT_PROT_SHIFT     (0)       /* Bits 0-15: Indicates which data registers are read-protected */
#define OTP_RPROT_PROT_MASK      (0xffff << OTP_RPROT_PROT_SHIFT)
#  define OTP_RPROT_PROT(n)      ((1 << (n)) << TP_RPROT_PROT_SHIFT)
#define OTP_RPROT_LOCK           (1 << 31) /* Bit 31: Register values are 'sticky' */

/* Write-protect Register */

#define OTP_WPROT_PROT_SHIFT     (0)       /* Bits 0-15: Indicates which data registers are write-protected */
#define OTP_WPROT_PROT_MASK      (0xffff << OTP_RPROT_PROT_SHIFT)
#  define OTP_WPROT_PROT(n)      ((1 << (n)) << TP_RPROT_PROT_SHIFT)
#define OTP_WPROT_LOCK           (1 << 31) /* Bit 31: Register values are 'sticky' */

/* Fuse-output data register 0-15: Fuse output 0-511 (32 per data register, 32*16 = 512) */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_OTP_H */
