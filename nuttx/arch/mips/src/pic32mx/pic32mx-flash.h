 /********************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-flash.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_FLASH_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_FLASH_H

 /********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

 /********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/
/* Register Offsets *************************************************************************/

#define PIC32MX_FLASH_NVMCON_OFFSET     0x0000 /* Programming Control Register */
#define PIC32MX_FLASH_NVMCONCLR_OFFSET  0x0004 /* Programming Control Clear Register */
#define PIC32MX_FLASH_NVMCONSET_OFFSET  0x0008 /* Programming Control Set Register */
#define PIC32MX_FLASH_NVMCONINV_OFFSET  0x000c /* Programming Control Invert Register */
#define PIC32MX_FLASH_NVMKEY_OFFSET     0x0010 /* Programming Unlock Register */
#define PIC32MX_FLASH_NVMADDR_OFFSET    0x0020 /* Flash Address Register */
#define PIC32MX_FLASH_NVMADDRCLR_OFFSET 0x0024 /* Flash Address Clear Register */
#define PIC32MX_FLASH_NVMADDRSET_OFFSET 0x0028 /* Flash Address Set Register */
#define PIC32MX_FLASH_NVMADDRINV_OFFSET 0x002c /* Flash Address Invert Register */
#define PIC32MX_FLASH_NVMDATA_OFFSET    0x0030 /* Flash Program Data Register */
#define PIC32MX_FLASH_NVMSRCADDR_OFFSET 0x0040 /* Source Data Address Register */

/* Register Addresses ***********************************************************************/

#define PIC32MX_FLASH_NVMCON            (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMCON_OFFSET)
#define PIC32MX_FLASH_NVMCONCLR         (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMCONCLR_OFFSET)
#define PIC32MX_FLASH_NVMCONSET         (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMCONSET_OFFSET)
#define PIC32MX_FLASH_NVMCONINV         (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMCONINV_OFFSET)
#define PIC32MX_FLASH_NVMKEY            (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMKEY_OFFSET)
#define PIC32MX_FLASH_NVMADDRCLR        (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMADDRCLR_OFFSET)
#define PIC32MX_FLASH_NVMADDRSET        (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMADDRSET_OFFSET)
#define PIC32MX_FLASH_NVMADDRINV        (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMADDRINV_OFFSET)
#define PIC32MX_FLASH_NVMADDR           (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMADDR_OFFSET)
#define PIC32MX_FLASH_NVMDATA           (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMDATA_OFFSET)
#define PIC32MX_FLASH_NVMSRCADDR        (PIC32MX_FLASH_K1BASE+PIC32MX_FLASH_NVMSRCADDR_OFFSET)

/* Register Bit-Field Definitions ***********************************************************/

/* Programming Control Register */

#define FLASH_NVMCON_NVMOP_SHIFT        (0)       /* Bits 0-3: NVM operation */
#define FLASH_NVMCON_NVMOP_MASK         (15 << FLASH_NVMCON_NVMOP_SHIFT)
#  define FLASH_NVMCON_NVMOP_NOP        (0 << FLASH_NVMCON_NVMOP_SHIFT) /* No operation */
#  define FLASH_NVMCON_NVMOP_WDPROG     (1 << FLASH_NVMCON_NVMOP_SHIFT) /* Word program operation */
#  define FLASH_NVMCON_NVMOP_ROWPROG    (3 << FLASH_NVMCON_NVMOP_SHIFT) /* Row program operation */
#  define FLASH_NVMCON_NVMOP_PFMERASE   (4 << FLASH_NVMCON_NVMOP_SHIFT) /* Page erase operation */
#  define FLASH_NVMCON_NVMOP_PFMERASE   (5 << FLASH_NVMCON_NVMOP_SHIFT) /* PFM erase operationxx */
#define FLASH_NVMCON_LVDSTAT            (1 << 11) /* Bit nn: Low-voltage detect status */
#define FLASH_NVMCON_LVDERR             (1 << 12) /* Bit nn: Low-voltage detect error */
#define FLASH_NVMCON_WRERR              (1 << 13) /* Bit nn: Write error */
#define FLASH_NVMCON_WREN               (1 << 14) /* Bit nn: Write enable */
#define FLASH_NVMCON_WR                 (1 << 15) /* Bit nn: Write control */

/* Programming Unlock Register -- 32 Bits of data */

/* Flash Address Register -- 32 Bits of data */

/* Flash Program Data Register -- 32 Bits of data */

/* Source Data Address Register -- 32 Bits of data */

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_FLASH_H */
