/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_eefc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_EEFC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_EEFC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* EEFC register offsets ****************************************************************/

#define SAM3U_EEFC_FMR_OFFSET          0x00 /* EEFC Flash Mode Register */
#define SAM3U_EEFC_FCR_OFFSET          0x04 /* EEFC Flash Command Register */
#define SAM3U_EEFC_FSR_OFFSET          0x08 /* EEFC Flash Status Register */
#define SAM3U_EEFC_FRR_OFFSET          0x0c /* EEFC Flash Result Register */

/* EEFC register adresses ***************************************************************/

#define SAM3U_EEFC_FMR(n)              (SAM3U_EEFCN_BASE(n)+SAM3U_EEFC_FMR_OFFSET)
#define SAM3U_EEFC_FCR(n)              (SAM3U_EEFCN_BASE(n)+SAM3U_EEFC_FCR_OFFSET)
#define SAM3U_EEFC_FSR(n)              (SAM3U_EEFCN_BASE(n)+SAM3U_EEFC_FSR_OFFSET)
#define SAM3U_EEFC_FRR(n)              (SAM3U_EEFCN_BASE(n)+SAM3U_EEFC_FRR_OFFSET)

#define SAM3U_EEFC0_FMR                (SAM3U_EEFC0_BASE+SAM3U_EEFC_FMR_OFFSET)
#define SAM3U_EEFC0_FCR                (SAM3U_EEFC0_BASE+SAM3U_EEFC_FCR_OFFSET)
#define SAM3U_EEFC0_FSR                (SAM3U_EEFC0_BASE+SAM3U_EEFC_FSR_OFFSET)
#define SAM3U_EEFC0_FRR                (SAM3U_EEFC0_BASE+SAM3U_EEFC_FRR_OFFSET)

#define SAM3U_EEFC1_FMR                (SAM3U_EEFC1_BASE+SAM3U_EEFC_FMR_OFFSET)
#define SAM3U_EEFC1_FCR                (SAM3U_EEFC1_BASE+SAM3U_EEFC_FCR_OFFSET)
#define SAM3U_EEFC1_FSR                (SAM3U_EEFC1_BASE+SAM3U_EEFC_FSR_OFFSET)
#define SAM3U_EEFC1_FRR                (SAM3U_EEFC1_BASE+SAM3U_EEFC_FRR_OFFSET)

/* EEFC register bit definitions ********************************************************/

#define EEFC_FMR_FRDY                  (1 << 0)  /* Bit 0:  Ready Interrupt Enable */
#define EEFC_FMR_FWS_SHIFT             (8)       /* Bits 8-11:  Flash Wait State */
#define EEFC_FMR_FWS_MASK              (15 << EEFC_FMR_FWS_SHIFT)
#define EEFC_FMR_FAM                   (1 << 24) /* Bit 24: Flash Access Mode */

#define EEFC_FCR_FCMD_SHIFT            (0)       /* Bits 0-7:  Flash Command */
#define EEFC_FCR_FCMD_MASK             (0xff << EEFC_FCR_FCMD_SHIFT)
#  define EEFC_FCR_FCMD_GETD           (0  << EEFC_FCR_FCMD_SHIFT) /* Get Flash Descriptor */
#  define EEFC_FCR_FCMD_WP             (1  << EEFC_FCR_FCMD_SHIFT) /* Write page */
#  define EEFC_FCR_FCMD_WPL            (2  << EEFC_FCR_FCMD_SHIFT) /* Write page and lock */
#  define EEFC_FCR_FCMD_EWP            (3  << EEFC_FCR_FCMD_SHIFT) /* Erase page and write page */
#  define EEFC_FCR_FCMD_EWPL           (4  << EEFC_FCR_FCMD_SHIFT) /* Erase page and write page then lock */
#  define EEFC_FCR_FCMD_EA             (5  << EEFC_FCR_FCMD_SHIFT) /* Erase all */
#  define EEFC_FCR_FCMD_SLB            (8  << EEFC_FCR_FCMD_SHIFT) /* Set Lock Bit */
#  define EEFC_FCR_FCMD_CLB            (9  << EEFC_FCR_FCMD_SHIFT) /* Clear Lock Bit */
#  define EEFC_FCR_FCMD_GLB            (10 << EEFC_FCR_FCMD_SHIFT) /* Get Lock Bit */
#  define EEFC_FCR_FCMD_SGPB           (11 << EEFC_FCR_FCMD_SHIFT) /* Set GPNVM Bit */
#  define EEFC_FCR_FCMD_CGPB           (12 << EEFC_FCR_FCMD_SHIFT) /* Clear GPNVM Bit */
#  define EEFC_FCR_FCMD_GGPB           (13 << EEFC_FCR_FCMD_SHIFT) /* Get GPNVM Bit */
#  define EEFC_FCR_FCMD_STUI           (14 << EEFC_FCR_FCMD_SHIFT) /* Start Read Unique Identifier */
#  define EEFC_FCR_FCMD_SPUI           (15 << EEFC_FCR_FCMD_SHIFT) /* Stop Read Unique Identifier */
#define EEFC_FCR_FARG_SHIFT            (8)       /* Bits 8-23:  Flash Command Argument */
#define EEFC_FCR_FARG_MASK             (0xffff << EEFC_FCR_FARG_SHIFT)
#define EEFC_FCR_FKEY_SHIFT            (24)      /* Bits 24-31:  Flash Writing Protection Key */
#define EEFC_FCR_FKEY__MASK            (0xff << EEFC_FCR_FKEY_SHIFT)

#define EEFC_FSR_FRDY                  (1 << 0)  /* Bit 0:  Flash Ready Status */
#define EEFC_FSR_FCMDE                 (1 << 1)  /* Bit 1:  Flash Command Error Status */
#define EEFC_FSR_FLOCKE                (1 << 2)  /* Bit 2:  Flash Lock Error Status */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_EEFC_H */
