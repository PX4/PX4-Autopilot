/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_rstc.h
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_RSTC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_RSTC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* RSTC register offsets ****************************************************************/

#define SAM3U_RSTC_CR_OFFSET    0x00 /* Control Register */
#define SAM3U_RSTC_SR_OFFSET    0x04 /* Status Register */
#define SAM3U_RSTC_MR_OFFSET    0x08 /* Mode Register  */

/* RSTC register adresses ***************************************************************/

#define SAM3U_RSTC_CR           (SAM3U_RSTC_BASE+SAM3U_RSTC_CR_OFFSET)
#define SAM3U_RSTC_SR           (SAM3U_RSTC_BASE+SAM3U_RSTC_SR_OFFSET)
#define SAM3U_RSTC_MR           (SAM3U_RSTC_BASE+SAM3U_RSTC_MR_OFFSET)

/* RSTC register bit definitions ********************************************************/

#define RSTC_CR_PROCRST         (1 << 0)  /* Bit 0:  Processor Reset */
#define RSTC_CR_PERRST          (1 << 2)  /* Bit 2:  Peripheral Reset */
#define RSTC_CR_EXTRST          (1 << 3)  /* Bit 3:  External Reset */
#define RSTC_CR_KEY_SHIFT       (24)      /* Bits 24-31:  Password */
#define RSTC_CR_KEY_MASK        (0xff << RSTC_CR_KEY_SHIFT)

#define RSTC_SR_URSTS           (1 << 0)  /* Bit 0:  User Reset Status */
#define RSTC_SR_RSTTYP_SHIFT    (8)       /* Bits 8-10:  Reset Type */
#define RSTC_SR_RSTTYP_MASK     (7 << RSTC_SR_RSTTYP_SHIFT)
#  define RSTC_SR_RSTTYP_PWRUP  (0 << RSTC_SR_RSTTYP_SHIFT) /* General Reset */
#  define RSTC_SR_RSTTYP_BACKUP (1 << RSTC_SR_RSTTYP_SHIFT) /* Backup Reset */
#  define RSTC_SR_RSTTYP_WDOG   (2 << RSTC_SR_RSTTYP_SHIFT) /* Watchdog Reset */
#  define RSTC_SR_RSTTYP_SWRST  (3 << RSTC_SR_RSTTYP_SHIFT) /* Software Reset */
#  define RSTC_SR_RSTTYP_NRST   (4 << RSTC_SR_RSTTYP_SHIFT) /* User Reset NRST pin */
#define RSTC_SR_NRSTL           (1 << 16) /* Bit 16:  NRST Pin Level */
#define RSTC_SR_SRCMP           (1 << 17) /* Bit 17:  Software Reset Command in Progress */

#define RSTC_MR_URSTEN          (1 << 0)  /* Bit 0:  User Reset Enable */
#define RSTC_MR_URSTIEN         (1 << 4)  /* Bit 4:  User Reset Interrupt Enable */
#define RSTC_MR_ERSTL_SHIFT     (8)       /* Bits 8-11:  External Reset Length */
#define RSTC_MR_ERSTL_MASK      (15 << RSTC_MR_ERSTL_SHIFT)
#define RSTC_MR_KEY_SHIFT       (24)      /* Bits 24-31:  Password */
#define RSTC_MR_KEY_MASK        (0xff << RSTC_CR_KEY_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_RSTC_H */
