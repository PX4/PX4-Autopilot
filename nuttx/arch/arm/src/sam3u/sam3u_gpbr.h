/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_gpbr.h
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_GPBR_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_GPBR_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* GPBR register offsets ****************************************************************/

#define SAM3U_GPBR_OFFSET(n)  ((n)<<2) /* General purpose back-up registers */
#define SAM3U_GPBR0_OFFSET    0x00
#define SAM3U_GPBR1_OFFSET    0x04
#define SAM3U_GPBR2_OFFSET    0x08
#define SAM3U_GPBR3_OFFSET    0x0c
#define SAM3U_GPBR4_OFFSET    0x10
#define SAM3U_GPBR5_OFFSET    0x14
#define SAM3U_GPBR6_OFFSET    0x18
#define SAM3U_GPBR7_OFFSET    0x1c

/* GPBR register adresses ***************************************************************/

#define SAM3U_GPBR(n))        (SAM3U_GPBR_BASE+SAM3U_GPBR_OFFSET(n))
#define SAM3U_GPBR0           (SAM3U_GPBR_BASE+SAM3U_GPBR0_OFFSET)
#define SAM3U_GPBR1           (SAM3U_GPBR_BASE+SAM3U_GPBR1_OFFSET)
#define SAM3U_GPBR2           (SAM3U_GPBR_BASE+SAM3U_GPBR2_OFFSET)
#define SAM3U_GPBR3           (SAM3U_GPBR_BASE+SAM3U_GPBR3_OFFSET)
#define SAM3U_GPBR4           (SAM3U_GPBR_BASE+SAM3U_GPBR4_OFFSET)
#define SAM3U_GPBR5           (SAM3U_GPBR_BASE+SAM3U_GPBR5_OFFSET)
#define SAM3U_GPBR6           (SAM3U_GPBR_BASE+SAM3U_GPBR6_OFFSET)
#define SAM3U_GPBR7           (SAM3U_GPBR_BASE+SAM3U_GPBR7_OFFSET)

/* GPBR register bit definitions ********************************************************/

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_GPBR_H */
