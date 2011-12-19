/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_rtt.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_RTT_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_RTT_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* RTT register offsets *****************************************************************/

#define SAM3U_RTT_MR_OFFSET    0x00 /* Mode Register */
#define SAM3U_RTT_AR_OFFSET    0x04 /* Alarm Register */
#define SAM3U_RTT_VR_OFFSET    0x08 /* Value Register */
#define SAM3U_RTT_SR_OFFSET    0x0c /* Status Register */

/* RTT register adresses ***************************************************************/

#define SAM3U_RTT_MR           (SAM3U_RTT_BASE+SAM3U_RTT_MR_OFFSET)
#define SAM3U_RTT_AR           (SAM3U_RTT_BASE+SAM3U_RTT_AR_OFFSET)
#define SAM3U_RTT_VR           (SAM3U_RTT_BASE+SAM3U_RTT_VR_OFFSET)
#define SAM3U_RTT_SR           (SAM3U_RTT_BASE+SAM3U_RTT_SR_OFFSET)

/* RTT register bit definitions ********************************************************/

#define RTT_MR_RTPRES_SHIFT    (0)       /* Bits 0-15:  Real-time Timer Prescaler Value */
#define RTT_MR_RTPRES__MASK    (0xffff << RTT_MR_RTPRES_SHIFT)
#define RTT_MR_ALMIEN          (1 << 16) /* Bit 16: Alarm Interrupt Enable */
#define RTT_MR_RTTINCIEN       (1 << 17) /* Bit 17: Real-time Timer Increment Int Enable */
#define RTT_MR_RTTRST          (1 << 18) /* Bit 18: Real-time Timer Restart */

#define RTT_SR_ALMS            (1 << 0)  /* Bit 0:  Real-time Alarm Status */
#define RTT_SR_RTTINC          (1 << 1)  /* Bit 1:  Real-time Timer Increment */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_RTT_H */
