/************************************************************************************
 * arch/arm/src/imx/imx_rtc.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_IMX_RTC_H
#define __ARCH_ARM_IMX_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* RTC Register Offsets *************************************************************/

#define RTC_HOURMIN_OFFSET           0x0000
#define RTC_SECOND_OFFSET            0x0004
#define RTC_ALRM_HM_OFFSET           0x0008
#define RTC_ALRM_SEC_OFFSET          0x000c
#define RTC_RTCCTL_OFFSET            0x0010
#define RTC_RTCISR_OFFSET            0x0014
#define RTC_RTCIENR_OFFSET           0x0018
#define RTC_STPWCH_OFFSET            0x001c
#define RTC_DAYR_OFFSET              0x0020
#define RTC_DAYALARM_OFFSET          0x0024
#define RTC_TEST1_OFFSET             0x0028
#define RTC_TEST2_OFFSET             0x002c
#define RTC_TEST3_OFFSET             0x0030

/* RTC Register Addresses ***********************************************************/

#define IMX_RTC_HOURMIN             (IMX_RTC_VBASE + RTC_HOURMIN_OFFSET)
#define IMX_RTC_SECOND              (IMX_RTC_VBASE + RTC_SECOND_OFFSET)
#define IMX_RTC_ALRM_HM             (IMX_RTC_VBASE + RTC_ALRM_HM_OFFSET)
#define IMX_RTC_ALRM_SEC            (IMX_RTC_VBASE + RTC_ALRM_SEC_OFFSET)
#define IMX_RTC_RTCCTL              (IMX_RTC_VBASE + RTC_RTCCTL_OFFSET)
#define IMX_RTC_RTCISR              (IMX_RTC_VBASE + RTC_RTCISR_OFFSET)
#define IMX_RTC_RTCIENR             (IMX_RTC_VBASE + RTC_RTCIENR_OFFSET)
#define IMX_RTC_STPWCH              (IMX_RTC_VBASE + RTC_STPWCH_OFFSET)
#define IMX_RTC_DAYR                (IMX_RTC_VBASE + RTC_DAYR_OFFSET)
#define IMX_RTC_DAYALARM            (IMX_RTC_VBASE + RTC_DAYALARM_OFFSET)
#define IMX_RTC_TEST1               (IMX_RTC_VBASE + RTC_TEST1_OFFSET)
#define IMX_RTC_TEST2               (IMX_RTC_VBASE + RTC_TEST2_OFFSET)
#define IMX_RTC_TEST3               (IMX_RTC_VBASE + RTC_TEST3_OFFSET)

/* RTC Register Bit Definitions *****************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_RTC_H */
