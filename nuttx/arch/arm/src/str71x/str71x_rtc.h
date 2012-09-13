/************************************************************************************
 * arch/arm/src/str71x/str71x_rtc.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_RTC_H
#define __ARCH_ARM_SRC_STR71X_STR71X_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* RTC Registers ********************************************************************/

#define STR71X_RTC_CRH     (STR71X_RTC_BASE + 0x0000) /* 16-bits wide */
#define STR71X_RTC_CRL     (STR71X_RTC_BASE + 0x0004) /* 16-bits wide */
#define STR71X_RTC_PRLH    (STR71X_RTC_BASE + 0x0008) /* 16-bits wide */
#define STR71X_RTC_PRLL    (STR71X_RTC_BASE + 0x000c) /* 16-bits wide */
#define STR71X_RTC_DIVH    (STR71X_RTC_BASE + 0x0010) /* 16-bits wide */
#define STR71X_RTC_DIVL    (STR71X_RTC_BASE + 0x0014) /* 16-bits wide */
#define STR71X_RTC_CNTH    (STR71X_RTC_BASE + 0x0018) /* 16-bits wide */
#define STR71X_RTC_CNTL    (STR71X_RTC_BASE + 0x001c) /* 16-bits wide */
#define STR71X_RTC_ALRH    (STR71X_RTC_BASE + 0x0020) /* 16-bits wide */
#define STR71X_RTC_ALRL    (STR71X_RTC_BASE + 0x0024) /* 16-bits wide */

/* Register bit settings ***********************************************************/

/* RTC control register */

#define STR71X_RTCCRH_SEN    (0x0001) /* Bit 0: Second interrupt enable */
#define STR71X_RTCCRH_AEN    (0x0002) /* Bit 1: Alarm interrupt enable */
#define STR71X_RTCCRH_OWEN   (0x0004) /* Bit 2: Overflow interrupt enable */
#define STR71X_RTCCRH_GEN    (0x0008) /* Bit 3: Global interrupt enable */

#define STR71X_RTCCRL_SIR    (0x0001) /* Bit 0: Second interrupt request */
#define STR71X_RTCCRL_AIR    (0x0002) /* Bit 1: Alarm interrupt request */
#define STR71X_RTCCRL_OWIR   (0x0004) /* Bit 2: Overflow interrupt request */
#define STR71X_RTCCRL_GIR    (0x0008) /* Bit 3: Global interrupt request */
#define STR71X_RTCCRL_CNF    (0x0010) /* Bit 4: Enter configuration mode */
#define STR71X_RTCCRL_RTOFF  (0x0020) /* Bit 5: RTC Operation Off */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_RTC_H */
