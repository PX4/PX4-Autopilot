/************************************************************************************
 * arch/arm/src/str71x/str71x_xti.h
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_XTI_H
#define __ARCH_ARM_SRC_STR71X_STR71X_XTI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* External Interupt Controller (XTI) registers *************************************/

#define STR71X_XTI_SR               (STR71X_XTI_BASE + 0x001c)  /* 8-bits wide */
#define STR71X_XTI_CTRL             (STR71X_XTI_BASE + 0x0024)  /* 8-bits wide */
#define STR71X_XTI_MRH              (STR71X_XTI_BASE + 0x0028)  /* 8-bits wide */
#define STR71X_XTI_MRL              (STR71X_XTI_BASE + 0x002c)  /* 8-bits wide */
#define STR71X_XTI_TRH              (STR71X_XTI_BASE + 0x0030)  /* 8-bits wide */
#define STR71X_XTI_TRL              (STR71X_XTI_BASE + 0x0034)  /* 8-bits wide */
#define STR71X_XTI_PRH              (STR71X_XTI_BASE + 0x0038)  /* 8-bits wide */
#define STR71X_XTI_PRL              (STR71X_XTI_BASE + 0x003c)  /* 8-bits wide */

/* Register bit settings ************************************************************/

/* Control register (CTRL) */

#define STR71X_XTICTRL_WKUPINT      (0x01)
#define STR71X_XTICTRL_ID1S         (0x02)
#define STR71X_XTICTRL_STOP         (0x04)

/* Most registers are address by external interrupt line in two 8-bit high and low
 * registers
 */

#define STR71X_XTI_LINE(n)          (1 << (n))
#define STR71X_XTI_LINE0            STR71X_XTI_LINE(0) /* Low register */
#define STR71X_XTI_LINE1            STR71X_XTI_LINE(1)
#define STR71X_XTI_LINE2            STR71X_XTI_LINE(2)
#define STR71X_XTI_LINE3            STR71X_XTI_LINE(3)
#define STR71X_XTI_LINE4            STR71X_XTI_LINE(4)
#define STR71X_XTI_LINE5            STR71X_XTI_LINE(5)
#define STR71X_XTI_LINE6            STR71X_XTI_LINE(6)
#define STR71X_XTI_LINE7            STR71X_XTI_LINE(7)

#define STR71X_XTI_LINE8            STR71X_XTI_LINE(8) /* High register */
#define STR71X_XTI_LINE9            STR71X_XTI_LINE(9)
#define STR71X_XTI_LINE10           STR71X_XTI_LINE(10)
#define STR71X_XTI_LINE11           STR71X_XTI_LINE(11)
#define STR71X_XTI_LINE12           STR71X_XTI_LINE(12)
#define STR71X_XTI_LINE13           STR71X_XTI_LINE(13)
#define STR71X_XTI_LINE14           STR71X_XTI_LINE(14)
#define STR71X_XTI_LINE15           STR71X_XTI_LINE(15)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* _ARCH_ARM_SRC_STR71X_STR71X_XTI_H */
