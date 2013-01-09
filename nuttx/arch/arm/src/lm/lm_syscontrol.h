/************************************************************************************
 * arch/arm/src/lm/lm_syscontrol.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_LM_SYSCONTROL_H
#define __ARCH_ARM_SRC_LM_LM_SYSCONTROL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lm_clockconfig
 *
 * Description:
 *   Called to change to new clock based on desired rcc and rcc2 settings.
 *   This is use to set up the initial clocking but can be used later to
 *   support slow clocked, low power consumption modes.
 *
 ****************************************************************************/

void lm_clockconfig(uint32_t newrcc, uint32_t newrcc2);

/****************************************************************************
 * Name: up_clockconfig
 *
 * Description:
 *   Called early in the bootsequence (before .data and .bss are available)
 *   in order to configure initial clocking.
 *
 ****************************************************************************/

void up_clockconfig(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LM_LM_SYSCONTROL_H */
