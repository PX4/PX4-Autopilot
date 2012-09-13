/************************************************************************************
 * arch/arm/src/kinetis/kinetis_ewm.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_EWM_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_EWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_EWM_CTRL_OFFSET    0x0000 /* Control Register */
#define KINETIS_EWM_SERV_OFFSET    0x0001 /* Service Register */
#define KINETIS_EWM_CMPL_OFFSET    0x0002 /* Compare Low Register */
#define KINETIS_EWM_CMPH_OFFSET    0x0003 /* Compare High Register */

/* Register Addresses ***************************************************************/

#define KINETIS_EWM_CTRL           (KINETIS_EWM_BASE+KINETIS_EWM_CTRL_OFFSET)
#define KINETIS_EWM_SERV           (KINETIS_EWM_BASE+KINETIS_EWM_SERV_OFFSET)
#define KINETIS_EWM_CMPL           (KINETIS_EWM_BASE+KINETIS_EWM_CMPL_OFFSET)
#define KINETIS_EWM_CMPH           (KINETIS_EWM_BASE+KINETIS_EWM_CMPH_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Control Register (8-bit) */

#define EWM_CTRL_EWMEN             (1 << 0)  /* Bit 0:  EWM enable */
#define EWM_CTRL_ASSIN             (1 << 2)  /* Bit 1:  EWM_in's Assertion State Select */
#define EWM_CTRL_INEN              (1 << 3)  /* Bit 2:  Input Enable */
                                             /* Bits 7–3: Reserved */

/* Service Register (8-bit values:  0xb4 followed by 0x2c) */
/* Compare Low Register (8-bit compare low value) */
/* Compare High Register (8-bit compare high value) */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_EWM_H */
