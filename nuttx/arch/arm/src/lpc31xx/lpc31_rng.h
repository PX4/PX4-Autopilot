/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_rng.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_RNG_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_RNG_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* RNG register base address offset into the APB0 domain ****************************************/

#define LPC31_RNG_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_RNG_OFFSET)
#define LPC31_RNG_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_RNG_OFFSET)

/* RNG register offsets (with respect to the RNG base) ******************************************/

#define LPC31_RNG_RAND_OFFSET          0x0000 /* Random number */
#define LPC31_RNG_PWRDWN_OFFSET        0x0ff4 /* Power-down mode */

/* RNG register (virtual) addresses *************************************************************/

#define LPC31_RNG_RAND                 (LPC31_RNG_VBASE+LPC31_RNG_RAND_OFFSET)
#define LPC31_RNG_PWRDWN               (LPC31_RNG_VBASE+LPC31_RNG_PWRDWN_OFFSET)

/* RNG register bit definitions *****************************************************************/

/* POWERDOWN, address 0x13006ff4 */

#define RNG_PWRDWN_PWRDWN                (1 << 2)  /* Block all accesses to standard registers */
#define RNG_PWRDWN_FORCERST              (1 << 1)  /* With SOFTRST forces immediate RNG reset */
#define RNG_PWRDWN_SOFTRST               (1 << 0)  /* Software RNG reset (delayed) */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_RNG_H */
