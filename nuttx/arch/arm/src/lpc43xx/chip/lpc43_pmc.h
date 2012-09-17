/************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_pmc.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_PMC_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_PMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define LPC43_PD0_SLEEP0_HWENA_OFFSET 0x0000 /* Hardware sleep event enable register */
#define LPC43_PD0_SLEEP0_MODE_OFFSET  0x001c /* Power-down mode control register */

/* Register Addresses ***************************************************************/

#define LPC43_PD0_SLEEP0_HWENA        (LPC43_PMC_BASE+LPC43_PD0_SLEEP0_HWENA_OFFSET)
#define LPC43_PD0_SLEEP0_MODE         (LPC43_PMC_BASE+LPC43_PD0_SLEEP0_MODE_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Hardware sleep event enable register */

#define PD0_SLEEP0_HWENA              (1 << 0)  /* Bit 0:  Enable power down mode */
                                                /* Bits 1-31:  Reserved */
/* Power-down mode control register */

#define PD0_DEEP_SLEEP_MODE           0x003000aa
#define PD0_PWRDOWN_MODE              0x0030fcba
#define PD0_DEEP_PWRDOWN_MODE         0x0030ff7f

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_PMC_H */
