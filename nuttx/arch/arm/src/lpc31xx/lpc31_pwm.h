/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_pwm.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_PWM_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_PWM_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* PWM register base address offset into the APB1 domain ****************************************/

#define LPC31_PWM_VBASE                (LPC31_APB1_VADDR+LPC31_APB1_PWM_OFFSET)
#define LPC31_PWM_PBASE                (LPC31_APB1_PADDR+LPC31_APB1_PWM_OFFSET)

/* PWM register offsets (with respect to the PWM base) ******************************************/

#define LPC31_PWM_TMR_OFFSET           0x000 /* Timer Register */
#define LPC31_PWM_CNTL_OFFSET          0x004 /* Control Register */

/* PWM register (virtual) addresses *************************************************************/

#define LPC31_PWM_TMR                 (LPC31_PWM_VBASE+LPC31_PWM_TMR_OFFSET)
#define LPC31_PWM_CNTL                (LPC31_PWM_VBASE+LPC31_PWM_CNTL_OFFSET)

/* PWM register bit definitions *****************************************************************/

/* Timer register TMR, address 0x13009000 */

#define PWM_TMR_SHIFT                   (0)    /* Bits 0-11: Timer used for PWM and PDM */
#define PWM_TMR_MASK                    (0xfff << PWM_TMR_SHIFT)

/* Control register CNTL, address 0x13009004 */

#define PWM_CNTL_PDM                    (1 << 7)  /* Bit 7:  PDM Select PDM mode */
#define PWM_CNTL_LOOP                   (1 << 6)  /* Bit 6:  Output inverted with top 4 TMR bits */
#define PWM_CNTL_HI                     (1 << 4)  /* Bit 4:  PWM output forced high */
                                                  /* Bits 2-3: Reserved */
#define PWM_CNTL_CLK_SHIFT              (0)       /* Bits 0-1: Configure PWM_CLK for output pulses */
#define PWM_CNTL_CLK_MASK               (3 << PWM_CNTL_CLK_SHIFT)
#  define PWM_CNTL_CLKDIV1              (0 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK */
#  define PWM_CNTL_CLKDIV2              (1 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK/2 */
#  define PWM_CNTL_CLKDIV4              (2 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK/4 */
#  define PWM_CNTL_CLKDIV8              (3 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK/8 */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_PWM_H */
