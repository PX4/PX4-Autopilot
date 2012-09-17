/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_timer.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_TIMER_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_TIMER_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* TIMER register base address offset into the APB1 domain **************************************/

#define LPC31_TIMER0_VBASE             (LPC31_APB1_VADDR+LPC31_APB1_TIMER0_OFFSET)
#define LPC31_TIMER0_PBASE             (LPC31_APB1_PADDR+LPC31_APB1_TIMER0_OFFSET)

#define LPC31_TIMER1_VBASE             (LPC31_APB1_VADDR+LPC31_APB1_TIMER1_OFFSET)
#define LPC31_TIMER1_PBASE             (LPC31_APB1_PADDR+LPC31_APB1_TIMER1_OFFSET)

#define LPC31_TIMER2_VBASE             (LPC31_APB1_VADDR+LPC31_APB1_TIMER2_OFFSET)
#define LPC31_TIMER2_PBASE             (LPC31_APB1_PADDR+LPC31_APB1_TIMER2_OFFSET)

#define LPC31_TIMER3_VBASE             (LPC31_APB1_VADDR+LPC31_APB1_TIMER3_OFFSET)
#define LPC31_TIMER3_PBASE             (LPC31_APB1_PADDR+LPC31_APB1_TIMER3_OFFSET)

/* TIMER register offsets (with respect to the TIMERn base) *************************************/

#define LPC31_TIMER_LOAD_OFFSET         0x00 /* Timer reload value */
#define LPC31_TIMER_VALUE_OFFSET        0x04 /* Current timer value */
#define LPC31_TIMER_CTRL_OFFSET         0x08 /* Timer nable/disable and pre-scale */
#define LPC31_TIMER_CLEAR_OFFSET        0x0c /* Clear timer interrupt */

/* TIMER register (virtual) addresses ***********************************************************/

#define LPC31_TIMER0_LOAD              (LPC31_TIMER0_VBASE+LPC31_TIMER_LOAD_OFFSET)
#define LPC31_TIMER0_VALUE             (LPC31_TIMER0_VBASE+LPC31_TIMER_VALUE_OFFSET)
#define LPC31_TIMER0_CTRL              (LPC31_TIMER0_VBASE+LPC31_TIMER_CTRL_OFFSET)
#define LPC31_TIMER0_CLEAR             (LPC31_TIMER0_VBASE+LPC31_TIMER_CLEAR_OFFSET)

#define LPC31_TIMER1_LOAD              (LPC31_TIMER1_VBASE+LPC31_TIMER_LOAD_OFFSET)
#define LPC31_TIMER1_VALUE             (LPC31_TIMER1_VBASE+LPC31_TIMER_VALUE_OFFSET)
#define LPC31_TIMER1_CTRL              (LPC31_TIMER1_VBASE+LPC31_TIMER_CTRL_OFFSET)
#define LPC31_TIMER1_CLEAR             (LPC31_TIMER1_VBASE+LPC31_TIMER_CLEAR_OFFSET)

#define LPC31_TIMER2_LOAD              (LPC31_TIMER2_VBASE+LPC31_TIMER_LOAD_OFFSET)
#define LPC31_TIMER2_VALUE             (LPC31_TIMER2_VBASE+LPC31_TIMER_VALUE_OFFSET)
#define LPC31_TIMER2_CTRL              (LPC31_TIMER2_VBASE+LPC31_TIMER_CTRL_OFFSET)
#define LPC31_TIMER2_CLEAR             (LPC31_TIMER2_VBASE+LPC31_TIMER_CLEAR_OFFSET)

#define LPC31_TIMER3_LOAD              (LPC31_TIMER3_VBASE+LPC31_TIMER_LOAD_OFFSET)
#define LPC31_TIMER3_VALUE             (LPC31_TIMER3_VBASE+LPC31_TIMER_VALUE_OFFSET)
#define LPC31_TIMER3_CTRL              (LPC31_TIMER3_VBASE+LPC31_TIMER_CTRL_OFFSET)
#define LPC31_TIMER3_CLEAR             (LPC31_TIMER3_VBASE+LPC31_TIMER_CLEAR_OFFSET)

/* TIMER register bit definitions ***************************************************************/

/* Timer Control register TIMER0_CTRL, address 0x13008008 TIMER1_CTRL, address 0x13008408
 * TIMER2_CTRL, address 0x13008808 TIMER3_CTRL, adddress 0x13008c08
 */

#define TIMER_CTRL_ENABLE                (1 << 7)  /* Bit 7:  Timer enable */
#define TIMER_CTRL_PERIODIC              (1 << 6)  /* Bit 6:  Periodic timer mode */
#define TIMER_CTRL_PRESCALE_SHIFT        (2)       /* Bits 2-3: Timer pre-scale */
#define TIMER_CTRL_PRESCALE_MASK         (3 << TIMER_CTRL_PRESCALE_SHIFT)
#  define TIMER_CTRL_PRESCALE_DIV1       (0 << TIMER_CTRL_PRESCALE_SHIFT) /* Divider=1 Stages=0 */
#  define TIMER_CTRL_PRESCALE_DIV16      (1 << TIMER_CTRL_PRESCALE_SHIFT) /* Divider=16 Stages4 */
#  define TIMER_CTRL_PRESCALE_DIV256     (2 << TIMER_CTRL_PRESCALE_SHIFT) /* Divider=256 Stages=8 */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_TIMER_H */
