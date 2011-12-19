/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32_dbgmcu.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_DBGMCU_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_DBGMCU_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Addresses *******************************************************************************/

#define STM32_DBGMCU_IDCODE       0xe0042000  /* MCU identifier */
#define STM32_DBGMCU_CR           0xe0042004  /* MCU debug */
#ifdef CONFIG_STM32_STM32F40XX
#  define STM32_DBGMCU_APB1_FZ    0xe0042008  /* Debug MCU APB1 freeze register */
#  define STM32_DBGMCU_APB2_FZ    0xe004200c  /* Debug MCU APB2 freeze register */
#endif

/* Register Bitfield Definitions ********************************************************************/

/* MCU identifier */

#define DBGMCU_IDCODE_DEVID_SHIFT (0)       /* Bits 11-0: Device Identifier */
#define DBGMCU_IDCODE_DEVID_MASK  (0x0fff << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_REVID_SHIFT (16)      /* Bits 31-16:  Revision Identifier */
#define DBGMCU_IDCODE_REVID_MASK  (0xffff << DBGMCU_IDCODE_REVID_SHIFT)

/* MCU debug */

#define DBGMCU_CR_SLEEP           (1 << 0)  /* Bit 0: Debug Sleep Mode */
#define DBGMCU_CR_STOP            (1 << 1)  /* Bit 1: Debug Stop Mode */
#define DBGMCU_CR_STANDBY         (1 << 2)  /* Bit 2: Debug Standby mode */
#define DBGMCU_CR_TRACEIOEN       (1 << 5)  /* Bit 5: Trace enabled */

#define DBGMCU_CR_TRACEMODE_SHIFT (6)        /* Bits 7-6: Trace mode pin assignement */
#define DBGMCU_CR_TRACEMODE_MASK  (3 << DBGMCU_CR_TRACEMODE_SHIFT)
#  define DBGMCU_CR_ASYNCH        (0 << DBGMCU_CR_TRACEMODE_SHIFT) /* Asynchronous Mode */
#  define DBGMCU_CR_SYNCH1        (1 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=1 */
#  define DBGMCU_CR_SYNCH2        (2 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=2 */
#  define DBGMCU_CR_SYNCH4        (3 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=4 */

#ifdef CONFIG_STM32_STM32F10XX
#  define DBGMCU_CR_IWDGSTOP      (1 << 8)   /* Bit 8: Independent Watchdog stopped when core is halted */
#  define DBGMCU_CR_WWDGSTOP      (1 << 9)   /* Bit 9: Window Watchdog stopped when core is halted */
#  define DBGMCU_CR_TIM1STOP      (1 << 10)  /* Bit 10: TIM1 stopped when core is halted */
#  define DBGMCU_CR_TIM2STOP      (1 << 11)  /* Bit 11: TIM2 stopped when core is halted */
#  define DBGMCU_CR_TIM3STOP      (1 << 12)  /* Bit 12: TIM3 stopped when core is halted */
#  define DBGMCU_CR_TIM4STOP      (1 << 13)  /* Bit 13: TIM4 stopped when core is halted */
#  define DBGMCU_CR_CAN1STOP      (1 << 14)  /* Bit 14: CAN1 stopped when core is halted */
#  define DBGMCU_CR_SMBUS1STOP    (1 << 15)  /* Bit 15: I2C1 SMBUS timeout mode stopped when core is halted */
#  define DBGMCU_CR_SMBUS2STOP    (1 << 16)  /* Bit 16: I2C2 SMBUS timeout mode stopped when core is halted */
#  define DBGMCU_CR_TIM8STOP      (1 << 17)  /* Bit 17: TIM8 stopped when core is halted */
#  define DBGMCU_CR_TIM5STOP      (1 << 18)  /* Bit 18: TIM5 stopped when core is halted */
#  define DBGMCU_CR_TIM6STOP      (1 << 19)  /* Bit 19: TIM6 stopped when core is halted */
#  define DBGMCU_CR_TIM7STOP      (1 << 20)  /* Bit 20: TIM7 stopped when core is halted */
#  define DBGMCU_CR_CAN2STOP      (1 << 21)  /* Bit 21: CAN2 stopped when core is halted */
#endif

/* Debug MCU APB1 freeze register */

#ifdef CONFIG_STM32_STM32F40XX
#  define DBGMCU_APB1_TIM2STOP    (1 << 0)   /* Bit 0: TIM2 stopped when core is halted */
#  define DBGMCU_APB1_TIM3STOP    (1 << 1)   /* Bit 1: TIM3 stopped when core is halted */
#  define DBGMCU_APB1_TIM4STOP    (1 << 2)   /* Bit 2: TIM4 stopped when core is halted */
#  define DBGMCU_APB1_TIM5STOP    (1 << 3)   /* Bit 3: TIM5 stopped when core is halted */
#  define DBGMCU_APB1_TIM6STOP    (1 << 4)   /* Bit 4: TIM6 stopped when core is halted */
#  define DBGMCU_APB1_TIM7STOP    (1 << 5)   /* Bit 5: TIM7 stopped when core is halted */
#  define DBGMCU_APB1_TIM12STOP   (1 << 6)   /* Bit 6: TIM12 stopped when core is halted */
#  define DBGMCU_APB1_TIM13STOP   (1 << 7)   /* Bit 7: TIM13 stopped when core is halted */
#  define DBGMCU_APB1_TIM14STOP   (1 << 8)   /* Bit 7: TIM14 stopped when core is halted */
#  define DBGMCU_CR_RTCSTOP       (1 << 10)  /* Bit 11: RTC stopped when Core is halted */
#  define DBGMCU_CR_WWDGSTOP      (1 << 11)  /* Bit 11: Window Watchdog stopped when core is halted */
#  define DBGMCU_CR_IWDGSTOP      (1 << 12)  /* Bit 12: Independent Watchdog stopped when core is halted */
#  define DBGMCU_APB1_I2C1STOP    (1 << 21)  /* Bit 21: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_I2C2STOP    (1 << 22)  /* Bit 22: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_I2C3STOP    (1 << 23)  /* Bit 23: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_CAN1STOP    (1 << 25)  /* Bit 25: CAN1 stopped when core is halted */
#  define DBGMCU_APB1_CAN2STOP    (1 << 26)  /* Bit 26: CAN2 stopped when core is halted */
#endif

/* Debug MCU APB2 freeze register */

#ifdef CONFIG_STM32_STM32F40XX
#  define DBGMCU_APB2_TIM1STOP    (1 << 0)   /* Bit 0:  TIM1 stopped when core is halted */
#  define DBGMCU_APB2_TIM8STOP    (1 << 1)   /* Bit 1:  TIM8 stopped when core is halted */
#  define DBGMCU_APB2_TIM9STOP    (1 << 16)  /* Bit 16: TIM9 stopped when core is halted */
#  define DBGMCU_APB2_TIM10STOP   (1 << 17)  /* Bit 17: TIM10 stopped when core is halted */
#  define DBGMCU_APB2_TIM11STOP   (1 << 18)  /* Bit 18: TIM11 stopped when core is halted */
#endif

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_DBGMCU_H */
