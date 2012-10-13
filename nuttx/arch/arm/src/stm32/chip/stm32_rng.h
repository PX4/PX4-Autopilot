/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_rng.h
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
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

#ifndef __ARCH_ARM_STC_STM32_CHIP_STM32_RNG_H
#define __ARCH_ARM_STC_STM32_CHIP_STM32_RNG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_RNG_CR_OFFSET       0x0000  /* RNG Control Register */
#define STM32_RNG_SR_OFFSET       0x0004  /* RNG Status Register */
#define STM32_RNG_DR_OFFSET       0x0008  /* RNG Data Register */

/* Register Addresses ***************************************************************/

#define STM32_RNG_CR              (STM32_RNG_BASE+STM32_RNG_CR_OFFSET)
#define STM32_RNG_SR              (STM32_RNG_BASE+STM32_RNG_SR_OFFSET)
#define STM32_RNG_DR              (STM32_RNG_BASE+STM32_RNG_DR_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* RNG Control Register */

#define RNG_CR_RNGEN              (1 << 2)  /* Bit 2: RNG enable */
#define RNG_CR_IE                 (1 << 3)  /* Bit 3: Interrupt enable */

/* RNG Status Register */

#define RNG_SR_DRDY               (1 << 0) /* Bit 0: Data ready */
#define RNG_SR_CECS               (1 << 1) /* Bit 1: Clock error current status */
#define RNG_SR_SECS               (1 << 2) /* Bit 2: Seed error current status */
#define RNG_SR_CEIS               (1 << 5) /* Bit 5: Clock error interrupt status */
#define RNG_SR_SEIS               (1 << 6) /* Bit 6: Seed error interrupt status */

#endif	/* __ARCH_ARM_STC_STM32_CHIP_STM32_RNG_H */
