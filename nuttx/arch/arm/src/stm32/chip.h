/************************************************************************************
 * arch/arm/src/stm32/chip.h
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_H
#define __ARCH_ARM_SRC_STM32_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities file */

#include <arch/stm32/chip.h>

/* Include the chip pin configuration file */

/* STM32 F1 Family ******************************************************************/
#if defined(CONFIG_STM32_STM32F10XX)

/* STM32F100 Value Line */

#  if defined(CONFIG_STM32_VALUELINE)
#    include "chip/stm32f100_pinmap.h"

/* STM32 F103 High Density Family */
/* STM32F103RC, STM32F103RD, and STM32F103RE are all provided in 64 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#  elif defined(CONFIG_ARCH_CHIP_STM32F103RET6)
#    include "chip/stm32f103re_pinmap.h"

/* STM32F103VC, STM32F103VD, and STM32F103VE are all provided in 100 pin packages and differ
 * only in the available FLASH and SRAM.
 */

#  elif defined(CONFIG_ARCH_CHIP_STM32F103VCT6) || defined(CONFIG_ARCH_CHIP_STM32F103VET6)
#    include "chip/stm32f103vc_pinmap.h"

/* STM32F103ZC, STM32F103ZD, and STM32F103ZE are all provided in 144 pin packages and differ
 * only in the available FLASH and SRAM.
 */
#  elif defined(CONFIG_ARCH_CHIP_STM32F103ZET6) 
#    include "chip/stm32f103ze_pinmap.h"

/* STM32 F105/F107 Connectivity Line */

#  elif defined(CONFIG_ARCH_CHIP_STM32F105VBT7)
#    include "chip/stm32f105vb_pinmap.h"

#  elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#    include "chip/stm32f107vc_pinmap.h"
#  else
#    error "Unsupported STM32F10XXX chip"
#  endif

/* STM32 F2 Family ******************************************************************/
#elif defined(CONFIG_STM32_STM32F20XX)
#  include "chip/stm32f20xxx_pinmap.h"

/* STM32 F4 Family ******************************************************************/
#elif defined(CONFIG_STM32_STM32F40XX)
#  include "chip/stm32f40xxx_pinmap.h"
#else
#  error "No pinmap file for this STM32 chip"
#endif

/* If the common ARMv7-M vector handling logic is used, then include the
 * required vector definitions as well.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR
#  if defined(CONFIG_STM32_STM32F10XX)
#    include "chip/stm32f10xxx_vectors.h"
#  elif defined(CONFIG_STM32_STM32F20XX)
#    include "chip/stm32f20xxx_vectors.h"
#  elif defined(CONFIG_STM32_STM32F40XX)
#    include "chip/stm32f40xxx_vectors.h"
#  else
#    error "No vector file for this STM32 family"
#  endif
#endif

/* Include the chip memory map. */

#include "chip/stm32_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_CHIP_H */

