/************************************************************************************
 * arch/arm/src/lpc43xx/chip.h
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities file */

#include <arch/lpc43xx/chip.h>

/* For each chip supported in chip.h, the following are provided to customize the
 * environment for the specific LPC43XX chip:
 *
 * Define ARMV7M_PERIPHERAL_INTERRUPTS - This is needed by common/up_vectors.c.  This
 *   definition provides the number of "external" interrupt vectors supported by
 *   the specific LPC43 chip.
 *
 *   For the Cortex-M3 core, this should always be equal to the value
 *   LPC43M4_IRQ_NEXTINT defined in include/lpc43xx/irq.h.  For the Cortex-M0
 *   core, this should always be equal to the value LPC43M0_IRQ_NEXTINT defined
 *   in include/lpc43xx/irq.h (At present, only the Cortex-M4 core is supported)
 *
 * Include the chip-specific memory map header file, and
 * Include the chip-specific pin configuration.
 *
 * These header files may or may not be shared between different chips.  That decisions
 * depends on the similarity of the chip peripheral.
 */

#if defined(CONFIG_ARCH_CHIP_LPC4310FBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4310FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4320FBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4320FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FBD144)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET100)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FBD208)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc4310203050_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FBD208)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc435357_memorymap.h"
#  include "chip/lpc4353fbd208_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc435357_memorymap.h"
#  include "chip/lpc4353fet180_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc435357_memorymap.h"
#  include "chip/lpc4353fet256_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET180)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc435357_memorymap.h"
#  include "chip/lpc4357fet180_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FBD208)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc435357_memorymap.h"
#  include "chip/lpc4357fbd208_pinconfig.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET256)
#  define  ARMV7M_PERIPHERAL_INTERRUPTS 53
#  include "chip/lpc435357_memorymap.h"
#  include "chip/lpc4357fet256_pinconfig.h"
#else
#  error "Unsupported LPC43xx chip"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_H */
