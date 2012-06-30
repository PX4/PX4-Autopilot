/************************************************************************************
 * arch/arm/src/lpc43xx/chip.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities file */

#include <arch/lpc43xx/chip.h>

/* Include the chip  memory map, pin configuration, and vector definition.  These
 * header files may or may not be shared between different chips.  That decisions
 * depends on the similarity of the chip peripheral.
 */

#if defined(CONFIG_ARCH_CHIP_LPC4310FBD144)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4310fbd144_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4310FET100)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4310fet100_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4320FBD144)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4320fbd144_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4320FET100)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4320fet100_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FBD144)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4330fbd144_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET100)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4330fet100_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET180)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4330fet180_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4330FET256)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4330fet256_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FBD208)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4350fbd208_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET180)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4350fet180_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4350FET256)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4310203050_pinconfig.h"
#  include "chip/lpc4350fet256_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FBD208)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4353fbd208_pinconfig.h"
#  include "chip/lpc4353fbd208_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET180)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4353fet180_pinconfig.h"
#  include "chip/lpc4353fet180_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4353FET256)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4353fet256_pinconfig.h"
#  include "chip/lpc4353fet256_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET180)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4357fet180_pinconfig.h"
#  include "chip/lpc4357fet180_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FBD208)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4357fbd208_pinconfig.h"
#  include "chip/lpc4357fbd208_vectors.h"
#elif defined(CONFIG_ARCH_CHIP_LPC4357FET256)
#  include "chip/lpc43_memorymap.h"
#  include "chip/lpc4357fet256_pinconfig.h"
#  include "chip/lpc4357fet256_vectors.h"
#else
#  error "Unsupported LPC43xx chip"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-31. The lower the value, the greater
 * the priority of the corresponding interrupt.
 *
 * The Cortex-M4 core supports up to 53 interrupts an 8 prgrammable interrupt
 * priority levels; The Cortex-M0 core supports up to 32 interrupts with 4
 * programmable interrupt priorities.
 */

#define LPC43M4_SYSH_PRIORITY_MIN     0xe0 /* All bits[7:5] set is minimum priority */
#define LPC43M4_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define LPC43M4_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

#define LPC43M0_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define LPC43M0_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define LPC43M0_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */

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
