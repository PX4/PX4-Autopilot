/************************************************************************************
 * arch/arm/src/lpc31xx/chip.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC31XX_CHIP_H
#define __ARCH_ARM_SRC_LPC31XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_ARCH_CHIP_LPC3130)
#  undef  HAVE_INTSRAM1                  /* 96Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#elif defined(CONFIG_ARCH_CHIP_LPC3131)
#  define HAVE_INTSRAM1  1               /* 192Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#elif defined(CONFIG_ARCH_CHIP_LPC3152)
#  define HAVE_INTSRAM1  1               /* 192Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#elif defined(CONFIG_ARCH_CHIP_LPC3154)
#  define HAVE_INTSRAM1  1               /* 192Kb internal SRAM */
#  define LPC31_NDMACH   12              /* 12 DMA channels */
#  define HAVE_AESENGINE 1               /* AES engine */
#else
#  error "Unsupported LPC31XX architecture"
#  undef  HAVE_INTSRAM1                  /* No INTSRAM1 */
#  define LPC31_NDMACH   0               /* No DMA channels */
#  undef  HAVE_AESENGINE                 /* No AES engine */
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_CHIP_H */
