/************************************************************************************
 * arch/arm/include/sam3u/chip.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_SAM3U_CHIP_H
#define __ARCH_ARM_INCLUDE_SAM3U_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#ifdef CONFIG_ARCH_CHIP_AT91SAM3U4E
/* Internal memory */

#  define CONFIG_SAM3U_SRAM0_SIZE   0x00008000  /* 32Kb */
#  define CONFIG_SAM3U_SRAM1_SIZE   0x00004000  /* 16Kb */
#  define CONFIG_SAM3U_NFCSRAM_SIZE 0x00001000  /*  4Kb */

/* DMA */

#  define CONFIG_SAM3U_NDMACHAN     4           /* 4 DMA Channels */

/* Memory card interface */

#  define CONFIG_SAM3U_MCI2         1
#else
#  error "Unknown SAM3U chip type"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-15. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:4] of each field, bits[3:0] read as zero and ignore writes.
 */
 
#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#define NVIC_SYSH_DISABLE_PRIORITY (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#define NVIC_SYSH_SVCALL_PRIORITY  NVIC_SYSH_PRIORITY_MAX

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_SAM3U_CHIP_H */
