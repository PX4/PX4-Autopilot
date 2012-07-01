/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_dac.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_DAC_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_DAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC43_DAC_CR_OFFSET     0x0000 /* D/A Converter Register */
#define LPC43_DAC_CTRL_OFFSET   0x0004 /* DAC Control register */
#define LPC43_DAC_CNTVAL_OFFSET 0x0008 /* DAC Counter Value register */

/* Register addresses ***************************************************************/

#define LPC43_DAC_CR            (LPC43_DAC_BASE+LPC43_DAC_CR_OFFSET)
#define LPC43_DAC_CTRL          (LPC43_DAC_BASE+LPC43_DAC_CTRL_OFFSET)
#define LPC43_DAC_CNTVAL        (LPC43_DAC_BASE+LPC43_DAC_CNTVAL_OFFSET)

/* Register bit definitions *********************************************************/

/* D/A Converter Register */
                                          /* Bits 0-5: Reserved */
#define DAC_CR_VALUE_SHIFT      (6)       /* Bits 6-15: Controls voltage on the AOUT pin */
#define DAC_CR_VALUE_MASK       (0x3ff << DAC_CR_VALUE_SHIFT)
#define DAC_CR_BIAS             (1 << 16) /* Bit 16: Controls DAC settling time */
                                          /* Bits 17-31: Reserved */
/* DAC Control register */

#define DAC_CTRL_INTDMAREQ      (1 << 0) /* Bit 0: Timer timed out */
#define DAC_CTRL_DBLBUFEN       (1 << 1) /* Bit 1: Enable DACR double-buffering */
#define DAC_CTRL_CNTEN          (1 << 2) /* Bit 2: Enable timeout counter */
#define DAC_CTRL_DMAEN          (1 << 3) /* Bit 3: Enable DMA access */
                                         /* Bits 4-31: Reserved */
/* DAC Counter Value register */

#define DAC_CNTVAL_SHIFT        (0)      /* Bits 0-15: Reload value for DAC interrupt/DMA timer */
#define DAC_CNTVAL_MASK         (0xffff << DAC_CNTVAL_SHIFT)
                                         /* Bits 8-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_DAC_H */
