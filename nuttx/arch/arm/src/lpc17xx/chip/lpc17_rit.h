/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_rit.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_RIT_H
#define __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_RIT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC17_RIT_COMPVAL_OFFSET 0x0000  /* Compare register */
#define LPC17_RIT_MASK_OFFSET    0x0004  /* Mask register */
#define LPC17_RIT_CTRL_OFFSET    0x0008  /* Control register */
#define LPC17_RIT_COUNTER_OFFSET 0x000c  /* 32-bit counter */

/* Register addresses ***************************************************************/

#define LPC17_RIT_COMPVAL        (LPC17_RIT_BASE+LPC17_RIT_COMPVAL_OFFSET)
#define LPC17_RIT_MASK           (LPC17_RIT_BASE+LPC17_RIT_MASK_OFFSET)
#define LPC17_RIT_CTRL           (LPC17_RIT_BASE+LPC17_RIT_CTRL_OFFSET)
#define LPC17_RIT_COUNTER        (LPC17_RIT_BASE+LPC17_RIT_COUNTER_OFFSET)

/* Register bit definitions *********************************************************/
/* Compare register (Bits 0-31: value compared to the counter) */

/* Mask register (Bits 0-31: 32-bit mask value) */

/* Control register */

#define RIT_CTRL_INT             (1 << 0)  /* Bit 0: Interrupt flag */
#define RIT_CTRL_ENCLR           (1 << 1)  /* Bit 1: Timer enable clear */
#define RIT_CTRL_ENBR            (1 << 2)  /* Bit 2: Timer enable for debug */
#define RIT_CTRL_EN              (1 << 3)  /* Bit 3: Timer enable */
                                           /* Bits 4-31: Reserved */
/* 32-bit counter (Bits 0-31: 32-bit up counter) */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_CHIP_LPC17_RIT_H */
