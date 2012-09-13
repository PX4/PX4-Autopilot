/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_dmamux.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_DMAMUX_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_DMAMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_DMAMUX_CHCFG_OFFSET(n)  (n)    /* Channel n Configuration Register */
#define KINETIS_DMAMUX_CHCFG0_OFFSET    0x0000 /* Channel 0 Configuration Register */
#define KINETIS_DMAMUX_CHCFG1_OFFSET    0x0001 /* Channel 1 Configuration Register */
#define KINETIS_DMAMUX_CHCFG2_OFFSET    0x0002 /* Channel 2 Configuration Register */
#define KINETIS_DMAMUX_CHCFG3_OFFSET    0x0003 /* Channel 3 Configuration Register */
#define KINETIS_DMAMUX_CHCFG4_OFFSET    0x0004 /* Channel 4 Configuration Register */
#define KINETIS_DMAMUX_CHCFG5_OFFSET    0x0005 /* Channel 5 Configuration Register */
#define KINETIS_DMAMUX_CHCFG6_OFFSET    0x0006 /* Channel 6 Configuration Register */
#define KINETIS_DMAMUX_CHCFG7_OFFSET    0x0007 /* Channel 7 Configuration Register */
#define KINETIS_DMAMUX_CHCFG8_OFFSET    0x0008 /* Channel 8 Configuration Register */
#define KINETIS_DMAMUX_CHCFG9_OFFSET    0x0009 /* Channel 9 Configuration Register */
#define KINETIS_DMAMUX_CHCFG10_OFFSET   0x000a /* Channel 10 Configuration Register */
#define KINETIS_DMAMUX_CHCFG11_OFFSET   0x000b /* Channel 11 Configuration Register */
#define KINETIS_DMAMUX_CHCFG12_OFFSET   0x000c /* Channel 12 Configuration Register */
#define KINETIS_DMAMUX_CHCFG13_OFFSET   0x000d /* Channel 13 Configuration Register */
#define KINETIS_DMAMUX_CHCFG14_OFFSET   0x000e /* Channel 14 Configuration Register */
#define KINETIS_DMAMUX_CHCFG15_OFFSET   0x000f /* Channel 15 Configuration Register */

/* Register Addresses ***********************************************************************/

#define KINETIS_DMAMUX_CHCFG(n)         (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG_OFFSET(n))
#define KINETIS_DMAMUX_CHCFG0           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG0_OFFSET)
#define KINETIS_DMAMUX_CHCFG1           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG1_OFFSET)
#define KINETIS_DMAMUX_CHCFG2           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG2_OFFSET)
#define KINETIS_DMAMUX_CHCFG3           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG3_OFFSET)
#define KINETIS_DMAMUX_CHCFG4           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG4_OFFSET)
#define KINETIS_DMAMUX_CHCFG5           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG5_OFFSET)
#define KINETIS_DMAMUX_CHCFG6           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG6_OFFSET)
#define KINETIS_DMAMUX_CHCFG7           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG7_OFFSET)
#define KINETIS_DMAMUX_CHCFG8           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG8_OFFSET)
#define KINETIS_DMAMUX_CHCFG9           (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG9_OFFSET)
#define KINETIS_DMAMUX_CHCFG10          (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG10_OFFSET)
#define KINETIS_DMAMUX_CHCFG11          (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG11_OFFSET)
#define KINETIS_DMAMUX_CHCFG12          (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG12_OFFSET)
#define KINETIS_DMAMUX_CHCFG13          (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG13_OFFSET)
#define KINETIS_DMAMUX_CHCFG14          (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG14_OFFSET)
#define KINETIS_DMAMUX_CHCFG15          (KINETIS_DMAMUX0_BASE+KINETIS_DMAMUX_CHCFG15_OFFSET)

/* Register Bit Definitions *****************************************************************/
/* Channel n Configuration Register */

#define DMAMUX_CHCFG_SOURCE_SHIFT       (0)       /* Bits 0-5: DMA Channel Source (slot) */
#define DMAMUX_CHCFG_SOURCE_MASK        (63 << DMAMUX_CHCFG_SOURCE_SHIFT)
#define DMAMUX_CHCFG_TRIG               (1 << 6)  /* Bit 6:  DMA Channel Trigger Enable */
#define DMAMUX_CHCFG_ENBL               (1 << 7)  /* Bit 7:  DMA Channel Enable */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_DMAMUX_H */
