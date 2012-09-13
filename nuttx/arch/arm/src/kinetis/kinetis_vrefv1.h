/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_vrefv1.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_VREFV1_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_VREFV1_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_VREF_TRM_OFFSET   0x0000 /* VREF Trim Register */
#define KINETIS_VREF_SC_OFFSET    0x0001 /* VREF Status and Control Register */

/* Register Addresses ***********************************************************************/

#define KINETIS_VREF_TRM          (KINETIS_VREF_BASE+KINETIS_VREF_TRM_OFFSET)
#define KINETIS_VREF_SC           (KINETIS_VREF_BASE+KINETIS_VREF_SC_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* VREF Trim Register (8-bit) */

#define VREF_TRM_SHIFT            (0)       /* Bits 0-5: Trim bits */
#define VREF_TRM_MASK             (63 << VREF_TRM_SHIFT)
                                            /* Bits 6-7: Reserved */
/* VREF Status and Control Register (8-bit) */

#define VREF_SC_MODE_LV_SHIFT     (0)       /* Bits 0-1: Buffer Mode selection */
#define VREF_SC_MODE_LV_MASK      (3 << VREF_SC_MODE_LV_SHIFT)
#  define VREF_SC_MODE_LV_BANDGAP (0 << VREF_SC_MODE_LV_SHIFT) /* Bandgap on only */
#  define VREF_SC_MODE_LV_LOWPWR  (1 << VREF_SC_MODE_LV_SHIFT) /* Low-power buffer enabled */
#  define VREF_SC_MODE_LV_TIGHT   (2 << VREF_SC_MODE_LV_SHIFT) /* Tight-regulation buffer enabled */
#define VREF_SC_VREFST            (1 << 2)  /* Bit 2:  Internal Voltage Reference stable */
                                            /* Bits 3-5: Reserved */
#define VREF_SC_REGEN             (1 << 6)  /* Bit 6:  Regulator enable */
#define VREF_SC_VREFEN            (1 << 7)  /* Bit 7:  Internal Voltage Reference enable */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_VREFV1_H */
