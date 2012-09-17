/************************************************************************************
 * arch/arm/src/kinetis/kinetis_pmc.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_PMC_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_PMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_PMC_LVDSC1_OFFSET 0x0000 /* Low Voltage Detect Status and Control 1 Register */
#define KINETIS_PMC_LVDSC2_OFFSET 0x0001 /* Low Voltage Detect Status and Control 2 Register */
#define KINETIS_PMC_REGSC_OFFSET  0x0002 /* Regulator Status and Control Register */

/* Register Addresses ***************************************************************/

#define KINETIS_PMC_LVDSC1        (KINETIS_PMC_BASE+KINETIS_PMC_LVDSC1_OFFSET)
#define KINETIS_PMC_LVDSC2        (KINETIS_PMC_BASE+KINETIS_PMC_LVDSC2_OFFSET)
#define KINETIS_PMC_REGSC         (KINETIS_PMC_BASE+KINETIS_PMC_REGSC_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Low Voltage Detect Status and Control 1 Register */

#define PMC_LVDSC1_LVDV_SHIFT     (0)       /* Bits 0-1: Low-Voltage Detect Voltage Select */
#define PMC_LVDSC1_LVDV_MASK      (3 << PMC_LVDSC1_LVDV_SHIFT)
#  define PMC_LVDSC1_LVDV_LOW     (0 << PMC_LVDSC1_LVDV_SHIFT) /* Low trip point selected (VLVD = VLVDL) */
#  define PMC_LVDSC1_LVDV_HIGH    (1 << PMC_LVDSC1_LVDV_SHIFT) /* High trip point selected (VLVD = VLVDH) */
                                            /* Bits 2-3: Reserved */
#define PMC_LVDSC1_LVDRE          (1 << 4)  /* Bit 4:  Low-Voltage Detect Reset Enable */
#define PMC_LVDSC1_LVDIE          (1 << 5)  /* Bit 5:  Low-Voltage Detect Interrupt Enable */
#define PMC_LVDSC1_LVDACK         (1 << 6)  /* Bit 6:  Low-Voltage Detect Acknowledge */
#define PMC_LVDSC1_LVDF           (1 << 7)  /* Bit 7:  Low-Voltage Detect Flag */

/* Low Voltage Detect Status and Control 2 Register */

#define PMC_LVDSC2_LVWV_SHIFT     (0)       /* Bits 0-1: Low-Voltage Warning Voltage Select */
#define PMC_LVDSC2_LVWV_MASK      (3 << PMC_LVDSC2_LVWV_SHIFT)
#  define PMC_LVDSC2_LVWV_ LOW    (0 << PMC_LVDSC2_LVWV_SHIFT) /* Low trip point selected (VLVW = VLVW1H/L) */
#  define PMC_LVDSC2_LVWV_ MID1   (1 << PMC_LVDSC2_LVWV_SHIFT) /* Mid 1 trip point selected (VLVW = VLVW2H/L) */
#  define PMC_LVDSC2_LVWV_ MID2   (2 << PMC_LVDSC2_LVWV_SHIFT) /* Mid 2 trip point selected (VLVW = VLVW3H/L) */
#  define PMC_LVDSC2_LVWV_ HIGH   (3 << PMC_LVDSC2_LVWV_SHIFT) /* High trip point selected (VLVW = VLVW4H/L) */
                                            /* Bits 2-4: Reserved */
#define PMC_LVDSC2_LVWIE          (1 << 5)  /* Bit 5:  Low-Voltage Warning Interrupt Enable */
#define PMC_LVDSC2_LVWACK         (1 << 6)  /* Bit 6:  Low-Voltage Warning Acknowledge */
#define PMC_LVDSC2_LVWF           (1 << 7)  /* Bit 7:  Low-Voltage Warning Flag */

/* Regulator Status and Control Register */

#define PMC_REGSC_BGBE            (1 << 0)  /* Bit 0:  Bandgap Buffer Enable */
                                            /* Bit 1: Reserved */
#define PMC_REGSC_REGONS          (1 << 2)  /* Bit 2:  Regulator in Run Regulation Status */
#define PMC_REGSC_VLPRS           (1 << 3)  /* Bit 3:  Very Low Power Run Status */
#define PMC_REGSC_TRAMPO          (1 << 4)  /* Bit 4:  For devices with FlexNVM: Traditional RAM Power Option */
                                            /* Bits 5-7: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_PMC_H */
