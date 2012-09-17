/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_dac.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_DACE_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_DACE_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_DAC_DATL_OFFSET(n) (0x0000+((n)<<1))
#define KINETIS_DAC_DATH_OFFSET(n) (0x0001+((n)<<1))

#define KINETIS_DAC_DAT0L_OFFSET   0x0000 /* DAC Data Low Register */
#define KINETIS_DAC_DAT0H_OFFSET   0x0001 /* DAC Data High Register */
#define KINETIS_DAC_DAT1L_OFFSET   0x0002 /* DAC Data Low Register */
#define KINETIS_DAC_DAT1H_OFFSET   0x0003 /* DAC Data High Register */
#define KINETIS_DAC_DAT2L_OFFSET   0x0004 /* DAC Data Low Register */
#define KINETIS_DAC_DAT2H_OFFSET   0x0005 /* DAC Data High Register */
#define KINETIS_DAC_DAT3L_OFFSET   0x0006 /* DAC Data Low Register */
#define KINETIS_DAC_DAT3H_OFFSET   0x0007 /* DAC Data High Register */
#define KINETIS_DAC_DAT4L_OFFSET   0x0008 /* DAC Data Low Register */
#define KINETIS_DAC_DAT4H_OFFSET   0x0009 /* DAC Data High Register */
#define KINETIS_DAC_DAT5L_OFFSET   0x000a /* DAC Data Low Register */
#define KINETIS_DAC_DAT5H_OFFSET   0x000b /* DAC Data High Register */
#define KINETIS_DAC_DAT6L_OFFSET   0x000c /* DAC Data Low Register */
#define KINETIS_DAC_DAT6H_OFFSET   0x000d /* DAC Data High Register */
#define KINETIS_DAC_DAT7L_OFFSET   0x000e /* DAC Data Low Register */
#define KINETIS_DAC_DAT7H_OFFSET   0x000f /* DAC Data High Register */
#define KINETIS_DAC_DAT8L_OFFSET   0x0010 /* DAC Data Low Register */
#define KINETIS_DAC_DAT8H_OFFSET   0x0011 /* DAC Data High Register */
#define KINETIS_DAC_DAT9L_OFFSET   0x0012 /* DAC Data Low Register */
#define KINETIS_DAC_DAT9H_OFFSET   0x0013 /* DAC Data High Register */
#define KINETIS_DAC_DAT10L_OFFSET  0x0014 /* DAC Data Low Register */
#define KINETIS_DAC_DAT10H_OFFSET  0x0015 /* DAC Data High Register */
#define KINETIS_DAC_DAT11L_OFFSET  0x0016 /* DAC Data Low Register */
#define KINETIS_DAC_DAT11H_OFFSET  0x0017 /* DAC Data High Register */
#define KINETIS_DAC_DAT12L_OFFSET  0x0018 /* DAC Data Low Register */
#define KINETIS_DAC_DAT12H_OFFSET  0x0019 /* DAC Data High Register */
#define KINETIS_DAC_DAT13L_OFFSET  0x001a /* DAC Data Low Register */
#define KINETIS_DAC_DAT13H_OFFSET  0x001b /* DAC Data High Register */
#define KINETIS_DAC_DAT14L_OFFSET  0x001c /* DAC Data Low Register */
#define KINETIS_DAC_DAT14H_OFFSET  0x001d /* DAC Data High Register */
#define KINETIS_DAC_DAT15L_OFFSET  0x001e /* DAC Data Low Register */
#define KINETIS_DAC_DAT15H_OFFSET  0x001f /* DAC Data High Register */
#define KINETIS_DAC_SR_OFFSET      0x0020 /* DAC Status Register */
#define KINETIS_DAC_C0_OFFSET      0x0021 /* DAC Control Register */
#define KINETIS_DAC_C1_OFFSET      0x0022 /* DAC Control Register 1 */
#define KINETIS_DAC_C2_OFFSET      0x0023 /* DAC Control Register 2 */

/* Register Addresses ***********************************************************************/

#define KINETIS_DAC0_DATL(n)        (KINETIS_DAC0_BASE+KINETIS_DAC_DATL_OFFSET(n))
#define KINETIS_DAC0_DATH(n)        (KINETIS_DAC0_BASE+KINETIS_DAC_DATH_OFFSET(n))

#define KINETIS_DAC0_DAT0L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT0L_OFFSET)
#define KINETIS_DAC0_DAT0H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT0H_OFFSET)
#define KINETIS_DAC0_DAT1L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT1L_OFFSET)
#define KINETIS_DAC0_DAT1H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT1H_OFFSET)
#define KINETIS_DAC0_DAT2L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT2L_OFFSET)
#define KINETIS_DAC0_DAT2H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT2H_OFFSET)
#define KINETIS_DAC0_DAT3L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT3L_OFFSET)
#define KINETIS_DAC0_DAT3H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT3H_OFFSET)
#define KINETIS_DAC0_DAT4L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT4L_OFFSET)
#define KINETIS_DAC0_DAT4H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT4H_OFFSET)
#define KINETIS_DAC0_DAT5L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT5L_OFFSET)
#define KINETIS_DAC0_DAT5H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT5H_OFFSET)
#define KINETIS_DAC0_DAT6L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT6L_OFFSET)
#define KINETIS_DAC0_DAT6H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT6H_OFFSET)
#define KINETIS_DAC0_DAT7L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT7L_OFFSET)
#define KINETIS_DAC0_DAT7H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT7H_OFFSET)
#define KINETIS_DAC0_DAT8L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT8L_OFFSET)
#define KINETIS_DAC0_DAT8H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT8H_OFFSET)
#define KINETIS_DAC0_DAT9L          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT9L_OFFSET)
#define KINETIS_DAC0_DAT9H          (KINETIS_DAC0_BASE+KINETIS_DAC_DAT9H_OFFSET)
#define KINETIS_DAC0_DAT10L         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT10L_OFFSET)
#define KINETIS_DAC0_DAT10H         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT10H_OFFSET)
#define KINETIS_DAC0_DAT11L         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT11L_OFFSET)
#define KINETIS_DAC0_DAT11H         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT11H_OFFSET)
#define KINETIS_DAC0_DAT12L         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT12L_OFFSET)
#define KINETIS_DAC0_DAT12H         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT12H_OFFSET)
#define KINETIS_DAC0_DAT13L         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT13L_OFFSET)
#define KINETIS_DAC0_DAT13H         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT13H_OFFSET)
#define KINETIS_DAC0_DAT14L         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT14L_OFFSET)
#define KINETIS_DAC0_DAT14H         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT14H_OFFSET)
#define KINETIS_DAC0_DAT15L         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT15L_OFFSET)
#define KINETIS_DAC0_DAT15H         (KINETIS_DAC0_BASE+KINETIS_DAC_DAT15H_OFFSET)
#define KINETIS_DAC0_SR             (KINETIS_DAC0_BASE+KINETIS_DAC_SR_OFFSET)
#define KINETIS_DAC0_C0             (KINETIS_DAC0_BASE+KINETIS_DAC_C0_OFFSET)
#define KINETIS_DAC0_C1             (KINETIS_DAC0_BASE+KINETIS_DAC_C1_OFFSET)
#define KINETIS_DAC0_C2             (KINETIS_DAC0_BASE+KINETIS_DAC_C2_OFFSET)

#define KINETIS_DAC1_DATL(n)        (KINETIS_DAC1_BASE+KINETIS_DAC_DATL_OFFSET(n))
#define KINETIS_DAC1_DATH(n)        (KINETIS_DAC1_BASE+KINETIS_DAC_DATH_OFFSET(n))

#define KINETIS_DAC1_DAT0L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT0L_OFFSET)
#define KINETIS_DAC1_DAT0H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT0H_OFFSET)
#define KINETIS_DAC1_DAT1L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT1L_OFFSET)
#define KINETIS_DAC1_DAT1H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT1H_OFFSET)
#define KINETIS_DAC1_DAT2L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT2L_OFFSET)
#define KINETIS_DAC1_DAT2H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT2H_OFFSET)
#define KINETIS_DAC1_DAT3L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT3L_OFFSET)
#define KINETIS_DAC1_DAT3H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT3H_OFFSET)
#define KINETIS_DAC1_DAT4L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT4L_OFFSET)
#define KINETIS_DAC1_DAT4H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT4H_OFFSET)
#define KINETIS_DAC1_DAT5L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT5L_OFFSET)
#define KINETIS_DAC1_DAT5H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT5H_OFFSET)
#define KINETIS_DAC1_DAT6L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT6L_OFFSET)
#define KINETIS_DAC1_DAT6H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT6H_OFFSET)
#define KINETIS_DAC1_DAT7L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT7L_OFFSET)
#define KINETIS_DAC1_DAT7H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT7H_OFFSET)
#define KINETIS_DAC1_DAT8L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT8L_OFFSET)
#define KINETIS_DAC1_DAT8H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT8H_OFFSET)
#define KINETIS_DAC1_DAT9L          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT9L_OFFSET)
#define KINETIS_DAC1_DAT9H          (KINETIS_DAC1_BASE+KINETIS_DAC_DAT9H_OFFSET)
#define KINETIS_DAC1_DAT10L         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT10L_OFFSET)
#define KINETIS_DAC1_DAT10H         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT10H_OFFSET)
#define KINETIS_DAC1_DAT11L         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT11L_OFFSET)
#define KINETIS_DAC1_DAT11H         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT11H_OFFSET)
#define KINETIS_DAC1_DAT12L         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT12L_OFFSET)
#define KINETIS_DAC1_DAT12H         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT12H_OFFSET)
#define KINETIS_DAC1_DAT13L         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT13L_OFFSET)
#define KINETIS_DAC1_DAT13H         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT13H_OFFSET)
#define KINETIS_DAC1_DAT14L         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT14L_OFFSET)
#define KINETIS_DAC1_DAT14H         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT14H_OFFSET)
#define KINETIS_DAC1_DAT15L         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT15L_OFFSET)
#define KINETIS_DAC1_DAT15H         (KINETIS_DAC1_BASE+KINETIS_DAC_DAT15H_OFFSET)
#define KINETIS_DAC1_SR             (KINETIS_DAC1_BASE+KINETIS_DAC_SR_OFFSET)
#define KINETIS_DAC1_C0             (KINETIS_DAC1_BASE+KINETIS_DAC_C0_OFFSET)
#define KINETIS_DAC1_C1             (KINETIS_DAC1_BASE+KINETIS_DAC_C1_OFFSET)
#define KINETIS_DAC1_C2             (KINETIS_DAC1_BASE+KINETIS_DAC_C2_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* DAC Data Low Register (8-bits of data DATA[7:0]) */
/* DAC Data High Register */

#define DAC_DAT0H_MASK              (0x0f)   /* Bits 0-3: DATA[11:8] */

/* DAC Status Register */

#define DAC_SR_DACBFRPBF            (1 << 0)  /* Bit 0:  DAC buffer read pointer bottom position flag */
#define DAC_SR_DACBFRPTF            (1 << 1)  /* Bit 1:  DAC buffer read pointer top position flag */
#define DAC_SR_DACBFWMF             (1 << 2)  /* Bit 2:  DAC buffer watermark flag
                                              /* Bits 3-7: Reserved */
/* DAC Control Register */

#define DAC_C0_DACBBIEN             (1 << 0)  /* Bit 0:  DAC buffer read pointer bottom flag interrupt enable */
#define DAC_C0_DACBTIEN             (1 << 1)  /* Bit 1:  DAC buffer read pointer top flag interrupt enable */
#define DAC_C0_LPEN                 (1 << 3)  /* Bit 3:  DAC low power control */
#define DAC_C0_DACBWIEN             (1 << 2)  /* Bit 2:  DAC buffer watermark interrupt enable */
#define DAC_C0_DACSWTRG             (1 << 4)  /* Bit 4:  DAC software trigger */
#define DAC_C0_DACTRGSEL            (1 << 5)  /* Bit 5:  DAC trigger select */
#define DAC_C0_DACRFS               (1 << 6)  /* Bit 6:  DAC Reference Select */
#define DAC_C0_DACEN                (1 << 7)  /* Bit 7:  DAC enable */

/* DAC Control Register 1 */

#define DAC_C1_DACBFEN              (1 << 0)  /* Bit nn:  DAC buffer enable */
#define DAC_C1_DACBFMD_SHIFT        (1)       /* Bits 1-2: DAC buffer work mode select00 Normal Mode */
#define DAC_C1_DACBFMD_MASK         (3 << DAC_C1_DACBFMD_SHIFT)
#  define DAC_C1_DACBFMD_NORMAL     (0 << DAC_C1_DACBFMD_SHIFT) /* Normal Mode */
#  define DAC_C1_DACBFMD_SWING      (1 << DAC_C1_DACBFMD_SHIFT) /* Swing Mode */
#  define DAC_C1_DACBFMD_OTSCAN     (2 << DAC_C1_DACBFMD_SHIFT) /* One-Time Scan Mode */
#define DAC_C1_DACBFWM_SHIFT        (3)       /* Bits 3-4: DAC buffer watermark select */
#define DAC_C1_DACBFWM_MASK         (3 << DAC_C1_DACBFWM_SHIFT)
#  define DAC_C1_DACBFWM_1WORD      (0 << DAC_C1_DACBFWM_SHIFT)
#  define DAC_C1_DACBFWM_2WORDS     (1 << DAC_C1_DACBFWM_SHIFT)
#  define DAC_C1_DACBFWM_3WORDS     (2 << DAC_C1_DACBFWM_SHIFT)
#  define DAC_C1_DACBFWM_4WORDS     (3 << DAC_C1_DACBFWM_SHIFT)
                                              /* Bits 5-6: Reserved */
#define DAC_C1_DMAEN                (1 << 7)  /* Bit 7:  DMA enable select */

/* DAC Control Register 2 */

#define DAC_C2_DACBFRP_SHIFT        (4)       /* Bits 4-7: DAC buffer read pointer */
#define DAC_C2_DACBFRP_MASK         (15 << DAC_C2_DACBFRP_SHIFT)
#define DAC_C2_DACBFUP_SHIFT        (0)       /* Bits 0-3: DAC buffer upper limit */
#define DAC_C2_DACBFUP_MASK         (15 << DAC_C2_DACBFUP_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_DACE_H */
