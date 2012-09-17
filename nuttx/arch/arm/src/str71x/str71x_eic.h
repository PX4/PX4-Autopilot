/************************************************************************************
 * arch/arm/src/str71x/str71x_eic.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_EIC_H
#define __ARCH_ARM_SRC_STR71X_STR71X_EIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* Enhanced Interupt Controller (EIC) register offsets ******************************/

#define STR71X_EIC_ICR_OFFSET      (0x0000)  /* 32-bits wide */
#define STR71X_EIC_CICR_OFFSET     (0x0004)  /* 32-bits wide */
#define STR71X_EIC_CIPR_OFFSET     (0x0008)  /* 32-bits wide */
#define STR71X_EIC_IVR_OFFSET      (0x0018)  /* 32-bits wide */
#define STR71X_EIC_FIR_OFFSET      (0x001c)  /* 32-bits wide */
#define STR71X_EIC_IER_OFFSET      (0x0020)  /* 32-bits wide */
#define STR71X_EIC_IPR_OFFSET      (0x0040)  /* 32-bits wide */

#define STR71X_EIC_SIR_OFFSET      (0x0060)  /* 32 x 32-bits */
#define STR71X_EIC_SIR0_OFFSET     (0x0060)  /* 32-bits wide */
#define STR71X_EIC_SIR1_OFFSET     (0x0064)  /* 32-bits wide */
#define STR71X_EIC_SIR2_OFFSET     (0x0068)  /* 32-bits wide */
#define STR71X_EIC_SIR3_OFFSET     (0x006c)  /* 32-bits wide */
#define STR71X_EIC_SIR4_OFFSET     (0x0070)  /* 32-bits wide */
#define STR71X_EIC_SIR5_OFFSET     (0x0074)  /* 32-bits wide */
#define STR71X_EIC_SIR6_OFFSET     (0x0078)  /* 32-bits wide */
#define STR71X_EIC_SIR7_OFFSET     (0x007c)  /* 32-bits wide */
#define STR71X_EIC_SIR8_OFFSET     (0x0080)  /* 32-bits wide */
#define STR71X_EIC_SIR9_OFFSET     (0x0084)  /* 32-bits wide */
#define STR71X_EIC_SIR10_OFFSET    (0x0088)  /* 32-bits wide */
#define STR71X_EIC_SIR11_OFFSET    (0x008c)  /* 32-bits wide */
#define STR71X_EIC_SIR12_OFFSET    (0x0090)  /* 32-bits wide */
#define STR71X_EIC_SIR13_OFFSET    (0x0094)  /* 32-bits wide */
#define STR71X_EIC_SIR14_OFFSET    (0x0098)  /* 32-bits wide */
#define STR71X_EIC_SIR15_OFFSET    (0x009c)  /* 32-bits wide */
#define STR71X_EIC_SIR16_OFFSET    (0x00a0)  /* 32-bits wide */
#define STR71X_EIC_SIR17_OFFSET    (0x00a4)  /* 32-bits wide */
#define STR71X_EIC_SIR18_OFFSET    (0x00a8)  /* 32-bits wide */
#define STR71X_EIC_SIR19_OFFSET    (0x00ac)  /* 32-bits wide */
#define STR71X_EIC_SIR20_OFFSET    (0x00b0)  /* 32-bits wide */
#define STR71X_EIC_SIR21_OFFSET    (0x00b4)  /* 32-bits wide */
#define STR71X_EIC_SIR22_OFFSET    (0x00b8)  /* 32-bits wide */
#define STR71X_EIC_SIR23_OFFSET    (0x00bc)  /* 32-bits wide */
#define STR71X_EIC_SIR24_OFFSET    (0x00c0)  /* 32-bits wide */
#define STR71X_EIC_SIR25_OFFSET    (0x00c4)  /* 32-bits wide */
#define STR71X_EIC_SIR26_OFFSET    (0x00c8)  /* 32-bits wide */
#define STR71X_EIC_SIR27_OFFSET    (0x00cc)  /* 32-bits wide */
#define STR71X_EIC_SIR28_OFFSET    (0x00d0)  /* 32-bits wide */
#define STR71X_EIC_SIR29_OFFSET    (0x00d4)  /* 32-bits wide */
#define STR71X_EIC_SIR30_OFFSET    (0x00d8)  /* 32-bits wide */
#define STR71X_EIC_SIR31_OFFSET    (0x00dc)  /* 32-bits wide */

#define STR71X_EIC_NCHANNELS       (32)
#define STR71X_EIC_SIR_BASE        (STR71X_EIC_BASE + STR71X_EIC_SIR_OFFSET)

/* Enhanced Interupt Controller (EIC) registers *************************************/

#define STR71X_EIC_ICR             (STR71X_EIC_BASE + STR71X_EIC_ICR_OFFSET)
#define STR71X_EIC_CICR            (STR71X_EIC_BASE + STR71X_EIC_CICR_OFFSET)
#define STR71X_EIC_CIPR            (STR71X_EIC_BASE + STR71X_EIC_CIPR_OFFSET)
#define STR71X_EIC_IVR             (STR71X_EIC_BASE + STR71X_EIC_IVR_OFFSET)
#define STR71X_EIC_FIR             (STR71X_EIC_BASE + STR71X_EIC_FIR_OFFSET)
#define STR71X_EIC_IER             (STR71X_EIC_BASE + STR71X_EIC_IER_OFFSET)
#define STR71X_EIC_IPR             (STR71X_EIC_BASE + STR71X_EIC_IPR_OFFSET)

#define STR71X_EIC_SIR(n)          (STR71X_EIC_SIR_BASE + ((n) << 2))

#define STR71X_EIC_SIR0            (STR71X_EIC_BASE + STR71X_EIC_SIR0_OFFSET)
#define STR71X_EIC_SIR1            (STR71X_EIC_BASE + STR71X_EIC_SIR1_OFFSET)
#define STR71X_EIC_SIR2            (STR71X_EIC_BASE + STR71X_EIC_SIR2_OFFSET)
#define STR71X_EIC_SIR3            (STR71X_EIC_BASE + STR71X_EIC_SIR3_OFFSET)
#define STR71X_EIC_SIR4            (STR71X_EIC_BASE + STR71X_EIC_SIR4_OFFSET)
#define STR71X_EIC_SIR5            (STR71X_EIC_BASE + STR71X_EIC_SIR5_OFFSET)
#define STR71X_EIC_SIR6            (STR71X_EIC_BASE + STR71X_EIC_SIR6_OFFSET)
#define STR71X_EIC_SIR7            (STR71X_EIC_BASE + STR71X_EIC_SIR7_OFFSET)
#define STR71X_EIC_SIR8            (STR71X_EIC_BASE + STR71X_EIC_SIR8_OFFSET)
#define STR71X_EIC_SIR9            (STR71X_EIC_BASE + STR71X_EIC_SIR9_OFFSET)
#define STR71X_EIC_SIR10           (STR71X_EIC_BASE + STR71X_EIC_SIR10_OFFSET)
#define STR71X_EIC_SIR11           (STR71X_EIC_BASE + STR71X_EIC_SIR11_OFFSET)
#define STR71X_EIC_SIR12           (STR71X_EIC_BASE + STR71X_EIC_SIR12_OFFSET)
#define STR71X_EIC_SIR13           (STR71X_EIC_BASE + STR71X_EIC_SIR13_OFFSET)
#define STR71X_EIC_SIR14           (STR71X_EIC_BASE + STR71X_EIC_SIR14_OFFSET)
#define STR71X_EIC_SIR15           (STR71X_EIC_BASE + STR71X_EIC_SIR15_OFFSET)
#define STR71X_EIC_SIR16           (STR71X_EIC_BASE + STR71X_EIC_SIR16_OFFSET)
#define STR71X_EIC_SIR17           (STR71X_EIC_BASE + STR71X_EIC_SIR17_OFFSET)
#define STR71X_EIC_SIR18           (STR71X_EIC_BASE + STR71X_EIC_SIR18_OFFSET)
#define STR71X_EIC_SIR19           (STR71X_EIC_BASE + STR71X_EIC_SIR19_OFFSET)
#define STR71X_EIC_SIR20           (STR71X_EIC_BASE + STR71X_EIC_SIR20_OFFSET)
#define STR71X_EIC_SIR21           (STR71X_EIC_BASE + STR71X_EIC_SIR21_OFFSET)
#define STR71X_EIC_SIR22           (STR71X_EIC_BASE + STR71X_EIC_SIR22_OFFSET)
#define STR71X_EIC_SIR23           (STR71X_EIC_BASE + STR71X_EIC_SIR23_OFFSET)
#define STR71X_EIC_SIR24           (STR71X_EIC_BASE + STR71X_EIC_SIR24_OFFSET)
#define STR71X_EIC_SIR25           (STR71X_EIC_BASE + STR71X_EIC_SIR25_OFFSET)
#define STR71X_EIC_SIR26           (STR71X_EIC_BASE + STR71X_EIC_SIR26_OFFSET)
#define STR71X_EIC_SIR27           (STR71X_EIC_BASE + STR71X_EIC_SIR27_OFFSET)
#define STR71X_EIC_SIR28           (STR71X_EIC_BASE + STR71X_EIC_SIR28_OFFSET)
#define STR71X_EIC_SIR29           (STR71X_EIC_BASE + STR71X_EIC_SIR29_OFFSET)
#define STR71X_EIC_SIR30           (STR71X_EIC_BASE + STR71X_EIC_SIR30_OFFSET)
#define STR71X_EIC_SIR31           (STR71X_EIC_BASE + STR71X_EIC_SIR31_OFFSET)

/* Register bit settings ************************************************************/

/* Interrupt control register (ICR) bit definitions */

#define STR71X_EICICR_IRQEN        (0x00000001) /* Bit 0: IRQ output enable */
#define STR71X_EICICR_FIQEN        (0x00000002) /* Bit 1: FIQ output enable */

/* Current interrupt channel register (CICR) bit definitions */

#define STR71X_EICCICR_MASK        0x1f         /* Bits: 0-4: CIC */

/* Fast interrupt register (FIR) bit definitions */

#define STR71X_EICFIR_FIE          (0x00000001) /* Bit 0: FIQ channel 1/0 enable */
#define STR71X_EICFIR_FIP          (0x00000002) /* Bit 1: channel 1/0 FIQ pending */

/* Source interrrupt register definitions */

#define STR71X_EICSIR_SIPLMASK     (0x0000000f) /* Bits 0-3: Source interrupt priority level */
#define STR71X_EICSIR_SIVMASK      (0xffff0000) /* Bits 16-31: Source interrupt vector */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_EIC_H */
