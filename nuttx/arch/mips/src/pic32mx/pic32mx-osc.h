/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx-osc.h
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
 ****************************************************************************/

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OSC_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Register Offsets *********************************************************/

#define PIC32MX_OSCCON_OFFSET    0x0000 /* Oscillator control register offset */
#define PIC32MX_OSCTUN_OFFSET    0x0010 /* FRC tuning register offset */

/* Register Addresses *******************************************************/

#define PIC32MX_OSCCON           (PIC32MX_OSC_K1BASE+PIC32MX_OSCCON_OFFSET)
#define PIC32MX_OSCTUN           (PIC32MX_OSC_K1BASE+PIC32MX_OSCTUN_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Oscillator control register offset */

#define OSCCON_OSWEN            (1 << 0)  /* Bit 0: Oscillator switch enable */
#define OSCCON_SOSCEN           (1 << 1)  /* Bit 1: 32.768kHz secondary oscillator enable */
#define OSCCON_UFRCEN           (1 << 2)  /* Bit 2: USB FRC clock enable */
#define OSCCON_CF               (1 << 3)  /* Bit 3: Clock fail detect */
#define OSCCON_SLPEN            (1 << 4)  /* Bit 4: Sleep mode enable */
#define OSCCON_SLOCK            (1 << 5)  /* Bit 5: PLL lock status */
#define OSCCON_ULOCK            (1 << 6)  /* Bit 6: USB PLL lock status */
#define OSCCON_CLKLOCK          (1 << 7)  /* Bit 7: Clock selection lock enable */
#define OSCCON_NOSC_SHIFT       (8)       /* Bits 8-10: New oscillator selection */
#define OSCCON_NOSC_MASK        (7 << OSCCON_NOSC_SHIFT)
#  define OSCCON_NOSC_FRC       (0 << OSCCON_NOSC_SHIFT) /* FRC oscillator */
#  define OSCCON_NOSC_FRCPLL    (1 << OSCCON_NOSC_SHIFT) /* FRC w/PLL postscaler */
#  define OSCCON_NOSC_POSC      (2 << OSCCON_NOSC_SHIFT) /* Primary oscillator */
#  define OSCCON_NOSC_POSCPLL   (3 << OSCCON_NOSC_SHIFT) /* Primary oscillator with PLL */
#  define OSCCON_NOSC_SOSC      (4 << OSCCON_NOSC_SHIFT) /* Secondary oscillator */
#  define OSCCON_NOSC_LPRC      (5 << OSCCON_NOSC_SHIFT) /* Low power RC oscillator */
#  define OSCCON_NOSC_FRCDIV16  (6 << OSCCON_NOSC_SHIFT) /* FRC divided by 16 */
#  define OSCCON_NOSC_FRCDIV    (7 << OSCCON_NOSC_SHIFT) /* FRC dived by FRCDIV */
#define OSCCON_COSC_SHIFT       (12)      /* Bits 12-14: Current oscillator selection */
#define OSCCON_COSC_MASK        (7 << OSCCON_COSC_SHIFT)
#  define OSCCON_COSC_FRC       (0 << OSCCON_COSC_SHIFT) /* FRC oscillator */
#  define OSCCON_COSC_FRCPLL    (1 << OSCCON_COSC_SHIFT) /* FRC w/PLL postscaler */
#  define OSCCON_COSC_POSC      (2 << OSCCON_COSC_SHIFT) /* Primary oscillator */
#  define OSCCON_COSC_POSCPLL   (3 << OSCCON_COSC_SHIFT) /* Primary oscillator with PLL */
#  define OSCCON_COSC_SOSC      (4 << OSCCON_COSC_SHIFT) /* Secondary oscillator */
#  define OSCCON_COSC_LPRC      (5 << OSCCON_COSC_SHIFT) /* Low power RC oscillator */
#  define OSCCON_COSC_FRCDIV16  (6 << OSCCON_COSC_SHIFT) /* FRC divided by 16 */
#  define OSCCON_COSC_FRCDIV    (7 << OSCCON_COSC_SHIFT) /* FRC dived by FRCDIV */
#define OSCCON_PLLMULT_SHIFT    (16)      /* Bits 16-18: PLL multiplier */
#define OSCCON_PLLMULT_MASK     (7 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL15  (0 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL16  (1 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL17  (2 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL18  (3 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL19  (4 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL20  (5 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL21  (6 << OSCCON_PLLMULT_SHIFT)
#  define OSCCON_PLLMULT_MUL24  (7 << OSCCON_PLLMULT_SHIFT)
#define OSCCON_PBDIV_SHIFT      (19)      /* Bits 19-20: PBVLK divisor */
#define OSCCON_PBDIV_SMASK      (3 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV1     (0 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV2     (1 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV4     (2 << OSCCON_PBDIV_SHIFT)
#  define OSCCON_PBDIV_DIV8     (3 << OSCCON_PBDIV_SHIFT)
#define OSCCON_SOSCRDY          (1 << 22) /* Bit 22: Secondary oscillator ready */
#define OSCCON_FRCDIV_SHIFT     (24)      /* Bits 24-26: FRC oscillator divider */
#define OSCCON_FRCDIV_MASK      (7 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV1    (0 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV2    (1 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV4    (2 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV8    (3 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV16   (4 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV32   (5 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV64   (6 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV256  (7 << OSCCON_FRCDIV_SHIFT)
#define OSCCON_PLL0DIV_SHIFT    (27)      /* Bits 27-29: Output divider for PLL */
#define OSCCON_PLL0DIV_MASK     (7 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV1   (0 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV2   (1 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV4   (2 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV8   (3 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV16  (4 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV32  (5 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV64  (6 << OSCCON_PLL0DIV_SHIFT)
#  define OSCCON_PLL0DIV_DIV256 (7 << OSCCON_PLL0DIV_SHIFT)

/* FRC tuning register offset (6-bit, signed twos complement) */

#define OSCTUN_SHIFT         (0)       /* Bits 0-5: FRC tuning bits */
#define OSCTUN_MASK          (0x3f << OSCTUN_SHIFT)
#  define OSCTUN_MIN         (0x20 << OSCTUN_SHIFT)
#  define OSCTUN_CENTER      (0x00 << OSCTUN_SHIFT)
#  define OSCTUN_MAX         (0x1f << OSCTUN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_OSC_H */
