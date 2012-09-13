/************************************************************************************
 * arch/z80/src/ez80/chip.h
 * arch/z80/src/chip/chip.h
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

#ifndef __ARCH_Z80_SRC_EZ80_CHIP_H
#define __ARCH_Z80_SRC_EZ80_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Bits in the Z80 FLAGS register ***************************************************/

#define EZ80_C_FLAG            0x01        /* Bit 0: Carry flag */
#define EZ80_N_FLAG            0x02        /* Bit 1: Add/Subtract flag  */
#define EZ80_PV_FLAG           0x04        /* Bit 2: Parity/Overflow flag */
#define EZ80_H_FLAG            0x10        /* Bit 4: Half carry flag */
#define EZ80_Z_FLAG            0x40        /* Bit 5: Zero flag */
#define EZ80_S_FLAG            0x80        /* Bit 7: Sign flag */

/* Include chip-specific regiser definitions */

#if defined(CONFIG_ARCH_CHIP_EZ80F91)
#  include "ez80f91.h"
#elif defined(CONFIG_ARCH_CHIP_EZ80F92)
#  include "ez80f92.h"
#elif defined(CONFIG_ARCH_CHIP_EZ80F93)
#  include "ez80f93.h"
#else
#  error "Register definitions not provided for this chip"
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN uint32_t ez80_systemclock;

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

 #undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* __ARCH_Z80_SRC_EZ80_CHIP_H */
