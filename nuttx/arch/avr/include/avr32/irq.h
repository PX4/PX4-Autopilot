/****************************************************************************
 * arch/avr/include/avr32/irq.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_AVR_INCLUDE_AVR32_IRQ_H
#define __ARCH_AVR_INCLUDE_AVR32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr32/avr32.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* General notes about the AVR32 ABI:
 *
 * Scratch/Volatile Registers: r8-r12
 * Preserved/Static Registers: r0-r7
 * Parameter Passing:          r12-R8 (in that order)
 */

/* Register state save array indices.
 *
 * The following registers are saved by the AVR32 hardware (for the case of
 * interrupts only).  Note the registers are order in the opposite order the
 * they appear in memory (i.e., in the order of increasing address) because
 * this makes it easier to following the ordering of pushing on a push-down
 * stack.
 */
 
#define REG_R8           16
#define REG_R9           15
#define REG_R10          14
#define REG_R11          13
#define REG_R12          12
#define REG_R14          11
#define REG_R15          10
#define REG_SR            9

#define REG_LR           REG_R14
#define REG_PC           REG_R15

/* Additional registers saved in order have the full CPU context */

#define REG_R13           8
#define REG_SP           REG_R13

#define REG_R0            7
#define REG_R1            6
#define REG_R2            5
#define REG_R3            4
#define REG_R4            3
#define REG_R5            2
#define REG_R6            1
#define REG_R7            0

/* Size of the register state save array (in 32-bit words) */

#define INTCONTEXT_REGS   8 /* r8-r12, lr, pc, sr */
#define XCPTCONTEXT_REGS 17 /* Plus r0-r7, sp */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored. */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of PC and SR used during signal processing.*/

  uint32_t saved_pc;
  uint32_t saved_sr;
#endif

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Read the AVR32 status register */

static inline uint32_t avr32_sr(void)
{
  uint32_t sr;
  __asm__ __volatile__ (
    "mfsr\t%0,%1\n\t"
    : "=r" (sr)
    : "i" (AVR32_SR)
  );
  return sr;
}

/* Read the interrupt vector base address */

static inline uint32_t avr32_evba(void)
{
  uint32_t evba;
  __asm__ __volatile__ (
    "mfsr\t%0,%1\n\t"
    : "=r" (evba)
    : "i" (AVR32_EVBA)
  );
  return evba;
}

/* Save the current interrupt enable state & disable all interrupts */

static inline irqstate_t irqsave(void)
{
  irqstate_t sr = (irqstate_t)avr32_sr();
  __asm__ __volatile__ (
    "ssrf\t%0\n\t"
    "nop\n\t"
    "nop"
    :
    : "i" (AVR32_SR_GM_SHIFT)
  );
  return sr;
}

/* Restore saved interrupt state */

static inline void irqrestore(irqstate_t flags)
{
  if ((flags & AVR32_SR_GM_MASK) == 0)
    {
      __asm__ __volatile__ (
        "csrf\t%0\n\t"
        "nop\n\t"
        "nop"
        :
        : "i" (AVR32_SR_GM_SHIFT)
      );
    }
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_AVR_INCLUDE_AVR32_IRQ_H */

