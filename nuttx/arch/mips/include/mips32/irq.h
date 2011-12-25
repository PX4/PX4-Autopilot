/****************************************************************************
 * arch/mips/include/mips32/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_MIPS_INCLUDE_MIPS32_IRQ_H
#define __ARCH_MIPS_INCLUDE_MIPS32_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/types.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The global pointer (GP) does not need to be saved in the "normal," flat
 * NuttX build.  However, it would be necessary to save the GP if this is
 * a KERNEL build or if NXFLAT is supported.
 */

#undef MIPS32_SAVE_GP
#if defined(CONFIG_NUTTX_KERNEL) || defined(CONFIG_NXFLAT)
#  define MIPS32_SAVE_GP 1
#endif

/* Register save state structure ********************************************/
/* Co processor registers */

#define REG_MFLO_NDX        0
#define REG_MFHI_NDX        1
#define REG_EPC_NDX         2
#define REG_STATUS_NDX      3

/* General pupose registers */
/* $0: Zero register does not need to be saved */
/* $1: at_reg, assembler temporary */

#define REG_R1_NDX          4

/* $2-$3 = v0-v1: Return value registers */

#define REG_R2_NDX          5
#define REG_R3_NDX          6

/* $4-$7 = a0-a3: Argument registers */

#define REG_R4_NDX          7
#define REG_R5_NDX          8
#define REG_R6_NDX          9
#define REG_R7_NDX          10

/* $8-$15 = t0-t7: Volatile registers */

#define REG_R8_NDX          11
#define REG_R9_NDX          12
#define REG_R10_NDX         13
#define REG_R11_NDX         14
#define REG_R12_NDX         15
#define REG_R13_NDX         16
#define REG_R14_NDX         17
#define REG_R15_NDX         18

/* $16-$23 = s0-s7: Static registers */

#define REG_R16_NDX         19
#define REG_R17_NDX         20
#define REG_R18_NDX         21
#define REG_R19_NDX         22
#define REG_R20_NDX         23
#define REG_R21_NDX         24
#define REG_R22_NDX         25
#define REG_R23_NDX         26

/* $24-25 = t8-t9: More Volatile registers */

#define REG_R24_NDX         27
#define REG_R25_NDX         28

/* $26-$27 = ko-k1: Reserved for use in exeption handers.  These do not need
 * to be saved.
 */

/* $28 = gp: Only needs to be saved under conditions where there are
 * multiple, per-thread values for the GP.
 */

#ifdef MIPS32_SAVE_GP

#  define REG_R28_NDX       29

/* $29 = sp:  The value of the stack pointer on return from the exception */

#  define REG_R29_NDX       30

/* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

#  define REG_R30_NDX       31

/* $31 = ra: Return address */

#  define REG_R31_NDX       32
#  define XCPTCONTEXT_REGS  33
#else

/* $29 = sp:  The value of the stack pointer on return from the exception */

#  define REG_R29_NDX       29

/* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

#  define REG_R30_NDX       30

/* $31 = ra: Return address */

#  define REG_R31_NDX       31
#  define XCPTCONTEXT_REGS  32
#endif
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/* In assembly language, values have to be referenced as byte address
 * offsets.  But in C, it is more convenient to reference registers as
 * register save table offsets.
 */

#ifdef __ASSEMBLY__
#  define REG_MFLO          (4*REG_MFLO_NDX)
#  define REG_MFHI          (4*REG_MFHI_NDX)
#  define REG_EPC           (4*REG_EPC_NDX)
#  define REG_STATUS        (4*REG_STATUS_NDX)
#  define REG_R1            (4*REG_R1_NDX)
#  define REG_R2            (4*REG_R2_NDX)
#  define REG_R3            (4*REG_R3_NDX)
#  define REG_R4            (4*REG_R4_NDX)
#  define REG_R5            (4*REG_R5_NDX)
#  define REG_R6            (4*REG_R6_NDX)
#  define REG_R7            (4*REG_R7_NDX)
#  define REG_R8            (4*REG_R8_NDX)
#  define REG_R9            (4*REG_R9_NDX)
#  define REG_R10           (4*REG_R10_NDX)
#  define REG_R11           (4*REG_R11_NDX)
#  define REG_R12           (4*REG_R12_NDX)
#  define REG_R13           (4*REG_R13_NDX)
#  define REG_R14           (4*REG_R14_NDX)
#  define REG_R15           (4*REG_R15_NDX)
#  define REG_R16           (4*REG_R16_NDX)
#  define REG_R17           (4*REG_R17_NDX)
#  define REG_R18           (4*REG_R18_NDX)
#  define REG_R19           (4*REG_R19_NDX)
#  define REG_R20           (4*REG_R20_NDX)
#  define REG_R21           (4*REG_R21_NDX)
#  define REG_R22           (4*REG_R22_NDX)
#  define REG_R23           (4*REG_R23_NDX)
#  define REG_R24           (4*REG_R24_NDX)
#  define REG_R25           (4*REG_R25_NDX)
#  ifdef MIPS32_SAVE_GP
#    define REG_R28         (4*REG_R28_NDX)
#  endif
#  define REG_R29           (4*REG_R29_NDX)
#  define REG_R30           (4*REG_R30_NDX)
#  define REG_R31           (4*REG_R31_NDX)
#else
#  define REG_MFLO          REG_MFLO_NDX
#  define REG_MFHI          REG_MFHI_NDX
#  define REG_EPC           REG_EPC_NDX
#  define REG_STATUS        REG_STATUS_NDX
#  define REG_R1            REG_R1_NDX
#  define REG_R2            REG_R2_NDX
#  define REG_R3            REG_R3_NDX
#  define REG_R4            REG_R4_NDX
#  define REG_R5            REG_R5_NDX
#  define REG_R6            REG_R6_NDX
#  define REG_R7            REG_R7_NDX
#  define REG_R8            REG_R8_NDX
#  define REG_R9            REG_R9_NDX
#  define REG_R10           REG_R10_NDX
#  define REG_R11           REG_R11_NDX
#  define REG_R12           REG_R12_NDX
#  define REG_R13           REG_R13_NDX
#  define REG_R14           REG_R14_NDX
#  define REG_R15           REG_R15_NDX
#  define REG_R16           REG_R16_NDX
#  define REG_R17           REG_R17_NDX
#  define REG_R18           REG_R18_NDX
#  define REG_R19           REG_R19_NDX
#  define REG_R20           REG_R20_NDX
#  define REG_R21           REG_R21_NDX
#  define REG_R22           REG_R22_NDX
#  define REG_R23           REG_R23_NDX
#  define REG_R24           REG_R24_NDX
#  define REG_R25           REG_R25_NDX
#  ifdef MIPS32_SAVE_GP
#    define REG_R28         REG_R28_NDX
#  endif
#  define REG_R29           REG_R29_NDX
#  define REG_R30           REG_R30_NDX
#  define REG_R31           REG_R31_NDX
#endif

/* Now define more user friendly alternative name that can be used either
 * in assembly or C contexts.
 */

/* $1: at_reg, assembler temporary */

#define REG_AT              REG_R1

/* $2-$3 = v0-v1: Return value registers */

#define REG_V0              REG_R2
#define REG_V1              REG_R3

/* $4-$7 = a0-a3: Argument registers */

#define REG_A0              REG_R4
#define REG_A1              REG_R5
#define REG_A2              REG_R6
#define REG_A3              REG_R7

/* $8-$15 = t0-t7: Volatile registers */

#define REG_T0              REG_R8
#define REG_T1              REG_R9
#define REG_T2              REG_R10
#define REG_T3              REG_R11
#define REG_T4              REG_R12
#define REG_T5              REG_R13
#define REG_T6              REG_R14
#define REG_T7              REG_R15

/* $16-$23 = s0-s7: Static registers */

#define REG_S0              REG_R16
#define REG_S1              REG_R17
#define REG_S2              REG_R18
#define REG_S3              REG_R19
#define REG_S4              REG_R20
#define REG_S5              REG_R21
#define REG_S6              REG_R22
#define REG_S7              REG_R23

/* $24-25 = t8-t9: More Volatile registers */

#define REG_T8              REG_R24
#define REG_T9              REG_R25

/* $28 = gp: Only needs to be saved under conditions where there are
 * multiple, per-thread values for the GP.
 */

#ifdef MIPS32_SAVE_GP
#  define REG_GP            REG_R28
#endif

/* $29 = sp:  The value of the stack pointer on return from the exception */

#define REG_SP              REG_R29

/* $30 = either s8 or fp:  Depends if a frame pointer is used or not */

#define REG_S8              REG_R30
#define REG_FP              REG_R30

/* $31 = ra: Return address */

#define REG_RA              REG_R31

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct xcptcontext
{
  /* The following function pointer is non-NULL if there are pending signals
   * to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These additional register save locations are used to implement the
   * signal delivery trampoline.
   */

  uint32_t saved_epc;    /* Trampoline PC */
  uint32_t saved_status; /* Status with interrupts disabled. */
#endif

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: cp0_getstatus
 *
 * Description:
 *   Read the CP0 STATUS register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline irqstate_t cp0_getstatus(void)
{
  register irqstate_t status;
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t mfc0   %0, $12, 0\n"           /* Get CP0 status register */
      "\t.set    pop\n"
      : "=r" (status)
      :
      : "memory"
    );

  return status;
}

/****************************************************************************
 * Name: cp0_putstatus
 *
 * Description:
 *   Write the CP0 STATUS register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp0_putstatus(irqstate_t status)
{
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t.set    noreorder\n"
      "\tmtc0   %0, $12, 0\n"            /* Set the status to the provided value */
      "\tnop\n"                          /* MTC0 status hazard: */
      "\tnop\n"                          /* Recommended spacing: 3 */
      "\tnop\n"
      "\tnop\n"                          /* Plus one for good measure */
      "\t.set    pop\n"
      : 
      : "r" (status)
      : "memory"
    );
}

/****************************************************************************
 * Name: cp0_getcause
 *
 * Description:
 *   Get the CP0 CAUSE register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t cp0_getcause(void)
{
  register uint32_t cause;
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t mfc0   %0, $13, 0\n"           /* Get CP0 cause register */
      "\t.set    pop\n"
      : "=r" (cause)
      :
      : "memory"
    );

  return cause;
}

/****************************************************************************
 * Name: cp0_putcause
 *
 * Description:
 *   Write the CP0 CAUSE register
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void cp0_putcause(uint32_t cause)
{
  __asm__ __volatile__
    (
      "\t.set    push\n"
      "\t.set    noat\n"
      "\t.set    noreorder\n"
      "\tmtc0   %0, $13, 0\n"            /* Set the cause to the provided value */
      "\t.set    pop\n"
      : 
      : "r" (cause)
      : "memory"
    );
}

/****************************************************************************
 * Public Variables
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

/****************************************************************************
 * Name: irqsave
 *
 * Description:
 *   Save the current interrupt state and disable interrupts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Interrupt state prior to disabling interrupts.
 *
 ****************************************************************************/

EXTERN irqstate_t irqsave(void);

/****************************************************************************
 * Name: irqrestore
 *
 * Description:
 *   Restore the previous interrupt state (i.e., the one previously returned
 *   by irqsave())
 *
 * Input Parameters:
 *   state - The interrupt state to be restored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void irqrestore(irqstate_t irqtate);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY */
#endif /* __ARCH_MIPS_INCLUDE_MIPS32_IRQ_H */

