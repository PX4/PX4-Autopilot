/****************************************************************************
 * arch/arm/include/armv7-m/irq.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_H
#define __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format: */

/* The following additional registers are stored by the interrupt handling
 * logic.
 */

#define REG_R13             (0)  /* R13 = SP at time of interrupt */
#define REG_PRIMASK         (1)  /* PRIMASK */
#define REG_R4              (2)  /* R4 */
#define REG_R5              (3)  /* R5 */
#define REG_R6              (4)  /* R6 */
#define REG_R7              (5)  /* R7 */
#define REG_R8              (6)  /* R8 */
#define REG_R9              (7)  /* R9 */
#define REG_R10             (8)  /* R10 */
#define REG_R11             (9)  /* R11 */

#ifdef CONFIG_NUTTX_KERNEL
#  define REG_EXC_RETURN    (10) /* EXC_RETURN */
#  define SW_INT_REGS       (11)
#else
#  define SW_INT_REGS       (10)
#endif

/* If the MCU supports a floating point unit, then it will be necessary
 * to save the state of the FPU status register and data registers on
 * each context switch.  These registers are not saved during interrupt
 * level processing, however. So, as a consequence, floating point
 * operations may NOT be performed in interrupt handlers.
 *
 * The FPU provides an extension register file containing 32 single-
 * precision registers. These can be viewed as:
 *
 * - Sixteen 64-bit doubleword registers, D0-D15
 * - Thirty-two 32-bit single-word registers, S0-S31
 *   S<2n> maps to the least significant half of D<n>
 *   S<2n+1> maps to the most significant half of D<n>.
 */

#ifdef CONFIG_ARCH_FPU
#  define REG_D0            (SW_INT_REGS+0)  /* D0 */
#  define REG_S0            (SW_INT_REGS+0)  /* S0 */
#  define REG_S1            (SW_INT_REGS+1)  /* S1 */
#  define REG_D1            (SW_INT_REGS+2)  /* D1 */
#  define REG_S2            (SW_INT_REGS+2)  /* S2 */
#  define REG_S3            (SW_INT_REGS+3)  /* S3 */
#  define REG_D2            (SW_INT_REGS+4)  /* D2 */
#  define REG_S4            (SW_INT_REGS+4)  /* S4 */
#  define REG_S5            (SW_INT_REGS+5)  /* S5 */
#  define REG_D3            (SW_INT_REGS+6)  /* D3 */
#  define REG_S6            (SW_INT_REGS+6)  /* S6 */
#  define REG_S7            (SW_INT_REGS+7)  /* S7 */
#  define REG_D4            (SW_INT_REGS+8)  /* D4 */
#  define REG_S8            (SW_INT_REGS+8)  /* S8 */
#  define REG_S9            (SW_INT_REGS+9)  /* S9 */
#  define REG_D5            (SW_INT_REGS+10) /* D5 */
#  define REG_S10           (SW_INT_REGS+10) /* S10 */
#  define REG_S11           (SW_INT_REGS+11) /* S11 */
#  define REG_D6            (SW_INT_REGS+12) /* D6 */
#  define REG_S12           (SW_INT_REGS+12) /* S12 */
#  define REG_S13           (SW_INT_REGS+13) /* S13 */
#  define REG_D7            (SW_INT_REGS+14) /* D7 */
#  define REG_S14           (SW_INT_REGS+14) /* S14 */
#  define REG_S15           (SW_INT_REGS+15) /* S15 */
#  define REG_D8            (SW_INT_REGS+16) /* D8 */
#  define REG_S16           (SW_INT_REGS+16) /* S16 */
#  define REG_S17           (SW_INT_REGS+17) /* S17 */
#  define REG_D9            (SW_INT_REGS+18) /* D9 */
#  define REG_S18           (SW_INT_REGS+18) /* S18 */
#  define REG_S19           (SW_INT_REGS+19) /* S19 */
#  define REG_D10           (SW_INT_REGS+20) /* D10 */
#  define REG_S20           (SW_INT_REGS+20) /* S20 */
#  define REG_S21           (SW_INT_REGS+21) /* S21 */
#  define REG_D11           (SW_INT_REGS+22) /* D11 */
#  define REG_S22           (SW_INT_REGS+22) /* S22 */
#  define REG_S23           (SW_INT_REGS+23) /* S23 */
#  define REG_D12           (SW_INT_REGS+24) /* D12 */
#  define REG_S24           (SW_INT_REGS+24) /* S24 */
#  define REG_S25           (SW_INT_REGS+25) /* S25 */
#  define REG_D13           (SW_INT_REGS+26) /* D13 */
#  define REG_S26           (SW_INT_REGS+26) /* S26 */
#  define REG_S27           (SW_INT_REGS+27) /* S27 */
#  define REG_D14           (SW_INT_REGS+28) /* D14 */
#  define REG_S28           (SW_INT_REGS+28) /* S28 */
#  define REG_S29           (SW_INT_REGS+29) /* S29 */
#  define REG_D15           (SW_INT_REGS+30) /* D15 */
#  define REG_S30           (SW_INT_REGS+30) /* S30 */
#  define REG_S31           (SW_INT_REGS+31) /* S31 */
#  define REG_FPSCR         (SW_INT_REGS+32) /* Floating point status and control */
#  define SW_FPU_REGS       (33)
#else
#  define SW_FPU_REGS       (0)
#endif

/* The total number of registers saved by software */

#define SW_XCPT_REGS        (SW_INT_REGS + SW_FPU_REGS)
#define SW_XCPT_SIZE        (4 * SW_XCPT_REGS)

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define REG_R0              (SW_XCPT_REGS+0) /* R0 */
#define REG_R1              (SW_XCPT_REGS+1) /* R1 */
#define REG_R2              (SW_XCPT_REGS+2) /* R2 */
#define REG_R3              (SW_XCPT_REGS+3) /* R3 */
#define REG_R12             (SW_XCPT_REGS+4) /* R12 */
#define REG_R14             (SW_XCPT_REGS+5) /* R14 = LR */
#define REG_R15             (SW_XCPT_REGS+6) /* R15 = PC */
#define REG_XPSR            (SW_XCPT_REGS+7) /* xPSR */

#define HW_XCPT_REGS        (8)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

#define XCPTCONTEXT_REGS    (HW_XCPT_REGS + SW_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (HW_XCPT_SIZE + SW_XCPT_SIZE)

/* Alternate register names */

#define REG_A1              REG_R0
#define REG_A2              REG_R1
#define REG_A3              REG_R2
#define REG_A4              REG_R3
#define REG_V1              REG_R4
#define REG_V2              REG_R5
#define REG_V3              REG_R6
#define REG_V4              REG_R7
#define REG_V5              REG_R8
#define REG_V6              REG_R9
#define REG_V7              REG_R10
#define REG_SB              REG_R9
#define REG_SL              REG_R10
#define REG_FP              REG_R11
#define REG_IP              REG_R12
#define REG_SP              REG_R13
#define REG_LR              REG_R14
#define REG_PC              REG_R15

/* The PIC register is usually R10. It can be R9 is stack checking is enabled
 * or if the user changes it with -mpic-register on the GCC command line.
 */

#define REG_PIC             REG_R10

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of LR, PRIMASK, and xPSR used during
   * signal processing.
   */

  uint32_t saved_pc;
  uint32_t saved_primask;
  uint32_t saved_xpsr;
#endif

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Disable IRQs */

static inline void irqdisable(void)
{
  __asm__ __volatile__ ("\tcpsid  i\n");
}

/* Save the current primask state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  unsigned short primask;

  /* Return the current value of primask register and set
   * bit 0 of the primask register to disable interrupts
   */

  __asm__ __volatile__
    (
     "\tmrs    %0, primask\n"
     "\tcpsid  i\n"
     : "=r" (primask)
     :
     : "memory");
  return primask;
}

/* Enable IRQs */

static inline void irqenable(void)
{
  __asm__ __volatile__ ("\tcpsie  i\n");
}

/* Restore saved primask state */

static inline void irqrestore(irqstate_t primask)
{
  /* If bit 0 of the primask is 0, then we need to restore
   * interupts.
   */

  __asm__ __volatile__
    (
      "\ttst    %0, #1\n"
      "\tbne    1f\n"
      "\tcpsie  i\n"
      "1:\n"
      :
      : "r" (primask)
      : "memory");
}

/* Get/set the primask register */

static inline uint8_t getprimask(void)
{
  uint32_t primask;
  __asm__ __volatile__
    (
     "\tmrs  %0, primask\n"
     : "=r" (primask)
     :
     : "memory");
  return (uint8_t)primask;
}

static inline void setprimask(uint32_t primask)
{
  __asm__ __volatile__
    (
      "\tmsr primask, %0\n"
      :
      : "r" (primask)
      : "memory");
}

/* Get/set the basepri register */

static inline uint8_t getbasepri(void)
{
  uint32_t basepri;
  __asm__ __volatile__
    (
     "\tmrs  %0, basepri\n"
     : "=r" (basepri)
     :
     : "memory");
  return (uint8_t)basepri;
}

static inline void setbasepri(uint32_t basepri)
{
  __asm__ __volatile__
    (
      "\tmsr basepri, %0\n"
      :
      : "r" (basepri)
      : "memory");
}

/* Get/set IPSR */

static inline uint32_t getipsr(void)
{
  uint32_t ipsr;
  __asm__ __volatile__
    (
     "\tmrs  %0, ipsr\n"
     : "=r" (ipsr)
     :
     : "memory");
  return ipsr;
}

static inline void setipsr(uint32_t ipsr)
{
  __asm__ __volatile__
    (
      "\tmsr ipsr, %0\n"
      :
      : "r" (ipsr)
      : "memory");
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

#endif /* __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_H */

