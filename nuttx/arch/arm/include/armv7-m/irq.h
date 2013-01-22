/****************************************************************************
 * arch/arm/include/armv7-m/irq.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

/* Included implementation-dependent register save structure layouts */

#ifdef CONFIG_ARMV7M_CMNVECTOR
#  include <arch/armv7-m/irq_cmnvector.h>
#else
#  include <arch/armv7-m/irq_lazyfpu.h>
#endif

#ifdef CONFIG_ARMV7M_USEBASEPRI
#  include <arch/chip/chip.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* Get/set the primask register */

static inline uint8_t getprimask(void) inline_function;
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

static inline void setprimask(uint32_t primask) inline_function;
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

static inline uint8_t getbasepri(void) inline_function;
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

static inline void setbasepri(uint32_t basepri) inline_function;
static inline void setbasepri(uint32_t basepri)
{
  __asm__ __volatile__
    (
      "\tmsr basepri, %0\n"
      :
      : "r" (basepri)
      : "memory");
}

/* Disable IRQs */

static inline void irqdisable(void) inline_function;
static inline void irqdisable(void)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI
  setbasepri(NVIC_SYSH_DISABLE_PRIORITY);
#else
  __asm__ __volatile__ ("\tcpsid  i\n");
#endif
}

/* Save the current primask state & disable IRQs */

static inline irqstate_t irqsave(void) inline_function;
static inline irqstate_t irqsave(void)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI

  uint8_t basepri = getbasepri();
  setbasepri(NVIC_SYSH_DISABLE_PRIORITY);
  return basepri;

#else

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
#endif
}

/* Enable IRQs */

static inline void irqenable(void) inline_function;
static inline void irqenable(void)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI
  setbasepri(NVIC_SYSH_PRIORITY_MIN);
#else
  __asm__ __volatile__ ("\tcpsie  i\n");
#endif
}

/* Restore saved primask state */

static inline void irqrestore(irqstate_t flags) inline_function;
static inline void irqrestore(irqstate_t flags)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI
  setbasepri(flags);
#else
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
      : "r" (flags)
      : "memory");
#endif
}

/* Get/set IPSR */

static inline uint32_t getipsr(void) inline_function;
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

static inline void setipsr(uint32_t ipsr) inline_function;
static inline void setipsr(uint32_t ipsr)
{
  __asm__ __volatile__
    (
      "\tmsr ipsr, %0\n"
      :
      : "r" (ipsr)
      : "memory");
}

/* Get/set CONTROL */

static inline uint32_t getcontrol(void) inline_function;
static inline uint32_t getcontrol(void)
{
  uint32_t control;
  __asm__ __volatile__
    (
     "\tmrs  %0, control\n"
     : "=r" (control)
     :
     : "memory");

  return control;
}

static inline void setcontrol(uint32_t control) inline_function;
static inline void setcontrol(uint32_t control)
{
  __asm__ __volatile__
    (
      "\tmsr control, %0\n"
      :
      : "r" (control)
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

