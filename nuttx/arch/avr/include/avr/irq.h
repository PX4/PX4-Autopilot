/****************************************************************************
 * arch/avr/include/avr/irq.h
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

#ifndef __ARCH_AVR_INCLUDE_AVR_IRQ_H
#define __ARCH_AVR_INCLUDE_AVR_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <arch/avr/avr.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Register state save array indices */

#define REG_SPH           0 /* Stack pointer on exception entry */
#define REG_SPL           1
#define REG_R27           2 /* r26-r27 */
#define REG_R26           3
#define REG_R31           4 /* r18-r31 */
#define REG_R30           5
#define REG_R29           6
#define REG_R28           7
#define REG_R23           8 /* r2-r23 */
#define REG_R22           9
#define REG_R21          10
#define REG_R20          11
#define REG_R19          12
#define REG_R18          13
#define REG_R17          14
#define REG_R16          15
#define REG_R15          16
#define REG_R14          17
#define REG_R13          18
#define REG_R12          19
#define REG_R11          20
#define REG_R10          21
#define REG_R9           22
#define REG_R8           23
#define REG_R7           24
#define REG_R6           25
#define REG_R5           26
#define REG_R4           27
#define REG_R3           28
#define REG_R2           29
#define REG_R1           30 /* r1 - the "zero" register */
#define REG_R0           31 /* r0 */
#define REG_SREG         32 /* Status register */
#define REG_R25          33 /* r24-r25 */
#define REG_R24          34

/* The program counter is automatically pushed when the interrupt occurs */

#define REG_PCH          35 /* PC */
#define REG_PCL          36

/* Size of the register state save array (in bytes) */

#define XCPTCONTEXT_REGS 37

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

  uint8_t saved_pcl;
  uint8_t saved_pch;
  uint8_t saved_sreg;
#endif

  /* Register save area */

  uint8_t regs[XCPTCONTEXT_REGS];
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Read/write the SREG */

static inline irqstate_t getsreg(void)
{
  irqstate_t sreg;
  asm volatile ("in %0, __SREG__" : "=r" (sreg) :: );
  return sreg;
}

static inline void putsreg(irqstate_t sreg)
{
  asm volatile ("out __SREG__, %s" : : "r" (sreg) : );
}

/* Interrupt enable/disable */

static inline void irqenable()
{
  asm volatile ("sei" ::);
}

static inline void irqdisable()
{
  asm volatile ("cli" ::);
}

/* Save the current interrupt enable state & disable all interrupts */

static inline irqstate_t irqsave(void)
{
  irqstate_t sreg;
  asm volatile
    (
      "\tin %0, __SREG__\n"
	  "\tcli\n"
	  : "=&r" (sreg) :: 
	);
  return sreg;
}

/* Restore saved interrupt state */

static inline void irqrestore(irqstate_t flags)
{
  asm volatile ("out __SREG__, %0" : : "r" (flags) : );
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

#endif /* __ARCH_AVR_INCLUDE_AVR_IRQ_H */

