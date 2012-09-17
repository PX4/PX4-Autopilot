/************************************************************************************
 * arch/hc/include/hcs12/irq.h
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
 ************************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_HC_INCLUDE_HCS12_IRQ_H
#define __ARCH_HC_INCLUDE_HCS12_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* CCR bit definitions */

#define HCS12_CCR_C (1 << 0) /* Bit 0: Carry/Borrow status bit */
#define HCS12_CCR_V (1 << 1) /* Bit 1: Two’s complement overflow status bit */
#define HCS12_CCR_Z (1 << 2) /* Bit 2: Zero status bit */
#define HCS12_CCR_N (1 << 3) /* Bit 3: Negative status bit */
#define HCS12_CCR_I (1 << 4) /* Bit 4: Maskable interrupt control bit */
#define HCS12_CCR_H (1 << 5) /* Bit 5: Half-carry status bit */
#define HCS12_CCR_X (1 << 6) /* Bit 6: Non-maskable interrupt control bit */
#define HCS12_CCR_S (1 << 7) /* Bit 7: STOP instruction control bit */

/************************************************************************************
 *	Register state save strucure
 *   Low Address        <-- SP after state save
 *                [PPAGE]
 *                [soft regisers]
 *                XYH
 *                XYL
 *                ZH
 *                ZL
 *                TMPH
 *                TMPL
 *                FRAMEH
 *                FRAMEL
 *                SP    <-- SP after interrupt
 *                CCR
 *                B
 *                A
 *                XH
 *                XL
 *                YH
 *                YL
 *                PCH
 *   High Address PCL    <-- SP before interrupt
 *
 ************************************************************************************/

/* Byte offsets */
/* PPAGE register (only in banked mode) */

#ifndef CONFIG_HCS12_NONBANKED
#  define REG_PPAGE          0
#  define REG_FIRST_SOFTREG  1
#else
#  define REG_FIRST_SOFTREG  0
#endif

/* Soft registers (as configured) */

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#elif CONFIG_HCS12_MSOFTREGS == 2
#  define REG_SOFTREG1       REG_FIRST_SOFTREG
#  define REG_SOFTREG2       (REG_FIRST_SOFTREG+2)
#  define REG_FIRST_HARDREG  (REG_FIRST_SOFTREG+4)
#elif CONFIG_HCS12_MSOFTREGS == 1
#  define REG_SOFTREG1       REG_FIRST_SOFTREG
#  define REG_FIRST_HARDREG  (REG_FIRST_SOFTREG+2)
#else
#  define REG_FIRST_HARDREG  REG_FIRST_SOFTREG
#endif

#define REG_XY               REG_FIRST_HARDREG
#define REG_Z                (REG_FIRST_HARDREG+2)
#  define REG_ZH             (REG_FIRST_HARDREG+2)
#  define REG_ZL             (REG_FIRST_HARDREG+3)
#define REG_TMP              (REG_FIRST_HARDREG+4)
#  define REG_TMPH           (REG_FIRST_HARDREG+4)
#  define REG_TMPL           (REG_FIRST_HARDREG+5)
#define REG_FRAME            (REG_FIRST_HARDREG+6)
#  define REG_FRAMEH         (REG_FIRST_HARDREG+6)
#  define REG_FRAMEL         (REG_FIRST_HARDREG+7)

/* Stack pointer before the interrupt */

#define REG_SP               (REG_FIRST_HARDREG+8)
#  define REG_SPH            (REG_FIRST_HARDREG+8)
#  define REG_SPL            (REG_FIRST_HARDREG+9)

/* On entry into an I- or X-interrupt, into an SWI, or into an undefined instruction
 * interrupt, the stack frame created by hardware looks like:
 *
 * Low Address       <-- SP after interrupt
 *              CCR
 *              B
 *              A
 *              XH
 *              XL
 *              YH
 *              YL
 *              PCH
 * High Address PCL  <-- SP before interrupt
 */

#define REG_CCR              (REG_FIRST_HARDREG+10)
#define REG_BA               (REG_FIRST_HARDREG+11)
#  define REG_B              (REG_FIRST_HARDREG+11)
#  define REG_A              (REG_FIRST_HARDREG+12)
#define REG_X                (REG_FIRST_HARDREG+13)
#  define REG_XH             (REG_FIRST_HARDREG+13)
#  define REG_XL             (REG_FIRST_HARDREG+14)
#define REG_Y                (REG_FIRST_HARDREG+15)
#  define REG_YH             (REG_FIRST_HARDREG+15)
#  define REG_YL             (REG_FIRST_HARDREG+16)
#define REG_PC               (REG_FIRST_HARDREG+17)
#  define REG_PCH            (REG_FIRST_HARDREG+17)
#  define REG_PCL            (REG_FIRST_HARDREG+18)

#define TOTALFRAME_SIZE      (REG_FIRST_HARDREG+17)
#define INTFRAME_SIZE        9
#define XCPTCONTEXT_REGS     TOTALFRAME_SIZE

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This structure defines the way the registers are stored. */

#ifndef __ASSEMBLY__
struct xcptcontext
{
  uint8_t regs[XCPTCONTEXT_REGS];
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Enable/Disable interrupts */

#define ienable()  __asm("cli");
#define idisable() __asm("orcc #0x10")
#define xenable()  __asm("andcc #0xbf")
#define xdisable() __asm("orcc #0x40")

/* Get the current value of the stack pointer */

static inline uint16_t up_getsp(void)
{
  uint16_t ret;
  __asm__
  (
    "\tsts %0\n"
	: "=m"(ret) :
  );
  return ret;
}

/* Get the current value of the CCR */

static inline irqstate_t up_getccr(void)
{
  irqstate_t ccr;
  __asm__
  (
    "\ttpa\n"
    "\tstaa %0\n"
	: "=m"(ccr) :
  );
  return ccr;
}

/* Save the current interrupt enable state & disable IRQs */

static inline irqstate_t irqsave(void)
{
  irqstate_t ccr;
  __asm__
  (
    "\ttpa\n"
    "\tstaa %0\n"
    "\torcc #0x50\n"
	: "=m"(ccr) :
  );
  return ccr;
}

/* Restore saved interrupt state */

static inline void irqrestore(irqstate_t flags)
{
  /* Should interrupts be enabled? */

  if ((flags & HCS12_CCR_I) == 0)
    {
      /* Yes.. unmask I- and Z-interrupts */

      __asm("andcc #0xaf");
    }
}

/* System call */

static inline void system_call3(unsigned int nbr, uintptr_t parm1,
			                    uintptr_t parm2, uintptr_t parm3)
{
  /* To be provided */
  /* __asm("swi") */
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_HC_INCLUDE_HCS12_IRQ_H */
