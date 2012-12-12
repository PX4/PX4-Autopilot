/****************************************************************************
 * arch/z80/include/z180/irq.h
 * arch/chip/irq.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 * through nuttx/irq.h (via arch/irq.h)
 */

#ifndef __ARCH_Z80_INCLUDE_Z180_IRQ_H
#define __ARCH_Z80_INCLUDE_Z180_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Z180 Interrupts */
/* Resets */
                                /* RST 0 is the power-up interrupt vector */
#define Z180_RST1          (0)  /* RST 1 */
#define Z180_RST2          (1)  /* RST 2 */
#define Z180_RST3          (2)  /* RST 3 */
#define Z180_RST4          (3)  /* RST 4 */
#define Z180_RST5          (4)  /* RST 5 */
#define Z180_RST6          (5)  /* RST 6 */
#define Z180_RST7          (6)  /* RST 7 */

/* TRAP Interrupt
 *
 * The Z8X180 generates a non-maskable TRAP interrupt when an undefined Op
 * Code fetch occurs. When a TRAP interrupt occurs the Z8X180 operates as
 * follows:
 *
 * 1. The TRAP bit in the Interrupt TRAP/Control (ITC) register is set to 1.
 * 2. The current PC (Program Counter) is saved on the stack.
 * 3. The Z8X180 vectors to logical address 0 (which may or may not be the
 *    same as reset which vectors to physical address 0x00000).
 *
 * The state of the UFO (Undefined Fetch Object) bit in ITC allows TRAP
 * manipulation software to correctly adjust the stacked PC, depending on
 * whether the second or third byte of the Op Code generated the TRAP. If
 * UFO is 0, the starting address of the invalid instruction is equal to
 * the stacked PC-1. If UFO is 1, the starting address of the invalid
 * instruction is equal to the stacked PC-2.
 */

#define Z180_TRAP          (7)

/* INT0
 *
 * INT0 (only) has 3 different software programmable interrupt response
 * modes—Mode 0, Mode 1 and Mode 2.
 *
 * - INT0 Mode 0. During the interrupt acknowledge cycle, an instruction
 *   is fetched from the data bus (DO–D7) at the rising edge of T3.
 *
 * - INT0 Mode 1. The PC is stacked and instruction execution restarts at
 *   logical address 0x0038.
 *
 * - INT0 Mode 2. The restart address is obtained by reading the contents
 *   of a table residing in memory. The vector table consists of up to
 *   128 two-byte restart addresses stored in low byte, high byte order.
 *
 *   This is similar to normal vector mode interrupts (like INT1 and 2):
 *   The 256-bit table address comes from I, however the lower-order 8
 *   bits of the vector is fetched from the data bus.
 */

#define Z180_INT0          (8)

/* Vector Interrupts
 *
 * Normal vector interrupts use a vector table with 16 entries (2 bytes
 * per entry).  Each entry holds the address of the interrupt handler.
 * The vector table address is determined by 11-bits from the I and IL
 * registers.  The vector table must be aligned on 32-byte address
 * boundaries.
 */

#define Z180_INT1          (9)  /* Vector offset 0: External /INT1 */
#define Z180_INT2          (10) /* Vector offset 2: External /INT2 */
#define Z180_PRT0          (11) /* Vector offset 4: PRT channel 0 */
#define Z180_PRT1          (12) /* Vector offset 6: PRT channel 1 */
#define Z180_DMA0          (13) /* Vector offset 8: DMA channel 0 */
#define Z180_DMA1          (14) /* Vector offset 10: DMA Channel 1 */
#define Z180_CSIO          (15) /* Vector offset 12: Clocked serial I/O */
#define Z180_ASCI0         (16) /* Vector offset 14: Async channel 0 */
#define Z180_ASCI1         (18) /* Vector offset 16: Async channel 1 */
#define Z180_UNUSED        (19) /* Vector offset 18-20: unused */

#define Z180_IRQ_SYSTIMER Z180_RST7
#define NR_IRQS            (20)

/* IRQ Stack Frame Format
 *
 * This stack frame is created on each interrupt.  These registers are stored
 * in the TCB to many context switches.
 */

#define XCPT_I             (0) /* Offset 0: Saved I w/interrupt state in carry */
#define XCPT_BC            (1) /* Offset 1: Saved BC register */
#define XCPT_DE            (2) /* Offset 2: Saved DE register */
#define XCPT_IX            (3) /* Offset 3: Saved IX register */
#define XCPT_IY            (4) /* Offset 4: Saved IY register */
#define XCPT_SP            (5) /* Offset 5: Offset to SP at time of interrupt */
#define XCPT_HL            (6) /* Offset 6: Saved HL register */
#define XCPT_AF            (7) /* Offset 7: Saved AF register */
#define XCPT_PC            (8) /* Offset 8: Offset to PC at time of interrupt */

#define XCPTCONTEXT_REGS   (9)
#define XCPTCONTEXT_SIZE   (2 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This is the type of the register save array */

typedef uint16_t chipreg_t;

/* Common Area 1 holds the code and data that is unique to a particular task
 * and shared by all pthreads created from that task.  Each task will then
 * have its own copy of struct z180_cbr_s.  This structure is created with
 * a reference count of one when the task is created.
 *
 * When the task creates additional threads, the reference count is
 * incremented and the CBR value is shared.  When each thread exits, the
 * reference count id decremented.  When the reference count is decremented,
 * the physical memory underlying the CBR is finally released.
 */

struct z180_cbr_s
{
  uint8_t cbr;   /* The CBR value used by the thread */
  uint8_t crefs; /* The number of threads sharing this CBR value */
  uint8_t pages; /* The number of 4KB pages of physical memory in the allocation */
};

/* This struct defines the way the registers and z180-state information are
 * stored.
 */

struct xcptcontext
{
  /* CBR allocation */

  FAR struct z180_cbr_s *cbr;

  /* Register save area */

  chipreg_t regs[XCPTCONTEXT_REGS];

  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  CODE void *sigdeliver; /* Actual type is sig_deliver_t */

  /* The following retains that state during signal execution */

  uint16_t saved_pc;    /* Saved return address */
  uint16_t saved_i;     /* Saved interrupt state */
#endif
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

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

EXTERN irqstate_t irqsave(void) __naked;
EXTERN void       irqrestore(irqstate_t flags) __naked;

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_Z80_INCLUDE_Z180_IRQ_H */

