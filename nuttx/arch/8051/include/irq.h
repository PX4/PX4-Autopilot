/************************************************************************
 * irq.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_IRQ_H
#define __ARCH_IRQ_H

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/************************************************************************
 * Definitions
 ************************************************************************/

#define EXT_INT0_IRQ 0
#define TIMER0_IRQ   1
#define EXT_INT1_IRQ 2
#define TIMER1_IRQ   3
#define UART_IRQ     4
#define TIMER2_IRQ   5

#define NR_IRQS      6

/* The stack for all tasks/threads lie at the same position
 * in IRAM.  On context switches, the STACK contents will be
 * copied into the TCB.
 */

#define IRAM_BASE    0x0000
#ifdef CONFIG_ARCH_CHIP_8052
# define IRAM_SIZE   0x0100
#else
# define IRAM_SIZE   0x0080
#endif

#define STACK_BASE   0x0024
#define STACK_SIZE   (IRAM_SIZE - STACK_BASE)

/* This is the form of initial stack frame
 *
 * This initial stack frame will be configured to hold.
 * (1) The 16-bit return address of either:
 *
 *     void task_start(void);
 *     void pthread_start(void)
 *
 *     The return address is stored at the top of stack.
 *     so that the RETI instruction will work:
 *
 *     PC15-8 <- ((SP))
 *     (SP)   <- (SP) -1
 *     PC7-0  <- ((SP))
 *     (SP)   <- (SP) -1
 */

#define FRAME_RETLS 0
#define FRAME_RETMS 1

/* Then a partial context context save area that can be
 * indexed with the following definitions (relative to the
 * beginning of the initial frame.
 */

#define FRAME_ACC   2
#define FRAME_IE    3
#define FRAME_DPL   4
#define FRAME_DPH   5

#define FRAME_SIZE  6

/* The remaining registers are not saved on the stack (due
 * to the limited stack size of the 8051/2) but in an array
 * in the TCB:
 */

#define REGS_B      0
#define REGS_R2     1
#define REGS_R3     2
#define REGS_R4     3
#define REGS_R5     4
#define REGS_R6     5
#define REGS_R7     6
#define REGS_R0     7
#define REGS_R1     8
#define REGS_PSW    9
#define REGS_BP     10

#define REGS_SIZE   11

/* Note that the stack pointer is not saved.  Rather, the
 * size of the saved stack frame is saved in the 'nbytes'
 * field.  Since that stack begins at a fixed location, the
 * top-of-stack pointer can be derived from the saved size.
 */

/* These are offsets into struct xcptcontext that can be
 * used from assembly language to access the structure.
 */

#define XCPT_NBYTES 0
#define XCPT_STACK  1
#define XCPT_REGS   (STACK_SIZE+1)

#define XCPT_SIZE   (STACK_SIZE+REGS_SIZE+1)

/************************************************************************
 * Public Types
 ************************************************************************/

/* This struct defines the way the registers are stored */

#ifndef __ASSEMBLY__
struct xcptcontext
{
   /* This is the number of valid bytes currently saved in
    * stack[].  Since that stack begins at a fixed location,
    * the top-of-stack pointer can be derived from this size.
    */

   uint8_t nbytes;

   /* This is the saved stack.  Space is allocated for the 
    * entire 256 byte IRAM (minus register and bit usage at
    * the beginning).
    */

   uint8_t stack[STACK_SIZE];

   /* These are save 8051/2 registers.  These are saved
    * separately from the stack to increase the effective
    * stack size.
    */

   uint8_t regs[REGS_SIZE];
};
#endif /* __ASSEMBLY */

/************************************************************************
 * Public Variables
 ************************************************************************/

/************************************************************************
 * Public Function Prototypes
 ************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN irqstate_t irqsave(void);
EXTERN void       irqrestore(irqstate_t flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY */
#endif /* __ARCH_IRQ_H */

