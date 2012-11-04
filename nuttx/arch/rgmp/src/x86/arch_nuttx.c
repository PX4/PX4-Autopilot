/****************************************************************************
 * arch/rgmp/src/x86/arch_nuttx.c
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Author: Yu Qiang <yuq825@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <rgmp/mmu.h>
#include <rgmp/string.h>
#include <rgmp/arch/fpu.h>

#include <arch/arch.h>
#include <nuttx/sched.h>
#include <os_internal.h>


void nuttx_arch_init(void)
{
    extern void e1000_mod_init(void);
    extern void up_serialinit(void);

    // setup COM device
    up_serialinit();

#ifdef CONFIG_NET_E1000
    // setup e1000
    e1000_mod_init();
#endif

}

void nuttx_arch_exit(void)
{
    extern void e1000_mod_exit(void);

#ifdef CONFIG_NET_E1000
    e1000_mod_exit();
#endif

}

void up_initial_state(_TCB *tcb)
{
    struct Trapframe *tf;

    if (tcb->pid) {
		tf = (struct Trapframe *)tcb->adj_stack_ptr - 1;
		rgmp_setup_context(&tcb->xcp.ctx, tf, tcb->start, 1);
    }
	else
		rgmp_setup_context(&tcb->xcp.ctx, NULL, NULL, 0);
}

void push_xcptcontext(struct xcptcontext *xcp)
{
    xcp->save_eip = xcp->ctx.tf->tf_eip;
    xcp->save_eflags = xcp->ctx.tf->tf_eflags;

    // set up signal entry with interrupts disabled
    xcp->ctx.tf->tf_eip = (uint32_t)up_sigentry;
    xcp->ctx.tf->tf_eflags = 0;
}

void pop_xcptcontext(struct xcptcontext *xcp)
{
    xcp->ctx.tf->tf_eip = xcp->save_eip;
    xcp->ctx.tf->tf_eflags = xcp->save_eflags;
}

