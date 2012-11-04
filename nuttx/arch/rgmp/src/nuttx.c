/****************************************************************************
 * arch/rgmp/src/nuttx.c
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

#include <rgmp/boot.h>
#include <rgmp/cxx.h>
#include <rgmp/memlayout.h>
#include <rgmp/allocator.h>
#include <rgmp/assert.h>
#include <rgmp/string.h>
#include <rgmp/arch/arch.h>

#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <stdio.h>
#include <stdlib.h>
#include <arch/irq.h>
#include <arch/arch.h>
#include <os_internal.h>

_TCB *current_task = NULL;


/**
 * This function is called in non-interrupt context
 * to switch tasks.
 * Assumption: global interrupt is disabled.
 */
static inline void up_switchcontext(_TCB *ctcb, _TCB *ntcb)
{
    // do nothing if two tasks are the same
    if (ctcb == ntcb)
		return;

    // this function can not be called in interrupt
    if (up_interrupt_context()) {
		panic("%s: try to switch context in interrupt\n", __func__);
    }

    // start switch
    current_task = ntcb;
    rgmp_context_switch(ctcb ? &ctcb->xcp.ctx : NULL, &ntcb->xcp.ctx);
}

void up_initialize(void)
{
    extern pidhash_t g_pidhash[];
	extern void vdev_init(void);
    extern void nuttx_arch_init(void);

    // intialize the current_task to g_idletcb
    current_task = g_pidhash[PIDHASH(0)].tcb;

	// OS memory alloc system is ready
	use_os_kmalloc = 1;

    // rgmp vdev init
	vdev_init();

	nuttx_arch_init();

    // enable interrupt
    local_irq_enable();
}

void up_idle(void)
{
    arch_hlt();
}

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
	void *boot_freemem = boot_alloc(0, sizeof(int));
    *heap_start = boot_freemem;
    *heap_size = KERNBASE + kmem_size - (uint32_t)boot_freemem;
}

int up_create_stack(_TCB *tcb, size_t stack_size)
{
    int ret = ERROR;
    size_t *adj_stack_ptr;

    /* Move up to next even word boundary if necessary */

    size_t adj_stack_size = (stack_size + 3) & ~3;
    size_t adj_stack_words = adj_stack_size >> 2;

    /* Allocate the memory for the stack */

    uint32_t *stack_alloc_ptr = (uint32_t*)kmalloc(adj_stack_size);
    if (stack_alloc_ptr) {
		/* This is the address of the last word in the allocation */

		adj_stack_ptr = &stack_alloc_ptr[adj_stack_words - 1];

		/* Save the values in the TCB */

		tcb->adj_stack_size  = adj_stack_size;
		tcb->stack_alloc_ptr = stack_alloc_ptr;
		tcb->adj_stack_ptr   = (void *)((unsigned int)adj_stack_ptr & ~7);
		ret = OK;
    }
    return ret;
}

int up_use_stack(_TCB *tcb, void *stack, size_t stack_size)
{
    /* Move up to next even word boundary if necessary */

    size_t adj_stack_size = stack_size & ~3;
    size_t adj_stack_words = adj_stack_size >> 2;

    /* This is the address of the last word in the allocation */

    size_t *adj_stack_ptr = &((size_t*)stack)[adj_stack_words - 1];

    /* Save the values in the TCB */

    tcb->adj_stack_size  = adj_stack_size;
    tcb->stack_alloc_ptr = stack;
    tcb->adj_stack_ptr   = (void *)((unsigned int)adj_stack_ptr & ~7);
    return OK;
}

void up_release_stack(_TCB *dtcb)
{
    if (dtcb->stack_alloc_ptr) {
		free(dtcb->stack_alloc_ptr);
    }

    dtcb->stack_alloc_ptr = NULL;
    dtcb->adj_stack_size  = 0;
    dtcb->adj_stack_ptr   = NULL;
}

/****************************************************************************
 * Name: up_block_task
 *
 * Description:
 *   The currently executing task at the head of
 *   the ready to run list must be stopped.  Save its context
 *   and move it to the inactive list specified by task_state.
 *
 *   This function is called only from the NuttX scheduling
 *   logic.  Interrupts will always be disabled when this
 *   function is called.
 *
 * Inputs:
 *   tcb: Refers to a task in the ready-to-run list (normally
 *     the task at the head of the list).  It most be
 *     stopped, its context saved and moved into one of the
 *     waiting task lists.  It it was the task at the head
 *     of the ready-to-run list, then a context to the new
 *     ready to run task must be performed.
 *   task_state: Specifies which waiting task list should be
 *     hold the blocked task TCB.
 *
 ****************************************************************************/
void up_block_task(_TCB *tcb, tstate_t task_state)
{
    /* Verify that the context switch can be performed */
    if ((tcb->task_state < FIRST_READY_TO_RUN_STATE) ||
		(tcb->task_state > LAST_READY_TO_RUN_STATE)) {
		warn("%s: task sched error\n", __func__);
		return;
    }
    else {
		_TCB *rtcb = current_task;
		bool switch_needed;

		/* Remove the tcb task from the ready-to-run list.  If we
		 * are blocking the task at the head of the task list (the
		 * most likely case), then a context switch to the next
		 * ready-to-run task is needed. In this case, it should
		 * also be true that rtcb == tcb.
		 */
		switch_needed = sched_removereadytorun(tcb);

		/* Add the task to the specified blocked task list */
		sched_addblocked(tcb, (tstate_t)task_state);

		/* Now, perform the context switch if one is needed */
		if (switch_needed) {
			_TCB *nexttcb;
			// this part should not be executed in interrupt context
			if (up_interrupt_context()) {
				panic("%s: %d\n", __func__, __LINE__);
			}
			// If there are any pending tasks, then add them to the g_readytorun
			// task list now. It should be the up_realease_pending() called from
			// sched_unlock() to do this for disable preemption. But it block 
			// itself, so it's OK.
			if (g_pendingtasks.head) {
				warn("Disable preemption failed for task block itself\n");
				sched_mergepending();
			}
			nexttcb = (_TCB*)g_readytorun.head;
			// context switch
			up_switchcontext(rtcb, nexttcb);
        }
    }
}

/****************************************************************************
 * Name: up_unblock_task
 *
 * Description:
 *   A task is currently in an inactive task list
 *   but has been prepped to execute.  Move the TCB to the
 *   ready-to-run list, restore its context, and start execution.
 *
 * Inputs:
 *   tcb: Refers to the tcb to be unblocked.  This tcb is
 *     in one of the waiting tasks lists.  It must be moved to
 *     the ready-to-run list and, if it is the highest priority
 *     ready to run taks, executed.
 *
 ****************************************************************************/
void up_unblock_task(_TCB *tcb)
{
    /* Verify that the context switch can be performed */
    if ((tcb->task_state < FIRST_BLOCKED_STATE) ||
		(tcb->task_state > LAST_BLOCKED_STATE)) {
		warn("%s: task sched error\n", __func__);
		return;
    }
    else {
		_TCB *rtcb = current_task;

		/* Remove the task from the blocked task list */
		sched_removeblocked(tcb);

		/* Reset its timeslice.  This is only meaningful for round
		 * robin tasks but it doesn't here to do it for everything
		 */
#if CONFIG_RR_INTERVAL > 0
		tcb->timeslice = CONFIG_RR_INTERVAL / MSEC_PER_TICK;
#endif
	
		// Add the task in the correct location in the prioritized
		// g_readytorun task list.
		if (sched_addreadytorun(tcb) && !up_interrupt_context()) {
			/* The currently active task has changed! */
			_TCB *nexttcb = (_TCB*)g_readytorun.head;
			// context switch
			up_switchcontext(rtcb, nexttcb);
		}
    }
}

/**
 * This function is called from sched_unlock() which will check not
 * in interrupt context and disable interrupt.
 */
void up_release_pending(void)
{
    _TCB *rtcb = current_task;

    /* Merge the g_pendingtasks list into the g_readytorun task list */

    if (sched_mergepending()) {
		/* The currently active task has changed! */
		_TCB *nexttcb = (_TCB*)g_readytorun.head;

		// context switch
		up_switchcontext(rtcb, nexttcb);
    }
}

void up_reprioritize_rtr(_TCB *tcb, uint8_t priority)
{
    /* Verify that the caller is sane */

    if (tcb->task_state < FIRST_READY_TO_RUN_STATE ||
		tcb->task_state > LAST_READY_TO_RUN_STATE
#if SCHED_PRIORITY_MIN > UINT8_MIN
		|| priority < SCHED_PRIORITY_MIN
#endif
#if SCHED_PRIORITY_MAX < UINT8_MAX
		|| priority > SCHED_PRIORITY_MAX
#endif
		) {
		warn("%s: task sched error\n", __func__);
		return;
    }
    else {
		_TCB *rtcb = current_task;
		bool switch_needed;

		/* Remove the tcb task from the ready-to-run list.
		 * sched_removereadytorun will return true if we just
		 * remove the head of the ready to run list.
		 */
		switch_needed = sched_removereadytorun(tcb);

		/* Setup up the new task priority */
		tcb->sched_priority = (uint8_t)priority;

		/* Return the task to the specified blocked task list.
		 * sched_addreadytorun will return true if the task was
		 * added to the new list.  We will need to perform a context
		 * switch only if the EXCLUSIVE or of the two calls is non-zero
		 * (i.e., one and only one the calls changes the head of the
		 * ready-to-run list).
		 */
		switch_needed ^= sched_addreadytorun(tcb);

		/* Now, perform the context switch if one is needed */
		if (switch_needed && !up_interrupt_context()) {
			_TCB *nexttcb;
			// If there are any pending tasks, then add them to the g_readytorun
			// task list now. It should be the up_realease_pending() called from
			// sched_unlock() to do this for disable preemption. But it block 
			// itself, so it's OK.
			if (g_pendingtasks.head) {
				warn("Disable preemption failed for reprioritize task\n");
				sched_mergepending();
            }

			nexttcb = (_TCB*)g_readytorun.head;
			// context switch
			up_switchcontext(rtcb, nexttcb);
        }
    }
}

void _exit(int status)
{
    _TCB* tcb;

    /* Destroy the task at the head of the ready to run list. */

    (void)task_deletecurrent();

    /* Now, perform the context switch to the new ready-to-run task at the
     * head of the list.
     */

    tcb = (_TCB*)g_readytorun.head;

    /* Then switch contexts */

    up_switchcontext(NULL, tcb);
}

void up_assert(const uint8_t *filename, int line)
{
    fprintf(stderr, "Assertion failed at file:%s line: %d\n", filename, line);

    // in interrupt context or idle task means kernel error 
    // which will stop the OS
    // if in user space just terminate the task
    if (up_interrupt_context() || current_task->pid == 0) {
		panic("%s: %d\n", __func__, __LINE__);
    }
    else {
		exit(EXIT_FAILURE);
    }
}

void up_assert_code(const uint8_t *filename, int line, int code)
{
    fprintf(stderr, "Assertion failed at file:%s line: %d error code: %d\n", 
			filename, line, code);

    // in interrupt context or idle task means kernel error 
    // which will stop the OS
    // if in user space just terminate the task
    if (up_interrupt_context() || current_task->pid == 0) {
		panic("%s: %d\n", __func__, __LINE__);
    }
    else {
		exit(EXIT_FAILURE);
    }
}


#ifndef CONFIG_DISABLE_SIGNALS

void up_schedule_sigaction(_TCB *tcb, sig_deliver_t sigdeliver)
{
    /* Refuse to handle nested signal actions */
    if (!tcb->xcp.sigdeliver) {
		int flags;

		/* Make sure that interrupts are disabled */
		local_irq_save(flags);

		// First, handle some special cases when the signal is
		// being delivered to the currently executing task.
		if (tcb == current_task) {
			// CASE 1:  We are not in an interrupt handler and
			// a task is signalling itself for some reason.
			if (!up_interrupt_context()) {
				// In this case just deliver the signal now.
				sigdeliver(tcb);
			}
			// CASE 2:  We are in an interrupt handler AND the
			// interrupted task is the same as the one that
			// must receive the signal.
			else {
				tcb->xcp.sigdeliver = sigdeliver;
            }
        }

		// Otherwise, we are (1) signaling a task is not running
		// from an interrupt handler or (2) we are not in an
		// interrupt handler and the running task is signalling
		// some non-running task.
		else {
			tcb->xcp.sigdeliver = sigdeliver;
			push_xcptcontext(&tcb->xcp);
        }

		local_irq_restore(flags);
    }
}

#endif /* !CONFIG_DISABLE_SIGNALS */


bool up_interrupt_context(void)
{
    if (nest_irq)
		return true;
    return false;
}

#ifndef CONFIG_ARCH_NOINTC
void up_disable_irq(int irq)
{

}

void up_enable_irq(int irq)
{

}
#endif

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{

}
#endif

void up_sigdeliver(struct Trapframe *tf)
{
    sig_deliver_t sigdeliver;
    
    pop_xcptcontext(&current_task->xcp);
    sigdeliver = current_task->xcp.sigdeliver;
    current_task->xcp.sigdeliver = NULL;
    local_irq_enable();
    sigdeliver(current_task);
    local_irq_disable();
}

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)

void up_cxxinitialize(void)
{
	rgmp_cxx_init();
}

#endif








