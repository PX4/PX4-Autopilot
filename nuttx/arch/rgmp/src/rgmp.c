/****************************************************************************
 * arch/rgmp/src/rgmp.c
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

#include <rgmp/trap.h>
#include <rgmp/mmu.h>
#include <rgmp/arch/arch.h>

#include <nuttx/config.h>
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <semaphore.h>
#include <queue.h>
#include <stdlib.h>
#include <arch/arch.h>
#include <os_internal.h>

int nest_irq = 0;

// the default time is 10ms
#ifdef CONFIG_MSEC_PER_TICK
unsigned int rtos_tick_time = CONFIG_MSEC_PER_TICK;
#else
unsigned int rtos_tick_time = 10;
#endif

void rtos_entry(void)
{
    os_start();
}

void *rtos_get_page(void)
{
    return memalign(PTMEMSIZE, PTMEMSIZE);
}

void rtos_free_page(void *page)
{
    free(page);
}

/**
 * The interrupt can be nested. The pair of rtos_enter_interrupt()
 * and rtos_exit_interrupt() make sure the context switch is 
 * performed only in the last IRQ exit.
 */
void rtos_enter_interrupt(struct Trapframe *tf)
{
    nest_irq++;
}

void rtos_exit_interrupt(struct Trapframe *tf)
{
    local_irq_disable();
    nest_irq--;
    if (!nest_irq) {
	_TCB *rtcb = current_task;
	_TCB *ntcb;

	if (rtcb->xcp.sigdeliver) {
	    rtcb->xcp.tf = tf;
	    push_xcptcontext(&rtcb->xcp);
	}
	ntcb = (_TCB*)g_readytorun.head;
	// switch needed
	if (rtcb != ntcb) {
	    rtcb->xcp.tf = tf;
	    current_task = ntcb;
	    rgmp_pop_tf(ntcb->xcp.tf);
	}
    }
}

void rtos_timer_isr(struct Trapframe *tf)
{
    sched_process_timer();
}

/**
 * RTOS mutex operation
 */
const int rtos_mutex_size = sizeof(sem_t);
void rtos_mutex_init(void *lock)
{
    sem_init(lock, 0, 1);
}

int rtos_mutex_lock(void *lock)
{
    return sem_wait(lock);
}

int rtos_mutex_unlock(void *lock)
{
    return sem_post(lock);
}

/**
 * RTOS semaphore operation
 */
const int rtos_semaphore_size = sizeof(sem_t);

void rtos_sem_init(void *sem, int val)
{
    sem_init(sem, 0, val);
}

int rtos_sem_up(void *sem)
{
    return sem_post(sem);
}

int rtos_sem_down(void *sem)
{
    return sem_wait(sem);
}

void rtos_stop_running(void)
{
    extern void nuttx_arch_exit(void);

    local_irq_disable();

    nuttx_arch_exit();

    while(1) {
	arch_hlt();
    }
}


