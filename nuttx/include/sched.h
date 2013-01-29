/********************************************************************************
 * include/sched.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

#ifndef __INCLUDE_SCHED_H
#define __INCLUDE_SCHED_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/sched.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* Task Management Definitions **************************************************/

/* POSIX-like scheduling policies */

#define SCHED_FIFO     1  /* FIFO per priority scheduling policy */
#define SCHED_RR       2  /* Round robin scheduling policy */
#define SCHED_SPORADIC 3  /* Not supported */
#define SCHED_OTHER    4  /* Not supported */

/* Pthread definitions **********************************************************/

#define PTHREAD_KEYS_MAX CONFIG_NPTHREAD_KEYS

/* Non-standard Helper **********************************************************/
/* One processor family supported by NuttX has a single, fixed hardware stack.
 * That is the 8051 family.  So for that family only, there is a variant form
 * of task_create() that does not take a stack size parameter.  The following
 * helper macros are provided to work around the ugliness of that exception.
 */

#ifndef CONFIG_CUSTOM_STACK
#  define TASK_INIT(t,n,p,m,s,e,a) task_init(t,n,p,m,s,e,a)
#  define TASK_CREATE(n,p,s,e,a)   task_create(n,p,s,e,a)
#else
#  define TASK_INIT(t,n,p,m,s,e,a) task_init(t,n,p,e,a)
#  define TASK_CREATE(n,p,s,e,a)   task_create(n,p,e,a)
#endif

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

/* This is the POSIX-like scheduling parameter structure */

struct sched_param
{
  int sched_priority;
};

/********************************************************************************
 * Public Data
 ********************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/* Task Control Interfaces (non-standard) */

#ifndef CONFIG_CUSTOM_STACK
int    task_init(FAR _TCB *tcb, const char *name, int priority,
                 FAR uint32_t *stack, uint32_t stack_size, main_t entry,
                 FAR const char *argv[]);
#else
int    task_init(FAR _TCB *tcb, const char *name, int priority, main_t entry,
                 FAR const char *argv[]);
#endif
int    task_activate(FAR _TCB *tcb);
#ifndef CONFIG_CUSTOM_STACK
int    task_create(FAR const char *name, int priority, int stack_size, main_t entry,
                   FAR const char *argv[]);
#else
int    task_create(FAR const char *name, int priority, main_t entry,
                   FAR const char *argv[]);
#endif
int    task_delete(pid_t pid);
int    task_restart(pid_t pid);

/* Task Scheduling Interfaces (based on POSIX APIs) */

int    sched_setparam(pid_t pid, const struct sched_param *param);
int    sched_getparam(pid_t pid, struct sched_param *param);
int    sched_setscheduler(pid_t pid, int policy,
                          FAR const struct sched_param *param);
int    sched_getscheduler(pid_t pid);
int    sched_yield(void);
int    sched_get_priority_max(int policy);
int    sched_get_priority_min(int policy);
int    sched_rr_get_interval(pid_t pid, FAR struct timespec *interval);

/* Task Switching Interfaces (non-standard) */

int    sched_lock(void);
int    sched_unlock(void);
int    sched_lockcount(void);

/* If instrumentation of the scheduler is enabled, then some outboard logic
 * must provide the following interfaces.
 */

#ifdef CONFIG_SCHED_INSTRUMENTATION

void   sched_note_start(FAR _TCB *tcb);
void   sched_note_stop(FAR _TCB *tcb);
void   sched_note_switch(FAR _TCB *pFromTcb, FAR _TCB *pToTcb);

#else
# define sched_note_start(t)
# define sched_note_stop(t)
# define sched_note_switch(t1, t2)
#endif /* CONFIG_SCHED_INSTRUMENTATION */

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_SCHED_H */

