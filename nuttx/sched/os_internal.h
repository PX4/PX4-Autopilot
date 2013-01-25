/****************************************************************************
 * sched/os_internal.h
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_OS_INTERNAL_H
#define __SCHED_OS_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <queue.h>
#include <sched.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OS CRASH CODES:  All must lie in the range 0-99 */

enum os_crash_codes_e
{
  OSERR_NOERROR = 0,          /* No error */
  OSERR_NOTIMPLEMENTED,       /* Feature is not implemented */
  OSERR_INTERNAL,             /* Internal logic error */
  OSERR_UNEXPECTEDISR,        /* Received unexpected interrupt */
  OSERR_UNDEFINEDINSN,        /* Undefined instruction */
  OSERR_ERREXCEPTION,         /* Other CPU-detected errors */
  OSERR_OUTOFMEMORY,          /* Insufficient memory */
  OSERR_OUTOFMESSAGES,        /* Out of messages */
  OSERR_NOIDLETASK,           /* There is no idle task */
  OSERR_MQNONEMPTYCOUNT,      /* Expected waiter for non-empty queue */
  OSERR_MQNOTFULLCOUNT,       /* Expected waiter for non-full queue */
  OSERR_MQNOWAITER,           /* Expected a queue for the waiter */
  OSERR_BADWAITSEM,           /* Already waiting for a semaphore */
  OSERR_BADMSGTYPE,           /* Tried to free a bad message type */
  OSERR_FAILEDTOADDSIGNAL,    /* Failed to add pending signal */
  OSERR_FAILEDTOREMOVESIGNAL, /* Failed to remove pending signal */
  OSERR_TIMEOUTNOTCB,         /* Timed out, but not TCB registered */
  OSERR_NOPENDINGSIGNAL,      /* Expected a signal to be pending */
  OSERR_BADDELETESTATE,       /* Bad state in task deletion */
  OSERR_WDOGNOTFOUND,         /* Active watchdog not found */
  OSERR_EXITFROMINTERRUPT,    /* Interrupt code attempted to exit */
  OSERR_BADUNBLOCKSTATE,      /* Attempt to unblock from bad state */
  OSERR_BADBLOCKSTATE,        /* Attempt to block from bad state */
  OSERR_BADREPRIORITIZESTATE  /* Attempt to reprioritize in bad state or priority */
};

/* Special task IDS.  Any negative PID is invalid. */

#define NULL_TASK_PROCESS_ID (pid_t)0
#define INVALID_PROCESS_ID   (pid_t)-1

/* Although task IDs can take the (positive, non-zero)
 * range of pid_t, the number of tasks that will be supported
 * at any one time is (artificially) limited by the CONFIG_MAX_TASKS
 * configuration setting. Limiting the number of tasks speeds certain
 * OS functions (this is the only limitation in the number of
 * tasks built into the design).
 */

#define MAX_TASKS_MASK      (CONFIG_MAX_TASKS-1)
#define PIDHASH(pid)        ((pid) & MAX_TASKS_MASK)

/* Stubs used when there are no file descriptors */

#if CONFIG_NFILE_DESCRIPTORS <= 0 && CONFIG_NSOCKET_DESCRIPTORS <= 0
# define sched_setupidlefiles(t)    (OK)
# define sched_setuptaskfiles(t)    (OK)
# define sched_setuppthreadfiles(t) (OK)
# define sched_releasefiles(t)      (OK)
#endif

/* One processor family supported by NuttX has a single, fixed hardware stack.
 * That is the 8051 family.  So for that family only, there is a variant form
 * of kernel_thread() that does not take a stack size parameter.  The following
 * helper macro is provided to work around the ugliness of that exception.
 */

#ifndef CONFIG_CUSTOM_STACK
#  define KERNEL_THREAD(n,p,s,e,a)   kernel_thread(n,p,s,e,a)
#else
#  define KERNEL_THREAD(n,p,s,e,a)   kernel_thread(n,p,e,a)
#endif

/* A more efficient ways to access the errno */

#define SET_ERRNO(e) \
  { _TCB *rtcb = _TCB*)g_readytorun.head; rtcb->pterrno = (e); }

#define _SET_TCB_ERRNO(t,e) \
  { (t)->pterrno = (e); }

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This structure defines the format of the hash table that
 * is used to (1) determine if a task ID is unique, and (2)
 * to map a process ID to its corresponding TCB.
 */

struct pidhash_s
{
  FAR _TCB *tcb;
  pid_t     pid;
};

typedef struct pidhash_s  pidhash_t;

/* This structure defines an element of the g_tasklisttable[].
 * This table is used to map a task_state enumeration to the
 * corresponding task list.
 */

struct tasklist_s
{
  DSEG volatile dq_queue_t *list; /* Pointer to the task list */
  bool prioritized;               /* true if the list is prioritized */
};

typedef struct tasklist_s tasklist_t;

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Declared in os_start.c ***************************************************/

/* The state of a task is indicated both by the task_state field of the TCB
 * and by a series of task lists.  All of these tasks lists are declared
 * below. Although it is not always necessary, most of these lists are
 * prioritized so that common list handling logic can be used (only the
 * g_readytorun, the g_pendingtasks, and the g_waitingforsemaphore lists need
 * to be prioritized).
 */

/* This is the list of all tasks that are ready to run.  The head of this
 * list is the currently active task; the tail of this list is always the
 * IDLE task.
 */

extern volatile dq_queue_t g_readytorun;

/* This is the list of all tasks that are ready-to-run, but cannot be placed
 * in the g_readytorun list because:  (1) They are higher priority than the
 * currently active task at the head of the g_readytorun list, and (2) the
 * currently active task has disabled pre-emption.
 */

extern volatile dq_queue_t g_pendingtasks;

/* This is the list of all tasks that are blocked waiting for a semaphore */

extern volatile dq_queue_t g_waitingforsemaphore;

/* This is the list of all tasks that are blocked waiting for a signal */

#ifndef CONFIG_DISABLE_SIGNALS
extern volatile dq_queue_t g_waitingforsignal;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-empty.
 */

#ifndef CONFIG_DISABLE_MQUEUE
extern volatile dq_queue_t g_waitingformqnotempty;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-full.
 */

#ifndef CONFIG_DISABLE_MQUEUE
extern volatile dq_queue_t g_waitingformqnotfull;
#endif

/* This is the list of all tasks that are blocking waiting for a page fill */

#ifdef CONFIG_PAGING
extern volatile dq_queue_t g_waitingforfill;
#endif

/* This the list of all tasks that have been initialized, but not yet
 * activated. NOTE:  This is the only list that is not prioritized.
 */

extern volatile dq_queue_t g_inactivetasks;

/* This is the list of dayed memory deallocations that need to be handled
 * within the IDLE loop.  These deallocations get queued by sched_free()
 * if the OS attempts to deallocate memory while it is within an interrupt
 * handler.
 */

extern volatile sq_queue_t g_delayeddeallocations;

/* This is the value of the last process ID assigned to a task */

extern volatile pid_t g_lastpid;

/* The following hash table is used for two things:
 *
 * 1. This hash table greatly speeds the determination of a new unique
 *    process ID for a task, and
 * 2. Is used to quickly map a process ID into a TCB.
 *
 * It has the side effects of using more memory and limiting the number
 * of tasks to CONFIG_MAX_TASKS.
 */

extern pidhash_t g_pidhash[CONFIG_MAX_TASKS];

/* This is a table of task lists.  This table is indexed by the task state
 * enumeration type (tstate_t) and provides a pointer to the associated
 * static task list (if there is one) as well as a boolean indication as to
 * if the list is an ordered list or not.
 */

extern const tasklist_t g_tasklisttable[NUM_TASK_STATES];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  os_bringup(void);
void task_start(void);
int  task_schedsetup(FAR _TCB *tcb, int priority, start_t start,
                     main_t main, uint8_t ttype);
int  task_argsetup(FAR _TCB *tcb, FAR const char *name, FAR const char *argv[]);
void task_exithook(FAR _TCB *tcb, int status);
int  task_deletecurrent(void);

#ifdef CONFIG_SCHED_HAVE_PARENT
int  task_reparent(pid_t ppid, pid_t chpid);

#ifdef HAVE_TASK_GROUP
int group_allocate(FAR _TCB *tcb);
int group_initialize(FAR _TCB *tcb);
int group_bind(FAR _TCB *tcb);
int group_join(FAR _TCB *tcb);
void group_leave(FAR _TCB *tcb);
#ifndef CONFIG_DISABLE_SIGNALS
int group_signal(FAR _TCB *tcb, FAR siginfo_t *info);
#else
# define group_signal(tcb,info) (0)
#endif
#else
# define group_allocate(tcb)    (0)
# define group_initialize(tcb)  (0)
# define group_bind(tcb)        (0)
# define group_join(tcb)        (0)
# define group_leave(tcb)
# define group_signal(tcb,info) (0)
#endif

#ifdef CONFIG_SCHED_CHILD_STATUS
void weak_function task_initialize(void);
FAR struct child_status_s *task_allocchild(void);
void task_freechild(FAR struct child_status_s *status);
void task_addchild(FAR _TCB *tcb, FAR struct child_status_s *child);
FAR struct child_status_s *task_exitchild(FAR _TCB *tcb);
FAR struct child_status_s *task_findchild(FAR _TCB *tcb, pid_t pid);
FAR struct child_status_s *task_removechild(FAR _TCB *tcb, pid_t pid);
void task_removechildren(FAR _TCB *tcb);
#endif
#endif

#ifndef CONFIG_CUSTOM_STACK
int  kernel_thread(FAR const char *name, int priority, int stack_size,
                   main_t entry, FAR const char *argv[]);
#else
int  kernel_thread(FAR const char *name, int priority, main_t entry,
                   FAR const char *argv[]);
#endif
bool sched_addreadytorun(FAR _TCB *rtrtcb);
bool sched_removereadytorun(FAR _TCB *rtrtcb);
bool sched_addprioritized(FAR _TCB *newTcb, DSEG dq_queue_t *list);
bool sched_mergepending(void);
void sched_addblocked(FAR _TCB *btcb, tstate_t task_state);
void sched_removeblocked(FAR _TCB *btcb);
int  sched_setpriority(FAR _TCB *tcb, int sched_priority);
#ifdef CONFIG_PRIORITY_INHERITANCE
int  sched_reprioritize(FAR _TCB *tcb, int sched_priority);
#else
#  define sched_reprioritize(tcb,sched_priority) sched_setpriority(tcb,sched_priority)
#endif
FAR _TCB *sched_gettcb(pid_t pid);
bool sched_verifytcb(FAR _TCB *tcb);

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
int  sched_setupidlefiles(FAR _TCB *tcb);
int  sched_setuptaskfiles(FAR _TCB *tcb);
int  sched_setuppthreadfiles(FAR _TCB *tcb);
#if CONFIG_NFILE_STREAMS > 0
int  sched_setupstreams(FAR _TCB *tcb);
#endif
int  sched_releasefiles(FAR _TCB *tcb);
#endif

int  sched_releasetcb(FAR _TCB *tcb);
void sched_garbagecollection(void);

#endif /* __SCHED_OS_INTERNAL_H */
