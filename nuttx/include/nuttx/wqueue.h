/****************************************************************************
 * include/nuttx/wqueue.h
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

#ifndef __INCLUDE_NUTTX_WQUEUE_H
#define __INCLUDE_NUTTX_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <signal.h>
#include <queue.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SCHED_WORKQUEUE.  Create a dedicated "worker" thread to
 *   handle delayed processing from interrupt handlers.  This feature
 *   is required for some drivers but, if there are not complaints,
 *   can be safely disabled.  The worker thread also performs
 *   garbage collection -- completing any delayed memory deallocations
 *   from interrupt handlers.  If the worker thread is disabled,
 *   then that clean will be performed by the IDLE thread instead
 *   (which runs at the lowest of priority and may not be appropriate
 *   if memory reclamation is of high priority).  If CONFIG_SCHED_WORKQUEUE
 *   is enabled, then the following options can also be used:
 * CONFIG_SCHED_WORKPRIORITY - The execution priority of the worker
 *   thread.  Default: 192
 * CONFIG_SCHED_WORKPERIOD - How often the worker thread checks for
 *   work in units of microseconds.  Default: 50*1000 (50 MS).
 * CONFIG_SCHED_WORKSTACKSIZE - The stack size allocated for the worker
 *   thread.  Default: CONFIG_IDLETHREAD_STACKSIZE.
 * CONFIG_SIG_SIGWORK - The signal number that will be used to wake-up
 *   the worker thread.  Default: 17
 *
 * CONFIG_SCHED_LPWORK. If CONFIG_SCHED_WORKQUEUE is defined, then a single
 *   work queue is created by default.  If CONFIG_SCHED_LPWORK is also defined
 *   then an additional, lower-priority work queue will also be created.  This
 *   lower priority work queue is better suited for more extended processing
 *   (such as file system clean-up operations)
 * CONFIG_SCHED_LPWORKPRIORITY - The execution priority of the lower priority
 *   worker thread.  Default: 50
 * CONFIG_SCHED_LPWORKPERIOD - How often the lower priority worker thread
 *  checks for work in units of microseconds.  Default: 50*1000 (50 MS).
 * CONFIG_SCHED_LPWORKSTACKSIZE - The stack size allocated for the lower
 *   priority worker thread.  Default: CONFIG_IDLETHREAD_STACKSIZE.
 */

#ifndef CONFIG_SCHED_WORKPRIORITY
#  define CONFIG_SCHED_WORKPRIORITY 192
#endif

#ifndef CONFIG_SCHED_WORKPERIOD
#  define CONFIG_SCHED_WORKPERIOD (50*1000) /* 50 milliseconds */
#endif

#ifndef CONFIG_SCHED_WORKSTACKSIZE
#  define CONFIG_SCHED_WORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#endif

#ifdef CONFIG_SCHED_LPWORK
#  ifndef CONFIG_SCHED_LPWORKPRIORITY
#    define CONFIG_SCHED_LPWORKPRIORITY 50
#  endif

#  ifndef CONFIG_SCHED_LPWORKPERIOD
#    define CONFIG_SCHED_LPWORKPERIOD (50*1000) /* 50 milliseconds */
#  endif

#  ifndef CONFIG_SCHED_LPWORKSTACKSIZE
#    define CONFIG_SCHED_LPWORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif
#endif

/* Work queue IDs (indices):
 *
 * Kernel Work Queues:
 *   HPWORK: This ID of the high priority work queue that should only be used for
 *   hi-priority, time-critical, driver bottom-half functions.
 *
 *   LPWORK: This is the ID of the low priority work queue that can be used for any
 *   purpose.  if CONFIG_SCHED_LPWORK is not defined, then there is only one kernel
 *   work queue and LPWORK == HPWORK.
 *
 * User Work Queue:
 *   USRWORK:  CONFIG_NUTTX_KERNEL and CONFIG_SCHED_USRWORK are defined, then NuttX
 *   will also support a user-accessible work queue.  Otherwise, USRWORK == LPWORK.
 */

#define HPWORK 0
#ifdef CONFIG_SCHED_LPWORK
#  define LPWORK (HPWORK+1)
#else
#  define LPWORK HPWORK
#endif

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_SCHED_USRWORK)
#  warning "Feature not implemented"
#  define USRWORK (LPWORK+1)
#else
#  define USRWORK LPWORK
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Defines the work callback */

typedef void (*worker_t)(FAR void *arg);

/* Defines one entry in the work queue.  The user only needs this structure
 * in order to declare instances of the work structure.  Handling of all
 * fields is performed by the work APIs
 */

struct work_s
{
  struct dq_entry_s dq; /* Implements a doubly linked list */
  worker_t  worker;     /* Work callback */
  FAR void *arg;        /* Callback argument */
  uint32_t  qtime;      /* Time work queued */
  uint32_t  delay;      /* Delay until work performed */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: work_queue
 *
 * Description:
 *   Queue work to be performed at a later time.  All queued work will be
 *   performed on the worker thread of of execution (not the caller's).
 *
 *   The work structure is allocated by caller, but completely managed by
 *   the work queue logic.  The caller should never modify the contents of
 *   the work queue structure; the caller should not call work_queue()
 *   again until either (1) the previous work has been performed and removed
 *   from the queue, or (2) work_cancel() has been called to cancel the work
 *   and remove it from the work queue.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *   work   - The work structure to queue
 *   worker - The worker callback to be invoked.  The callback will invoked
 *            on the worker thread of execution.
 *   arg    - The argument that will be passed to the workder callback when
 *            int is invoked.
 *   delay  - Delay (in clock ticks) from the time queue until the worker
 *            is invoked. Zero means to perform the work immediately.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

EXTERN int work_queue(int qid, FAR struct work_s *work, worker_t worker,
                      FAR void *arg, uint32_t delay);

/****************************************************************************
 * Name: work_cancel
 *
 * Description:
 *   Cancel previously queued work.  This removes work from the work queue.
 *   After work has been canceled, it may be re-queue by calling work_queue()
 *   again.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *   work   - The previously queue work structure to cancel
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

EXTERN int work_cancel(int qid, FAR struct work_s *work);

/****************************************************************************
 * Name: work_signal
 *
 * Description:
 *   Signal the worker thread to process the work queue now.  This function
 *   is used internally by the work logic but could also be used by the
 *   user to force an immediate re-assessment of pending work.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

EXTERN int work_signal(int qid);

/****************************************************************************
 * Name: work_available
 *
 * Description:
 *   Check if the work structure is available.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   true if available; false if busy (i.e., there is still pending work).
 *
 ****************************************************************************/

#define work_available(work) ((work)->worker == NULL)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WQUEUE_H */
