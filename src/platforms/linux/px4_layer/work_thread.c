/****************************************************************************
 * libc/wqueue/work_thread.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <unistd.h>
#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* The state of each work queue. */

#ifdef CONFIG_NUTTX_KERNEL

  /* Play some games in the kernel mode build to assure that different
   * naming is used for the global work queue data structures.  This may
   * not be necessary but it safer.
   *
   * In this case g_work is #define'd to be either g_kernelwork or
   * g_usrwork in include/nuttx/wqueue.h
   */

#  ifdef __KERNEL__
struct wqueue_s g_kernelwork[NWORKERS];
#  else
struct wqueue_s g_usrwork[NWORKERS];
#  endif

#else /* CONFIG_NUTTX_KERNEL */

struct wqueue_s g_work[NWORKERS];

#endif /* CONFIG_NUTTX_KERNEL */

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_process
 *
 * Description:
 *   This is the logic that performs actions placed on any work list.
 *
 * Input parameters:
 *   wqueue - Describes the work queue to be processed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void work_process(FAR struct wqueue_s *wqueue)
{
  volatile FAR struct work_s *work;
  worker_t  worker;
  irqstate_t flags;
  FAR void *arg;
  uint32_t elapsed;
  uint32_t remaining;
  uint32_t next;

  /* Then process queued work.  We need to keep interrupts disabled while
   * we process items in the work list.
   */

  next  = CONFIG_SCHED_WORKPERIOD / USEC_PER_TICK;
  flags = irqsave();
  work  = (FAR struct work_s *)wqueue->q.head;
  while (work)
    {
      /* Is this work ready?  It is ready if there is no delay or if
       * the delay has elapsed. qtime is the time that the work was added
       * to the work queue.  It will always be greater than or equal to
       * zero.  Therefore a delay of zero will always execute immediately.
       */

      elapsed = clock_systimer() - work->qtime;
      if (elapsed >= work->delay)
        {
          /* Remove the ready-to-execute work from the list */

          (void)dq_rem((struct dq_entry_s *)work, &wqueue->q);

          /* Extract the work description from the entry (in case the work
           * instance by the re-used after it has been de-queued).
           */

          worker = work->worker;
          arg    = work->arg;

          /* Mark the work as no longer being queued */

          work->worker = NULL;

          /* Do the work.  Re-enable interrupts while the work is being
           * performed... we don't have any idea how long that will take!
           */

          irqrestore(flags);
          worker(arg);

          /* Now, unfortunately, since we re-enabled interrupts we don't
           * know the state of the work list and we will have to start
           * back at the head of the list.
           */

          flags = irqsave();
          work  = (FAR struct work_s *)wqueue->q.head;
        }
      else
        {
          /* This one is not ready.. will it be ready before the next
           * scheduled wakeup interval?
           */

          /* Here: elapsed < work->delay */
          remaining = work->delay - elapsed;
          if (remaining < next)
            {
              /* Yes.. Then schedule to wake up when the work is ready */

              next = remaining;
            }
              
          /* Then try the next in the list. */

          work = (FAR struct work_s *)work->dq.flink;
        }
    }

  /* Wait awhile to check the work list.  We will wait here until either
   * the time elapses or until we are awakened by a signal.
   */

  usleep(next * USEC_PER_TICK);
  irqrestore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: work_hpthread, work_lpthread, and work_usrthread
 *
 * Description:
 *   These are the worker threads that performs actions placed on the work
 *   lists.
 *
 *   work_hpthread and work_lpthread:  These are the kernel mode work queues
 *     (also build in the flat build).  One of these threads also performs
 *     periodic garbage collection (that is otherwise performed by the idle
 *     thread if CONFIG_SCHED_WORKQUEUE is not defined).
 *
 *     These worker threads are started by the OS during normal bringup.
 *
 *   work_usrthread:  This is a user mode work queue.  It must be built into
 *     the applicatino blob during the user phase of a kernel build.  The
 *     user work thread will then automatically be started when the system
 *     boots by calling through the pointer found in the header on the user
 *     space blob.
 *
 *   All of these entrypoints are referenced by OS internally and should not
 *   not be accessed by application logic.
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK

int work_hpthread(int argc, char *argv[])
{
  /* Loop forever */

  for (;;)
    {
      /* First, perform garbage collection.  This cleans-up memory de-allocations
       * that were queued because they could not be freed in that execution
       * context (for example, if the memory was freed from an interrupt handler).
       * NOTE: If the work thread is disabled, this clean-up is performed by
       * the IDLE thread (at a very, very low priority).
       */

#ifndef CONFIG_SCHED_LPWORK
      sched_garbagecollection();
#endif

      /* Then process queued work.  We need to keep interrupts disabled while
       * we process items in the work list.
       */

      work_process(&g_work[HPWORK]);
    }

  return OK; /* To keep some compilers happy */
}

#ifdef CONFIG_SCHED_LPWORK

int work_lpthread(int argc, char *argv[])
{
  /* Loop forever */

  for (;;)
    {
      /* First, perform garbage collection.  This cleans-up memory de-allocations
       * that were queued because they could not be freed in that execution
       * context (for example, if the memory was freed from an interrupt handler).
       * NOTE: If the work thread is disabled, this clean-up is performed by
       * the IDLE thread (at a very, very low priority).
       */

      sched_garbagecollection();

      /* Then process queued work.  We need to keep interrupts disabled while
       * we process items in the work list.
       */

      work_process(&g_work[LPWORK]);
    }

  return OK; /* To keep some compilers happy */
}

#endif /* CONFIG_SCHED_LPWORK */
#endif /* CONFIG_SCHED_HPWORK */

#ifdef CONFIG_SCHED_USRWORK

int work_usrthread(int argc, char *argv[])
{
  /* Loop forever */

  for (;;)
    {
      /* Then process queued work.  We need to keep interrupts disabled while
       * we process items in the work list.
       */

      work_process(&g_work[USRWORK]);
    }

  return OK; /* To keep some compilers happy */
}

#endif /* CONFIG_SCHED_USRWORK */

#endif /* CONFIG_SCHED_WORKQUEUE */
