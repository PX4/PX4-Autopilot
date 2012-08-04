/****************************************************************************
 * sched/mq_timedsend.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <mqueue.h>
#include <wdog.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>

#include "clock_internal.h"
#include "os_internal.h"
#include "mq_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_sndtimeout
 *
 * Description:
 *   This function is called if the timeout elapses before the message queue
 *   becomes non-full.
 *
 * Parameters:
 *   argc  - the number of arguments (should be 1)
 *   pid   - the task ID of the task to wakeup
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void mq_sndtimeout(int argc, uint32_t pid)
{
  FAR _TCB *wtcb;
  irqstate_t saved_state;

  /* Disable interrupts.  This is necessary because an interrupt handler may
   * attempt to send a message while we are doing this.
   */

  saved_state = irqsave();

  /* Get the TCB associated with this pid.  It is possible that task may no
   * longer be active when this watchdog goes off.
   */

  wtcb = sched_gettcb((pid_t)pid);

  /* It is also possible that an interrupt/context switch beat us to the
   * punch and already changed the task's state.
   */

  if (wtcb && wtcb->task_state == TSTATE_WAIT_MQNOTFULL)
    {
      /* Restart with task with a timeout error */

      mq_waitirq(wtcb, ETIMEDOUT);
    }

  /* Interrupts may now be re-enabled. */

  irqrestore(saved_state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_send
 *
 * Description:
 *   This function adds the specificied message (msg) to the message queue
 *   (mqdes).  The "msglen" parameter specifies the length of the message
 *   in bytes pointed to by "msg."  This length must not exceed the maximum
 *   message length from the mq_getattr().
 *
 *   If the message queue is not full, mq_timedsend() place the message in the
 *   message queue at the position indicated by the "prio" argrument.
 *   Messages with higher priority will be inserted before lower priority
 *   messages.  The value of "prio" must not exceed MQ_PRIO_MAX.
 *
 *   If the specified message queue is full and O_NONBLOCK is not set in the
 *   message queue, then mq_timedsend() will block until space becomes available
 *   to the queue the message or a timeout occurs.
 *
 *   mq_timedsend() behaves just like mq_send(), except that if the queue
 *   is full and the O_NONBLOCK flag is not enabled for the message queue
 *   description, then abstime points to a structure which specifies a
 *   ceiling on the time for which the call will block.  This ceiling is an
 *   absolute timeout in seconds and nanoseconds since the Epoch (midnight
 *   on the morning of 1 January 1970).
 *
 *   If the message queue is full, and the timeout has already expired by
 *   the time of the call, mq_timedsend() returns immediately.
 * 
 * Parameters:
 *   mqdes - Message queue descriptor
 *   msg - Message to send
 *   msglen - The length of the message in bytes
 *   prio - The priority of the message
 *   abstime - the absolute time to wait until a timeout is decleared
 *
 * Return Value:
 *   On success, mq_send() returns 0 (OK); on error, -1 (ERROR)
 *   is returned, with errno set to indicate the error:
 *
 *   EAGAIN   The queue was empty, and the O_NONBLOCK flag was set for the
 *            message queue description referred to by mqdes.
 *   EINVAL   Either msg or mqdes is NULL or the value of prio is invalid.
 *   EPERM    Message queue opened not opened for writing.
 *   EMSGSIZE 'msglen' was greater than the maxmsgsize attribute of the
 *            message queue.
 *   EINTR    The call was interrupted by a signal handler.
 *
 * Assumptions/restrictions:
 *
 ****************************************************************************/

int mq_timedsend(mqd_t mqdes, const char *msg, size_t msglen, int prio,
                 const struct timespec *abstime)
{
  WDOG_ID      wdog;
  FAR msgq_t  *msgq;
  FAR mqmsg_t *mqmsg = NULL;
  irqstate_t   saved_state;
  int          ret = ERROR;

  DEBUGASSERT(up_interrupt_context() == false);

  /* Verify the input parameters -- setting errno appropriately
   * on any failures to verify.
   */

  if (mq_verifysend(mqdes, msg, msglen, prio) != OK)
    {
      return ERROR;
    }

  if (!abstime || abstime->tv_sec < 0 || abstime->tv_nsec > 1000000000)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Get a pointer to the message queue */

  msgq = mqdes->msgq;

  /* Create a watchdog.  We will not actually need this watchdog
   * unless the queue is full, but we will reserve it up front
   * before we enter the following critical section.
   */

  wdog = wd_create();
  if (!wdog)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Allocate a message structure:
   * - If we are called from an interrupt handler, or
   * - If the message queue is not full, or
   */

  sched_lock();
  saved_state = irqsave();
  if (up_interrupt_context()      || /* In an interrupt handler */
      msgq->nmsgs < msgq->maxmsgs)   /* OR Message queue not full */
    {
      /* Allocate the message */

      irqrestore(saved_state);
      mqmsg = mq_msgalloc();
    }
  else
    {
      int ticks;

      /* We are not in an interupt handler and the message queue is full.
       * set up a timed wait for the message queue to become non-full.
       *
       * Convert the timespec to clock ticks.  We must have interrupts
       * disabled here so that this time stays valid until the wait begins.
       */

      int result = clock_abstime2ticks(CLOCK_REALTIME, abstime, &ticks);

      /* If the time has already expired and the message queue is empty,
       * return immediately.
       */

      if (result == OK && ticks <= 0)
        {
          result = ETIMEDOUT;
        }

      /* Handle any time-related errors */

      if (result != OK)
        {
          set_errno(result);
          ret = ERROR;
        }

      /* Start the watchdog and begin the wait for MQ not full */

      if (result == OK)
        {
          /* Start the watchdog */

          wd_start(wdog, ticks, (wdentry_t)mq_sndtimeout, 1, getpid());

          /* And wait for the message queue to be non-empty */

          ret = mq_waitsend(mqdes);

          /* This may return with an error and errno set to either EINTR
           * or ETIMEOUT.  Cancel the watchdog timer in any event.
           */

          wd_cancel(wdog);
        }

      /* That is the end of the atomic operations */

      irqrestore(saved_state);

      /* If any of the above failed, set the errno.  Otherwise, there should
       * be space for another message in the message queue.  NOW we can allocate
       * the message structure.
       */

      if (ret == OK)
        {
          mqmsg = mq_msgalloc();
        }
    }

  /* Check if we were able to get a message structure -- this can fail
   * either because we cannot send the message (and didn't bother trying
   * to allocate it) or because the allocation failed.
   */

  if (mqmsg)
    {
      /* Yes, peform the message send. */

      ret = mq_dosend(mqdes, mqmsg, msg, msglen, prio);
    }

  sched_unlock();
  wd_delete(wdog);
  return ret;
}

