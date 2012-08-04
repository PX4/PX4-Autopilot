/************************************************************************
 * sched/mq_notify.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <mqueue.h>
#include <sched.h>
#include <errno.h>

#include "os_internal.h"
#include "mq_internal.h"

/************************************************************************
 * Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: mq_notify
 *
 * Description:
 *   If "notification" is not NULL, this function connects the task with
 *   the message queue such that the specified signal will be sent to the
 *   task whenever the message changes from empty to non-empty.  Only one
 *   notification can be attached to a message queue.
 *
 *   If "notification" is NULL, the attached notification is detached (if
 *   it was held by the calling task) and the queue is available to attach
 *   another notification.
 *
 *   When the notification is sent to the registered process, its
 *   registration will be removed.  The message queue will then be
 *   available for registration.
 *
 * Parameters:
 *   mqdes - Message queue descriptor
 *   notification - Real-time signal structure containing:
 *      sigev_notify - Should be SIGEV_SIGNAL (but actually ignored)
 *      sigev_signo - The signo to use for the notification
 *      sigev_value - Value associated with the signal
 *
 * Return Value:
 *   On success mq_notify() returns 0; on error, -1 is returned, with
 *   errno set to indicate the error.
 *
 *   EBADF The descriptor specified in mqdes is invalid. 
 *   EBUSY Another process has already registered to receive notification
 *     for this message queue. 
 *   EINVAL sevp->sigev_notify is not one of the permitted values; or
 *     sevp->sigev_notify is SIGEV_SIGNAL and sevp->sigev_signo is not a
 *     valid signal number. 
 *   ENOMEM
 *     Insufficient memory.
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   int mq_notify(mqd_t mqdes, const struct sigevent *notification);
 *
 *   The notification will be sent to the registered task even if another
 *   task is waiting for the message queue to become non-empty.  This is
 *   inconsistent with the POSIX specification which says, "If a process
 *   has registered for notification of message a arrival at a message
 *   queue and some process is blocked in mq_receive() waiting to receive
 *   a message when a message arrives at the queue, the arriving message
 *   message shall satisfy mq_receive()... The resulting behavior is as if
 *   the message queue remains empty, and no notification shall be sent."
 *
 ************************************************************************/

int mq_notify(mqd_t mqdes, const struct sigevent *notification)
{
  _TCB   *rtcb;
  msgq_t *msgq;
  int     errval;

  /* Was a valid message queue descriptor provided? */

  if (!mqdes)
    {
      /* No.. return EBADF */

      errval = EBADF;
      goto errout;
    }

  /* Get a pointer to the message queue */

  sched_lock();
  msgq = mqdes->msgq;

  /* Get the current process ID */

  rtcb = (_TCB*)g_readytorun.head;

  /* Is there already a notification attached */

  if (!msgq->ntmqdes)
    {
      /* No... Have we been asked to establish one? */

      if (notification)
        {
          /* Yes... Was a valid signal number supplied? */

          if (!GOOD_SIGNO(notification->sigev_signo))
            {
              /* No... Return EINVAL */

              errval = EINVAL;
              goto errout;
            }

          /* Yes... Assign it to the current task. */

          msgq->ntvalue.sival_ptr = notification->sigev_value.sival_ptr;
          msgq->ntsigno           = notification->sigev_signo;
          msgq->ntpid             = rtcb->pid;
          msgq->ntmqdes           = mqdes;
        }
    }

  /* Yes... a notification is attached.  Does this task own it?
   * Is it trying to remove it?
   */

  else if ((msgq->ntpid != rtcb->pid) || (notification))
    {
      /* This thread does not own the notification OR it is
       * not trying to remove it.  Return EBUSY.
       */

      errval = EBUSY;
      goto errout;
    }
  else
    {
      /* Yes, the notification belongs to this thread.  Allow the
       * thread to detach the notification.
       */

      msgq->ntpid             = INVALID_PROCESS_ID;
      msgq->ntsigno           = 0;
      msgq->ntvalue.sival_ptr = NULL;
      msgq->ntmqdes           = NULL;
    }

  sched_unlock();
  return OK;

errout:
  set_errno(errval);
  sched_unlock();
  return ERROR;
}
