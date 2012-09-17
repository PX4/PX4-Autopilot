/****************************************************************************
 * sched/mq_rcvinternal.c
 *
 *   Copyright (C) 2007, 2008, 2012 Gregory Nutt. All rights reserved.
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
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <mqueue.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "mq_internal.h"

/****************************************************************************
 * Definitions
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_verifyreceive
 *
 * Description:
 *   This is internal, common logic shared by both mq_receive and
 *   mq_timedreceive.  This function verifies the input parameters that are
 *   common to both functions.
 *
 * Parameters:
 *   mqdes - Message Queue Descriptor
 *   msg - Buffer to receive the message
 *   msglen - Size of the buffer in bytes
 *
 * Return Value:
 *   One success, 0 (OK) is returned. On failure, -1 (ERROR) is returned and
 *   the errno is set appropriately:
 *
 *   EPERM    Message queue opened not opened for reading.
 *   EMSGSIZE 'msglen' was less than the maxmsgsize attribute of the message
 *            queue.
 *   EINVAL   Invalid 'msg' or 'mqdes'
 *
 * Assumptions:
 *
 ****************************************************************************/

int mq_verifyreceive(mqd_t mqdes, void *msg, size_t msglen)
{
  /* Verify the input parameters */

  if (!msg || !mqdes)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if ((mqdes->oflags & O_RDOK) == 0)
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (msglen < (size_t)mqdes->msgq->maxmsgsize)
    {
      set_errno(EMSGSIZE);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: mq_waitreceive
 *
 * Description:
 *   This is internal, common logic shared by both mq_receive and
 *   mq_timedreceive.  This function waits for a message to be received on
 *   the specified message queue, removes the message from the queue, and
 *   returns it.
 *
 * Parameters:
 *   mqdes - Message queue descriptor
 *
 * Return Value:
 *   On success, a reference to the received message.  If the wait was
 *   interrupted by a signal or a timeout, then the errno will be set
 *   appropriately and NULL will be returned.
 *
 * Assumptions:
 * - The caller has provided all validity checking of the input parameters
 *   using mq_verifyreceive.
 * - Interrupts should be disabled throughout this call.  This is necessary
 *   because messages can be sent from interrupt level processing.
 * - For mq_timedreceive, setting of the timer and this wait must be atomic.
 *
 ****************************************************************************/

FAR mqmsg_t *mq_waitreceive(mqd_t mqdes)
{
  FAR _TCB    *rtcb;
  FAR msgq_t  *msgq;
  FAR mqmsg_t *rcvmsg;

  /* Get a pointer to the message queue */

  msgq = mqdes->msgq;

  /* Get the message from the head of the queue */

  while ((rcvmsg = (FAR mqmsg_t*)sq_remfirst(&msgq->msglist)) == NULL)
    {
      /* The queue is empty!  Should we block until there the above condition
       * has been satisfied?
       */

      if ((mqdes->oflags & O_NONBLOCK) == 0)
        {
          /* Yes.. Block and try again */

          rtcb = (FAR _TCB*)g_readytorun.head;
          rtcb->msgwaitq = msgq;
          msgq->nwaitnotempty++;

          set_errno(OK);
          up_block_task(rtcb, TSTATE_WAIT_MQNOTEMPTY);

          /* When we resume at this point, either (1) the message queue
           * is no longer empty, or (2) the wait has been interrupted by
           * a signal.  We can detect the latter case be examining the
           * errno value (should be either EINTR or ETIMEDOUT).
           */

          if (get_errno() != OK)
            {
              break;
            }
        }
      else
        {
          /* The queue was empty, and the O_NONBLOCK flag was set for the
           * message queue description referred to by 'mqdes'.
           */

          set_errno(EAGAIN);
          break;
        }
    }

  /* If we got message, then decrement the number of messages in
   * the queue while we are still in the critical section
   */

  if (rcvmsg)
    {
      msgq->nmsgs--;
    }

  return rcvmsg;
}

/****************************************************************************
 * Name: mq_doreceive
 *
 * Description:
 *   This is internal, common logic shared by both mq_receive and
 *   mq_timedreceive.  This function accepts the message obtained by
 *   mq_waitmsg, provides the message content to the user, notifies any
 *   threads that were waiting for the message queue to become non-full,
 *   and disposes of the message structure
 *
 * Parameters:
 *   mqdes - Message queue descriptor
 *   mqmsg   - The message obtained by mq_waitmsg()
 *   ubuffer - The address of the user provided buffer to receive the message
 *   prio    - The user-provided location to return the message priority.
 *
 * Return Value:
 *   Returns the length of the received message.  This function does not fail.
 *
 * Assumptions:
 * - The caller has provided all validity checking of the input parameters
 *   using mq_verifyreceive.
 * - The user buffer, ubuffer, is known to be large enough to accept the
 *   largest message that an be sent on this message queue
 * - Pre-emption should be disabled throughout this call.
 *
 ****************************************************************************/

ssize_t mq_doreceive(mqd_t mqdes, mqmsg_t *mqmsg, void *ubuffer, int *prio)
{
  FAR _TCB   *btcb;
  irqstate_t  saved_state;
  FAR msgq_t *msgq;
  ssize_t     rcvmsglen;

  /* Get the length of the message (also the return value) */

  rcvmsglen = mqmsg->msglen;

  /* Copy the message into the caller's buffer */

  memcpy(ubuffer, (const void*)mqmsg->mail, rcvmsglen);

  /* Copy the message priority as well (if a buffer is provided) */

  if (prio)
    {
      *prio = mqmsg->priority;
    }

  /* We are done with the message.  Deallocate it now. */

  mq_msgfree(mqmsg);

  /* Check if any tasks are waiting for the MQ not full event. */

  msgq = mqdes->msgq;
  if (msgq->nwaitnotfull > 0)
    {
      /* Find the highest priority task that is waiting for
       * this queue to be not-full in g_waitingformqnotfull list.
       * This must be performed in a critical section because
       * messages can be sent from interrupt handlers.
       */

      saved_state = irqsave();
      for (btcb = (FAR _TCB*)g_waitingformqnotfull.head;
           btcb && btcb->msgwaitq != msgq;
           btcb = btcb->flink);

      /* If one was found, unblock it.  NOTE:  There is a race
       * condition here:  the queue might be full again by the
       * time the task is unblocked
       */

      if (!btcb)
        {
          PANIC(OSERR_MQNOTFULLCOUNT);
        }
      else
        {
          btcb->msgwaitq = NULL;
          msgq->nwaitnotfull--;
          up_unblock_task(btcb);
        }

      irqrestore(saved_state);
    }

  /* Return the length of the message transferred to the user buffer */

  return rcvmsglen;
}
