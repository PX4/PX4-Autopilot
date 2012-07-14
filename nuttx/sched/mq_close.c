/****************************************************************************
 * sched/mq_close.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#include <mqueue.h>
#include <sched.h>

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
 * Name: mq_desfree
 *
 * Description:
 *   Deallocate a message queue descriptor but returning it to the free list
 *
 * Inputs:
 *   mqdes - message queue descriptor to free
 *
 ****************************************************************************/

#define mq_desfree(mqdes) sq_addlast((FAR sq_entry_t*)mqdes, &g_desfree)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_close
 *
 * Description:
 *   This function is used to indicate that the calling task is finished
 *   with the specified message queued mqdes.  The mq_close() deallocates
 *   any system resources allocated by the system for use by this task for
 *   its message queue.
 *
 *   If the calling task has attached a notification to the message queue
 *   via this mqdes, this attachment will be removed and the message queue
 *   is available for another process to attach a notification.
 *
 * Parameters:
 *   mqdes - Message queue descriptor.
 *
 * Return Value:
 *   0 (OK) if the message queue is closed successfully,
 *   otherwise, -1 (ERROR).
 *
 * Assumptions:
 * - The behavior of a task that is blocked on either a mq_send() or
 *   mq_receive() is undefined when mq_close() is called.
 * - The results of using this message queue descriptor after a successful
 *   return from mq_close() is undefined.
 *
 ****************************************************************************/

int mq_close(mqd_t mqdes)
{
  FAR _TCB    *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR msgq_t *msgq;
  irqstate_t  saved_state;
  int         ret = ERROR;

  /* Verify the inputs */

   if (mqdes)
     {
       sched_lock();

       /* Remove the message descriptor from the current task's
        * list of message descriptors.
        */

       sq_rem((FAR sq_entry_t*)mqdes, &rtcb->msgdesq);

       /* Find the message queue associated with the message descriptor */

       msgq = mqdes->msgq;

       /* Check if the calling task has a notification attached to
        * the message queue via this mqdes.
        */

#ifndef CONFIG_DISABLE_SIGNALS
       if (msgq->ntmqdes == mqdes)
         {
           msgq->ntpid   = INVALID_PROCESS_ID;
           msgq->ntsigno = 0;
           msgq->ntvalue.sival_int = 0;
           msgq->ntmqdes = NULL;
         }
#endif

       /* Decrement the connection count on the message queue. */

       if (msgq->nconnect)
         {
           msgq->nconnect--;
         }

       /* If it is no longer connected to any message descriptor and if the
        * message queue has already been unlinked, then we can discard the
        * message queue.
        */

       if (!msgq->nconnect && msgq->unlinked)
         {
           /* Remove the message queue from the list of all
            * message queues
            */

           saved_state = irqsave();
           (void)sq_rem((FAR sq_entry_t*)msgq, &g_msgqueues);
           irqrestore(saved_state);

           /* Then deallocate it (and any messages left in it) */

           mq_msgqfree(msgq);
         }

       /* Deallocate the message descriptor */

       mq_desfree(mqdes);

       sched_unlock();
       ret = OK;
     }

   return ret;
}

