/************************************************************************
 * sched.mq_unlink.c
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <mqueue.h>
#include <sched.h>

#include "os_internal.h"
#include "mq_internal.h"

/************************************************************************
 * Pre-processor Definitions
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
 * Name: mq_unlink
 *
 * Description:
 *   This function removes the message queue named by "mq_name." If one
 *   or more tasks have the message queue open when mq_unlink() is called,
 *   removal of the message queue is postponed until all references to the
 *   message queue have been closed.
 * 
 * Parameters:
 *   mq_name - Name of the message queue
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ************************************************************************/

int mq_unlink(const char *mq_name)
{
  FAR msgq_t *msgq;
  irqstate_t  saved_state;
  int         ret = ERROR;

  /* Verify the input values */

  if (mq_name)
    {
      sched_lock();

      /* Find the named message queue */

      msgq = mq_findnamed(mq_name);
      if (msgq)
        {
          /* If it is no longer connected, then we can just
           * discard the message queue now.
           */

          if (!msgq->nconnect)
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

          /* If the message queue is still connected to a message descriptor,
           * then mark it for deletion when the last message descriptor is
           * closed
           */

          else
            {
              msgq->unlinked = true;
            }

          ret = OK;
        }

      sched_unlock();
    }

  return ret;
}

