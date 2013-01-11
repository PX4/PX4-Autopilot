/****************************************************************************
 * sched/mq_open.c
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
#include <stdarg.h>
#include <mqueue.h>
#include <fcntl.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

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
 * Name: mq_open
 *
 * Description:
 *   This function establish a connection between a named message queue and
 *   the calling task.  After a successful call of mq_open(), the task can
 *   reference the message queue using the address returned by the call. The
 *   message queue remains usable until it is closed by a successful call to
 *   mq_close().
 *
 * Parameters:
 *   mq_name - Name of the queue to open
 *   oflags - open flags
 *   Optional parameters.  When the O_CREAT flag is specified, two optional
 *   parameters are expected:
 *
 *     1. mode_t mode (ignored), and
 *     2. struct mq_attr *attr.  The mq_maxmsg attribute
 *        is used at the time that the message queue is
 *        created to determine the maximum number of
 *        messages that may be placed in the message queue.
 *
 * Return Value:
 *   A message queue descriptor or -1 (ERROR)
 *
 * Assumptions:
 *
 ****************************************************************************/

mqd_t mq_open(const char *mq_name, int oflags, ...)
{
  FAR _TCB       *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR msgq_t     *msgq;
  mqd_t           mqdes = NULL;
  va_list         arg;          /* Points to each un-named argument */
  struct mq_attr *attr;         /* MQ creation attributes */
  int             namelen;      /* Length of MQ name */

  /* Make sure that a non-NULL name is supplied */

  if (mq_name)
    {
      sched_lock();
      namelen = strlen(mq_name);
      if (namelen > 0)
        {
          /* See if the message queue already exists */

          msgq = mq_findnamed(mq_name);
          if (msgq)
            {
              /* It does.  Check if the caller wanted to create a new
               * message queue with this name (i.e., O_CREAT|O_EXCL)
               */

              if ((oflags & O_CREAT) == 0 || (oflags & O_EXCL) == 0)
                {
                  /* Create a message queue descriptor for the TCB */

                  mqdes = mq_descreate(rtcb, msgq, oflags);
                  if (mqdes)
                    {
                      /* Allow a new connection to the message queue */

                      msgq->nconnect++;
                    }
                }
            }

          /* It doesn't exist.  Should we create one? */

          else if ((oflags & O_CREAT) != 0)
            {
              /* Allocate memory for the new message queue.  The size to
               * allocate is the size of the msgq_t header plus the size
               * of the message queue name+1.
               */

              msgq = (FAR msgq_t*)kzalloc(SIZEOF_MQ_HEADER + namelen + 1);
              if (msgq)
                {
                  /* Create a message queue descriptor for the TCB */

                  mqdes = mq_descreate(rtcb, msgq, oflags);
                  if (mqdes)
                    {
                      /* Set up to get the optional arguments needed to create
                       * a message queue.
                       */

                      va_start(arg, oflags);
                      (void)va_arg(arg, mode_t); /* MQ creation mode parameter (ignored) */
                      attr = va_arg(arg, struct mq_attr*);

                      /* Initialize the new named message queue */

                      sq_init(&msgq->msglist);
                      if (attr)
                        {
                          msgq->maxmsgs = (int16_t)attr->mq_maxmsg;
                          if (attr->mq_msgsize <= MQ_MAX_BYTES)
                            {
                              msgq->maxmsgsize = (int16_t)attr->mq_msgsize;
                            }
                          else
                            {
                              msgq->maxmsgsize = MQ_MAX_BYTES;
                            }
                        }
                      else
                        {
                          msgq->maxmsgs = MQ_MAX_MSGS;
                          msgq->maxmsgsize = MQ_MAX_BYTES;
                        }

                      msgq->nconnect = 1;
#ifndef CONFIG_DISABLE_SIGNALS
                      msgq->ntpid    = INVALID_PROCESS_ID;
#endif
                      strcpy(msgq->name, mq_name);

                      /* Add the new message queue to the list of
                       * message queues
                       */

                      sq_addlast((FAR sq_entry_t*)msgq, &g_msgqueues);

                      /* Clean-up variable argument stuff */

                      va_end(arg);
                    }
                  else
                    {
                      /* Deallocate the msgq structure.  Since it is
                       * uninitialized, mq_deallocate() is not used.
                       */

                      sched_free(msgq);
                    }
                }
            }
        }

      sched_unlock();
    }

  if (mqdes == NULL)
    {
      return (mqd_t)ERROR;
    }
  else
    {
      return mqdes;
    }
}
