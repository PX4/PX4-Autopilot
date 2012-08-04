/****************************************************************************
 * sched/mq_descreate.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <mqueue.h>
#include <sched.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "os_internal.h"
#include "sig_internal.h"

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
 * Name: mq_desalloc
 *
 * Description:
 *   Allocate a message queue descriptor.
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   Reference to the allocated mq descriptor.
 *
 ****************************************************************************/

static mqd_t mq_desalloc(void)
{
  mqd_t mqdes;

  /* Try to get the message descriptorfrom the free list */

  mqdes = (mqd_t)sq_remfirst(&g_desfree);

  /* Check if we got one. */

  if (!mqdes)
    {
      /* Add another block of message descriptors to the list */

      mq_desblockalloc();

      /* And try again */

      mqdes = (mqd_t)sq_remfirst(&g_desfree);
    }

  return mqdes;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_descreate
 *
 * Description:
 *   Create a message queue descriptor for the specified TCB
 *
 * Inputs:
 *   TCB - task that needs the descriptor.
 *   msgq - Named message queue containing the message
 *   oflags - access rights for the descriptor
 *
 * Return Value:
 *   On success, the message queue descriptor is returned.  NULL is returned
 *   on a failure to allocate.
 *
 ****************************************************************************/

mqd_t mq_descreate(FAR _TCB* mtcb, FAR msgq_t* msgq, int oflags)
{
  mqd_t mqdes;

  /* Create a message queue descriptor for the TCB */

  mqdes = mq_desalloc();
  if (mqdes)
    {
      /* Initialize the MQ descriptor */

      memset(mqdes, 0, sizeof(struct mq_des));
      mqdes->msgq   = msgq;
      mqdes->oflags = oflags;

      /* And add it to the specified tasks's TCB */

      sq_addlast((FAR sq_entry_t*)mqdes, &mtcb->msgdesq);
    }

  return mqdes;
}
