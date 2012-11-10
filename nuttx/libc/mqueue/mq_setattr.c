/************************************************************************
 * libc/mqueue/mq_setattr.c
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

#include <fcntl.h>          /* O_NONBLOCK */
#include <mqueue.h>

#include <nuttx/mqueue.h>

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
 * Function:  mq_setattr
 *
 * Description:
 *   This function sets the attributes associated with the
 *   specified message queue "mqdes."  Only the "O_NONBLOCK"
 *   bit of the "mq_flags" can be changed.
 *
 *   If "oldstat" is non-null, mq_setattr() will store the
 *   previous message queue attributes at that location (just
 *   as would have been returned by mq_getattr()).
 *
 * Parameters:
 *   mqdes - Message queue descriptor
 *   mq_stat - New attributes
 *   oldstate - Old attributes
 *
 * Return Value:
 *   0 (OK) if attributes are set successfully, otherwise
 *   -1 (ERROR).
 *
 * Assumptions:
 *
 ************************************************************************/

int mq_setattr(mqd_t mqdes, const struct mq_attr *mq_stat,
               struct mq_attr *oldstat)
{
  int ret = ERROR;

  if (mqdes && mq_stat)
    {
      /* Return the attributes if so requested */

      if (oldstat)
        {
          (void)mq_getattr(mqdes, oldstat);
        }

      /* Set the new value of the O_NONBLOCK flag. */

      mqdes->oflags = ((mq_stat->mq_flags & O_NONBLOCK) |
                       (mqdes->oflags & (~O_NONBLOCK)));
      ret = OK;
    }

  return ret;
}
