/******************************************************************************************
 * pthread_setcancelstate.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
 ******************************************************************************************/

/******************************************************************************************
 * Included Files
 ******************************************************************************************/

#include <pthread.h>
#include <errno.h>
#include "os_internal.h"

/******************************************************************************************
 * Private Definitions
 ******************************************************************************************/

/******************************************************************************************
 * Private Types
 ******************************************************************************************/

/******************************************************************************************
 * Private Function Prototypes
 ******************************************************************************************/

/******************************************************************************************
 * Global Variables
 ******************************************************************************************/

/******************************************************************************************
 * Private Variables
 ******************************************************************************************/

/******************************************************************************************
 * Private Functions
 ******************************************************************************************/

/******************************************************************************************
 * Public Functions
 ******************************************************************************************/

/******************************************************************************************
 * Name: pthread_setcancelstate
 ******************************************************************************************/

int pthread_setcancelstate(int state, FAR int *oldstate)
{
  _TCB *tcb = (_TCB*)g_readytorun.head;
  int ret = OK;

  /* Suppress context changes for a bit so that the flags are stable. (the
   * flags should not change in interrupt handling).
   */

  sched_lock();

  /* Return the current state if so requrested */

  if (oldstate)
    {
      if ((tcb->flags & TCB_FLAG_NONCANCELABLE) != 0)
        {
          *oldstate = PTHREAD_CANCEL_DISABLE;
        }
      else
        {
          *oldstate = PTHREAD_CANCEL_ENABLE;
        }
    }

  /* Set the new cancellation state */

  if (state == PTHREAD_CANCEL_ENABLE)
    {
      unsigned flags = tcb->flags;

      /* Clear the non-cancelable and cancel pending flags */

      tcb->flags &= ~(TCB_FLAG_NONCANCELABLE|TCB_FLAG_CANCEL_PENDING);

      /* If the cancel was pending, then just exit as requested */

      if (flags & TCB_FLAG_CANCEL_PENDING)
        {
          pthread_exit(PTHREAD_CANCELED);
        }
    }
  else if (state == PTHREAD_CANCEL_DISABLE)
    {
      /* Set the non-cancelable state */

      tcb->flags |= TCB_FLAG_NONCANCELABLE;
    }
  else
    {
      ret = EINVAL;
    }

  sched_unlock();
  return ret;
}
