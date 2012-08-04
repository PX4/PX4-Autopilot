/**************************************************************************
 * sched/pthread_cancel.c
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include "os_internal.h"
#include "pthread_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

int pthread_cancel(pthread_t thread)
{
  _TCB *tcb;

  /* First, make sure that the handle references a valid thread */

  if (!thread)
    {
      /* pid == 0 is the IDLE task.  Callers cannot cancel the
       * IDLE task.
       */

      return ESRCH;
    }

  tcb = sched_gettcb((pid_t)thread);
  if (!tcb)
    {
      /* The pid does not correspond to any known thread */

      return ESRCH;
    }

  /* Check to see if this thread has the non-cancelable bit set in its
   * flags. Suppress context changes for a bit so that the flags are stable.
   * (the flags should not change in interrupt handling.
   */

  sched_lock();
  if ((tcb->flags & TCB_FLAG_NONCANCELABLE) != 0)
    {
      /* Then we cannot cancel the thread now.  Here is how this is
       * supposed to work:
       *
       * "When cancelability is disabled, all cancels are held pending
       *  in the target thread until the thread changes the cancelability.
       *  When cancelability is deferred, all cancels are held pending in
       *  the target thread until the thread changes the cancelability, calls
       *  a function which is a cancellation point or calls pthread_testcancel(),
       *  thus creating a cancellation point. When cancelability is asynchronous,
       *  all cancels are acted upon immediately, interrupting the thread with its
       *  processing."
       */

      tcb->flags |= TCB_FLAG_CANCEL_PENDING;
      sched_unlock();
      return OK;
    }

  sched_unlock();

  /* Check to see if the ID refers to ourselves.. this would be the
   * same as pthread_exit(PTHREAD_CANCELED).
   */

  if (tcb == (_TCB*)g_readytorun.head)
    {
      pthread_exit(PTHREAD_CANCELED);
    }

  /* Complete pending join operations */

  (void)pthread_completejoin((pid_t)thread, PTHREAD_CANCELED);

  /* Then let pthread_delete do the real work */

  task_delete((pid_t)thread);
  return OK;
}


