/****************************************************************************
 * pthread_getschedparam.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>
#include "pthread_internal.h"

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
 *****************************************************************************/

/****************************************************************************
 * Name: pthread_getschedparam
 *
 * Description:
 *   The pthread_getschedparam() functions will get the scheduling policy and
 *   parameters of threads. For SCHED_FIFO and SCHED_RR, the only required
 *   member of the sched_param structure is the priority sched_priority.
 *
 *   The pthread_getschedparam() function will retrieve the scheduling
 *   policy and scheduling parameters for the thread whose thread ID is
 *   given by 'thread' and will store those values in 'policy' and 'param',
 *   respectively. The priority value returned from pthread_getschedparam()
 *   will be the value specified by the most recent pthread_setschedparam(),
 *   pthread_setschedprio(), or pthread_create() call affecting the target
 *   thread. It will not reflect any temporary adjustments to its priority
 *   (such as might result of any priority inheritance, for example).
 *
 *   The policy parameter may have the value SCHED_FIFO, or SCHED_RR
 *   (SCHED_OTHER and SCHED_SPORADIC, in particular, are not supported).
 *   The SCHED_FIFO and SCHED_RR policies will have a single scheduling
 *   parameter, sched_priority.
*
 * Parameters:
 *   thread - The ID of thread whose scheduling parameters will be queried.
 *   policy - The location to store the thread's scheduling policy.
 *   param  - The location to store the thread's priority.
 *
 * Return Value:
 *   0 if successful.  Otherwise, the error code ESRCH if the value specified
 *   by thread does not refer to an existing thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_getschedparam(pthread_t thread, FAR int *policy,
                          FAR struct sched_param *param)
{
  int ret;

  sdbg("Thread ID=%d policy=0x%p param=0x%p\n", thread, policy, param);

  if (!policy || !param)
    {
      ret = EINVAL;
    }
  else
    {
      /* Get the schedparams of the thread. */

      ret = sched_getparam((pid_t)thread, param);
      if (ret != OK)
        {
          ret = EINVAL;
        }

      /* Return the policy. */

      *policy = sched_getscheduler((pid_t)thread);
      if (*policy == ERROR)
        {
          ret = get_errno();
        }
    }

  sdbg("Returning %d\n", ret);
  return ret;
}

