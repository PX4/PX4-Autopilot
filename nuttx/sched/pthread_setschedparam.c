/****************************************************************************
 * pthread_setschedparam.c
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
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setschedparam
 *
 * Description:
 *   The pthread_setschedparam() functions will set the scheduling policy
 *   and parameters of threads. For SCHED_FIFO and SCHED_RR, the only
 *   required member of the sched_param structure is the priority
 *   sched_priority.
 *
 *   The pthread_setschedparam() function will set the scheduling policy
 *   and associated scheduling parameters for the thread whose thread ID
 *   is given by 'thread' to the policy and associated parameters provided
 *   in 'policy' and 'param', respectively.
 *
 *   The policy parameter may have the value SCHED_FIFO, or SCHED_RR
 *   (SCHED_OTHER and SCHED_SPORADIC, in particular, are not supported).
 *   The SCHED_FIFO and SCHED_RR policies will have a single scheduling
 *   parameter, sched_priority.
 *
 *   If the pthread_setschedparam() function fails, the scheduling parameters
 *   will not be changed for the target thread.
 *
 * Parameters:
 *   thread - The ID of thread whose scheduling parameters will be modified.
 *   policy - The new scheduling policy of the thread.  Either SCHED_FIFO or
 *            SCHED_RR. SCHED_OTHER and SCHED_SPORADIC are not supported.
 *   param  - Provides the new priority of the thread.
 *
 * Return Value:
 *   0 if successful.  Otherwise, an error code identifying the cause of the
 *   failure:
 *
 *   EINVAL  The value specified by 'policy' or one of the scheduling
 *           parameters associated with the scheduling policy 'policy' is
 *           invalid.
 *   ENOTSUP An attempt was made to set the policy or scheduling parameters
 *           to an unsupported value (SCHED_OTHER and SCHED_SPORADIC in
 *           particular are not supported)
 *   EPERM   The caller does not have the appropriate permission to set either
 *           the scheduling parameters or the scheduling policy of the
 *           specified thread. Or, the implementation does not allow the
 *           application to modify one of the parameters to the value
 *           specified.
 *   ESRCH   The value specified by thread does not refer to a existing thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_setschedparam(pthread_t thread, int policy, FAR const struct sched_param *param)
{
  int ret;

  sdbg("thread ID=%d policy=%d param=0x%p\n", thread, policy, param);

  /* Set the errno to some non-zero value (failsafe) */

  set_errno(EINVAL);

  /* Let sched_setscheduler do all of the work */

  ret = sched_setscheduler((pid_t)thread, policy, param);
  if (ret != OK)
    {
      /* If sched_setscheduler() fails, return the errno */

      ret = get_errno();
    }

  return ret;
}
