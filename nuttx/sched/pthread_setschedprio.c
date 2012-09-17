/****************************************************************************
 * pthread_schedsetprio.c
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
#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/arch.h>
#include "os_internal.h"

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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_setsetprio
 *
 * Description:
 *   The pthread_setschedprio() function sets the scheduling priority for the
 *   thread whose thread ID is given by 'thread' to the value given by 'prio'.
 *   If the  thread_setschedprio() function fails, the scheduling priority
 *   of the target thread will not be changed.
 *
 * Inputs:
 *   thread - the thread ID of the task to reprioritize.
 *   prio - The new thread priority.  The range of valid priority numbers is
 *     from SCHED_PRIORITY_MIN through SCHED_PRIORITY_MAX.
 *
 * Return Value:
 *    OK if successful, otherwise an error number.  This function can
 *    fail for the following reasons:
 *
 *    EINVAL - prio is out of range.
 *    ESRCH  - thread ID does not correspond to any thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_setschedprio(pthread_t thread, int prio)
{
  struct sched_param param;
  int ret;

  /* Set the errno to some non-zero value (failsafe) */

  errno = EINVAL;

  /* Call sched_setparam() to change the priority */

  param.sched_priority = prio;
  ret = sched_setparam((pid_t)thread, &param);
  if (ret != OK)
    {
      /* If sched_setparam() fails, return the errno */

      ret = errno;
    }
  return ret;
}
