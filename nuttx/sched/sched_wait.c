/*****************************************************************************
 * sched/sched_wait.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sys/wait.h>
#include <signal.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "os_internal.h"

#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: wait
 *
 * Description:
 *   The wait() function will suspend execution of the calling thread until
 *   status information for one of its terminated child processes is
 *   available, or until delivery of a signal whose action is either to
 *   execute a signal-catching function or to terminate the process. If more
 *   than one thread is suspended in wait() or waitpid() awaiting termination
 *   of the same process, exactly one thread will return the process status
 *   at the time of the target process termination. If status information is
 *   available prior to the call to wait(), return will be immediate.
 *
 *   The waitpid() function will behave identically to wait(), if the pid
 *   argument is (pid_t)-1 and the options argument is 0. Otherwise, its
 *   behaviour will be modified by the values of the pid and options arguments.
 *
 * Input Parameters:
 *   stat_loc - The location to return the exit status
 *
 * Returned Value:
 *   See waitpid();
 *
 *****************************************************************************/

pid_t wait(FAR int *stat_loc)
{
  return waitpid((pid_t)-1, stat_loc, 0);
}

#endif /* CONFIG_SCHED_WAITPID && CONFIG_SCHED_HAVE_PARENT*/
