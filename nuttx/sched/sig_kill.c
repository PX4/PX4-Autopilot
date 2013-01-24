/************************************************************************
 * sched/sig_kill.c
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

#include <sys/types.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include "os_internal.h"
#include "sig_internal.h"

/************************************************************************
 * Global Functions
 ************************************************************************/

/************************************************************************
 * Name: kill
 *
 * Description:
 *   The kill() system call can be used to send any signal to any task.
 *
 *   Limitation: Sending of signals to 'process groups' is not
 *   supported in NuttX
 *
 * Parameters:
 *   pid - The id of the task to receive the signal.  The POSIX kill
 *     specification encodes process group information as zero and
 *     negative pid values.  Only positive, non-zero values of pid are
 *     supported by this implementation.
 *   signo - The signal number to send.  If signo is zero, no signal is
 *     sent, but all error checking is performed.
 *
 * Returned Value:
 *    On success (at least one signal was sent), zero is returned.  On
 *    error, -1 is returned, and errno is set appropriately:
 *
 *    EINVAL An invalid signal was specified.
 *    EPERM  The process does not have permission to send the
 *           signal to any of the target processes.
 *    ESRCH  The pid or process group does not exist.
 *    ENOSYS Do not support sending signals to process groups.
 *
 * Assumptions:
 *
 ************************************************************************/

int kill(pid_t pid, int signo)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR _TCB *rtcb = (FAR _TCB *)g_readytorun.head;
#endif
  FAR _TCB *stcb;
  siginfo_t info;
  int       ret = ERROR;

  /* We do not support sending signals to process groups */

  if (pid <= 0)
    {
      errno = ENOSYS;
      return ERROR;
    }

  /* Make sure that the signal is valid */

  if (!GOOD_SIGNO(signo))
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Keep things stationary through the following */

  sched_lock();

  /* Get the TCB of the receiving task */

  stcb = sched_gettcb(pid);
  sdbg("TCB=0x%08x signo=%d\n", stcb, signo);
  if (!stcb)
    {
      errno = ESRCH;
      sched_unlock();
      return ERROR;
    }

  /* Create the siginfo structure */

  info.si_signo           = signo;
  info.si_code            = SI_USER;
  info.si_value.sival_ptr = NULL;
#ifdef CONFIG_SCHED_HAVE_PARENT
  info.si_pid             = rtcb->pid;
  info.si_status          = OK;
#endif

  /* Send the signal */

  ret = sig_received(stcb, &info);
  sched_unlock();
  return ret;
}


