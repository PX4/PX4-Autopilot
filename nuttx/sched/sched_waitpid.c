/*****************************************************************************
 * sched/sched_waitpid.c
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <signal.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "os_internal.h"

#ifdef CONFIG_SCHED_WAITPID

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: waitpid
 *
 * Description:
 *
 *   The waitpid() functions will obtain status information pertaining to one
 *   of the caller's child processes. The waitpid() function will suspend
 *   execution of the calling thread until status information for one of the
 *   terminated child processes of the calling process is available, or until
 *   delivery of a signal whose action is either to execute a signal-catching
 *   function or to terminate the process. If more than one thread is suspended
 *   in waitpid() awaiting termination of the same process, exactly one thread
 *   will return the process status at the time of the target process
 *   termination. If status information is available prior to the call to
 *   waitpid(), return will be immediate.
 *
 *   The pid argument specifies a set of child processes for which status is
 *   requested. The waitpid() function will only return the status of a child
 *   process from this set:
 *
 *   - If pid is equal to (pid_t)-1, status is requested for any child process.
 *     In this respect, waitpid() is then equivalent to wait().
 *   - If pid is greater than 0, it specifies the process ID of a single child
 *     process for which status is requested.
 *   - If pid is 0, status is requested for any child process whose process
 *     group ID is equal to that of the calling process.
 *   - If pid is less than (pid_t)-1, status is requested for any child process
 *     whose process group ID is equal to the absolute value of pid.
 *
 *   The options argument is constructed from the bitwise-inclusive OR of zero
 *   or more of the following flags, defined in the <sys/wait.h> header:
 *
 *   WCONTINUED - The waitpid() function will report the status of any
 *     continued child process specified by pid whose status has not been
 *     reported since it continued from a job control stop.  
 *   WNOHANG - The waitpid() function will not suspend execution of the
 *    calling thread if status is not immediately available for one of the
 *    child processes specified by pid. 
 *   WUNTRACED - The status of any child processes specified by pid that are
 *    stopped, and whose status has not yet been reported since they stopped,
 *    will also be reported to the requesting process. 
 *
 *   If the calling process has SA_NOCLDWAIT set or has SIGCHLD set to
 *   SIG_IGN, and the process has no unwaited-for children that were
 *   transformed into zombie processes, the calling thread will block until all
 *   of the children of the process containing the calling thread terminate, and
 *   waitpid() will fail and set errno to ECHILD. 
 *
 *   If waitpid() returns because the status of a child process is available,
 *   these functions will return a value equal to the process ID of the child
 *   process. In this case, if the value of the argument stat_loc is not a
 *   null pointer, information will be stored in the location pointed to by
 *   stat_loc. The value stored at the location pointed to by stat_loc will be
 *   0 if and only if the status returned is from a terminated child process
 *   that terminated by one of the following means:
 *
 *   1. The process returned 0 from main().
 *   2. The process called _exit() or exit() with a status argument of 0.
 *   3. The process was terminated because the last thread in the process terminated.
 *
 *   Regardless of its value, this information may be interpreted using the
 *   following macros, which are defined in <sys/wait.h> and evaluate to
 *   integral expressions; the stat_val argument is the integer value pointed
 *   to by stat_loc.
 *
 *   WIFEXITED(stat_val) - Evaluates to a non-zero value if status was
 *     returned for a child process that terminated normally. 
 *   WEXITSTATUS(stat_val) - If the value of WIFEXITED(stat_val) is non-zero,
 *     this macro evaluates to the low-order 8 bits of the status argument
 *     that the child process passed to _exit() or exit(), or the value the
 *     child process returned from main(). 
 *   WIFSIGNALED(stat_val) - Evaluates to a non-zero value if status was
 *     returned for a child process that terminated due to the receipt of a
 *     signal that was not caught (see <signal.h>). 
 *   WTERMSIG(stat_val)  - If the value of WIFSIGNALED(stat_val) is non-zero,
 *     this macro evaluates to the number of the signal that caused the
 *     termination of the child process. 
 *   WIFSTOPPED(stat_val) - Evaluates to a non-zero value if status was
 *     returned for a child process that is currently stopped. 
 *   WSTOPSIG(stat_val) - If the value of WIFSTOPPED(stat_val) is non-zero,
 *     this macro evaluates to the number of the signal that caused the child
 *     process to stop. 
 *   WIFCONTINUED(stat_val) - Evaluates to a non-zero value if status was
 *    returned for a child process that has continued from a job control stop.
 *
 * Parameters:
 *   pid - The task ID of the thread to waid for
 *   stat_loc - The location to return the exit status
 *   options - ignored
 *
 * Return Value:
 *   If waitpid() returns because the status of a child process is available,
 *   it will return a value equal to the process ID of the child process for
 *   which status is reported.
 *
 *   If waitpid() returns due to the delivery of a signal to the calling
 *   process, -1 will be returned and errno set to EINTR.
 *
 *   If waitpid() was invoked with WNOHANG set in options, it has at least
 *   one child process specified by pid for which status is not available, and
 *   status is not available for any process specified by pid, 0 is returned.
 *
 *   Otherwise, (pid_t)-1 will be returned, and errno set to indicate the error:
 *
 *   ECHILD - The process specified by pid does not exist or is not a child of
 *     the calling process, or the process group specified by pid does not exist
 *     or does not have any member process that is a child of the calling process. 
 *   EINTR - The function was interrupted by a signal. The value of the location
 *     pointed to by stat_loc is undefined. 
 *   EINVAL - The options argument is not valid. 
 *
 * Assumptions:
 *
 *****************************************************************************/

/***************************************************************************
 * NOTE: This is a partially functional, experimental version of waitpid()
 *
 * If there is no SIGCHLD signal supported (CONFIG_SCHED_HAVE_PARENT not
 * defined), then waitpid() is still available, but does not obey the
 * restriction that the pid be a child of the caller.
 *
 ***************************************************************************/

#ifndef CONFIG_SCHED_HAVE_PARENT
pid_t waitpid(pid_t pid, int *stat_loc, int options)
{
  _TCB *ctcb;
  bool mystat;
  int err;
  int ret;

  DEBUGASSERT(stat_loc);

  /* None of the options are supported */

#ifdef CONFIG_DEBUG
  if (options != 0)
    {
      set_errno(ENOSYS);
      return ERROR;
    }
#endif

  /* Disable pre-emption so that nothing changes in the following tests */

  sched_lock();

  /* Get the TCB corresponding to this PID */

  ctcb = sched_gettcb(pid);
  if (!ctcb)
    {
      err = ECHILD;
      goto errout_with_errno;
    }

  /* "If more than one thread is suspended in waitpid() awaiting termination of
   * the same process, exactly one thread will return the process status at the
   * time of the target process termination."  Hmmm.. what do we return to the
   * others?
   */

  if (stat_loc != NULL && ctcb->stat_loc == NULL)
    {
      ctcb->stat_loc = stat_loc;
      mystat         = true;
    }

  /* Then wait for the task to exit */
 
  ret = sem_wait(&ctcb->exitsem);
  if (ret < 0)
    {
      /* Unlock pre-emption and return the ERROR (sem_wait has already set
       * the errno).  Handle the awkward case of whether or not we need to
       * nullify the stat_loc value.
       */

      if (mystat)
        {
          ctcb->stat_loc = NULL;
        }

      goto errout;
    }

  /* On success, return the PID */

  sched_unlock();
  return pid;

errout_with_errno:
  set_errno(err);
errout:
  sched_unlock();
  return ERROR;
}

/***************************************************************************
 *
 * If CONFIG_SCHED_HAVE_PARENT is defined, then waitpid will use the SIGHCLD
 * signal.  It can also handle the pid == (pid_t)-1 arguement.  This is
 * slightly more spec-compliant.
 *
 * But then I have to be concerned about the fact that NuttX does not queue
 * signals.  This means that a flurry of signals can cause signals to be
 * lost (or to have the data in the struct siginfo to be overwritten by
 * the next signal).
 *
 ***************************************************************************/

#else
pid_t waitpid(pid_t pid, int *stat_loc, int options)
{
  FAR _TCB *rtcb = (FAR _TCB *)g_readytorun.head;
  FAR _TCB *ctcb;
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
  bool retains;
#endif
  FAR struct siginfo info;
  sigset_t sigset;
  int err;
  int ret;

  DEBUGASSERT(stat_loc);

  /* None of the options are supported */

#ifdef CONFIG_DEBUG
  if (options != 0)
    {
      set_errno(ENOSYS);
      return ERROR;
    }
#endif

  /* Create a signal set that contains only SIGCHLD */

  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, SIGCHLD);

  /* Disable pre-emption so that nothing changes while the loop executes */

  sched_lock();

  /* Verify that this task actually has children and that the requested PID
   * is actually a child of this task.
   */

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Does this task retain child status? */

  retains = ((rtcb->group->tg_flags && GROUP_FLAG_NOCLDWAIT) == 0);

  if (rtcb->group->tg_children == NULL && retains)
    {
      err = ECHILD;
      goto errout_with_errno;
    }
  else if (pid != (pid_t)-1)
    {
      /* Get the TCB corresponding to this PID and make sure it is our child. */

      ctcb = sched_gettcb(pid);
      if (!ctcb || ctcb->parent != rtcb->pid)
        {
          err = ECHILD;
          goto errout_with_errno;
        }

      /* Does this task retain child status? */

       if (retains)
        {
           /* Check if this specific pid has allocated child status? */

           if (task_findchild(rtcb, pid) == NULL)
            {
              err = ECHILD;
              goto errout_with_errno;
            }
        }
    }
#else
  if (rtcb->nchildren == 0)
    {
      /* There are no children */

      err = ECHILD;
      goto errout_with_errno;
    }
  else if (pid != (pid_t)-1)
    {
     /* Get the TCB corresponding to this PID and make sure it is our child. */

      ctcb = sched_gettcb(pid);
      if (!ctcb || ctcb->parent != rtcb->pid)
        {
          err = ECHILD;
          goto errout_with_errno;
        }
    }
#endif

  /* Loop until the child that we are waiting for dies */

  for (;;)
    {
#ifdef CONFIG_SCHED_CHILD_STATUS
      /* Check if the task has already died. Signals are not queued in
       * NuttX.  So a possibility is that the child has died and we
       * missed the death of child signal (we got some other signal
       * instead).
       */

      if (pid == (pid_t)-1)
        {
          /* We are waiting for any child, check if there are still
           * chilren.
           */

          DEBUGASSERT(!retains || rtcb->group->tg_children);
          if (retains && (child = task_exitchild(rtcb)) != NULL)
            {
              /* A child has exited.  Apparently we missed the signal.
               * Return the saved exit status.
               */

              /* The child has exited. Return the saved exit status */

              *stat_loc = child->ch_status << 8;

              /* Discard the child entry and break out of the loop */

              (void)task_removechild(rtcb, child->ch_pid);
              task_freechild(child);
              break;
            }
        }

      /* We are waiting for a specific PID. Does this task retain child status? */

      else if (retains)
        {
          /* Get the current status of the child task. */

          child = task_findchild(rtcb, pid);
          DEBUGASSERT(child);

          /* Did the child exit? */

          if ((child->ch_flags & CHILD_FLAG_EXITED) != 0)
            {
              /* The child has exited. Return the saved exit status */

              *stat_loc = child->ch_status << 8;

              /* Discard the child entry and break out of the loop */

              (void)task_removechild(rtcb, pid);
              task_freechild(child);
              break;
            }
        }
      else
        {
          /* We can use kill() with signal number 0 to determine if that
           * task is still alive.
           */

          ret = kill(pid, 0);
          if (ret < 0)
            {
              /* It is no longer running.  We know that the child task
               * was running okay when we started, so we must have lost
               * the signal.  In this case, we know that the task exit'ed,
               * but we do not know its exit status.  It would be better
               * to reported ECHILD than bogus status.
               */

              err = ECHILD;
              goto errout_with_errno;
            }
        }
#else
      /* Check if the task has already died. Signals are not queued in
       * NuttX.  So a possibility is that the child has died and we
       * missed the death of child signal (we got some other signal
       * instead).
       */

      if (rtcb->nchildren == 0 ||
          (pid != (pid_t)-1 && (ret = kill(pid, 0)) < 0))
        {
          /* We know that the child task was running okay we stared,
           * so we must have lost the signal.  What can we do?
           * Let's claim we were interrupted by a signal.
           */

          err = EINTR;
          goto errout_with_errno;
        }
#endif

      /* Wait for any death-of-child signal */

      ret = sigwaitinfo(&sigset, &info);
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      /* Was this the death of the thread we were waiting for? In the of
       * pid == (pid_t)-1, we are waiting for any child thread.
       */

      if (info.si_signo == SIGCHLD &&
         (pid == (pid_t)-1 || info.si_pid == pid))
        {
          /* Yes... return the status and PID (in the event it was -1) */

          *stat_loc = info.si_status << 8;
          pid = info.si_pid;
          break;
        }
    }

  sched_unlock();
  return (int)pid;

errout_with_errno:
  set_errno(err);

errout_with_lock:
  sched_unlock();
  return ERROR;
}
#endif /* CONFIG_SCHED_HAVE_PARENT */

#endif /* CONFIG_SCHED_WAITPID */
