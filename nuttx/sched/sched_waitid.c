/*****************************************************************************
 * sched/sched_waitid.c
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
 * Name: exited_child
 *
 * Description:
 *   Handle the case where a child exited properlay was we (apparently) lost
 *   the detch of child signal.
 *
 *****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
static void exited_child(FAR _TCB *rtcb, FAR struct child_status_s *child,
                          FAR siginfo_t *info)
{
  /* The child has exited. Return the saved exit status (and some fudged
   * information.
   */

  info->si_signo           = SIGCHLD;
  info->si_code            = CLD_EXITED;
  info->si_value.sival_ptr = NULL;
  info->si_pid             = child->ch_pid;
  info->si_status          = child->ch_status;

  /* Discard the child entry */

  (void)task_removechild(rtcb, child->ch_pid);
  task_freechild(child);
}
#endif

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: waitid
 *
 * Description:
 *   The waitid() function suspends the calling thread until one child of
 *   the process containing the calling thread changes state. It records the
 *   current state of a child in the structure pointed to by 'info'. If a
 *   child process changed state prior to the call to waitid(), waitid()
 *   returns immediately. If more than one thread is suspended in wait() or
 *   waitpid() waiting termination of the same process, exactly one thread
 *   will return the process status at the time of the target process
 *   termination
 *
 *   The idtype and id arguments are used to specify which children waitid()
 *   will wait for.
 *
 *     If idtype is P_PID, waitid() will wait for the child with a process
 *     ID equal to (pid_t)id.
 *
 *     If idtype is P_PGID, waitid() will wait for any child with a process
 *     group ID equal to (pid_t)id.
 *
 *     If idtype is P_ALL, waitid() will wait for any children and id is
 *     ignored.
 *
 *   The options argument is used to specify which state changes waitid()
 *   will will wait for. It is formed by OR-ing together one or more of the
 *   following flags:
 *
 *     WEXITED - Wait for processes that have exited. 
 *     WSTOPPED - Status will be returned for any child that has stopped
 *       upon receipt of a signal. 
 *     WCONTINUED - Status will be returned for any child that was stopped
 *       and has been continued. 
 *     WNOHANG - Return immediately if there are no children to wait for. 
 *     WNOWAIT - Keep the process whose status is returned in 'info' in a
 *       waitable state. This will not affect the state of the process; the
 *       process may be waited for again after this call completes. 
 *
 *   The 'info' argument must point to a siginfo_t structure. If waitid()
 *   returns because a child process was found that satisfied the conditions
 *   indicated by the arguments idtype and options, then the structure pointed
 *   to by 'info' will be filled in by the system with the status of the
 *   process. The si_signo member will always be equal to SIGCHLD. 
 *
 * Input Parameters:
 *   See description.
 *
 * Returned Value:
 *   If waitid() returns due to the change of state of one of its children,
 *   0 is returned. Otherwise, -1 is returned and errno is set to indicate
 *   the error. 
 *
 *   The waitid() function will fail if:
 *
 *     ECHILD - The calling process has no existing unwaited-for child
 *       processes. 
 *     EINTR - The waitid() function was interrupted by a signal. 
 *     EINVAL - An invalid value was specified for options, or idtype and id
 *       specify an invalid set of processes. 
 *
 *****************************************************************************/

int waitid(idtype_t idtype, id_t id, FAR siginfo_t *info, int options)
{
  FAR _TCB *rtcb = (FAR _TCB *)g_readytorun.head;
  FAR _TCB *ctcb;
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
  bool retains;
#endif
  sigset_t sigset;
  int err;
  int ret;

  /* MISSING LOGIC:   If WNOHANG is provided in the options, then this function
   * should returned immediately.  However, there is no mechanism available now
   * know if the thread has child:  The children remember their parents (if
   * CONFIG_SCHED_HAVE_PARENT) but the parents do not remember their children.
   */

  /* None of the options are supported except for WEXITED (which must be
   * provided.  Currently SIGCHILD always reports CLD_EXITED so we cannot
   * distinguish any other events.
   */

#ifdef CONFIG_DEBUG
  if (options != WEXITED)
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

  /* Verify that this task actually has children and that the the requested
   * TCB is actually a child of this task.
   */

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* Does this task retain child status? */

  retains = ((rtcb->flags && TCB_FLAG_NOCLDWAIT) == 0);

  if (rtcb->children == NULL && retains)
    {
      /* There are no children */

      err = ECHILD;
      goto errout_with_errno;
    }
  else if (idtype == P_PID)
    {
      /* Get the TCB corresponding to this PID and make sure it is our child. */

      ctcb = sched_gettcb((pid_t)id);
      if (!ctcb || ctcb->parent != rtcb->pid)
        {
          err = ECHILD;
          goto errout_with_errno;
        }

      /* Does this task retain child status? */

       if (retains)
        {
           /* Check if this specific pid has allocated child status? */

          if (task_findchild(rtcb, (pid_t)id) == NULL)
            {
              /* This specific pid is not a child */

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
  else if (idtype == P_PID)
    {
     /* Get the TCB corresponding to this PID and make sure it is our child. */

      ctcb = sched_gettcb((pid_t)id);
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

      DEBUGASSERT(!retains || rtcb->children);
      if (idtype == P_ALL)
        {
          /* We are waiting for any child to exit */

          if (retains && (child = task_exitchild(rtcb)) != NULL)
            {
              /* A child has exited.  Apparently we missed the signal.
               * Return the exit status and break out of the loop.
               */

              exited_child(rtcb, child, info);
              break;
            }
        }

      /* We are waiting for a specific PID.  Does this task retain child status? */

      else if (retains)
        {
          /* Yes ... Get the current status of the child task. */

          child = task_findchild(rtcb, (pid_t)id);
          DEBUGASSERT(child);
      
          /* Did the child exit? */

          if ((child->ch_flags & CHILD_FLAG_EXITED) != 0)
            {
              /* The child has exited. Return the exit status and break out
               * of the loop.
               */

              exited_child(rtcb, child, info);
              break;
            }
        }
      else
        {
          /* We can use kill() with signal number 0 to determine if that
           * task is still alive.
           */

          ret = kill((pid_t)id, 0);
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
          (idtype == P_PID && (ret = kill((pid_t)id, 0)) < 0))
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

      ret = sigwaitinfo(&sigset, info);
      if (ret < 0)
        {
          goto errout;
        }

      /* Make there this was SIGCHLD */

      if (info->si_signo == SIGCHLD)
        {
          /* Yes.. Are we waiting for the death of a specific child? */

          if (idtype == P_PID)
            {
              /* Was this the death of the thread we were waiting for? */

              if (info->si_pid == (pid_t)id)
                {
                   /* Yes... return success */

                   break;
                }
            }

          /* Are we waiting for any child to change state? */

          else if (idtype == P_ALL)
            {
              /* Return success */

              break;
            }

          /* Other ID types are not supported */

          else /* if (idtype == P_PGID) */
            {
              set_errno(ENOSYS);
              goto errout;
            }
        }
    }

  sched_unlock();
  return OK;

errout_with_errno:
  set_errno(err);
errout:
  sched_unlock();
  return ERROR;
}

#endif /* CONFIG_SCHED_WAITPID && CONFIG_SCHED_HAVE_PARENT*/
