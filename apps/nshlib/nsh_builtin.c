/****************************************************************************
 * apps/nshlib/nsh_builtin.c
 *
 * Originally by:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With subsequent updates, modifications, and general maintenance by:
 *
 *   Copyright (C) 2011-2013 Gregory Nutt.  All rights reserved.
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

#ifdef CONFIG_SCHED_WAITPID
#  include <sys/wait.h>
#endif

#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include <nuttx/binfmt/builtin.h>
#include <apps/builtin.h>

#include "nsh.h"
#include "nsh_console.h"

#ifdef CONFIG_NSH_BUILTIN_APPS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_builtin
 *
 * Description:
 *    Attempt to execute the application task whose name is 'cmd'
 *
 * Returned Value:
 *   <0          If exec_builtin() fails, then the negated errno value
 *               is returned.
 *   -1 (ERROR)  if the application task corresponding to 'cmd' could not
 *               be started (possibly because it doesn not exist).
 *    0 (OK)     if the application task corresponding to 'cmd' was
 *               and successfully started.  If CONFIG_SCHED_WAITPID is
 *               defined, this return value also indicates that the
 *               application returned successful status (EXIT_SUCCESS)
 *    1          If CONFIG_SCHED_WAITPID is defined, then this return value
 *               indicates that the application task was spawned successfully
 *               but returned failure exit status.
 *
 ****************************************************************************/

int nsh_builtin(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd,
                FAR char **argv, FAR const char *redirfile, int oflags)
{
  int ret = OK;

  /* Lock the scheduler in an attempt to prevent the application from
   * running until waitpid() has been called.
   */

  sched_lock();

  /* Try to find and execute the command within the list of builtin
   * applications.
   */

  ret = exec_builtin(cmd, (FAR const char **)argv, redirfile, oflags);
  if (ret >= 0)
    {
      /* The application was successfully started with pre-emption disabled.
       * In the simplest cases, the application will not have run because the
       * the scheduler is locked.  But in the case where I/O was redirected, a 
       * proxy task ran and broke our lock.  As result, the application may
       * have aso ran if its priority was higher than than the priority of
       * this thread.
       *
       * If the application did not run to completion and if the application
       * was not backgrounded, then we need to wait here for the application
       * to exit.  This only works works with the following options:
       *
       * - CONFIG_NSH_DISABLEBG - Do not run commands in background
       * - CONFIG_SCHED_WAITPID - Required to run external commands in
       *     foreground
       */

#ifdef CONFIG_SCHED_WAITPID

      /* CONFIG_SCHED_WAITPID is selected, so we may run the command in
       * foreground unless we were specifically requested to run the command
       * in background (and running commands in background is enabled).
       */

#  ifndef CONFIG_NSH_DISABLEBG
      if (vtbl->np.np_bg == false)
#  endif /* CONFIG_NSH_DISABLEBG */
        {
          int rc = 0;

          /* Wait for the application to exit.  We did lock the scheduler
           * above, but that does not guarantee that the application did not
           * already run to completion in the case where I/O was redirected.
           * Here the scheduler will be unlocked while waitpid is waiting
           * and if the application has not yet run, it will now be able to
           * do so.
           *
           * Also, if CONFIG_SCHED_HAVE_PARENT is defined waitpid() might fail
           * even if task is still active:  If the I/O was re-directed by a 
           * proxy task, then the ask is a child of the proxy, and not this
           * task.  waitpid() fails with ECHILD in either case.
           */

          ret = waitpid(ret, &rc, 0);
          if (ret < 0)
            {
              /* If the child thread does not exist, waitpid() will return
               * the error ECHLD.  Since we know that the task was successfully
               * started, this must be one of the cases described above; we
               * have to assume that the task already exit'ed.  In this case,
               * we have no idea if the application ran successfully or not
               * (because NuttX does not retain exit status of child tasks).
               * Let's assume that is did run successfully.
               */

              int errcode = errno;
              if (errcode == ECHILD)
                {
                  ret = OK;
                }
              else
                {
                  nsh_output(vtbl, g_fmtcmdfailed, cmd, "waitpid",
                             NSH_ERRNO_OF(errcode));
                }
            }

          /* Waitpid completed the wait successfully */

          else
            {
              /* We can't return the exact status (nsh has nowhere to put it)
               * so just pass back zero/nonzero in a fashion that doesn't look 
               * like an error.
               */

              ret = (rc == 0) ? OK : 1;

             /* TODO:  Set the environment variable '?' to a string corresponding
              * to WEXITSTATUS(rc) so that $? will expand to the exit status of
              * the most recently executed task.
              */
            }
        }
#  ifndef CONFIG_NSH_DISABLEBG
      else
#  endif /* CONFIG_NSH_DISABLEBG */
#endif /* CONFIG_SCHED_WAITPID */

      /* We get here if either:
       *
       * - CONFIG_SCHED_WAITPID is not selected meaning that all commands
       *   have to be run in background, or
       * - CONFIG_SCHED_WAITPID and CONFIG_NSH_DISABLEBG are both selected, but the
       *   user requested to run the command in background.
       *
       * NOTE that the case of a) CONFIG_SCHED_WAITPID is not selected and
       * b) CONFIG_NSH_DISABLEBG selected cannot be supported.  In that event, all
       * commands will have to run in background.  The waitpid() API must be
       * available to support running the command in foreground.
       */

#if !defined(CONFIG_SCHED_WAITPID) || !defined(CONFIG_NSH_DISABLEBG)
        {
          struct sched_param param;
          sched_getparam(ret, &param);
          nsh_output(vtbl, "%s [%d:%d]\n", cmd, ret, param.sched_priority);

          /* Backgrounded commands always 'succeed' as long as we can start
           * them.
           */

          ret = OK;
        }
#endif /* !CONFIG_SCHED_WAITPID || !CONFIG_NSH_DISABLEBG */
    }

  sched_unlock();

  /* If exec_builtin() or waitpid() failed, then return -1 (ERROR) with the
   * errno value set appropriately.
   */

  if (ret < 0)
    {
      return ERROR;
    }

  return ret;
}

#endif /* CONFIG_NSH_BUILTIN_APPS */
