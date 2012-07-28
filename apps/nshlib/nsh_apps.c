/****************************************************************************
 * apps/nshlib/nsh_apps.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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

#include <apps/apps.h>

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
 * Name: nsh_execapp
 *
 * Description:
 *    Attempt to execute the application task whose name is 'cmd'
 *
 * Returned Value:
 *   -1 (ERRROR) if the application task corresponding to 'cmd' could not
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

int nsh_execapp(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd,
                FAR char **argv)
{
  int ret = OK;

  /* Try to find command within pre-built application list. */

  ret = exec_namedapp(cmd, (FAR const char **)argv);
  if (ret < 0)
    {
      return -errno;
    }

#ifdef CONFIG_SCHED_WAITPID
  if (vtbl->np.np_bg == false)
    {
      int rc = 0;

      waitpid(ret, &rc, 0);

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
  else
#endif
    {
      struct sched_param param;
      sched_getparam(0, &param);
      nsh_output(vtbl, "%s [%d:%d]\n", cmd, ret, param.sched_priority);

      /* Backgrounded commands always 'succeed' as long as we can start
       * them.
       */

      ret = OK;
    }

  return ret;
}

#endif /* CONFIG_NSH_BUILTIN_APPS */
