/****************************************************************************
 * apps/namedaps/exec_namedapp.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With updates, modifications, and general maintenance by:
 *
 *   Copyright (C) 2012 Gregory Nutt.  All rights reserved.
 *   Auther: Gregory Nutt <gnutt@nuttx.org>
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
#include <apps/apps.h>
#include <sched.h>

#include <string.h>
#include <errno.h>

#include "namedapp.h"

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: namedapp_getname
 *
 * Description:
 *   Return the name of the application at index in the table of named
 *   applications.
 *
 ****************************************************************************/

const char *namedapp_getname(int index)
{
  if (index < 0 || index >= number_namedapps())
   {
     return NULL;
   }
    
  return namedapps[index].name;
}
 
/****************************************************************************
 * Name: namedapp_isavail
 *
 * Description:
 *   Return the index into the table of applications for the applicaiton with
 *   the name 'appname'.
 *
 ****************************************************************************/

int namedapp_isavail(FAR const char *appname)
{
  int i;
    
  for (i = 0; namedapps[i].name; i++) 
    {
      if (!strcmp(namedapps[i].name, appname))
        {
          return i;
        }
    }

  set_errno(ENOENT);
  return ERROR;
}
 
/****************************************************************************
 * Name: namedapp_isavail
 *
 * Description:
 *   Execute the application with name 'appname', providing the arguments
 *   in the argv[] array.
 *
 * Returned Value:
 *   On success, the task ID of the named application is returned.  On
 *   failure, -1 (ERROR) is returned an the errno value is set appropriately.
 *
 ****************************************************************************/

int exec_namedapp(FAR const char *appname, FAR const char **argv)
{
  pid_t pid;
  int index;

  /* Verify that an application with this name exists */

  index = namedapp_isavail(appname);
  if (index >= 0)
    {
      /* Disable pre-emption.  This means that although we start the named
       * application here, it will not actually run until pre-emption is
       * re-enabled below.
       */

      sched_lock();

      /* Start the named application task */

      pid = TASK_CREATE(namedapps[index].name, namedapps[index].priority, 
                        namedapps[index].stacksize, namedapps[index].main, 
                        (argv) ? &argv[1] : (const char **)NULL);

      /* If robin robin scheduling is enabled, then set the scheduling policy
       * of the new task to SCHED_RR before it has a chance to run.
       */

#if CONFIG_RR_INTERVAL > 0
      if (pid > 0)
        {
          struct sched_param param;

          /* Pre-emption is disabled so the task creation and the
           * following operation will be atomic.  The priority of the
           * new task cannot yet have changed from its initial value.
           */

          param.sched_priority = namedapps[index].priority;
          sched_setscheduler(pid, SCHED_RR, &param);
        }
#endif
      /* Now let the named application run */

      sched_unlock();

      /* Return the task ID of the new task if the task was sucessfully
       * started.  Otherwise, pid will be ERROR (and the errno value will
       * be set appropriately).
       */

      return pid;
    }

  /* Return ERROR with errno set appropriately */

  return ERROR;
}
