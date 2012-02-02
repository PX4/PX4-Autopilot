/****************************************************************************
 * apps/nshlib/nsh_proccmds.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The returned value should be zero for sucess or TRUE or non zero for
 * failure or FALSE.
 */

typedef int (*exec_t)(void);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_PS
static const char *g_statenames[] =
{
  "INVALID ",
  "PENDING ",
  "READY   ", 
  "RUNNING ", 
  "INACTIVE", 
  "WAITSEM ", 
#ifndef CONFIG_DISABLE_MQUEUE
  "WAITSIG ", 
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  "MQNEMPTY", 
  "MQNFULL "
#endif
};

static const char *g_ttypenames[4] =
{
  "TASK   ",
  "PTHREAD",
  "KTHREAD",
  "--?--  "
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ps_task
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_PS
static void ps_task(FAR _TCB *tcb, FAR void *arg)
{
  struct nsh_vtbl_s *vtbl = (struct nsh_vtbl_s*)arg;
#if CONFIG_MAX_TASK_ARGS > 2
  int i;
#endif

  /* Show task status */

  nsh_output(vtbl, "%5d %3d %4s %7s%c%c %8s ",
             tcb->pid, tcb->sched_priority,
             tcb->flags & TCB_FLAG_ROUND_ROBIN ? "RR  " : "FIFO",
             g_ttypenames[(tcb->flags & TCB_FLAG_TTYPE_MASK) >> TCB_FLAG_TTYPE_SHIFT],
             tcb->flags & TCB_FLAG_NONCANCELABLE ? 'N' : ' ',
             tcb->flags & TCB_FLAG_CANCEL_PENDING ? 'P' : ' ',
             g_statenames[tcb->task_state]);

  /* Show task name and arguments */

  nsh_output(vtbl, "%s(", tcb->argv[0]);

  /* Special case 1st argument (no comma) */

  if (tcb->argv[1])
    {
     nsh_output(vtbl, "%p", tcb->argv[1]);
    }

  /* Then any additional arguments */

#if CONFIG_MAX_TASK_ARGS > 2
  for (i = 2; i <= CONFIG_MAX_TASK_ARGS && tcb->argv[i]; i++)
    {
      nsh_output(vtbl, ", %p", tcb->argv[i]);
     }
#endif
  nsh_output(vtbl, ")\n");
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_exec
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_EXEC
int cmd_exec(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *endptr;
  uintptr_t addr;

  addr = (uintptr_t)strtol(argv[1], &endptr, 0);
  if (!addr || endptr == argv[1] || *endptr != '\0')
    {
       nsh_output(vtbl, g_fmtarginvalid, argv[0]);
       return ERROR;
    }

  nsh_output(vtbl, "Calling %p\n", (exec_t)addr);
  return ((exec_t)addr)();
}
#endif

/****************************************************************************
 * Name: cmd_ps
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLE_PS
int cmd_ps(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  nsh_output(vtbl, "PID   PRI SCHD TYPE   NP STATE    NAME\n");
  sched_foreach(ps_task, vtbl);
  return OK;
}
#endif

/****************************************************************************
 * Name: cmd_kill
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
#ifndef CONFIG_NSH_DISABLE_KILL
int cmd_kill(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *ptr;
  char *endptr;
  long signal;
  long pid;

  /* Check incoming parameters.  The first parameter should be "-<signal>" */

  ptr = argv[1];
  if (*ptr != '-' || ptr[1] < '0' || ptr[1] > '9')
    {
      goto invalid_arg;
    }

  /* Extract the signal number */

  signal = strtol(&ptr[1], &endptr, 0);

  /* The second parameter should be <pid>  */

  ptr = argv[2];
  if (*ptr < '0' || *ptr > '9')
    {
      goto invalid_arg;
    }

  /* Extract athe pid */

  pid = strtol(ptr, &endptr, 0);

  /* Send the signal.  Kill return values:
   *
   *   EINVAL An invalid signal was specified.
   *   EPERM  The process does not have permission to send the signal to any
   *          of the target processes.
   *   ESRCH  The pid or process group does not exist.
   *   ENOSYS Do not support sending signals to process groups.
   */

  if (kill((pid_t)pid, (int)signal) == 0)
    {
      return OK;
    }

  switch (errno)
    {
    case EINVAL:
      goto invalid_arg;

    case ESRCH:
      nsh_output(vtbl, g_fmtnosuch, argv[0], "task", argv[2]);
      return ERROR;

    case EPERM:
    case ENOSYS:
    default:
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "kill", NSH_ERRNO);
      return ERROR;
    }

invalid_arg:
  nsh_output(vtbl, g_fmtarginvalid, argv[0]);
  return ERROR;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_sleep
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
#ifndef CONFIG_NSH_DISABLE_SLEEP
int cmd_sleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *endptr;
  long secs;

  secs = strtol(argv[1], &endptr, 0);
  if (!secs || endptr == argv[1] || *endptr != '\0')
    {
       nsh_output(vtbl, g_fmtarginvalid, argv[0]);
       return ERROR;
    }
  sleep(secs);
  return OK;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_usleep
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
#ifndef CONFIG_NSH_DISABLE_USLEEP
int cmd_usleep(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *endptr;
  long usecs;

  usecs = strtol(argv[1], &endptr, 0);
  if (!usecs || endptr == argv[1] || *endptr != '\0')
    {
       nsh_output(vtbl, g_fmtarginvalid, argv[0]);
       return ERROR;
    }
  usleep(usecs);
  return OK;
}
#endif
#endif
