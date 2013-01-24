/****************************************************************************
 * examples/ostest/waitpid.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/wait.h>
#include <stdbool.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>

#include "ostest.h"

#ifdef CONFIG_SCHED_WAITPID

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RETURN_STATUS 14
#define NCHILDREN     3
#define PRIORITY      100

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_waitpids[NCHILDREN];

/****************************************************************************
 * Priviate Functions
 ****************************************************************************/

static int waitpid_main(int argc, char *argv[])
{
  pid_t me = getpid();

  printf("waitpid_main: PID %d Started\n", me);
  sleep(3);
  printf("waitpid_main: PID %d exitting with result=%d\n", me, RETURN_STATUS);
  return RETURN_STATUS;
}

static void waitpid_start_children(void)
{
  int ret;
  int i;

  for (i = 0; i < NCHILDREN; i++)
    {
      ret = TASK_CREATE("waitpid", PRIORITY, STACKSIZE, waitpid_main, NULL);
      if (ret < 0)
        {
          printf("waitpid_start_child: ERROR Failed to start user_main\n");
        }
      else
        {
          printf("waitpid_start_child: Started waitpid_main at PID=%d\n", ret);
        }

      g_waitpids[i] = ret;
    }
}

static void waitpid_last(void)
{
  int stat_loc;
  int ret;

  printf("waitpid_last: Waiting for PID=%d with waitpid()\n",
         g_waitpids[NCHILDREN-1]);

  ret = (int)waitpid(g_waitpids[NCHILDREN-1], &stat_loc, 0);
  if (ret < 0)
    {
      int errcode = errno;
      printf("waitpid_last: ERROR: PID %d waitpid failed: %d\n",
             g_waitpids[NCHILDREN-1], errcode);
    }
  else if (stat_loc != RETURN_STATUS)
    {
      printf("waitpid_last: ERROR: PID %d return status is %d, expected %d\n",
             g_waitpids[NCHILDREN-1], stat_loc, RETURN_STATUS);
    }
  else
    {
      printf("waitpid_last: PID %d waitpid succeeded with stat_loc=%d\n",
             g_waitpids[NCHILDREN-1], stat_loc);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int waitpid_test(void)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  siginfo_t info;
#endif
  int stat_loc;
  int ret;

  /* Start the children and wait for first one to complete */

  printf("\nTest waitpid()\n");
  waitpid_start_children();

  printf("waitpid_test: Waiting for PID=%d with waitpid()\n", g_waitpids[0]);
  ret = (int)waitpid(g_waitpids[0], &stat_loc, 0);
  if (ret < 0)
    {
      int errcode = errno;
      printf("waitpid_test: ERROR: PID %d waitpid failed: %d\n",
             g_waitpids[0], errcode);
    }
  else if (ret != g_waitpids[0])
    {
      printf("waitpid_test: ERROR: PID %d wait returned PID %d\n",
             g_waitpids[0], ret);
    }
  else if (stat_loc != RETURN_STATUS)
    {
      printf("waitpid_test: ERROR: PID %d return status is %d, expected %d\n",
             g_waitpids[0], stat_loc, RETURN_STATUS);
    }
  else
    {
      printf("waitpid_test: PID %d waitpid succeeded with stat_loc=%d\n",
             g_waitpids[0], stat_loc);
    }

  /* Wait a big to make sure that the other threads complete */

  waitpid_last();
  sleep(1);

#ifdef CONFIG_SCHED_HAVE_PARENT
  /* Start the children and wait for first one to complete */

  printf("\nTest waitid(P_PID)\n");
  waitpid_start_children();

  printf("waitpid_test: Waiting for PID=%d with waitid()\n", g_waitpids[0]);
  ret = waitid(P_PID, (id_t)g_waitpids[0], &info, WEXITED);
  if (ret < 0)
    {
      int errcode = errno;
      printf("waitpid_test: ERROR: PID %d waitid failed: %d\n",
             g_waitpids[0], errcode);
    }
  else if (info.si_pid != g_waitpids[0])
    {
      printf("waitpid_test: ERROR: PID %d waitid returned PID %d\n",
             g_waitpids[0], info.si_pid);
    }
  else if (info.si_status != RETURN_STATUS)
    {
      printf("waitpid_test: ERROR: PID %d return status is %d, expected %d\n",
             info.si_pid, info.si_status, RETURN_STATUS);
    }
  else
    {
      printf("waitpid_test: waitid PID %d succeeded with si_status=%d\n",
             info.si_pid, info.si_status);
    }

  /* Wait a big to make sure that the other threads complete */

  waitpid_last();
  sleep(1);

  /* Start the children and wait for any one to complete */

  printf("\nTest waitid(P_ALL)\n");
  waitpid_start_children();

  printf("waitpid_test: Waiting for any child with waitid()\n");
  ret = waitid(P_ALL, 0, &info, WEXITED);
  if (ret < 0)
    {
      int errcode = errno;
      printf("waitpid_test: ERROR: waitid failed: %d\n", errcode);
    }
  else if (info.si_status != RETURN_STATUS)
    {
      printf("waitpid_test: ERROR: PID %d return status is %d, expected %d\n",
             info.si_pid, info.si_status, RETURN_STATUS);
    }
  else
    {
      printf("waitpid_test: PID %d waitid succeeded with si_status=%d\n",
             info.si_pid, info.si_status);
    }

  /* Wait a big to make sure that the other threads complete */

  waitpid_last();
  sleep(1);

  /* Start the children and wait for first one to complete */

  printf("\nTest wait()\n");
  waitpid_start_children();

  printf("waitpid_test: Waiting for any child with wait()\n");
  ret = (int)wait(&stat_loc);
  if (ret < 0)
    {
      int errcode = errno;
      printf("waitpid_test: ERROR: wait failed: %d\n", errcode);
    }
  else if (stat_loc != RETURN_STATUS)
    {
      printf("waitpid_test: ERROR: PID %d return status is %d, expected %d\n",
             ret, stat_loc, RETURN_STATUS);
    }
  else
    {
      printf("waitpid_test: PID %d wait succeeded with stat_loc=%d\n",
             ret, stat_loc);
    }

  /* Wait a big to make sure that the other threads complete */

  waitpid_last();
  sleep(1);
#endif

  return 0;
}

#endif /* CONFIG_SCHED_WAITPID */
