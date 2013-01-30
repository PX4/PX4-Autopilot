/****************************************************************************
 * apps/builtin/exec_builtin.c
 *
 * Originally by:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With subsequent updates, modifications, and general maintenance by:
 *
 *   Copyright (C) 2012-2013 Gregory Nutt.  All rights reserved.
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
#include <sched.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/binfmt/builtin.h>
#include <apps/builtin.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BUILTIN_PROXY_STACKSIZE
#  define CONFIG_BUILTIN_PROXY_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct builtin_parms_s
{
  /* Input values */

  FAR const char *redirfile;
  FAR const char **argv;
  int oflags;
  int index;

  /* Returned values */

  pid_t result;
  int errcode;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_builtin_parmsem = SEM_INITIALIZER(1);
#ifndef CONFIG_SCHED_WAITPID
static sem_t g_builtin_execsem = SEM_INITIALIZER(0);
#endif
static struct builtin_parms_s g_builtin_parms;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bultin_semtake and builtin_semgive
 *
 * Description:
 *   Give and take semaphores
 *
 * Input Parameters:
 *
 *   sem - The semaphore to act on.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bultin_semtake(FAR sem_t *sem)
{
  int ret;

  do
    {
      ret = sem_wait(sem);
      ASSERT(ret == 0 || get_errno() == EINTR);
    }
  while (ret != 0);
}

#define builtin_semgive(sem) sem_post(sem)

/****************************************************************************
 * Name: builtin_taskcreate
 *
 * Description:
 *   Execute the builtin task
 *
 * Returned Value:
 *   On success, the task ID of the builtin task is returned; On failure, -1
 *  (ERROR) is returned and the errno is set appropriately.
 *
 ****************************************************************************/

static int builtin_taskcreate(int index, FAR const char **argv)
{
  FAR const struct builtin_s *b;
  int ret;

  b = builtin_for_index(index);

  if (b == NULL)
    { 
      set_errno(ENOENT);
      return ERROR;
    }

  /* Disable pre-emption.  This means that although we start the builtin
   * application here, it will not actually run until pre-emption is
   * re-enabled below.
   */

  sched_lock();

  /* Start the builtin application task */

  ret = TASK_CREATE(b->name, b->priority, b->stacksize, b->main,
                    (argv) ? &argv[1] : (FAR const char **)NULL);

  /* If robin robin scheduling is enabled, then set the scheduling policy
   * of the new task to SCHED_RR before it has a chance to run.
   */

#if CONFIG_RR_INTERVAL > 0
  if (ret > 0)
    {
      struct sched_param param;

      /* Pre-emption is disabled so the task creation and the
       * following operation will be atomic.  The priority of the
       * new task cannot yet have changed from its initial value.
       */

      param.sched_priority = b->priority;
      (void)sched_setscheduler(ret, SCHED_RR, &param);
    }
#endif

  /* Now let the builtin application run */

  sched_unlock();

  /* Return the task ID of the new task if the task was sucessfully
   * started.  Otherwise, ret will be ERROR (and the errno value will
   * be set appropriately).
   */

  return ret;
}

/****************************************************************************
 * Name: builtin_proxy
 *
 * Description:
 *   Perform output redirection, then execute the builtin task.
 *
 * Input Parameters:
 *   Standard task start-up parameters
 *
 * Returned Value:
 *   Standard task return value.
 *
 ****************************************************************************/

static int builtin_proxy(int argc, char *argv[])
{
  int fd;
  int ret = ERROR;

  /* Open the output file for redirection */

  svdbg("Open'ing redirfile=%s oflags=%04x mode=0644\n",
        g_builtin_parms.redirfile, g_builtin_parms.oflags);

  fd = open(g_builtin_parms.redirfile, g_builtin_parms.oflags, 0644);
  if (fd < 0)
    {
      /* Remember the errno value.  ret is already set to ERROR */

      g_builtin_parms.errcode = get_errno();
      sdbg("ERROR: open of %s failed: %d\n",
           g_builtin_parms.redirfile, g_builtin_parms.errcode);
    }

  /* Does the return file descriptor happen to match the required file
   * desciptor number?
   */

  else if (fd != 1)
    {
      /* No.. dup2 to get the correct file number */

      svdbg("Dup'ing %d->1\n", fd);

      ret = dup2(fd, 1);
      if (ret < 0)
        {
          g_builtin_parms.errcode = get_errno();
          sdbg("ERROR: dup2 failed: %d\n", g_builtin_parms.errcode);
        }

      svdbg("Closing fd=%d\n", fd);
      close(fd);
    }

  /* Was the setup successful? */

  if (ret == OK)
    {
      /* Yes.. Start the task.  On success, the task ID of the builtin task
       * is returned; On failure, -1 (ERROR) is returned and the errno
       * is set appropriately.
       */

      ret = builtin_taskcreate(g_builtin_parms.index, g_builtin_parms.argv);
      if (ret < 0)
        {
          g_builtin_parms.errcode = get_errno();
          sdbg("ERROR: builtin_taskcreate failed: %d\n",
               g_builtin_parms.errcode);
        }
    }

  /* NOTE:  There is a logical error here if CONFIG_SCHED_HAVE_PARENT is
   * defined:  The new task is the child of this proxy task, not the
   * original caller.  As a consequence, operations like waitpid() will
   * fail on the caller's thread.
   */

  /* Post the semaphore to inform the parent task that we have completed
   * what we need to do.
   */

  g_builtin_parms.result = ret;
#ifndef CONFIG_SCHED_WAITPID
  builtin_semgive(&g_builtin_execsem);
#endif
  return 0;
}

/****************************************************************************
 * Name: builtin_startproxy
 *
 * Description:
 *   Perform output redirection, then execute the builtin task.
 *
 * Input Parameters:
 *   Standard task start-up parameters
 *
 * Returned Value:
 *   On success, the task ID of the builtin task is returned; On failure, -1
 *  (ERROR) is returned and the errno is set appropriately.
 *
 ****************************************************************************/

static inline int builtin_startproxy(int index, FAR const char **argv,
                                     FAR const char *redirfile, int oflags)
{
  struct sched_param param;
  pid_t proxy;
  int errcode;
#ifdef CONFIG_SCHED_WAITPID
  int status;
#endif
  int ret;

  svdbg("index=%d argv=%p redirfile=%s oflags=%04x\n",
        index, argv, redirfile, oflags);

  /* We will have to go through an intermediary/proxy task in order to
   * perform the I/O redirection.  This would be a natural place to fork().
   * However, true fork() behavior requires an MMU and most implementations
   * of vfork() are not capable of these operations.
   *
   * Even without fork(), we can still do the job, but parameter passing is
   * messier.  Unfortunately, there is no (clean) way to pass binary values
   * as a task parameter, so we will use a semaphore-protected global
   * structure.
   */

  /* Get exclusive access to the global parameter structure */

  bultin_semtake(&g_builtin_parmsem);

  /* Populate the parameter structure */

  g_builtin_parms.redirfile = redirfile;
  g_builtin_parms.argv      = argv;
  g_builtin_parms.result    = ERROR;
  g_builtin_parms.oflags    = oflags;
  g_builtin_parms.index     = index;

  /* Get the priority of this (parent) task */

  ret = sched_getparam(0, &param);
  if (ret < 0)
    {
      errcode = get_errno();
      sdbg("ERROR: sched_getparam failed: %d\n", errcode);
      goto errout_with_sem;
    }

  /* Disable pre-emption so that the proxy does not run until we waitpid
   * is called.  This is probably unnecessary since the builtin_proxy has
   * the same priority as this thread; it should be schedule behind this
   * task in the ready-to-run list.
   */

#ifdef CONFIG_SCHED_WAITPID
  sched_lock();
#endif

  /* Start the intermediary/proxy task at the same priority as the parent task. */

  proxy = TASK_CREATE("builtin_proxy", param.sched_priority,
                      CONFIG_BUILTIN_PROXY_STACKSIZE, (main_t)builtin_proxy,
                      (FAR const char **)NULL);
  if (proxy < 0)
    {
      errcode = get_errno();
      sdbg("ERROR: Failed to start builtin_proxy: %d\n", errcode);
      goto errout_with_lock;
    }

   /* Wait for the proxy to complete its job.  We could use waitpid()
    * for this.
    */

#ifdef CONFIG_SCHED_WAITPID
   ret = waitpid(proxy, &status, 0);
   if (ret < 0)
     {
       sdbg("ERROR: waitpid() failed: %d\n", get_errno());
       goto errout_with_lock;
     }
#else
   bultin_semtake(&g_builtin_execsem);
#endif

   /* Get the result and relinquish our access to the parameter structure */

   set_errno(g_builtin_parms.errcode);
   builtin_semgive(&g_builtin_parmsem);
   return g_builtin_parms.result;

errout_with_lock:
#ifdef CONFIG_SCHED_WAITPID
  sched_unlock();
#endif

errout_with_sem:
  set_errno(errcode);
  builtin_semgive(&g_builtin_parmsem);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_builtin
 *
 * Description:
 *   Executes builtin applications registered during 'make context' time.
 *   New application is run in a separate task context (and thread).
 *
 * Input Parameter:
 *   filename  - Name of the linked-in binary to be started.
 *   argv      - Argument list
 *   redirfile - If output if redirected, this parameter will be non-NULL
 *               and will provide the full path to the file.
 *   oflags    - If output is redirected, this parameter will provide the
 *               open flags to use.  This will support file replacement
 *               of appending to an existing file.
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns the PID of the exec'ed module.  On failure, it.returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

int exec_builtin(FAR const char *appname, FAR const char **argv,
                 FAR const char *redirfile, int oflags)
{
  int index;
  int ret = ERROR;

  /* Verify that an application with this name exists */

  index = builtin_isavail(appname);
  if (index >= 0)
    {
      /* Is output being redirected? */

      if (redirfile)
        {
          ret = builtin_startproxy(index, argv, redirfile, oflags);
        }
      else
        {
          /* Start the builtin application task */

          ret = builtin_taskcreate(index, argv);
        }
    }


  /* Return the task ID of the new task if the task was sucessfully
   * started.  Otherwise, ret will be ERROR (and the errno value will
   * be set appropriately).
   */

  return ret;
}
