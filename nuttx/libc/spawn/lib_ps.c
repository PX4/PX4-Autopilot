/****************************************************************************
 * libc/string/lib_ps.c
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
#include <semaphore.h>
#include <signal.h>
#include <sched.h>
#include <fcntl.h>
#include <spawn.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/binfmt/binfmt.h>

#include "spawn/spawn.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spawn_parms_s
{
  int result;
  FAR pid_t *pid;
  FAR const char *path;
  FAR const posix_spawn_file_actions_t *file_actions;
  FAR const posix_spawnattr_t *attr;
  FAR char *const *argv;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_ps_parmsem = SEM_INITIALIZER(1);
#ifndef CONFIG_SCHED_WAITPID
static sem_t g_ps_execsem = SEM_INITIALIZER(0);
#endif
static struct spawn_parms_s g_ps_parms;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ps_semtake and ps_semgive
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

static void ps_semtake(FAR sem_t *sem)
{
  int ret;

  do
    {
      ret = sem_wait(sem);
      ASSERT(ret == 0 || errno == EINTR);
    }
  while (ret != 0);
}

#define ps_semgive(sem) sem_post(sem)

/****************************************************************************
 * Name: ps_exec
 *
 * Description:
 *   Execute the task from the file system.
 *
 * Input Parameters:
 *
 *   pidp - Upon successful completion, this will return the task ID of the
 *     child task in the variable pointed to by a non-NULL 'pid' argument.|
 *
 *   path - The 'path' argument identifies the file to execute.  If
 *     CONFIG_BINFMT_EXEPATH is defined, this may be either a relative or
 *     or an absolute path.  Otherwise, it must be an absolute path.
 *
 *   attr - If the value of the 'attr' parameter is NULL, the all default
 *     values for the POSIX spawn attributes will be used.  Otherwise, the
 *     attributes will be set according to the spawn flags.  The
 *     following spawn flags are supported:
 *
 *     - POSIX_SPAWN_SETSCHEDPARAM: Set new tasks priority to the sched_param
 *       value.
 *     - POSIX_SPAWN_SETSCHEDULER: Set the new tasks scheduler priority to
 *       the sched_policy value.
 *
 *     NOTE: POSIX_SPAWN_SETSIGMASK is handled in ps_proxy().
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer.
 *
 * Returned Value:
 *   This function will return zero on success. Otherwise, an error number
 *   will be returned as the function return value to indicate the error.
 *   This errno value may be that set by execv(), sched_setpolicy(), or
 *   sched_setparam().
 *
 ****************************************************************************/

static int ps_exec(FAR pid_t *pidp, FAR const char *path,
                   FAR const posix_spawnattr_t *attr,
                   FAR char *const argv[])
{
  struct sched_param param;
  FAR const struct symtab_s *symtab;
  int nsymbols;
  int pid;
  int ret = OK;

  DEBUGASSERT(path);

  /* Get the current symbol table selection */

  exec_getsymtab(&symtab, &nsymbols);

  /* Disable pre-emption so that we can modify the task parameters after
   * we start the new task; the new task will not actually begin execution
   * until we re-enable pre-emption.
   */

  sched_lock();

  /* Start the task */

  pid = exec(path, (FAR const char **)argv, symtab, nsymbols);
  if (pid < 0)
    {
      ret = errno;
      sdbg("ERROR: exec failed: %d\n", ret);
      goto errout;
    }

  /* Return the task ID to the caller */

  if (pid)
    {
      *pidp = pid;
    }

  /* Now set the attributes.  Note that we ignore all of the return values
   * here because we have already successfully started the task.  If we
   * return an error value, then we would also have to stop the task.
   */

  if (attr)
    {
      /* If we are only setting the priority, then call sched_setparm()
       * to set the priority of the of the new task.
       */

      if ((attr->flags & POSIX_SPAWN_SETSCHEDPARAM) != 0)
        {
          /* Get the priority from the attrributes */

          param.sched_priority = attr->priority;

          /* If we are setting *both* the priority and the scheduler,
           * then we will call sched_setscheduler() below.
           */

          if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) == 0)
            {
              svdbg("Setting priority=%d for pid=%d\n",
                    param.sched_priority, pid);

              (void)sched_setparam(pid, &param);
            }
        }

      /* If we are only changing the scheduling policy, then reset
       * the priority to the default value (the same as this thread) in
       * preparation for the sched_setscheduler() call below.
       */

      else if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
        {
          (void)sched_getparam(0, &param);
        }

      /* Are we setting the scheduling policy?  If so, use the priority
       * setting determined above.
       */

      if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
        {
          svdbg("Setting policy=%d priority=%d for pid=%d\n",
                attr->policy, param.sched_priority, pid);

          (void)sched_setscheduler(pid, attr->policy, &param);
        }
    }

  /* Re-enable pre-emption and return */

errout:
  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: spawn_close, spawn_dup2, and spawn_open
 *
 * Description:
 *   Implement individual file actions
 *
 * Input Parameters:
 *   action - describes the action to be performed
 *
 * Returned Value:
 *   posix_spawn() and posix_spawnp() will return zero on success.
 *   Otherwise, an error number will be returned as the function return
 *   value to indicate the error.
 *
 ****************************************************************************/

static inline int spawn_close(FAR struct spawn_close_file_action_s *action)
{
  /* The return value from close() is ignored */

  svdbg("Closing fd=%d\n", action->fd);

  (void)close(action->fd);
  return OK;
}

static inline int spawn_dup2(FAR struct spawn_dup2_file_action_s *action)
{
  int ret;

  /* Perform the dup */

  svdbg("Dup'ing %d->%d\n", action->fd1, action->fd2);

  ret = dup2(action->fd1, action->fd2);
  if (ret < 0)
    {
      int errcode = errno;

      sdbg("ERROR: dup2 failed: %d\n", errcode);
      return errcode;
    }

  return OK;
}

static inline int spawn_open(FAR struct spawn_open_file_action_s *action)
{
  int fd;
  int ret = OK;

  /* Open the file */

  svdbg("Open'ing path=%s oflags=%04x mode=%04x\n",
        action->path, action->oflags, action->mode);

  fd = open(action->path, action->oflags, action->mode);
  if (fd < 0)
    {
      ret = errno;
      sdbg("ERROR: open failed: %d\n", ret);
    }

  /* Does the return file descriptor happen to match the required file
   * desciptor number?
   */

  else if (fd != action->fd)
    {
      /* No.. dup2 to get the correct file number */

      svdbg("Dup'ing %d->%d\n", fd, action->fd);

      ret = dup2(fd, action->fd);
      if (ret < 0)
        {
          ret = errno;
          sdbg("ERROR: dup2 failed: %d\n", ret);
        }

      svdbg("Closing fd=%d\n", fd);
      close(fd);
    }

  return ret;
}

/****************************************************************************
 * Name: spawn_proxy
 *
 * Description:
 *   Perform file_actions, then execute the task from the file system.
 *
 * Input Parameters:
 *   Standard task start-up parameters
 *
 * Returned Value:
 *   Standard task return value.
 *
 ****************************************************************************/

static int spawn_proxy(int argc, char *argv[])
{
  FAR struct spawn_general_file_action_s *entry;
  FAR const posix_spawnattr_t *attr = g_ps_parms.attr;
  int ret = OK;

  /* Perform file actions and/or set a custom signal mask.  We get here only
   * if the file_actions parameter to posix_spawn[p] was non-NULL and/or the
   * option to change the signal mask was selected.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  DEBUGASSERT((g_ps_parms.file_actions && *g_ps_parms.file_actions) ||
              (attr && (attr->flags & POSIX_SPAWN_SETSIGMASK) != 0));
#else
  DEBUGASSERT(g_ps_parms.file_actions && *g_ps_parms.file_actions);
#endif

  /* Check if we need to change the signal mask */

#ifndef CONFIG_DISABLE_SIGNALS
  if (attr && (attr->flags & POSIX_SPAWN_SETSIGMASK) != 0)
    {
      (void)sigprocmask(SIG_SETMASK, &attr->sigmask, NULL);
    }

  /* Were we also requested to perform file actions? */

  if (g_ps_parms.file_actions)
#endif
    {
      /* Execute each file action */

      for (entry = (FAR struct spawn_general_file_action_s *)*g_ps_parms.file_actions;
           entry && ret == OK;
           entry = entry->flink)
        {
          switch (entry->action)
            {
            case SPAWN_FILE_ACTION_CLOSE:
              ret = spawn_close((FAR struct spawn_close_file_action_s *)entry);
              break;

            case SPAWN_FILE_ACTION_DUP2:
              ret = spawn_dup2((FAR struct spawn_dup2_file_action_s *)entry);
              break;

            case SPAWN_FILE_ACTION_OPEN:
              ret = spawn_open((FAR struct spawn_open_file_action_s *)entry);
              break;

            case SPAWN_FILE_ACTION_NONE:
            default:
              sdbg("ERROR: Unknown action: %d\n", entry->action);
              ret = EINVAL;
              break;
            }
        }
    }

  /* Check for failures */

  if (ret == OK)
    {
      /* Start the task */

      ret = ps_exec(g_ps_parms.pid, g_ps_parms.path, attr, g_ps_parms.argv);
    }

  /* Post the semaphore to inform the parent task that we have completed
   * what we need to do.
   */

  g_ps_parms.result = ret;
#ifndef CONFIG_SCHED_WAITPID
  ps_semgive(&g_ps_execsem);
#endif
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn
 *
 * Description:
 *   The posix_spawn() and posix_spawnp() functions will create a new,
 *   child task, constructed from a regular executable file.
 *
 * Input Parameters:
 *
 *   pid - Upon successful completion, posix_spawn() and posix_spawnp() will
 *     return the task ID of the child task to the parent task, in the
 *     variable pointed to by a non-NULL 'pid' argument.  If the 'pid'
 *     argument is a null pointer, the process ID of the child is not
 *     returned to the caller.
 *
 *   path - The 'path' argument to posix_spawn() is the absolute path that
 *     identifies the file to execute.  The 'path' argument to posix_spawnp()
 *     may also be a relative path and will be used to construct a pathname
 *     that identifies the file to execute.  In the case of a relative path,
 *     the path prefix for the file will be obtained by a search of the
 *     directories passed as the environment variable PATH.
 *
 *     NOTE: NuttX provides only one implementation:  If
 *     CONFIG_BINFMT_EXEPATH is defined, then only posix_spawnp() behavior
 *     is supported; otherwise, only posix_spawn behavior is supported.
 *
 *   file_actions - If 'file_actions' is a null pointer, then file 
 *     descriptors open in the calling process will remain open in the
 *     child process (unless CONFIG_FDCLONE_STDIO is defined). If
 *     'file_actions' is not NULL, then the file descriptors open in the
 *     child process will be those open in the calling process as modified
 *     by the spawn file actions object pointed to by file_actions.
 *
 *   attr - If the value of the 'attr' parameter is NULL, the all default
 *     values for the POSIX spawn attributes will be used.  Otherwise, the
 *     attributes will be set according to the spawn flags.  The
 *     posix_spawnattr_t spawn attributes object type is defined in spawn.h.
 *     It will contains these attributes, not all of which are supported by
 *     NuttX:
 *
 *     - POSIX_SPAWN_SETPGROUP:  Setting of the new task's process group is
 *       not supported.  NuttX does not support process groups.
 *     - POSIX_SPAWN_SETSCHEDPARAM: Set new tasks priority to the sched_param
 *       value.
 *     - POSIX_SPAWN_SETSCHEDULER: Set the new task's scheduler policy to
 *       the sched_policy value.
 *     - POSIX_SPAWN_RESETIDS: Resetting of the effective user ID of the child
 *       process is not supported.  NuttX does not support effective user
 *       IDs.
 *     - POSIX_SPAWN_SETSIGMASK: Set the new task's signal mask.
 *     - POSIX_SPAWN_SETSIGDEF:  Resetting signal default actions is not
 *       supported.  NuttX does not support default signal actions.
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer.
 *
 *   envp - The envp[] argument is not used by NuttX and may be NULL.  In
 *     standard implementations, envp[] is an array of character pointers to
 *     null-terminated strings that provide the environment for the new
 *     process image. The environment array is terminated by a null pointer.
 *     In NuttX, the envp[] argument is ignored and the new task will simply
 *     inherit the environment of the parent task.
 *
 * Returned Value:
 *   posix_spawn() and posix_spawnp() will return zero on success.
 *   Otherwise, an error number will be returned as the function return
 *   value to indicate the error:
 *
 *   - EINVAL: The value specified by 'file_actions' or 'attr' is invalid.
 *   - Any errors that might have been return if vfork() and excec[l|v]()
 *     had been called.
 *
 * Assumptions/Limitations:
 *   - NuttX provides only posix_spawn() or posix_spawnp() behavior
 *     depending upon the setting of CONFIG_BINFMT_EXEPATH: If
 *     CONFIG_BINFMT_EXEPATH is defined, then only posix_spawnp() behavior
 *     is supported; otherwise, only posix_spawn behavior is supported.
 *   - The 'envp' argument is not used and the 'environ' variable is not
 *     altered (NuttX does not support the 'environ' variable).
 *   - Process groups are not supported (POSIX_SPAWN_SETPGROUP).
 *   - Effective user IDs are not supported (POSIX_SPAWN_RESETIDS).
 *   - Signal default actions cannot be modified in the newly task executed
 *     because NuttX does not support default signal actions
 *     (POSIX_SPAWN_SETSIGDEF).
 *
 * POSIX Compatibility
 *   - The value of the argv[0] received by the child task is assigned by
 *     NuttX.  For the caller of posix_spawn(), the provided argv[0] will
 *     correspond to argv[1] received by the new task.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_EXEPATH
int posix_spawnp(FAR pid_t *pid, FAR const char *path,
                 FAR const posix_spawn_file_actions_t *file_actions,
                 FAR const posix_spawnattr_t *attr,
                 FAR char *const argv[], FAR char *const envp[])
#else
int posix_spawn(FAR pid_t *pid, FAR const char *path,
                FAR const posix_spawn_file_actions_t *file_actions,
                FAR const posix_spawnattr_t *attr,
                FAR char *const argv[], FAR char *const envp[])
#endif
{
  struct sched_param param;
  pid_t proxy;
#ifdef CONFIG_SCHED_WAITPID
  int status;
#endif
  int ret;

  DEBUGASSERT(path);

  svdbg("pid=%p path=%s file_actions=%p attr=%p argv=%p\n",
        pid, path, file_actions, attr, argv);

  /* If there are no file actions to be performed and there is no change to
   * the signal mask, then start the new child task directly from the parent task.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  if ((file_actions == NULL || *file_actions == NULL) &&
      (attr == NULL || (attr->flags & POSIX_SPAWN_SETSIGMASK) == 0))
#else
  if (file_actions ==  NULL || *file_actions == NULL)
#endif
    {
      return ps_exec(pid, path, attr, argv);
    }

  /* Otherwise, we will have to go through an intermediary/proxy task in order
   * to perform the I/O redirection.  This would be a natural place to fork().
   * However, true fork() behavior requires an MMU and most implementations
   * of vfork() are not capable of these operations.
   *
   * Even without fork(), we can still do the job, but parameter passing is
   * messier.  Unfortunately, there is no (clean) way to pass binary values
   * as a task parameter, so we will use a semaphore-protected global
   * structure.
   */

  /* Get exclusive access to the global parameter structure */

  ps_semtake(&g_ps_parmsem);

  /* Populate the parameter structure */

  g_ps_parms.result       = ENOSYS;
  g_ps_parms.pid          = pid;
  g_ps_parms.path         = path;
  g_ps_parms.file_actions = file_actions;
  g_ps_parms.attr         = attr;
  g_ps_parms.argv         = argv;

  /* Get the priority of this (parent) task */

  ret = sched_getparam(0, &param);
  if (ret < 0)
    {
      int errcode = errno;

      sdbg("ERROR: sched_getparam failed: %d\n", errcode);
      ps_semgive(&g_ps_parmsem);
      return errcode;
    }

  /* Start the intermediary/proxy task at the same priority as the parent task. */

  proxy = TASK_CREATE("spawn_proxy", param.sched_priority,
                      CONFIG_POSIX_SPAWN_STACKSIZE, (main_t)spawn_proxy,
                      (FAR const char **)NULL);
  if (proxy < 0)
    {
      int errcode = errno;

      sdbg("ERROR: Failed to start spawn_proxy: %d\n", errcode);
      ps_semgive(&g_ps_parmsem);
      return errcode;
    }

   /* Wait for the proxy to complete its job */

#ifdef CONFIG_SCHED_WAITPID
   ret = waitpid(proxy, &status, 0);
   if (ret < 0)
     {
      sdbg("ERROR: waitpid() failed: %d\n", errno);
     }
#else
   ps_semtake(&g_ps_execsem);
#endif

   /* Get the result and relinquish our access to the parameter structure */

   ret = g_ps_parms.result;
   ps_semgive(&g_ps_parmsem);
   return ret;
}
