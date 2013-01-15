/****************************************************************************
 * sched/pthread_create.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/kmalloc.h>
#include <nuttx/pthread.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "clock_internal.h"
#include "env_internal.h"
#include "pthread_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Default pthread attributes */

pthread_attr_t g_default_pthread_attr = PTHREAD_ATTR_INITIALIZER;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* This is the name for name-less pthreads */

static const char g_pthreadname[] = "<pthread>";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_argsetup
 *
 * Description:
 *   This functions sets up parameters in the Task Control Block (TCB) in
 *   preparation for starting a new thread.
 *
 *   pthread_argsetup() is called from task_init() and task_start() to create
 *   a new task (with arguments cloned via strdup) or pthread_create() which
 *   has one argument passed by value (distinguished by the pthread boolean
 *   argument).
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   name       - Name of the new task (not used)
 *   argv       - A pointer to an array of input parameters. Up to
 *                CONFIG_MAX_TASK_ARG parameters may be provided. If fewer
 *                than CONFIG_MAX_TASK_ARG parameters are passed, the list
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Return Value:
 *  None
 *
 ****************************************************************************/

static void pthread_argsetup(FAR _TCB *tcb, pthread_addr_t arg)
{
  int i;

#if CONFIG_TASK_NAME_SIZE > 0
  /* Copy the pthread name into the TCB */

  strncpy(tcb->name, g_pthreadname, CONFIG_TASK_NAME_SIZE);

  /* Save the name as the first argument in the TCB */

  tcb->argv[0] = tcb->name;
#else
  /* Save the name as the first argument in the TCB */

  tcb->argv[0] = (char *)g_pthreadname;
#endif /* CONFIG_TASK_NAME_SIZE */

  /* For pthreads, args are strictly pass-by-value; that actual
   * type wrapped by pthread_addr_t is unknown.
   */

  tcb->argv[1]  = (char*)arg;

  /* Nullify the remaining, unused argument storage */

  for (i = 2; i < CONFIG_MAX_TASK_ARGS+1; i++)
    {
      tcb->argv[i] = NULL;
    }
}

/****************************************************************************
 * Name: pthread_addjoininfo
 *
 * Description:
 *   Add a join_t to the local data set.
 *
 * Parameters:
 *   pjoin
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   The caller has provided protection from re-entrancy.
 *
 ****************************************************************************/

static void pthread_addjoininfo(FAR join_t *pjoin)
{
  pjoin->next = NULL;
  if (!g_pthread_tail)
    {
      g_pthread_head = pjoin;
    }
  else
    {
      g_pthread_tail->next = pjoin;
    }

  g_pthread_tail = pjoin;
}

/****************************************************************************
 * Name:  pthread_start
 *
 * Description:
 *   This function is the low level entry point into the
 *   pthread
 *
 * Parameters:
 * None
 *
 ****************************************************************************/

static void pthread_start(void)
{
  FAR _TCB   *ptcb  = (FAR _TCB*)g_readytorun.head;
  FAR join_t *pjoin = (FAR join_t*)ptcb->joininfo;
  pthread_addr_t exit_status;

  /* Sucessfully spawned, add the pjoin to our data set.
   * Don't re-enable pre-emption until this is done.
   */

  (void)pthread_takesemaphore(&g_join_semaphore);
  pthread_addjoininfo(pjoin);
  (void)pthread_givesemaphore(&g_join_semaphore);

  /* Report to the spawner that we successfully started. */

  pjoin->started = true;
  (void)pthread_givesemaphore(&pjoin->data_sem);

  /* Pass control to the thread entry point.  The argument is
   * argv[1].  argv[0] (the thread name) and argv[2-4] are not made
   * available to the pthread.
   */

  exit_status = (*ptcb->entry.pthread)((pthread_addr_t)ptcb->argv[1]);

  /* The thread has returned */

  pthread_exit(exit_status);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_create
 *
 * Description:
 *   This function creates and activates a new thread with a specified
 *   attributes.
 *
 * Input Parameters:
 *    thread
 *    attr
 *    start_routine
 *    arg
 *
 * Returned value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set.
 *
 ****************************************************************************/

int pthread_create(FAR pthread_t *thread, FAR pthread_attr_t *attr,
                   pthread_startroutine_t start_routine, pthread_addr_t arg)
{
  FAR _TCB *ptcb;
  FAR join_t *pjoin;
  int ret;
  int priority;
#if CONFIG_RR_INTERVAL > 0
  int policy;
#endif
  pid_t pid;

  /* If attributes were not supplied, use the default attributes */

  if (!attr)
    {
      attr = &g_default_pthread_attr;
    }

  /* Allocate a TCB for the new task. */

  ptcb = (FAR _TCB*)kzalloc(sizeof(_TCB));
  if (!ptcb)
    {
      return ENOMEM;
    }

  /* Share the address environment of the parent task.  NOTE:  Only tasks
   * created throught the nuttx/binfmt loaders may have an address
   * environment.
   */

#ifdef CONFIG_ADDRENV
  ret = up_addrenv_share((FAR const _TCB *)g_readytorun.head, ptcb);
  if (ret < 0)
    {
      sched_releasetcb(ptcb);
      return -ret;
    }
#endif

  /* Associate file descriptors with the new task */

  ret = sched_setuppthreadfiles(ptcb);
  if (ret != OK)
    {
      sched_releasetcb(ptcb);
      return ret;
    }

  /* Share the parent's envionment */

  (void)env_share(ptcb);

  /* Allocate a detachable structure to support pthread_join logic */

  pjoin = (FAR join_t*)kzalloc(sizeof(join_t));
  if (!pjoin)
    {
      sched_releasetcb(ptcb);
      return ENOMEM;
    }

  /* Allocate the stack for the TCB */

  ret = up_create_stack(ptcb, attr->stacksize);
  if (ret != OK)
    {
      sched_releasetcb(ptcb);
      sched_free(pjoin);
      return ENOMEM;
    }

  /* Should we use the priority and scheduler specified in the
   * pthread attributes?  Or should we use the current thread's
   * priority and scheduler?
   */

  if (attr->inheritsched == PTHREAD_INHERIT_SCHED)
    {
      /* Get the priority for this thread. */

      struct sched_param param;
      ret = sched_getparam(0, &param);
      if (ret == OK)
        {
          priority = param.sched_priority;
        }
      else
        {
          priority = SCHED_FIFO;
        }

      /* Get the scheduler policy for this thread */

#if CONFIG_RR_INTERVAL > 0
      policy = sched_getscheduler(0);
      if (policy == ERROR)
        {
          policy = SCHED_FIFO;
        }
#endif
    }
  else
    {
      /* Use the priority and scheduler from the attributes */

      priority = attr->priority;
#if CONFIG_RR_INTERVAL > 0
      policy   = attr->policy;
#endif
    }

  /* Mark this task as a pthread (this setting will be needed in
   * task_schedsetup() when up_initial_state() is called.
   */

  ptcb->flags |= TCB_FLAG_TTYPE_PTHREAD;

  /* Initialize the task control block */

  ret  = task_schedsetup(ptcb, priority, pthread_start, (main_t)start_routine);
  if (ret != OK)
    {
      sched_releasetcb(ptcb);
      sched_free(pjoin);
      return EBUSY;
    }

  /* Configure the TCB for a pthread receiving on parameter
   * passed by value
   */

  pthread_argsetup(ptcb, arg);

  /* Attach the join info to the TCB. */

  ptcb->joininfo = (void*)pjoin;

  /* If round robin scheduling is selected, set the appropriate flag
   * in the TCB.
   */

#if CONFIG_RR_INTERVAL > 0
  if (policy == SCHED_RR)
    {
      ptcb->flags    |= TCB_FLAG_ROUND_ROBIN;
      ptcb->timeslice = CONFIG_RR_INTERVAL / MSEC_PER_TICK;
    }
#endif

  /* Get the assigned pid before we start the task (who knows what
   * could happen to ptcb after this!).  Copy this ID into the join structure
   * as well.
   */

  pid = (int)ptcb->pid;
  pjoin->thread = (pthread_t)pid;

  /* Initialize the semaphores in the join structure to zero. */

  ret = sem_init(&pjoin->data_sem, 0, 0);
  if (ret == OK)
    {
      ret = sem_init(&pjoin->exit_sem, 0, 0);
    }

  /* Activate the task */

  sched_lock();
  if (ret == OK)
    {
      ret = task_activate(ptcb);
    }

  if (ret == OK)
    {
      /* Wait for the task to actually get running and to register
       * its join_t
       */

      (void)pthread_takesemaphore(&pjoin->data_sem);

      /* Return the thread information to the caller */

      if (thread)
       {
         *thread = (pthread_t)pid;
       }

      if (!pjoin->started)
        {
          ret = EINVAL;
        }

      sched_unlock();
      (void)sem_destroy(&pjoin->data_sem);
    }
  else
    {
      sched_unlock();
      dq_rem((FAR dq_entry_t*)ptcb, (dq_queue_t*)&g_inactivetasks);
      (void)sem_destroy(&pjoin->data_sem);
      (void)sem_destroy(&pjoin->exit_sem);
      sched_releasetcb(ptcb);
      sched_free(pjoin);
      ret = EIO;
    }

  return ret;
}
