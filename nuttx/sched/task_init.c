/****************************************************************************
 * sched/task_init.c
 *
 *   Copyright (C) 2007, 2009, 2013 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "os_internal.h"
#include "group_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_init
 *
 * Description:
 *   This function initializes a Task Control Block (TCB) in preparation for
 *   starting a new thread.  It performs a subset of the functionality of
 *   task_create()
 *
 *   Unlike task_create():
 *     1. Allocate the TCB.  The pre-allocated TCB is passed in the arguments.
 *     2. Allocate the stack.  The pre-allocated stack is passed in the arguments.
 *     3. Activate the task. This must be done by calling task_activate().
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   name       - Name of the new task (not used)
 *   priority   - Priority of the new task
 *   stack      - Start of the pre-allocated stack
 *   stack_size - Size (in bytes) of the stack allocated
 *   entry      - Application start point of the new task
 *   arg        - A pointer to an array of input parameters. Up to
 *                CONFIG_MAX_TASK_ARG parameters may be provided.  If fewer
 *                than CONFIG_MAX_TASK_ARG parameters are passed, the list
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *
 * Return Value:
 *   OK on success; ERROR on failure with errno set appropriately.  (See
 *   task_schedsetup() for possible failure conditions).  On failure, the
 *   caller is responsible for freeing the stack memory and for calling
 *   sched_releasetcb() to free the TCB (which could be in most any state).
 *
 ****************************************************************************/

#ifndef CONFIG_CUSTOM_STACK
int task_init(FAR _TCB *tcb, const char *name, int priority,
              FAR uint32_t *stack, uint32_t stack_size,
              main_t entry, const char *argv[])
#else
int task_init(FAR _TCB *tcb, const char *name, int priority,
              main_t entry, const char *argv[])
#endif
{
  int errcode;
  int ret;

  /* Create a new task group */

#ifdef HAVE_TASK_GROUP
  ret = group_allocate(tcb);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout;
    }
#endif

 /* Associate file descriptors with the new task */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
  ret = sched_setuptaskfiles(tcb);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_group;
    }
#endif

  /* Configure the user provided stack region */

#ifndef CONFIG_CUSTOM_STACK
  up_use_stack(tcb, stack, stack_size);
#endif

  /* Initialize the task control block */

  ret = task_schedsetup(tcb, priority, task_start, entry,
                        TCB_FLAG_TTYPE_TASK);
  if (ret < OK)
    {
      errcode = -ret;
      goto errout_with_group;
    }

  /* Setup to pass parameters to the new task */

  (void)task_argsetup(tcb, name, argv);

  /* Now we have enough in place that we can join the group */

#ifdef HAVE_TASK_GROUP
  ret = group_initialize(tcb);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_group;
    }
#endif
  return OK;

errout_with_group:
#ifdef HAVE_TASK_GROUP
  group_leave(tcb);

errout:
#endif
  set_errno(errcode);
  return ERROR;
}

