/****************************************************************************
 * binfmt/binfmt_execmodule.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#include <stdlib.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt.h>

#include "os_internal.h"
#include "binfmt_internal.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: exec_module
 *
 * Description:
 *   Execute a module that has been loaded into memory by load_module().
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns the PID of the exec'ed module.  On failure, it.returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

int exec_module(FAR const struct binary_s *bin, int priority)
{
  FAR _TCB     *tcb;
#ifndef CONFIG_CUSTOM_STACK
  FAR uint32_t *stack;
#endif
  pid_t         pid;
  int           err;
  int           ret;

  /* Sanity checking */

#ifdef CONFIG_DEBUG
  if (!bin || !bin->ispace || !bin->entrypt || bin->stacksize <= 0)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  bdbg("Executing %s\n", bin->filename);

  /* Allocate a TCB for the new task. */

  tcb = (FAR _TCB*)zalloc(sizeof(_TCB));
  if (!tcb)
    {
      err = ENOMEM;
      goto errout;
    }

  /* Allocate the stack for the new task */

#ifndef CONFIG_CUSTOM_STACK
  stack = (FAR uint32_t*)malloc(bin->stacksize);
  if (!tcb)
    {
      err = ENOMEM;
      goto errout_with_tcb;
    }

  /* Initialize the task */

  ret = task_init(tcb, bin->filename, priority, stack, bin->stacksize, bin->entrypt, bin->argv);
#else
  /* Initialize the task */

  ret = task_init(tcb, bin->filename, priority, stack, bin->entrypt, bin->argv);
#endif
  if (ret < 0)
    {
      err = errno;
      bdbg("task_init() failed: %d\n", err);
      goto errout_with_stack;
    }

  /* Add the DSpace address as the PIC base address */

#ifdef CONFIG_PIC
  tcb->dspace = bin->dspace;

  /* Re-initialize the task's initial state to account for the new PIC base */

  up_initial_state(tcb);
#endif

  /* Get the assigned pid before we start the task */

  pid = tcb->pid;

  /* Then activate the task at the provided priority */

  ret = task_activate(tcb);
  if (ret < 0)
    {
      err = errno;
      bdbg("task_activate() failed: %d\n", err);
      goto errout_with_stack;
    }
  return (int)pid;

errout_with_stack:
#ifndef CONFIG_CUSTOM_STACK
  tcb->stack_alloc_ptr = NULL;
  sched_releasetcb(tcb);
  free(stack);
#else
  sched_releasetcb(tcb);
#endif
  goto errout;

errout_with_tcb:
  free(tcb);
errout:
  errno = err;
  bdbg("returning errno: %d\n", err);
  return ERROR;
}

#endif /* CONFIG_BINFMT_DISABLE */

