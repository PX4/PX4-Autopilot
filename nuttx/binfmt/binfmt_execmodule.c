/****************************************************************************
 * binfmt/binfmt_execmodule.c
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

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
 * Name: exec_ctors
 *
 * Description:
 *   Execute C++ static constructors.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_CONSTRUCTORS
static inline int exec_ctors(FAR const struct binary_s *binp)
{
  binfmt_ctor_t *ctor = binp->ctors;
#ifdef CONFIG_ADDRENV
  hw_addrenv_t oldenv;
  int ret;
#endif
  int i;

  /* Instantiate the address enviroment containing the constructors */

#ifdef CONFIG_ADDRENV
  ret = up_addrenv_select(binp->addrenv, &oldenv);
  if (ret < 0)
    {
      bdbg("up_addrenv_select() failed: %d\n", ret);
      return ret;
    }
#endif

  /* Execute each constructor */

  for (i = 0; i < binp->nctors; i++)
    {
      bvdbg("Calling ctor %d at %p\n", i, (FAR void *)ctor);

      (*ctor)();
      ctor++;
    }

  /* Restore the address enviroment */

#ifdef CONFIG_ADDRENV
  return up_addrenv_restore(oldenv);
#else
  return OK;
#endif
}
#endif

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

int exec_module(FAR const struct binary_s *binp, int priority)
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
  if (!binp || !binp->entrypt || binp->stacksize <= 0)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  bdbg("Executing %s\n", binp->filename);

  /* Allocate a TCB for the new task. */

  tcb = (FAR _TCB*)kzalloc(sizeof(_TCB));
  if (!tcb)
    {
      err = ENOMEM;
      goto errout;
    }

  /* Allocate the stack for the new task */

#ifndef CONFIG_CUSTOM_STACK
  stack = (FAR uint32_t*)kmalloc(binp->stacksize);
  if (!tcb)
    {
      err = ENOMEM;
      goto errout_with_tcb;
    }

  /* Initialize the task */

  ret = task_init(tcb, binp->filename, priority, stack,
                  binp->stacksize, binp->entrypt, binp->argv);
#else
  /* Initialize the task */

  ret = task_init(tcb, binp->filename, priority, stack,
                  binp->entrypt, binp->argv);
#endif
  if (ret < 0)
    {
      err = errno;
      bdbg("task_init() failed: %d\n", err);
      goto errout_with_stack;
    }

  /* Note that tcb->flags are not modified.  0=normal task */
  /* tcb->flags |= TCB_FLAG_TTYPE_TASK; */

  /* Add the D-Space address as the PIC base address.  By convention, this
   * must be the first allocated address space.
   */

#ifdef CONFIG_PIC
  tcb->dspace = binp->alloc[0];

  /* Re-initialize the task's initial state to account for the new PIC base */

  up_initial_state(tcb);
#endif

  /* Assign the address environment to the task */

#ifdef CONFIG_ADDRENV
  ret = up_addrenv_assign(binp->addrenv, tcb);
  if (ret < 0)
    {
      err = -ret;
      bdbg("up_addrenv_assign() failed: %d\n", ret);
      goto errout_with_stack;
    }
#endif

  /* Get the assigned pid before we start the task */

  pid = tcb->pid;

  /* Execute all of the C++ static constructors */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  ret = exec_ctors(binp);
  if (ret < 0)
    {
      err = -ret;
      bdbg("exec_ctors() failed: %d\n", ret);
      goto errout_with_stack;
    }
#endif

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
  kfree(stack);
#else
  sched_releasetcb(tcb);
#endif
  goto errout;

errout_with_tcb:
  kfree(tcb);
errout:
  errno = err;
  bdbg("returning errno: %d\n", err);
  return ERROR;
}

#endif /* CONFIG_BINFMT_DISABLE */

