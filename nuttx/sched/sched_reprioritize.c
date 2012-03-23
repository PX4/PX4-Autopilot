/****************************************************************************
 * sched/sched_reprioritize.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <sched.h>
#include <errno.h>

#include "os_internal.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

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
 * Name:  sched_reprioritize
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has a similar
 *   effect to sched_yield() -- The task will be moved to  after all other
 *   tasks with the same priority.
 *
 * Inputs:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Return Value:
 *   On success, sched_setparam() returns 0 (OK). On error, -1
 *  (ERROR) is returned, and errno is set appropriately.
 *
 *  EINVAL The parameter 'param' is invalid or does not make
 *         sense for the current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_reprioritize(FAR _TCB *tcb, int sched_priority)
{
  /* This function is equivalent to sched_setpriority() BUT it also has the
   * side effect of discarding all priority inheritance history.  This is
   * done only on explicit, user-initiated reprioritization.
   */

  int ret = sched_setpriority(tcb, sched_priority);
  if (ret == 0)
    {
       /* Reset the base_priority -- the priority that the thread would return
        * to once it posts the semaphore.
        */

       tcb->base_priority  = (uint8_t)sched_priority;

       /* Discard any pending reprioritizations as well */

#  if CONFIG_SEM_NNESTPRIO > 0
       tcb->npend_reprio   = 0;
#  endif
    }
  return ret;
}
#endif /* CONFIG_PRIORITY_INHERITANCE */
