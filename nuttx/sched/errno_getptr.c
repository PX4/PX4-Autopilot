/****************************************************************************
 * sched/errno_getptr.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <errno.h>
#include <nuttx/arch.h>
#include "os_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef get_errno_ptr
#undef errno

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_irqerrno;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_errno_ptr
 *
 * Description:
 *   Return a pointer to the thread specific errno.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   A pointer to the per-thread errno variable.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR int *get_errno_ptr(void)
{
  /* Check if this function was called from an interrupt handler.  In that
   * case, we have to do things a little differently to prevent the interrupt
   * handler from modifying the tasks errno value.
   */

  if (!up_interrupt_context())
    {
      /* We were called from the normal tasking context.  Verify that the
       * task at the head of the ready-to-run list is actually running.  It
       * may not be running during very brief times during context switching
       * logic (see, for example, task_deletecurrent.c).
       */

      FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
      if (rtcb->task_state == TSTATE_TASK_RUNNING)
        {
          /* Yes.. the task is running normally.  Return a reference to the
           * thread-private errno in the TCB of the running task.
           */

          return &rtcb->pterrno;
        }
    }

  /* We were called either from (1) an interrupt handler or (2) from normally
   * code but in an unhealthy state.  In either event, do not permit access to
   * the errno in the TCB of the task at the head of the ready-to-run list.
   * Instead, use a separate errno just for interrupt handlers.  Of course, this
   * would have to change if we ever wanted to support nested interrupts or if
   * we really cared about the stability of the errno during those "unhealthy
   * states."
   */

  return &g_irqerrno;
}


