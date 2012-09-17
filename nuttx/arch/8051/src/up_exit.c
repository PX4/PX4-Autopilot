/****************************************************************************************
 * up_exit.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <debug.h>

#include <8052.h>
#include <nuttx/arch.h>

#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************************
 * Private Definitions
 ****************************************************************************************/

/****************************************************************************************
 * Private Data
 ****************************************************************************************/

/****************************************************************************************
 * Private Functions
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

/****************************************************************************************
 * Name: _exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 ****************************************************************************************/

void _exit(int status)
{
  FAR _TCB* tcb;

  dbg("TCB=%p exitting\n", tcb);

  /* Disable interrupts.  Interrupts will remain disabled until
   * the new task is resumed below when the save IE is restored.
   */

  EA = 0;

  /* Destroy the task at the head of the ready to run list. */

  (void)task_deletecurrent();

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = (FAR _TCB*)g_readytorun.head;
  dbg("New Active Task TCB=%p\n", tcb);

  /* Then switch contexts */

  up_restorecontext(&tcb->xcp);
}

