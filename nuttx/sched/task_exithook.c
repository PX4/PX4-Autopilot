/****************************************************************************
 * sched/task_exithook.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdlib.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs.h>

#include "os_internal.h"
#include "sig_internal.h"

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
 * Function: task_hook
 *
 * Description:
 *   This function implements some of the internal logic of exit() and
 *   task_delete().  This function performs some cleanup and other actions
 *   required when a task exists:
 *
 *   - All open streams are flushed and closed.
 *   - All functions registered with atexit() and on_exit() are called, in
 *     the reverse order of their registration.
 *
 *   When called from exit(), the tcb still resides at the head of the ready-
 *   to-run list.  The following logic is safe because we will not be
 *   returning from the exit() call.
 *
 *   When called from task_delete() we are operating on a different thread;
 *   on the thread that called task_delete().  In this case, task_delete
 *   will have already removed the tcb from the ready-to-run list to prevent
 *   any further action on this task.
 *
 ****************************************************************************/

void task_exithook(FAR _TCB *tcb, int status)
{
  /* Inform the instrumentation layer that the task has stopped */

  sched_note_stop(tcb);

  /* Flush all streams (File descriptors will be closed when
   * the TCB is deallocated).
   */

#if CONFIG_NFILE_STREAMS > 0
  (void)lib_flushall(tcb->streams);
#endif

  /* Deallocate anything left in the TCB's queues */

#ifndef CONFIG_DISABLE_SIGNALS
  sig_cleanup(tcb); /* Deallocate Signal lists */
#endif

  /* Wakeup any tasks waiting for this task to exit */

#ifdef CONFIG_SCHED_WAITPID /* Experimental */
  while (tcb->exitsem.semcount < 0)
    {
      /* "If more than one thread is suspended in waitpid() awaiting
       *  termination of the same process, exactly one thread will return
       *  the process status at the time of the target process termination." 
       *  Hmmm.. what do we return to the others?
       */

      if (tcb->stat_loc)
        {
          *tcb->stat_loc = status << 8;
           tcb->stat_loc = NULL;
        }

      /* Wake up the thread */

      sem_post(&tcb->exitsem);
    }
#endif

  /* If an exit function was registered, call it now.  NOTE:  In the case
   * of task_delete(), the exit function will *not* be called on the thread
   * execution of the task being deleted!
   */

#ifdef CONFIG_SCHED_ATEXIT
  if (tcb->exitfunc)
    {
      (*tcb->exitfunc)();
    }
#endif
}
