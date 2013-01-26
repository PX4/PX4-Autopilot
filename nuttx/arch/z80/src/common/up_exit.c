/****************************************************************************
 * common/up_exit.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "chip/chip.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_DUMP_ON_EXIT
#include <nuttx/fs/fs.h>
#endif

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _up_dumponexit
 *
 * Description:
 *   Dump the state of all tasks whenever on task exits.  This is debug
 *   instrumentation that was added to check file-related reference counting
 *   but could be useful again sometime in the future.
 *
 ****************************************************************************/

#if defined(CONFIG_DUMP_ON_EXIT) && defined(CONFIG_DEBUG)
static void _up_dumponexit(FAR _TCB *tcb, FAR void *arg)
{
#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NFILE_STREAMS > 0
  int i;
#endif

  lldbg("  TCB=%p name=%s\n", tcb, tcb->argv[0]);
  lldbg("    priority=%d state=%d\n", tcb->sched_priority, tcb->task_state);

#if CONFIG_NFILE_DESCRIPTORS > 0
  if (tcb->filelist)
    {
      FAR struct filelist *list = tcb->group->tg_filelist;
      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          struct inode *inode = list->fl_files[i].f_inode;
          if (inode)
            {
              lldbg("      fd=%d refcount=%d\n",
                    i, inode->i_crefs);
            }
        }
    }
#endif

#if CONFIG_NFILE_STREAMS > 0
  if (tcb->streams)
    {
      lldbg("    streamlist refcount=%d\n",
            tcb->streams->sl_crefs);

      for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
        {
          struct file_struct *filep = &tcb->streams->sl_streams[i];
          if (filep->fs_filedes >= 0)
            {
#if CONFIG_STDIO_BUFFER_SIZE > 0
              lldbg("      fd=%d nbytes=%d\n",
                    filep->fs_filedes,
                    filep->fs_bufpos - filep->fs_bufstart);
#else
              lldbg("      fd=%d\n", filep->fs_filedes);
#endif
            }
        }
    }
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 ****************************************************************************/

void _exit(int status)
{
  FAR _TCB* tcb;

  /* Disable interrupts.  Interrupts will remain disabled until
   * the new task is resumed below.
   */

  (void)irqsave();

  slldbg("TCB=%p exitting\n", tcb);

#if defined(CONFIG_DUMP_ON_EXIT) && defined(CONFIG_DEBUG)
  lldbg("Other tasks:\n");
  sched_foreach(_up_dumponexit, NULL);
#endif

  /* Destroy the task at the head of the ready to run list. */

  (void)task_deletecurrent();

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = (FAR _TCB*)g_readytorun.head;
  slldbg("New Active Task TCB=%p\n", tcb);

  /* Then switch contexts */

  RESTORE_USERCONTEXT(tcb);
}

