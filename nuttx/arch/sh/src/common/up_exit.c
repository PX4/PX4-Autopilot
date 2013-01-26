/****************************************************************************
 * common/up_exit.c
 *
 *   Copyright (C) 2008-2009, 2013 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/arch.h>

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
#if CONFIG_NFILE_DESCRIPTORS > 0
  FAR struct filelist *filelist;
#if CONFIG_NFILE_STREAMS > 0
  FAR struct streamlist *streamlist;
#endif
  int i;
#endif

  sdbg("  TCB=%p name=%s\n", tcb, tcb->argv[0]);
  sdbg("    priority=%d state=%d\n", tcb->sched_priority, tcb->task_state);

#if CONFIG_NFILE_DESCRIPTORS > 0
  filelist = tcb->group->tg_filelist;
  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      struct inode *inode = filelist->fl_files[i].f_inode;
      if (inode)
        {
          sdbg("      fd=%d refcount=%d\n",
               i, inode->i_crefs);
        }
    }
#endif

#if CONFIG_NFILE_STREAMS > 0
  streamlist = tcb->group->tg_streamlist;
  for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
    {
      struct file_struct *filep = &streamlist->sl_streams[i];
      if (filep->fs_filedes >= 0)
        {
#if CONFIG_STDIO_BUFFER_SIZE > 0
          sdbg("      fd=%d nbytes=%d\n",
               filep->fs_filedes,
               filep->fs_bufpos - filep->fs_bufstart);
#else
          sdbg("      fd=%d\n", filep->fs_filedes);
#endif
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
  _TCB* tcb;

  /* Disable interrupts.  They will be restored when the next
   * task is started.
   */

  (void)irqsave();

  slldbg("TCB=%p exitting\n", g_readytorun.head);

#if defined(CONFIG_DUMP_ON_EXIT) && defined(CONFIG_DEBUG)
  slldbg("Other tasks:\n");
  sched_foreach(_up_dumponexit, NULL);
#endif

  /* Destroy the task at the head of the ready to run list. */

  (void)task_deletecurrent();

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = (_TCB*)g_readytorun.head;

  /* Then switch contexts */

  up_fullcontextrestore(tcb->xcp.regs);
}

