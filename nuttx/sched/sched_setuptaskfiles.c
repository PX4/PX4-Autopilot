/****************************************************************************
 * sched/sched_setuptaskfiles.c
 *
 *   Copyright (C) 2007-2008, 2010, 2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "os_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine how many file descriptors to clone.  If CONFIG_FDCLONE_DISABLE
 * is set, no file descriptors will be cloned.  If CONFIG_FDCLONE_STDIO is
 * set, only the first three descriptors (stdin, stdout, and stderr) will
 * be cloned.  Otherwise all file descriptors will be cloned.
 */

#if defined(CONFIG_FDCLONE_STDIO) && CONFIG_NFILE_DESCRIPTORS > 3
#  define NFDS_TOCLONE 3
#else
#  define NFDS_TOCLONE CONFIG_NFILE_DESCRIPTORS
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_dupfiles
 *
 * Description:
 *   Duplicate parent task's file descriptors.
 *
 * Input Parameters:
 *   tcb - tcb of the new task.
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_FDCLONE_DISABLE)
static inline void sched_dupfiles(FAR _TCB *tcb)
{
  /* The parent task is the one at the head of the ready-to-run list */

  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR struct file *parent;
  FAR struct file *child;
  int i;

  /* Duplicate the file descriptors.  This will be either all of the
   * file descriptors or just the first three (stdin, stdout, and stderr)
   * if CONFIG_FDCLONE_STDIO is defined.  NFSDS_TOCLONE is set
   * accordingly above.
   */

  if (rtcb->filelist)
    {
      /* Get pointers to the parent and child task file lists */

      parent = rtcb->filelist->fl_files;
      child  = tcb->filelist->fl_files;

      /* Check each file in the parent file list */

      for (i = 0; i < NFDS_TOCLONE; i++)
        {
          /* Check if this file is opened by the parent.  We can tell if
           * if the file is open because it contain a reference to a non-NULL
           * i-node structure.
           */

          if (parent[i].f_inode)
            {
              /* Yes... duplicate it for the child */

              (void)files_dup(&parent[i], &child[i]);
            }
        }
    }
}
#else /* CONFIG_NFILE_DESCRIPTORS && !CONFIG_FDCLONE_DISABLE */
#  define sched_dupfiles(tcb)
#endif /* CONFIG_NFILE_DESCRIPTORS && !CONFIG_FDCLONE_DISABLE */

/****************************************************************************
 * Name: sched_dupsockets
 *
 * Description:
 *   Duplicate the parent task's socket descriptors.
 *
 * Input Parameters:
 *   tcb - tcb of the new task.
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0 && !defined(CONFIG_SDCLONE_DISABLE)
static inline void sched_dupsockets(FAR _TCB *tcb)
{
  /* The parent task is the one at the head of the ready-to-run list */

  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  FAR struct socket *parent;
  FAR struct socket *child;
  int i;

 /* Duplicate the socket descriptors of all sockets opened by the parent
  * task.
  */

  if (rtcb->sockets)
    {
      /* Get pointers to the parent and child task socket lists */

      parent = rtcb->sockets->sl_sockets;
      child  = tcb->sockets->sl_sockets;

      /* Check each socket in the parent socket list */

      for (i = 0; i < CONFIG_NSOCKET_DESCRIPTORS; i++)
        {
          /* Check if this parent socket is allocated.  We can tell if the
           * socket is allocated because it will have a positive, non-zero
           * reference count.
           */

          if (parent[i].s_crefs > 0)
            {
              /* Yes... duplicate it for the child */

              (void)net_clone(&parent[i], &child[i]);
            }
        }
    }
}
#else /* CONFIG_NSOCKET_DESCRIPTORS && !CONFIG_SDCLONE_DISABLE */
#  define sched_dupsockets(tcb)
#endif /* CONFIG_NSOCKET_DESCRIPTORS && !CONFIG_SDCLONE_DISABLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_setuptaskfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit
 *   file descriptors and streams from the parent task.
 *
 * Parameters:
 *   tcb - tcb of the new task.
 *
 * Return Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setuptaskfiles(FAR _TCB *tcb)
{
  /* Allocate file descriptors for the TCB */

#if CONFIG_NFILE_DESCRIPTORS > 0
  tcb->filelist = files_alloclist();
  if (!tcb->filelist)
    {
      return -ENOMEM;
    }
#endif

  /* Allocate socket descriptors for the TCB */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  tcb->sockets = net_alloclist();
  if (!tcb->sockets)
    {
      return -ENOMEM;
    }
#endif

  /* Duplicate the parent task's file descriptors */

  sched_dupfiles(tcb);

  /* Duplicate the parent task's socket descriptors */

  sched_dupsockets(tcb);

  /* Allocate file/socket streams for the new TCB */

#if CONFIG_NFILE_STREAMS > 0
  return sched_setupstreams(tcb);
#else
  return OK;
#endif
}

#endif /* CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0*/
