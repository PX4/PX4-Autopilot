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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sched_setuptaskfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit
 *   file descriptors and streams from the parent task.
 *
 * Parameters:
 *   tcb - tcb of the new task.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setuptaskfiles(FAR _TCB *tcb)
{
#if CONFIG_NFILE_DESCRIPTORS > 0
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  int i;
#endif /* CONFIG_NFILE_DESCRIPTORS > 0 */
  int ret = OK;

#if CONFIG_NFILE_DESCRIPTORS > 0

  /* Allocate file descriptors for the TCB */

  tcb->filelist = files_alloclist();
  if (!tcb->filelist)
    {
      return -ENOMEM;
    }

#endif /* CONFIG_NFILE_DESCRIPTORS */

#if CONFIG_NSOCKET_DESCRIPTORS > 0

  /* Allocate socket descriptors for the TCB */

  tcb->sockets = net_alloclist();
  if (!tcb->sockets)
    {
      return -ENOMEM;
    }

#endif /* CONFIG_NSOCKET_DESCRIPTORS */

#if CONFIG_NFILE_DESCRIPTORS > 0
#if !defined(CONFIG_FDCLONE_DISABLE)

 /* Duplicate the file descriptors.  This will be either all of the
  * file descriptors or just the first three (stdin, stdout, and stderr)
  * if CONFIG_FDCLONE_STDIO is defined.  NFSDS_TOCLONE is set
  * accordingly above.
  */

  if (rtcb->filelist)
    {
      for (i = 0; i < NFDS_TOCLONE; i++)
        {
          /* Check if this file is opened */

          if (rtcb->filelist->fl_files[i].f_inode)
            {
              (void)files_dup(&rtcb->filelist->fl_files[i],
                              &tcb->filelist->fl_files[i]);
            }
        }
    }
#endif

#if CONFIG_NFILE_STREAMS > 0

  /* Allocate file streams for the TCB */

  ret = sched_setupstreams(tcb);

#endif /* CONFIG_NFILE_STREAMS */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if CONFIG_NSOCKET_DESCRIPTORS > 0 && !defined(CONFIG_SDCLONE_DISABLE)

 /* Duplicate the socket descriptors */

  if (rtcb->sockets)
    {
      for (i = 0; i < CONFIG_NSOCKET_DESCRIPTORS; i++)
        {
          /* Check if this socket is allocated */

          if (rtcb->sockets->sl_sockets[i].s_crefs > 0)
            {
              (void)net_clone(&rtcb->sockets->sl_sockets[i],
                              &tcb->sockets->sl_sockets[i]);
            }
        }
    }
#endif
  return ret;
}

#endif /* CONFIG_NFILE_DESCRIPTORS || CONFIG_NSOCKET_DESCRIPTORS */
