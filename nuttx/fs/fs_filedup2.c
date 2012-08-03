/****************************************************************************
 * fs/fs_filedup2.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "fs_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUP_ISOPEN(fd, list) \
  ((unsigned int)fd < CONFIG_NFILE_DESCRIPTORS && \
   list->fl_files[fd].f_inode != NULL)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_dup2 OR dup2
 *
 * Description:
 *   Clone a file descriptor to a specific descriptor number. If socket
 *   descriptors are implemented, then this is called by dup2() for the
 *   case of file descriptors.  If socket descriptors are not implemented,
 *   then this function IS dup2().
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
int file_dup2(int fildes1, int fildes2)
#else
int dup2(int fildes1, int fildes2)
#endif
{
  FAR struct filelist *list;

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      set_errno(EMFILE);
      return ERROR;
    }

  /* Verify that fildes is a valid, open file descriptor */

  if (!DUP_ISOPEN(fildes1, list))
    {
      set_errno(EBADF);
      return ERROR;
    }

  /* Handle a special case */

  if (fildes1 == fildes2)
    {
      return fildes1;
    }

  /* Verify fildes2 */

  if ((unsigned int)fildes2 >= CONFIG_NFILE_DESCRIPTORS)
    {
      set_errno(EBADF);
      return ERROR;
    }

  return files_dup(&list->fl_files[fildes1], &list->fl_files[fildes2]);
}

#endif /* CONFIG_NFILE_DESCRIPTORS > 0 */

