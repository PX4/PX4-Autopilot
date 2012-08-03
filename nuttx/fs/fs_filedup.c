/****************************************************************************
 * fs/fs_filedup.c
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

#include <sched.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

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
 * Name: file_dup OR dup
 *
 * Description:
 *   Clone a file descriptor 'fd' to an arbitray descriptor number (any value
 *   greater than or equal to 'minfd'). If socket descriptors are
 *   implemented, then this is called by dup() for the case of file
 *   descriptors.  If socket descriptors are not implemented, then this
 *   function IS dup().
 *
 ****************************************************************************/

int file_dup(int fildes, int minfd)
{
  FAR struct filelist *list;
  int fildes2;

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      set_errno(EMFILE);
      return ERROR;
    }

  /* Verify that fildes is a valid, open file descriptor */

  if (!DUP_ISOPEN(fildes, list))
    {
      set_errno(EBADF);
      return ERROR;
    }

  /* Increment the reference count on the contained inode */

  inode_addref(list->fl_files[fildes].f_inode);

  /* Then allocate a new file descriptor for the inode */

  fildes2 = files_allocate(list->fl_files[fildes].f_inode,
                           list->fl_files[fildes].f_oflags,
                           list->fl_files[fildes].f_pos,
                           minfd);
  if (fildes2 < 0)
    {
      set_errno(EMFILE);
      inode_release(list->fl_files[fildes].f_inode);
      return ERROR;
    }

  return fildes2;
}

#endif /* CONFIG_NFILE_DESCRIPTORS > 0 */

