/****************************************************************************
 * fs_open.c
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
#include <sys/types.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#ifdef CONFIG_FILE_MODE
#include <stdarg.h>
#endif
#include <nuttx/fs/fs.h>
#include "fs_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_checkflags
 *
 * Description:
 *   Check if the access described by 'oflags' is supported on 'inode'
 *
 ****************************************************************************/

int inode_checkflags(FAR struct inode *inode, int oflags)
{
  if (((oflags & O_RDOK) != 0 && !inode->u.i_ops->read) ||
      ((oflags & O_WROK) != 0 && !inode->u.i_ops->write))
    {
      return -EACCES;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: open
 *
 * Description:
 *   Standard 'open' interface
 *
 ****************************************************************************/

int open(const char *path, int oflags, ...)
{
  FAR struct filelist *list;
  FAR struct inode    *inode;
  FAR const char      *relpath = NULL;
#if defined(CONFIG_FILE_MODE) || !defined(CONFIG_DISABLE_MOUNTPOINT)
  mode_t               mode = 0666;
#endif
  int                  ret;
  int                  fd;

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      ret = EMFILE;
      goto errout;
    }

#ifdef CONFIG_FILE_MODE
#  ifdef CONFIG_CPP_HAVE_WARNING
#    warning "File creation not implemented"
#  endif

  /* If the file is opened for creation, then get the mode bits */

  if (oflags & (O_WRONLY|O_CREAT) != 0)
    {
      va_list ap;
      va_start(ap, oflags);
      mode = va_arg(ap, mode_t);
      va_end(ap);
    }
#endif

  /* Get an inode for this file */

  inode = inode_find(path, &relpath);
  if (!inode)
    {
      /* "O_CREAT is not set and the named file does not exist.  Or, a
       * directory component in pathname does not exist or is a dangling
       * symbolic link."
       */

      ret = ENOENT;
      goto errout;
    }

  /* Verify that the inode is valid and either a "normal" or a mountpoint.  We
   * specifically exclude block drivers.
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if ((!INODE_IS_DRIVER(inode) && !INODE_IS_MOUNTPT(inode)) || !inode->u.i_ops)
#else
  if (!INODE_IS_DRIVER(inode) || !inode->u.i_ops)
#endif
    {
      ret = ENXIO;
      goto errout_with_inode;
    }

  /* Make sure that the inode supports the requested access */

  ret = inode_checkflags(inode, oflags);
  if (ret < 0)
    {
      ret = -ret;
      goto errout_with_inode;
    }

  /* Associate the inode with a file structure */

  fd = files_allocate(inode, oflags, 0, 0);
  if (fd < 0)
    {
      ret = EMFILE;
      goto errout_with_inode;
    }

  /* Perform the driver open operation.  NOTE that the open method may be
   * called many times.  The driver/mountpoint logic should handled this
   * because it may also be closed that many times.
   */

  ret = OK;
  if (inode->u.i_ops->open)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          ret = inode->u.i_mops->open((FAR struct file*)&list->fl_files[fd],
                                      relpath, oflags, mode);
        }
      else
#endif
        {
          ret = inode->u.i_ops->open((FAR struct file*)&list->fl_files[fd]);
        }
    }

  if (ret < 0)
    {
      ret = -ret;
      goto errout_with_fd;
    }

  return fd;

 errout_with_fd:
  files_release(fd);
 errout_with_inode:
  inode_release(inode);
 errout:
  set_errno(ret);
  return ERROR;
}
