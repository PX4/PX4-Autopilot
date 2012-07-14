/****************************************************************************
 * fs/fs_lseek.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "fs_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lseek
 *
 * Description:
 *   The lseek() function repositions the offset of the open file associated
 *   with the file descriptor fildes to the argument 'offset' according to the
 *   directive 'whence' as follows:
 *
 *   SEEK_SET
 *      The offset is set to offset bytes.
 *   SEEK_CUR
 *      The offset is set to its current location plus offset bytes.
 *   SEEK_END
 *      The offset is set to the size of the file plus offset bytes.
 *
 *  The lseek() function allows the file offset to be set beyond the end of the
 *  file (but this does not change the size of the file). If data is later written
 *  at this point, subsequent reads of the data in the gap (a "hole") return null
 *  bytes ('\0') until data is actually written into the gap.
 *
 * Parameters:
 *   fd       File descriptor of device
 *   offset   Defines the offset to position to
 *   whence   Defines how to use offset
 *
 * Return:
 *   The resulting offset on success.  -1 on failure withi errno set properly:
 *
 *   EBADF      fildes is not an open file descriptor.
 *   EINVAL     whence  is  not one of SEEK_SET, SEEK_CUR, SEEK_END; or the
 *              resulting file offset would be negative, or beyond the end of a
 *              seekable device.
 *   EOVERFLOW  The resulting file offset cannot be represented in an off_t.
 *   ESPIPE     fildes is associated with a pipe, socket, or FIFO.
 *
 ****************************************************************************/

off_t lseek(int fd, off_t offset, int whence)
{
  FAR struct filelist *list;
  FAR struct file     *filep;
  FAR struct inode    *inode;
  int                  err;

  /* Did we get a valid file descriptor? */

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      err = EBADF;
      goto errout;
    }

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      err = EMFILE;
      goto errout;
    }

  /* Is a driver registered? */

  filep = &list->fl_files[fd];
  inode =  filep->f_inode;

  if (inode && inode->u.i_ops)
    {
      /* Does it support the seek method */

      if (inode->u.i_ops->seek)
        {
          /* Yes, then let it perform the seek */

          err = (int)inode->u.i_ops->seek(filep, offset, whence);
          if (err < 0)
            {
              err = -err;
              goto errout;
            }
         }
      else
        {
          /* No... there are a couple of default actions we can take */

          switch (whence)
            {
              case SEEK_CUR:
                offset += filep->f_pos;

              case SEEK_SET:
                if (offset >= 0)
                  {
                    filep->f_pos = offset; /* Might be beyond the end-of-file */
                    break;
                  }
                else
                  {
                    err = EINVAL;
                    goto errout;
                  }
                break;

              case SEEK_END:
                err = ENOSYS;
                goto errout;

              default:
                err = EINVAL;
                goto errout;
            }
        }
    }

  return filep->f_pos;

errout:
  set_errno(err);
  return (off_t)ERROR;
}

#endif
