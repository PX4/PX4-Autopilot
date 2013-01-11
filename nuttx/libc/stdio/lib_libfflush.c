/****************************************************************************
 * libc/stdio/lib_libfflush.c
 *
 *   Copyright (C) 2007-2008, 2011-2012 Gregory Nutt. All rights reserved.
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "lib_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fflush
 *
 * Description:
 *  The function lib_fflush() forces a write of all user-space buffered data for
 *  the given output or update stream via the stream's underlying write
 *  function.  The open status of the stream is unaffected.
 *
 * Parmeters:
 *  stream - the stream to flush
 *  bforce - flush must be complete.
 *
 * Return:
 *  A negated errno value on failure, otherwise the number of bytes remaining
 *  in the buffer.
 *
 ****************************************************************************/

ssize_t lib_fflush(FAR FILE *stream, bool bforce)
{
#if CONFIG_STDIO_BUFFER_SIZE > 0
  FAR const unsigned char *src;
  ssize_t bytes_written;
  ssize_t nbuffer;

  /* Return EBADF if the file is not opened for writing */

  if (stream->fs_filedes < 0 || (stream->fs_oflags & O_WROK) == 0)
    {
      return -EBADF;
    }

  /* Make sure that we have exclusive access to the stream */

  lib_take_semaphore(stream);

  /* Make sure that the buffer holds valid data */

  if (stream->fs_bufpos  != stream->fs_bufstart)
    {
       /* Make sure that the buffer holds buffered write data.  We do not
        * support concurrent read/write buffer usage.
        */

       if (stream->fs_bufread != stream->fs_bufstart)
        {
          /* The buffer holds read data... just return zero meaning "no bytes
           * remaining in the buffer."
           */

          lib_give_semaphore(stream);
          return 0;
        }

      /* How many bytes of write data are used in the buffer now */

      nbuffer = stream->fs_bufpos - stream->fs_bufstart;

      /* Try to write that amount */

      src = stream->fs_bufstart;
      do
        {
          /* Perform the write */

          bytes_written = write(stream->fs_filedes, src, nbuffer);
          if (bytes_written < 0)
            {
              /* Write failed.  The cause of the failure is in 'errno'.
               * returned the negated errno value.
               */

              lib_give_semaphore(stream);
              return -get_errno();
            }

          /* Handle partial writes.  fflush() must either return with
           * an error condition or with the data successfully flushed
           * from the buffer.
           */

          src     += bytes_written;
          nbuffer -= bytes_written;
        }
      while (bforce && nbuffer > 0);

      /* Reset the buffer position to the beginning of the buffer */

      stream->fs_bufpos = stream->fs_bufstart;

      /* For the case of an incomplete write, nbuffer will be non-zero
       * It will hold the number of bytes that were not written.
       * Move the data down in the buffer to handle this (rare) case
       */

      while (nbuffer)
       {
         *stream->fs_bufpos++ = *src++;
         --nbuffer;
       }
    }

  /* Restore normal access to the stream and return the number of bytes
   * remaining in the buffer.
   */

  lib_give_semaphore(stream);
  return stream->fs_bufpos - stream->fs_bufstart;
#else
  /* Return no bytes remaining in the buffer */

  return 0;
#endif
}

