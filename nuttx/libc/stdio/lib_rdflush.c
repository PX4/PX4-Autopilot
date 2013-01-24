/****************************************************************************
 * libc/stdio/lib_rdflush.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

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
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rdflush
 *
 * Description:
 *   Flush read data from the I/O buffer and adjust the file pointer to
 *   account for the unread data
 *
 ****************************************************************************/

#if CONFIG_STDIO_BUFFER_SIZE > 0
int lib_rdflush(FAR FILE *stream)
{
  if (!stream)
    {
      set_errno(EBADF);
      return ERROR;
    }

  /* Get exclusive access to the stream */

  lib_take_semaphore(stream);

  /* If the buffer is currently being used for read access, then discard all
   * of the read-ahead data.  We do not support concurrent buffered read/write
   * access.
   */

  if (stream->fs_bufread != stream->fs_bufstart)
    {
      /* Now adjust the stream pointer to account for the read-ahead data that
       * was not actually read by the user.
       */

#if CONFIG_NUNGET_CHARS > 0
      off_t rdoffset = stream->fs_bufread - stream->fs_bufpos + stream->fs_nungotten;
#else
      off_t rdoffset = stream->fs_bufread - stream->fs_bufpos;
#endif
      /* Mark the buffer as empty (do this before calling fseek() because fseek()
       * also calls this function).
       */

      stream->fs_bufpos = stream->fs_bufread = stream->fs_bufstart;
#if CONFIG_NUNGET_CHARS > 0
      stream->fs_nungotten = 0;
#endif
      /* Then seek to the position corresponding to the last data read by the user */

      if (fseek(stream, -rdoffset, SEEK_CUR) < 0)
        {
          lib_give_semaphore(stream);
          return ERROR;
        }
    }

  lib_give_semaphore(stream);
  return OK;
}
#endif /* CONFIG_STDIO_BUFFER_SIZE */

