/****************************************************************************
 * libc/stdio/lib_fseek.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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
#include <fcntl.h>
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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fseek
 *
 * Description:
 *   The fseek() function sets the file position indicator for the stream
 *   pointed to by stream. The new position, measured in bytes, is obtained
 *   by adding offset bytes to the position specified by whence. If whence is
 *   set to SEEK_SET, SEEK_CUR, or SEEK_END, the offset is relative to the
 *   start of the file, the current position indicator, or end-of-file,
 *   respectively. A successful call to the fseek() function clears the
 *   end-of-file indicator for the stream and undoes any effects of the ungetc(3)
 *   function on the same stream.
 *
 * Returned Value:
 *   Zero on succes; -1 on failure with errno set appropriately. 
 *
 ****************************************************************************/

int fseek(FAR FILE *stream, long int offset, int whence)
{
 #if CONFIG_STDIO_BUFFER_SIZE > 0
  /* Flush any valid read/write data in the buffer (also verifies stream) */

  if (lib_rdflush(stream) < 0 || lib_wrflush(stream) < 0)
    {
      return ERROR;
    }
#else
  /* Verify that we were provided with a stream */

  if (!stream)
    {
      set_errno(EBADF);
      return ERROR;
    }
#endif

  /* On success or failure, discard any characters saved by ungetc() */

#if CONFIG_NUNGET_CHARS > 0
  stream->fs_nungotten = 0;
#endif

  /* Perform the fseek on the underlying file descriptor */

  return lseek(stream->fs_filedes, offset, whence) == (off_t)-1 ? ERROR : OK;
}


