/****************************************************************************
 * libc/stdio/lib_ferror.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#if CONFIG_NFILE_STREAMS > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ferror
 *
 * Description:
 *   This function will test if the last operation resulted in an eror.  This
 *   is used to disambiguate EOF and error conditions.
 *
 * Return Value:
 *   A non-zero value is returned to indicate the error condition.
 *
 ****************************************************************************/

int ferror(FILE *stream)
{
#if 0
  /* If an error is encountered by any of the C-buffered I/O functions, they
   * should set the __FS_FLAG_ERROR in the fs_flags field of struct
   * file_struct.
   */

  return (stream->fs_flags & __FS_FLAG_ERROR) != 0;
#else
  /* However, nothing currenlty sets the __FS_FLAG_ERROR flag (that is a job
   * for another day).  The __FS_FLAG_EOF is set by operations that perform
   * read operations.  Since ferror()  is probably only called to disambiguate
   * the meaning of other functions that return EOF, to indicate either EOF or
   * an error, just testing for not EOF is probably sufficient for now.
   *
   * This approach would not work if ferror() is called in other contexts. In
   * those cases, ferror() will always report an error.
   */

  return (stream->fs_flags & __FS_FLAG_EOF) == 0;
#endif
}

#endif /* CONFIG_NFILE_STREAMS */

