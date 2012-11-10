/****************************************************************************
 * libc/stdio/lib_wrflush.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: lib_wrflush
 *
 * Description:
 *   This is simply a version of fflush that does not report an error if
 *   the file is not open for writing.
 *
 ****************************************************************************/

int lib_wrflush(FAR FILE *stream)
{
#if CONFIG_STDIO_BUFFER_SIZE > 0
  /* Verify that we were passed a valid (i.e., non-NULL) stream */

#ifdef CONFIG_DEBUG
  if (!stream)
    {
      return -EINVAL;
    }
#endif

  /* Verify that the stream is opened for writing... lib_fflush will
   * return an error if it is called for a stream that is not opened for
   * writing.  Check that first so that this function will not fail in
   * that case.
   */

  if ((stream->fs_oflags & O_WROK) == 0)
    {
      /* Report that the success was successful if we attempt to flush a
       * read-only stream.
       */

      return OK;
    }

  /* Flush the stream.   Return success if there is no buffered write data
   * -- i.e., that the stream is opened for writing and  that all of the
   * buffered write data was successfully flushed by lib_fflush().
   */

  return lib_fflush(stream, true);
#else
  /* Verify that we were passed a valid (i.e., non-NULL) stream */

#ifdef CONFIG_DEBUG
  if (!stream)
    {
      return -EINVAL;
    }
#endif

  return OK;
#endif
}
