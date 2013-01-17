/****************************************************************************
 * libc/stdio/lib_memoutstream.c
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

#include <assert.h>

#include "lib_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memoutstream_putc
 ****************************************************************************/

static void memoutstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  FAR struct lib_memoutstream_s *mthis = (FAR struct lib_memoutstream_s *)this;

  DEBUGASSERT(this);

  /* If this will not overrun the buffer, then write the character to the
   * buffer.  Not that buflen was pre-decremented when the stream was
   * created so it is okay to write past the end of the buflen by one.
   */

  if (this->nput < mthis->buflen)
    {
      mthis->buffer[this->nput] = ch;
      this->nput++;
      mthis->buffer[this->nput] = '\0';
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_memoutstream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input parameters:
 *   memoutstream - User allocated, uninitialized instance of struct
 *                  lib_memoutstream_s to be initialized.
 *   bufstart     - Address of the beginning of the fixed-size memory buffer
 *   buflen       - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (memoutstream initialized).
 *
 ****************************************************************************/

void lib_memoutstream(FAR struct lib_memoutstream_s *memoutstream,
                      FAR char *bufstart, int buflen)
{
  memoutstream->public.put   = memoutstream_putc;
#ifdef CONFIG_STDIO_LINEBUFFER
  memoutstream->public.flush = lib_noflush;
#endif
  memoutstream->public.nput  = 0;          /* Will be buffer index */
  memoutstream->buffer       = bufstart;   /* Start of buffer */
  memoutstream->buflen       = buflen - 1; /* Save space for null terminator */
  memoutstream->buffer[0]    = '\0';       /* Start with an empty string */
}


