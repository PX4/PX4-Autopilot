/****************************************************************************
 * libc/stdio/lib_meminstream.c
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
 * Name: meminstream_getc
 ****************************************************************************/

static int meminstream_getc(FAR struct lib_instream_s *this)
{
  FAR struct lib_meminstream_s *mthis = (FAR struct lib_meminstream_s *)this;
  int ret;

  DEBUGASSERT(this);

  /* Get the next character (if any) from the buffer */

  if (this->nget < mthis->buflen)
    {
      ret = mthis->buffer[this->nget];
      this->nget++;
    }
  else
    {
      ret = EOF;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_meminstream
 *
 * Description:
 *   Initializes a stream for use with a fixed-size memory buffer.
 *
 * Input parameters:
 *   meminstream - User allocated, uninitialized instance of struct
 *                 lib_meminstream_s to be initialized.
 *   bufstart    - Address of the beginning of the fixed-size memory buffer
 *   buflen      - Size of the fixed-sized memory buffer in bytes
 *
 * Returned Value:
 *   None (meminstream initialized).
 *
 ****************************************************************************/

void lib_meminstream(FAR struct lib_meminstream_s *meminstream,
                     FAR const char *bufstart, int buflen)
{
  meminstream->public.get  = meminstream_getc;
  meminstream->public.nget = 0;          /* Will be buffer index */
  meminstream->buffer      = bufstart;   /* Start of buffer */
  meminstream->buflen      = buflen;     /* Length of the buffer */
}


