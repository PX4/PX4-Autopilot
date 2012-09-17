/**********************************************************************
 * pextension.c
 * Manage file extensions
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 **********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdbool.h>
#include <string.h>

#include "keywords.h"
#include "pdefs.h"
#include "paslib.h"

/***********************************************************************/

bool extension(const char *inName, const char *ext, char *outName,
               bool force_default)
{
  int    namelen = strlen(inName);
  int    extlen;
  int    copylen;
  char  *lastdot;

  /* Find the position of the last occurrence of '.' in inName */

  lastdot = strrchr(inName, '.');

  /* If a file extension is in the input line and no default is forced, then
   * copy the rest of the input line
   */

  if ((lastdot != NULL) && (!force_default))
    {
      /* Make sure that the string (with its null terminator) will fit in
       * the allocated buffer.
       */

      if ((namelen + 1) > FNAME_SIZE)
        {
          /* It won't */

          return true;
        }
      else
        {
          /* Copy the string. */

          strcpy(outName, inName);
        }
    }
  else
    {
      /* The name has no extension or we must replace the extension. */

      extlen  = strlen(ext) + 1; /* extension + null terminator */

      if (lastdot != NULL)
        {
          /* It has an extension.  We must copy everything except the
           * last dot and the following extension.
           */

          copylen = namelen - strlen(lastdot); /* name - . - terminator */
        }
      else
        {
          /* It has no extension.  We must copy everything */

          copylen = namelen + 1; /* whole name with null termination */
        }

      /* Make sure that the string (with its null terminator) will fit in
       * the allocated buffer.
       */

      if ((copylen + extlen + 1) > FNAME_SIZE)
        {
          /* It won't */

          return true;
        }
      else
        {
          /* It will Copy file name up to, but excluding, the '.' */

          memcpy(outName, inName, copylen);

          /* Then copy the extension */

          outName[copylen] = '.';
          memcpy(&outName[copylen+1], ext, extlen);
        }
    }

  return false;

} /* end extension */

/***********************************************************************/
