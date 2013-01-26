/************************************************************************
 * libc/string//lib_strndup.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <string.h>

#include "lib_internal.h"

/************************************************************************
 * Global Functions
 ************************************************************************/
/************************************************************************
 * Name: strndup
 *
 * Description:
 *   The strndup() function is equivalent to the strdup() function,
 *   duplicating the provided 's' in a new block of memory allocated as
 *   if by using malloc(), with the exception being that strndup() copies
 *   at most 'size' plus one bytes into the newly allocated memory,
 *   terminating the new string with a NUL character. If the length of 's'
 *   is larger than 'size', only 'size' bytes will be duplicated. If
 *   'size' is larger than the length of 's', all bytes in s will be
 *   copied into the new memory buffer, including the terminating NUL
 *   character. The newly created string will always be properly
 *   terminated.
 *
 ************************************************************************/

FAR char *strndup(FAR const char *s, size_t size)
{
  FAR char *news = NULL;
  if (s)
    {
      /* Get the size of the new string (limited to size) */

      size_t allocsize = strnlen(s, size);

      /* Allocate the new string, adding 1 for the NUL terminator */

      news = (FAR char*)lib_malloc(allocsize + 1);
      if (news)
        {
          /* Copy the string into the allocated memory and add a NUL
           * terminator in any case.
           */

          memcpy(news, s, allocsize);
          news[allocsize] = '\0';
        }
    }

  return news;
}
