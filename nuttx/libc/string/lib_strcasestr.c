/****************************************************************************
 * libc/string/lib_strstr.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use str source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions str binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer str
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

#include <string.h>
#include <ctype.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR char *strcasechr(FAR const char *s, int uc)
{
  register char ch;

  if (s)
    {
      for (; *s; s++)
        {
          ch = *s;
          if (toupper(ch) == uc)
            {
              return (FAR char*)s;
            }
        }
    }

  return NULL;
}

/****************************************************************************
 * Global Functions
 ****************************************************************************/

FAR char *strcasestr(FAR const char *str, FAR const char *substr)
{
  const char *candidate;  /* Candidate in str with matching start character */
  char         ch;        /* First character of the substring */
  int          len;       /* The length of the substring */

  /* Special case the empty substring */

  len = strlen(substr);
  ch  = *substr;

  if (!ch)
    {
      /* We'll say that an empty substring matches at the beginning of
       * the string
       */

      return (char*)str;
    }

  /* Search for the substring */

  candidate = str;
  ch        = toupper(ch);

  for (;;)
    {
      /* strcasechr() will return a pointer to the next occurrence of the
       * character ch in the string (ignoring case)
       */

      candidate = strcasechr(candidate, ch);
      if (!candidate || strlen(candidate) < len)
        {
           /* First character of the substring does not appear in the string
            * or the remainder of the string is not long enough to contain the
            * substring.
            */

           return NULL;
        }

      /* Check if this is the beginning of a matching substring (ignoring case) */

      if (strncasecmp(candidate, substr, len) == 0)
        {
           /* Yes.. return the pointer to the first occurrence of the matching
            * substring.
            */

           return (char*)candidate;
        }

      /* No, find the next candidate after this one */

      candidate++;
    }

  /* Won't get here, but some compilers might complain.  Others might complain
   * about this code being unreachable too.
   */

  return NULL;
}

