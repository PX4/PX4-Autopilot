/****************************************************************************
 * libc/misc/lib_match.c - simple shell-style filename matcher
 *
 * Simple shell-style filename pattern matcher written by Jef Poskanzer
 * This pattern matcher only handles '?', '*' and '**', and  multiple
 * patterns separated by '|'.
 *
 *   Copyright © 1995,2000 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <nuttx/regex.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: match_one
 *
 * Description:
 *   Does all of the work for one '|' delimited pattern
 *
 * Returned Value:
 *   Returns 1 (match) or 0 (no-match).
 *
 ****************************************************************************/

static int match_one(const char *pattern, int patlen, const char *string)
{
  const char *p;
  int pl;
  int i;

  for (p = pattern; p - pattern < patlen; p++, string++)
    {
      if (*p == '?' && *string != '\0')
        {
          continue;
        }

      if (*p == '*')
        {
          p++;
          if (*p == '*')
            {
              /* Double-wildcard matches anything. */

              p++;
              i = strlen(string);
            }
          else
            {
              /* Single-wildcard matches anything but slash. */

              i = strcspn(string, "/");
            }

          pl = patlen - (p - pattern);
          for (; i >= 0; i--)
            {
              if (match_one(p, pl, &(string[i])))
                {
                  return 1;
                }
            }
          return 0;
        }

      if (*p != *string)
        {
          return 0;
        }
    }

  if (*string == '\0')
    {
      return 1;
    }
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: match
 *
 * Description:
 *   Simple shell-style filename pattern matcher written by Jef Poskanzer
 *   This pattern matcher only handles '?', '*' and '**', and  multiple
 *   patterns separated by '|'.
 *
 * Returned Value:
 *   Returns 1 (match) or 0 (no-match).
 *
 ****************************************************************************/

int match(const char *pattern, const char *string)
{
  const char *or;

  for (;;)
    {
      or = strchr(pattern, '|');
      if (or == (char *)0)
        {
          return match_one(pattern, strlen(pattern), string);
        }

      if (match_one(pattern, or - pattern, string))
        {
          return 1;
        }

      pattern = or + 1;
    }
}

