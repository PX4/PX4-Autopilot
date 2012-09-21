/****************************************************************************
 * lib/stdio/lib_sscanf.c
 *
 *   Copyright (C) 2007, 2008, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/compiler.h>
#include <sys/types.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <debug.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define MAXLN 128

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

int vsscanf(char *buf, const char *fmt, va_list ap);

/**************************************************************************
 * Global Constant Data
 **************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/**************************************************************************
 * Private Constant Data
 **************************************************************************/

static const char spaces[] = " \t\n\r\f\v";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  findwidth
 *
 * Description:
 *    Try to figure out the width of the input data.
 *
 ****************************************************************************/

static int findwidth(FAR const char *buf, FAR const char *fmt)
{
  FAR const char *next = fmt + 1;

  /* No... is there a space after the format? Or does the format string end
   * here?
   */

  if (isspace(*next) || *next == 0)
    {
      /* Use the input up until the first white space is encountered. */

      return strcspn(buf, spaces);
    }

  /* No.. Another possibility is the the format character is followed by
   * some recognizable delimiting value.
   */

  if (*next != '%')
    {
      /* If so we will say that the string ends there if we can find that
       * delimiter in the input string.
       */

      FAR const char *ptr = strchr(buf, *next);
      if (ptr)
        {
          return (int)(ptr - buf);
        }
    }

  /* No... the format has not delimiter and is back-to-back with the next
   * formats (or no is following by a delimiter that does not exist in the
   * input string).  At this point we just bail and Use the input up until
   * the first white space is encountered.
   *
   * NOTE:  This means that values from the following format may be
   * concatenated with the first. This is a bug.  We have no generic way of
   * determining the width of the data if there is no fieldwith, no space
   * separating the input, and no usable delimiter character.
   */

  return strcspn(buf, spaces);
}

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Function:  sscanf
 *
 * Description:
 *    ANSI standard sscanf implementation.
 *
 ****************************************************************************/

int sscanf(FAR const char *buf, FAR const char *fmt, ...)
{
  va_list ap;
  int     count;

  va_start(ap, fmt);
  count = vsscanf((FAR char*)buf, fmt, ap);
  va_end(ap);
  return count;
}

/****************************************************************************
 * Function:  vsscanf
 *
 * Description:
 *    ANSI standard vsscanf implementation.
 *
 ****************************************************************************/

int vsscanf(FAR char *buf, FAR const char *fmt, va_list ap)
{
  FAR char       *bufstart;
  FAR char       *tv;
  FAR const char *tc;
  int             count;
  int             noassign;
  int             width;
  int             base = 10;
  int             lflag;
  char            tmp[MAXLN];

  lvdbg("vsscanf: buf=\"%s\" fmt=\"%s\"\n", buf, fmt);

  /* Remember the start of the input buffer.  We will need this for %n
   * calculations.
   */

  bufstart = buf;

  /* Parse the format, extracting values from the input buffer as needed */

  count    = 0;
  noassign = 0;
  width    = 0;
  lflag    = 0;

  while (*fmt && *buf)
    {
      /* Skip over white space */

      while (isspace(*fmt))
        {
          fmt++;
        }

      /* Check for a conversion specifier */

      if (*fmt == '%')
        {
          lvdbg("vsscanf: Specifier found\n");

          /* Check for qualifiers on the conversion specifier */
          fmt++;
          for (; *fmt; fmt++)
            {
              lvdbg("vsscanf: Processing %c\n", *fmt);

              if (strchr("dibouxcsefgn%", *fmt))
                {
                  break;
                }

              if (*fmt == '*')
                {
                  noassign = 1;
                }
              else if (*fmt == 'l' || *fmt == 'L')
                {
                  lflag = 1;
                }
              else if (*fmt >= '1' && *fmt <= '9')
                {
                  for (tc = fmt; isdigit(*fmt); fmt++);
                  strncpy(tmp, tc, fmt - tc);
                  tmp[fmt - tc] = '\0';
                  width = atoi(tmp);
                  fmt--;
                }
            }

          /* Process %s:  String conversion */

          if (*fmt == 's')
            {
              lvdbg("vsscanf: Performing string conversion\n");

              while (isspace(*buf))
                {
                  buf++;
                }

              /* Was a fieldwidth specified? */

              if (!width)
                {
                  /* No... Guess a field width using some heuristics */

                  width = findwidth(buf, fmt);
                }

              if (!noassign)
                {
                  tv = va_arg(ap, char*);
                  strncpy(tv, buf, width);
                  tv[width] = '\0';
                }

              buf += width;
            }

          /* Process %c:  Character conversion */

          else if (*fmt == 'c')
            {
              lvdbg("vsscanf: Performing character conversion\n");

              /* Was a fieldwidth specified? */

              if (!width)
                {
                  /* No, then width is this one single character */

                  width = 1;
                }

              if (!noassign)
                {
                  tv = va_arg(ap, char*);
                  strncpy(tv, buf, width);
                  tv[width] = '\0';
                }

              buf += width;
            }

          /* Process %d, %o, %b, %x, %u:  Various integer conversions */

          else if (strchr("dobxu", *fmt))
            {
              lvdbg("vsscanf: Performing integer conversion\n");

              /* Skip over any white space before the integer string */

              while (isspace(*buf))
                {
                  buf++;
                }

              /* The base of the integer conversion depends on the specific
               * conversion specification.
               */

              if (*fmt == 'd' || *fmt == 'u')
                {
                  base = 10;
                }
              else if (*fmt == 'x')
                {
                  base = 16;
                }
              else if (*fmt == 'o')
                {
                  base = 8;
                }
              else if (*fmt == 'b')
                {
                  base = 2;
                }

              /* Was a fieldwidth specified? */

              if (!width)
                {
                  /* No... Guess a field width using some heuristics */

                  width = findwidth(buf, fmt);
                }

              /* Copy the numeric string into a temporary working buffer. */

              strncpy(tmp, buf, width);
              tmp[width] = '\0';

              lvdbg("vsscanf: tmp[]=\"%s\"\n", tmp);

              /* Perform the integer conversion */

              buf += width;
              if (!noassign)
                {
                  int *pint = va_arg(ap, int*);
#ifdef SDCC
                  char *endptr;
                  int tmpint = strtol(tmp, &endptr, base);
#else
                  int tmpint = strtol(tmp, NULL, base);
#endif
                  lvdbg("vsscanf: Return %d to 0x%p\n", tmpint, pint);
                  *pint = tmpint;
                }
            }

          /* Process %f:  Floating point conversion */

          else if (*fmt == 'f')
            {
#ifndef CONFIG_LIBC_FLOATINGPOINT
              /* No floating point conversions */

              void *pv = va_arg(ap, void*);

              lvdbg("vsscanf: Return 0.0 to %p\n", pv);
              *((double_t*)pv) = 0.0;
#else
              lvdbg("vsscanf: Performing floating point conversion\n");

              /* Skip over any white space before the real string */

              while (isspace(*buf))
                {
                  buf++;
                }

              /* Was a fieldwidth specified? */

              if (!width)
                {
                  /* No... Guess a field width using some heuristics */

                  width = findwidth(buf, fmt);
                }

              /* Copy the real string into a temporary working buffer. */

              strncpy(tmp, buf, width);
              tmp[width] = '\0';
              buf += width;

              lvdbg("vsscanf: tmp[]=\"%s\"\n", tmp);

              /* Perform the floating point conversion */

              if (!noassign)
                {
                  /* strtod always returns a double */
#ifdef SDCC
                  char *endptr;
                  double_t dvalue = strtod(tmp,&endptr);
#else
                  double_t dvalue = strtod(tmp, NULL);
#endif
                  void *pv = va_arg(ap, void*);

                  lvdbg("vsscanf: Return %f to %p\n", dvalue, pv);

                  /* But we have to check whether we need to return a
                   * float or a double.
                   */

#ifdef CONFIG_HAVE_DOUBLE
                  if (lflag)
                    {
                      *((double_t*)pv) = dvalue;
                    }
                  else
#endif
                    {
                      *((float*)pv) = (float)dvalue;
                    }
                }
#endif
            }

          /* Process %n:  Character count */

          else if (*fmt == 'n')
            {
              lvdbg("vsscanf: Performing character count\n");

              if (!noassign)
                {
                  int *pint = va_arg(ap, int*);
                  *pint = (int)(buf - bufstart);
                }
            }

          /* Note %n does not count as a conversion */

          if (!noassign && *fmt != 'n')
            {
              count++;
            }

          width = noassign = lflag = 0;
          fmt++;
        }

    /* Its is not a conversion specifier */

      else
        {
          while (isspace(*buf))
            {
              buf++;
            }

          if (*fmt != *buf)
            {
              break;
            }
          else
            {
              fmt++;
              buf++;
            }
        }
    }

  return count;
}
