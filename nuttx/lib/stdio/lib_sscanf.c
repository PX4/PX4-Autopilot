/****************************************************************************
 * lib/stdio/lib_sscanf.c
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

int vsscanf(char *buf, const char *s, va_list ap);

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
int vsscanf(FAR char *buf, FAR const char *s, va_list ap)
{
  int             count;
  int             noassign;
  int             width;
  int             base = 10;
  int             lflag;
  FAR char       *tv;
  FAR const char *tc;
  char            tmp[MAXLN];

  lvdbg("vsscanf: buf=\"%s\" fmt=\"%s\"\n", buf, s);

  count = noassign = width = lflag = 0;
  while (*s && *buf)
    {
      /* Skip over white space */

      while (isspace(*s))
        {
          s++;
        }

      /* Check for a conversion specifier */

      if (*s == '%')
        {
          lvdbg("vsscanf: Specifier found\n");

          /* Check for qualifiers on the conversion specifier */
          s++;
          for (; *s; s++)
            {
              lvdbg("vsscanf: Processing %c\n", *s);

              if (strchr("dibouxcsefg%", *s))
                {
                  break;
                }

              if (*s == '*')
                {
                  noassign = 1;
                }
              else if (*s == 'l' || *s == 'L')
                {
                  lflag = 1;
                }
              else if (*s >= '1' && *s <= '9')
                {
                  for (tc = s; isdigit(*s); s++);
                  strncpy(tmp, tc, s - tc);
                  tmp[s - tc] = '\0';
                  width = atoi(tmp);
                  s--;
                }
            }

          /* Process %s:  String conversion */

          if (*s == 's')
            {
              lvdbg("vsscanf: Performing string conversion\n");

              while (isspace(*buf))
                {
                  buf++;
                }

              if (!width)
                {
                  width = strcspn(buf, spaces);
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

          else if (*s == 'c')
            {
              lvdbg("vsscanf: Performing character conversion\n");

              if (!width)
                {
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

          else if (strchr("dobxu", *s))
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

              if (*s == 'd' || *s == 'u')
                {
                  base = 10;
                }
              else if (*s == 'x')
                {
                  base = 16;
                }
              else if (*s == 'o')
                {
                  base = 8;
                }
              else if (*s == 'b')
                {
                  base = 2;
                }

              /* Copy the integer string into a temporary working buffer. */

              if (!width)
                {
                  if (isspace(*(s + 1)) || *(s + 1) == 0)
                    {
                      width = strcspn(buf, spaces);
                    }
                  else
                    {
                      width = strchr(buf, *(s + 1)) - buf;
                    }
                }

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

          else if (*s == 'f')
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

              /* Copy the real string into a temporary working buffer. */

              if (!width)
                {
                  if (isspace(*(s + 1)) || *(s + 1) == 0)
                    {
                      width = strcspn(buf, spaces);
                    }
                  else
                    {
                      width = strchr(buf, *(s + 1)) - buf;
                    }
                }

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

          if (!noassign)
            {
              count++;
            }

          width = noassign = lflag = 0;
          s++;
        }

    /* Its is not a conversion specifier */

      else
        {
          while (isspace(*buf))
            {
              buf++;
            }

          if (*s != *buf)
            {
              break;
            }
          else
            {
              s++;
              buf++;
            }
        }
    }

  return count;
}
