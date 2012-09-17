/****************************************************************************
 * tools/csvparser.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include "csvparser.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

bool g_debug;
char g_line[LINESIZE+1];
char g_parm[MAX_FIELDS][MAX_PARMSIZE];
int g_lineno;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skip_space
 ****************************************************************************/

static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

/****************************************************************************
 * Name: copy_parm
 ****************************************************************************/

static char *copy_parm(char *src, char *dest)
{
  char *start = src;
  int i;

  /* De-quote the parameter and copy it into the parameter array */

  for (i = 0; i < MAX_PARMSIZE; i++)
    {
      if (*src == '"')
        {
          *dest = '\0';
          return src;
        }
      else if (*src == '\n' || *src == '\0')
        {
          fprintf(stderr, "%d: Unexpected end of line: \"%s\"\n", g_lineno, start);
          exit(4);
        }
      else
        {
          *dest++ = *src++;
        }
    }

  fprintf(stderr, "%d: Parameter too long: \"%s\"\n", g_lineno, start);
  exit(3);
}

/****************************************************************************
 * Name: find_parm
 ****************************************************************************/

static char *find_parm(char *ptr)
{
  char *start = ptr;

  if (*ptr != '"')
    {
      fprintf(stderr, "%d: I'm confused: \"%s\"\n", g_lineno, start);
      exit(5);
    }
  ptr++;

  ptr = skip_space(ptr);
  if (*ptr == '\n' || *ptr == '\0')
    {
      return NULL;
    }
  else if (*ptr != ',')
    {
      fprintf(stderr, "%d: Expected ',': \"%s\"\n", g_lineno, start);
      exit(6);
    }
  ptr++;

  ptr = skip_space(ptr);
  if (*ptr != '"')
    {
      fprintf(stderr, "%d: Expected \": \"%s\"\n", g_lineno, start);
      exit(7);
    }
  ptr++;

  return ptr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: read_line
 ****************************************************************************/

char *read_line(FILE *stream)
{
  char *ptr;

  for (;;)
    {
      g_line[LINESIZE] = '\0';
      if (!fgets(g_line, LINESIZE, stream))
        {
          return NULL;
        }
      else
        {
          g_lineno++;
          if (g_debug)
            {
              printf("Line: %s\n", g_line);
            }

          ptr = skip_space(g_line);
          if (*ptr && *ptr != '#' && *ptr != '\n')
            {
              return ptr;
            }
        }
    }
}

/****************************************************************************
 * Name: parse_csvline
 ****************************************************************************/

int parse_csvline(char *ptr)
{
  int nparms;
  int i;

  /* Format "arg1","arg2","arg3",... Spaces will be tolerated outside of the
   * quotes.  Any initial spaces have already been skipped so the first thing
   * should be '"'.
   */

  if (*ptr != '"')
    {
      fprintf(stderr, "%d: Bad line: \"%s\"\n", g_lineno, g_line);
      exit(2);
    }

  ptr++;
  nparms = 0;

  /* Copy each comma-separated value in an array (stripping quotes from each
   * of the values).
   */

  do
    {
      ptr = copy_parm(ptr, &g_parm[nparms][0]);
      nparms++;
      ptr = find_parm(ptr);
    }
  while (ptr);

  /* If debug is enabled, show what we got */

  if (g_debug)
    {
      printf("Parameters: %d\n", nparms);
      for (i = 0; i < nparms; i++)
        {
          printf("  Parm%d: \"%s\"\n", i+1, g_parm[i]);
        }
    }

  return nparms;
}
