/****************************************************************************
 * tools/b16.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr, "\nUSAGE: %s <b16_t>|<float>\n", progname);
  fprintf(stderr, "\nWhere:\n");
  fprintf(stderr, "  <b16_t>:\n");
  fprintf(stderr, "    A b16 fixed precision value in hexadecimal form: E.g., 0x00010000\n");
  fprintf(stderr, "    Any value begininning with '0' will assumed by be hexadecimal format\n");
  fprintf(stderr, "  <float>:\n");
  fprintf(stderr, "    A floating value in standard form: E.g., 5.1\n");
  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  double fvalue;
  unsigned long ulvalue;
  long lvalue;
  const char *str;
  char *endptr;

  /* There must be exactly one argument */

  if (argc != 2)
    {
      fprintf(stderr, "\nExpected a single argument\n");
      show_usage(argv[0]);
    }
  str = argv[1];

  /* If the value begins with a zero, we will assume that it is a hexadecimal
   * representation.
   */

  if (str[0] == '0')
    {
      endptr = NULL;
      ulvalue = strtoul(str, &endptr, 16);
      if (!endptr || *endptr != '\0')
        {
          fprintf(stderr, "\nHexadecimal argument not fully converted\n");
          show_usage(argv[0]);
        }

      if (ulvalue >= 0x80000000)
        {
          lvalue = ~ulvalue + 1;
        }
      else
        {
          lvalue = ulvalue;
        }

      fvalue = ((double)lvalue) / 65536.0;
      printf("0x%08lx -> %10.5f\n", ulvalue, fvalue);
    }
  else
    {
      endptr = NULL;
      fvalue = strtod(str, &endptr);
      if (!endptr || *endptr != '\0')
        {
          fprintf(stderr, "\nFloating point argument not fully converted\n");
          show_usage(argv[0]);
        }

      lvalue = 65536.0 * fvalue;
      printf("%10.5f -> 0x%08lx\n", fvalue, lvalue);
    }
   
  return 0;
}
