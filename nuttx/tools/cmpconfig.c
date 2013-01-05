/****************************************************************************
 * tools/cmpconfig.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

#include "cfgparser.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <config1> <config2>\n", progname);
  exit(EXIT_FAILURE);
}

static void compare_variables(struct variable_s *list1, struct variable_s *list2)
{
  char *varval1;
  char *varval2;
  int result;

  while (list1 || list2)
    {
      if (list1 && list1->val)
        {
          varval1 = list1->val;
        }
      else
        {
          varval1 = "<NULL>";
        }

      if (list2 && list2->val)
        {
          varval2 = list2->val;
        }
      else
        {
          varval2 = "<NULL>";
        }

      if (!list1)
        {
          printf("file1:\n");
          printf("file2: %s=%s\n\n", list2->var, varval2);
          list2 = list2->flink;
        }
      else if (!list2)
        {
          printf("file1: %s=%s\n", list1->var, varval1);
          printf("file2:\n\n");
          list1 = list1->flink;
        }
      else
        {
          result = strcmp(list1->var, list2->var);
          if (result < 0)
            {
              printf("file1: %s=%s\n", list1->var, varval1);
              printf("file2:\n\n");
              list1 = list1->flink;
            }
          else if (result > 0)
            {
              printf("file1:\n");
              printf("file2: %s=%s\n\n", list2->var, varval2);
              list2 = list2->flink;
            }
          else /* if (result == 0) */
            {
              result = strcmp(varval1, varval2);
              if (result != 0)
                {
                  printf("file1: %s=%s\n", list1->var, varval1);
                  printf("file2: %s=%s\n\n", list2->var, varval2);
                }

              list1 = list1->flink;
              list2 = list2->flink;
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct variable_s *list1 = 0;
  struct variable_s *list2 = 0;
  FILE *stream1;
  FILE *stream2;

  if (argc != 3)
    {
      fprintf(stderr, "Unexpected number of arguments: %d\n\n", argc);
      show_usage(argv[0]);
    }

  stream1 = fopen(argv[1], "r");
  if (!stream1)
    {
      fprintf(stderr, "Failed to open %s for reading: %s\n\n",
        argv[1], strerror(errno));
      show_usage(argv[0]);
    }

  stream2 = fopen(argv[2], "r");
  if (!stream2)
    {
      fprintf(stderr, "Failed to open %s for reading: %s\n\n",
        argv[2], strerror(errno));
      show_usage(argv[0]);
    }

  parse_file(stream1, &list1);
  parse_file(stream2, &list2);

  fclose(stream1);
  fclose(stream2);

  printf("Comparing:\n\n");
  printf("  file1 = %s\n", argv[1]);
  printf("  file2 = %s\n\n", argv[2]);
  compare_variables(list1, list2);
  return EXIT_SUCCESS;
}
