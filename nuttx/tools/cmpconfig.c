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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct variable_s
{
  struct variable_s *flink;
  char *var;
  char *val;
  char storage[1];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <config1> <config2>\n", progname);
  exit(EXIT_FAILURE);
}

static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

static char *read_line(FILE *stream, char *line, int len)
{
  char *ptr;

  for (;;)
    {
      line[len-1] = '\0';
      if (!fgets(line, len, stream))
        {
          return NULL;
        }
      else
        {
          ptr = skip_space(line);
          if (*ptr && *ptr != '#' && *ptr != '\n')
            {
              return ptr;
            }
        }
    }
}

static char *find_name_end(char *ptr)
{
  while (*ptr && (isalnum((int)*ptr) || *ptr == '_')) ptr++;
  return ptr;
}

static char *find_value_end(char *ptr)
{
  while (*ptr && !isspace((int)*ptr))
    {
      if (*ptr == '"')
        {
           do ptr++; while (*ptr && *ptr != '"');
           if (*ptr) ptr++;
        }
      else
        {
           do ptr++; while (*ptr && !isspace((int)*ptr) && *ptr != '"');
        }
    }
  return ptr;
}

static void parse_line(char *ptr, char **varname, char **varval)
{
  *varname = ptr;
  *varval = NULL;

   ptr = find_name_end(ptr);
   if (*ptr && *ptr != '=')
    {
      *ptr = '\0';
       ptr = skip_space(ptr + 1);
    }

  if (*ptr == '=')
    {
      *ptr = '\0';
      ptr = skip_space(ptr + 1);
      if (*ptr)
        {
          *varval = ptr;
          ptr = find_value_end(ptr);
          *ptr = '\0';
        }
    }
}

static void parse_file(FILE *stream, struct variable_s **list)
{
  char line[10242];
  struct variable_s *curr;
  struct variable_s *prev;
  struct variable_s *next;
  char *varname;
  char *varval;
  char *ptr;

  do
    {
      ptr = read_line(stream, line, 1024);
      if (ptr)
        {
          parse_line(ptr, &varname, &varval);
          if (!varval || strcmp(varval, "n") == 0)
            {
              continue;
            }

          if (varname)
            {
              int varlen = strlen(varname) + 1;
              int vallen = 0;

              if (varval)
                {
                  vallen = strlen(varval) + 1;
                }

              curr = (struct variable_s *)malloc(sizeof(struct variable_s) + varlen + vallen - 1);
              if (curr)
                {
                  curr->var = &curr->storage[0];
                  strcpy(curr->var, varname);

                  curr->val = NULL;
                  if (varval)
                    {
                      curr->val = &curr->storage[varlen];
                      strcpy(curr->val, varval);
                    }
                }

              prev = 0;
              next = *list;
              while (next && strcmp(next->var, curr->var) <= 0)
                {
                  prev = next;
                  next = next->flink;
                }

              if (prev)
                {
                  prev->flink = curr;
                }
              else
                {
                  *list = curr;
                }
              curr->flink = next;
            }
        }
    }
  while (ptr);
}

static void compare_variables(struct variable_s *list1, struct variable_s *list2)
{
  char *varval1;
  char *varval2;

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
          int result = strcmp(list1->var, list2->var);
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
          else
            {
              int result = strcmp(varval1, varval2);
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
