/****************************************************************************
 * tools/cfgpaser.c
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

#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "cfgparser.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

char line[LINESIZE+1];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 /* Skip over any spaces */
 
static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

/* Find the end of a variable string */

static char *find_name_end(char *ptr)
{
  while (*ptr && (isalnum((int)*ptr) || *ptr == '_')) ptr++;
  return ptr;
}

/* Find the end of a value string */

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

/* Read the next line from the configuration file */

static char *read_line(FILE *stream)
{
  char *ptr;

  for (;;)
    {
      line[LINESIZE] = '\0';
      if (!fgets(line, LINESIZE, stream))
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

/* Parse the line from the configuration file into a variable name
 * string and a value string.
 */

static void parse_line(char *ptr, char **varname, char **varval)
{
  /* Skip over any leading spaces */

  ptr = skip_space(ptr);

  /* The first no-space is the beginning of the variable name */

  *varname = skip_space(ptr);
  *varval = NULL;

  /* Parse to the end of the variable name */

  ptr = find_name_end(ptr);

  /* An equal sign is expected next, perhaps after some white space */

  if (*ptr && *ptr != '=')
    {
      /* Some else follows the variable name.  Terminate the variable
       * name and skip over any spaces.
       */

      *ptr = '\0';
       ptr = skip_space(ptr + 1);
    }

  /* Verify that the equal sign is present */

  if (*ptr == '=')
    {
      /* Make sure that the variable name is terminated (this was already
       * done if the name was followed by white space.
       */

      *ptr = '\0';

      /* The variable value should follow =, perhaps separated by some
       * white space.
       */
 
      ptr = skip_space(ptr + 1);
      if (*ptr)
        {
          /* Yes.. a variable follows.  Save the pointer to the start
           * of the variable string.
           */

          *varval = ptr;

          /* Find the end of the variable string and make sure that it
           * is terminated.
           */

          ptr = find_value_end(ptr);
          *ptr = '\0';
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void parse_file(FILE *stream, struct variable_s **list)
{
  struct variable_s *curr;
  struct variable_s *prev;
  struct variable_s *next;
  char *varname;
  char *varval;
  char *ptr;

  /* Loop until the entire file has been parsed. */

  do
    {
      /* Read the next line from the file */

      ptr = read_line(stream);
      if (ptr)
        {
          /* Parse the line into a variable and a value field */

          parse_line(ptr, &varname, &varval);

          /* If the variable has not value (or the special value 'n'), then
           * ignore it.
           */

          if (!varval || strcmp(varval, "n") == 0)
            {
              continue;
            }

          /* Make sure that a variable name was found. */

          if (varname)
            {
              int varlen = strlen(varname) + 1;
              int vallen = 0;

              /* Get the size of the value, including the NUL terminating
               * character.
               */

              if (varval)
                {
                  vallen = strlen(varval) + 1;
                }

              /* Allocate memory to hold the struct variable_s with the
               * variable name and the value.
               */

              curr = (struct variable_s *)malloc(sizeof(struct variable_s) + varlen + vallen - 1);
              if (curr)
                {
                  /* Add the variable to the list */

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

struct variable_s *find_variable(const char *varname, struct variable_s *list)
{
  while (list)
    {
      if (strcmp(varname, list->var) == 0)
        {
          return list;
        }
      
      list = list->flink;
    }

  return NULL;
}

