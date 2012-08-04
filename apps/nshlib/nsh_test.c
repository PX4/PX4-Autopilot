/****************************************************************************
 * apps/nshlib/nsh_test.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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

/* Test syntax:
 *
 * expression = simple-expression | !expression |
 *              expression -o expression | expression -a expression
 *
 * simple-expression = unary-expression | binary-expression
 *
 * unary-expression = string-unary | file-unary
 *
 * string-unary = -n string | -z string
 *
 * file-unary = -b file | -c file | -d file | -e file | -f file |
 *              -r file | -s file | -w file
 *
 * binary-expression = string-binary | numeric-binary
 *
 * string-binary = string = string | string == string | string != string
 *
 * numeric-binary = integer -eq integer | integer -ge integer |
 *                  integer -gt integer | integer -le integer |
 *                  integer -lt integer | integer -ne integer
 *
 * Note that the smallest expression consists of two strings.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>

#include "nsh.h"
#include "nsh_console.h"

#if !defined(CONFIG_NSH_DISABLESCRIPT) && !defined(CONFIG_NSH_DISABLE_TEST)

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define TEST_TRUE  OK
#define TEST_FALSE ERROR
#define TEST_ERROR 1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binaryexpression
 ****************************************************************************/

static inline int binaryexpression(FAR struct nsh_vtbl_s *vtbl, char **argv)
{
  char *endptr;
  long integer1;
  long integer2;

  /* STRING2 = STRING2 */

  if (strcmp(argv[1], "=") == 0 || strcmp(argv[1], "==") == 0)
    {
      /* Return true if the strings are identical */

      return strcmp(argv[0], argv[2]) == 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* STRING1 != STRING2 */

  if (strcmp(argv[1], "!=") == 0)
    {
      /* Return true if the strings are different */

      return strcmp(argv[0], argv[2]) != 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* The remaining operators assuming that the two values are integers */

  integer1 = strtol(argv[0], &endptr, 0);
  if (argv[0][0] == '\0' || *endptr != '\0')
    {
      return TEST_ERROR;
    }

  integer2 = strtol(argv[2], &endptr, 0);
  if (argv[2][0] == '\0' || *endptr != '\0')
    {
      return TEST_ERROR;
    }

  /* INTEGER1 -eq INTEGER2 */

  if (strcmp(argv[1], "-eq") == 0)
    {
      /* Return true if the strings are different */

      return integer1 == integer2 ? TEST_TRUE : TEST_FALSE;
    }

  /* INTEGER1 -ge INTEGER2 */

  if (strcmp(argv[1], "-ge") == 0)
    {
      /* Return true if the strings are different */

      return integer1 >= integer2 ? TEST_TRUE : TEST_FALSE;
    }

  /* INTEGER1 -gt INTEGER2 */

  if (strcmp(argv[1], "-gt") == 0)
    {
      /* Return true if the strings are different */

      return integer1 > integer2 ? TEST_TRUE : TEST_FALSE;
    }

  /* INTEGER1 -le INTEGER2 */

  if (strcmp(argv[1], "-le") == 0)
    {
      /* Return true if the strings are different */

      return integer1 <= integer2 ? TEST_TRUE : TEST_FALSE;
    }

  /* INTEGER1 -lt INTEGER2 */

  if (strcmp(argv[1], "-lt") == 0)
    {
      /* Return true if the strings are different */

      return integer1 < integer2 ? TEST_TRUE : TEST_FALSE;
    }

  /* INTEGER1 -ne INTEGER2 */

  if (strcmp(argv[1], "-ne") == 0)
    {
      /* Return true if the strings are different */

      return integer1 != integer2 ? TEST_TRUE : TEST_FALSE;
    }

  return TEST_ERROR;
}

/****************************************************************************
 * Name: unaryexpression
 ****************************************************************************/

static inline int unaryexpression(FAR struct nsh_vtbl_s *vtbl, char **argv)
{
  struct stat buf;
  char *fullpath;
  int   ret;

  /* -n STRING */

  if (strcmp(argv[0], "-n") == 0)
    {
      /* Return true if the length of the string is non-zero */

      return strlen(argv[1]) != 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* -z STRING */

  if (strcmp(argv[0], "-z") == 0)
    {
      /* Return true if the length of the string is zero */

      return strlen(argv[1]) == 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* All of the remaining assume that the following argument is the
   * path to a file.
   */

  fullpath = nsh_getfullpath(vtbl, argv[1]);
  if (!fullpath)
    {
       return TEST_FALSE;
    }

  ret = stat(fullpath, &buf);
  nsh_freefullpath(fullpath);

  if (ret != 0)
    {
       /* The file does not exist (or another error occurred) -- return FALSE */

       return TEST_FALSE;
    }

  /* -b FILE */

  if (strcmp(argv[0], "-b") == 0)
    {
      /* Return true if the path is a block device */

      return S_ISBLK(buf.st_mode) ? TEST_TRUE : TEST_FALSE;
    }

  /* -c FILE */

  if (strcmp(argv[0], "-c") == 0)
    {
      /* Return true if the path is a character device */

      return S_ISCHR(buf.st_mode) ? TEST_TRUE : TEST_FALSE;
    }

  /* -d FILE */

  if (strcmp(argv[0], "-d") == 0)
    {
      /* Return true if the path is a directory */

      return S_ISDIR(buf.st_mode) ? TEST_TRUE : TEST_FALSE;
    }

  /* -e FILE */

  if (strcmp(argv[0], "-e") == 0)
    {
      /* Return true if the file exists */

      return TEST_TRUE;
    }

  /* -f FILE */

  if (strcmp(argv[0], "-f") == 0)
    {
      /* Return true if the path refers to a regular file */

      return S_ISREG(buf.st_mode) ? TEST_TRUE : TEST_FALSE;
    }

  /* -r FILE */

  if (strcmp(argv[0], "-r") == 0)
    {
      /* Return true if the file is readable */

      return (buf.st_mode & (S_IRUSR|S_IRGRP|S_IROTH)) != 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* -s FILE */

  if (strcmp(argv[0], "-s") == 0)
    {
      /* Return true if the size of the file is greater than zero */

      return buf.st_size > 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* -w FILE */

  if (strcmp(argv[0], "-w") == 0)
    {
      /* Return true if the file is write-able */

      return (buf.st_mode & (S_IWUSR|S_IWGRP|S_IWOTH)) != 0 ? TEST_TRUE : TEST_FALSE;
    }

  /* Unrecognized operator */

  return TEST_ERROR;
}

/****************************************************************************
 * Name: expression
 ****************************************************************************/

static int expression(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  int value;
  int i = 0;

  /* Check for unary operations on expressions */

  if (strcmp(argv[0], "!") == 0)
    {
      if (argc < 2)
        {
          goto errout_syntax;
        }
      return expression(vtbl, argc-1, &argv[1]) == TEST_TRUE ? TEST_FALSE : TEST_TRUE;
    }

  /* Check for unary operations on simple, typed arguments */

  else if (argv[0][0] == '-')
    {
      if (argc < 2)
        {
          goto errout_syntax;
        }
      i += 2;
      value = unaryexpression(vtbl, argv);
    }

  /* Check for binary operations on simple, typed arguments */

  else
    {
      if (argc < 3)
        {
          goto errout_syntax;
        }
      i += 3;
      value = binaryexpression(vtbl, argv);
    }

  /* Test if there any failure */

  if (value == TEST_ERROR)
    {
      goto errout_syntax;
    }

  /* Is there anything after the simple expression? */

  if (i < argc)
    {
      /* EXPRESSION -a EXPRESSION */

       if (strcmp(argv[i], "-a") == 0)
         {
            if (value != TEST_TRUE)
              {
                 return TEST_FALSE;
              }
            else
              {
                 i++;
                 return expression(vtbl, argc-i, &argv[i]);
              }
         }

       /* EXPRESSION -o EXPRESSION */

       else if (strcmp(argv[i], "-o") == 0)
         {
           if (value == TEST_TRUE)
             {
                return TEST_TRUE;
             }
           else
             {
                i++;
                return expression(vtbl, argc-i, &argv[i]);
             }
         }
       else
         {
           goto errout_syntax;
         }
    }
  return value; 

errout_syntax:
  nsh_output(vtbl, g_nshsyntax, "test");
  return TEST_FALSE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_test
 ****************************************************************************/

int cmd_test(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return expression(vtbl, argc-1, &argv[1]);
}

/****************************************************************************
 * Name: cmd_lbracket
 ****************************************************************************/

int cmd_lbracket(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  if (strcmp(argv[argc-1], "]") != 0)
    {
      nsh_output(vtbl, g_nshsyntax, argv[0]);
      return ERROR;
    }
  else
    {
      return expression(vtbl, argc-2, &argv[1]);
    }
}

#endif /* !CONFIG_NSH_DISABLESCRIPT && !CONFIG_NSH_DISABLE_TEST */
