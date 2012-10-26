/****************************************************************************
 * examples/elf/tests/longjmp/longjmp.c
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
#include <setjmp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAIN_VAL        47
#define FUNC_VAL        92
#define LEAF_VAL       163

#define FUNCTION_ARG   MAIN_VAL
#define LEAF_ARG      (FUNCTION_ARG + FUNC_VAL)
#define SETJMP_RETURN (LEAF_ARG + LEAF_VAL)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static jmp_buf env;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int leaf(int *some_arg)
{
  int some_local_variable = *some_arg + LEAF_VAL;

  printf("leaf: received %d\n", *some_arg);

  if (*some_arg != LEAF_ARG)
    printf("leaf: ERROR: expected %d\n", LEAF_ARG);

  printf("leaf: Calling longjmp() with %d\n", some_local_variable);

  longjmp(env, some_local_variable);

  /* We should not get here */

  return -ERROR; 
}

static int function(int some_arg)
{
  int some_local_variable = some_arg + FUNC_VAL;
  int retval;

  printf("function: received %d\n", some_arg);

  if (some_arg != FUNCTION_ARG)
    printf("function: ERROR: expected %d\n", FUNCTION_ARG);

  printf("function: Calling leaf() with %d\n", some_local_variable);

  retval = leaf(&some_local_variable);

  printf("function: ERROR -- leaf returned!\n");
  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int value;

  printf("main: Calling setjmp\n");
  value = setjmp(env);
  printf("main: setjmp returned %d\n", value);

  if (value == 0)
    {
      printf("main: Normal setjmp return\n");
      printf("main: Calling function with %d\n", MAIN_VAL);
      function(MAIN_VAL);
      printf("main: ERROR -- function returned!\n");
      return 1;
    }
  else if (value != SETJMP_RETURN)
    {
      printf("main: ERROR: Expected %d\n", SETJMP_RETURN);
      return 1;
    }
  else
    {
      printf("main: SUCCESS: setjmp return from longjmp call\n");
      return 0;
    }
}

