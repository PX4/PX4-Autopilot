/****************************************************************************
 * examples/nxflat/tests/struct/struct_main.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include "struct.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct struct_dummy_s dummy_struct = 
{
   DUMMY_SCALAR_VALUE3
};

int dummy_scalar = DUMMY_SCALAR_VALUE2;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  const struct struct_s *mystruct = getstruct();

  printf("Calling getstruct()\n");
  mystruct = getstruct();
  printf("getstruct returned %p\n", mystruct);
  printf("  n = %d (vs %d) %s\n",
         mystruct->n, DUMMY_SCALAR_VALUE1,
         mystruct->n == DUMMY_SCALAR_VALUE1 ? "PASS" : "FAIL");

  printf("  pn = %p (vs %p) %s\n",
         mystruct->pn, &dummy_scalar,
         mystruct->pn == &dummy_scalar ? "PASS" : "FAIL");
  if (mystruct->pn == &dummy_scalar)
    {
      printf(" *pn = %d (vs %d) %s\n",
             *mystruct->pn, DUMMY_SCALAR_VALUE2,
             *mystruct->pn == DUMMY_SCALAR_VALUE2 ? "PASS" : "FAIL");
    }

  printf("  ps = %p (vs %p) %s\n",
         mystruct->ps, &dummy_struct,
         mystruct->ps == &dummy_struct ? "PASS" : "FAIL");
  if (mystruct->ps == &dummy_struct)
    {
      printf("  ps->n = %d (vs %d) %s\n",
             mystruct->ps->n, DUMMY_SCALAR_VALUE3,
             mystruct->ps->n == DUMMY_SCALAR_VALUE3 ? "PASS" : "FAIL");
    }

  printf("  pf = %p (vs %p) %s\n",
         mystruct->pf, dummyfunc,
         mystruct->pf == dummyfunc ? "PASS" : "FAIL");
  if (mystruct->pf == dummyfunc)
    {
      printf("Calling mystruct->pf()\n");
      mystruct->pf();
    }

  printf("Exit-ing\n");
  return 0;
}

void dummyfunc(void)
{
  printf("In dummyfunc() -- PASS\n");
}


