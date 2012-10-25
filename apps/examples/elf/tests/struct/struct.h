/****************************************************************************
 * examples/elf/tests/struct/struct.h
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

#ifndef __EXAMPLES_ELF_TESTS_STRUCT_STRUCT_H
#define __EXAMPLES_ELF_TESTS_STRUCT_STRUCT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUMMY_SCALAR_VALUE1 42
#define DUMMY_SCALAR_VALUE2 87
#define DUMMY_SCALAR_VALUE3 117

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*dummy_t)(void);

struct struct_dummy_s
{
  int  n;    /* This is a simple scalar value (DUMMY_SCALAR_VALUE3) */
};

struct struct_s
{
  int  n;                           /* This is a simple scalar value (DUMMY_SCALAR_VALUE1) */
  const int *pn;                    /* This is a pointer to a simple scalar value */
  const struct struct_dummy_s *ps;  /* This is a pointer to a structure */
  dummy_t pf;                       /* This is a pointer to a function */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern int    dummy_scalar; /* (DUMMY_SCALAR_VALUE2) */
extern const struct struct_dummy_s dummy_struct;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern void dummyfunc(void);
extern const struct struct_s *getstruct(void);

#endif /* __EXAMPLES_ELF_TESTS_STRUCT_STRUCT_H */


