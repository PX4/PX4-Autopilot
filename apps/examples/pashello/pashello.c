/****************************************************************************
 * examples/pashello/pashello.c
 *
 *   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <debug.h>

#include "pexec.h"
#include "pedefs.h"
#include "pashello.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_PASHELLO_VARSTACKSIZE
# define CONFIG_PASHELLO_VARSTACKSIZE 1024
#endif

#ifndef CONFIG_PASHELLO_STRSTACKSIZE
# define CONFIG_PASHELLO_STRSTACKSIZE 128
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prun
 *
 * Description:
 *   This function executes the P-Code program until a stopping condition
 *   is encountered.
 *
 ****************************************************************************/

static void prun(FAR struct pexec_s *st)
{
  int errcode;

  for (;;)
    {
      /* Execute the instruction; Check for exceptional conditions */

      errcode = pexec(st);
      if (errcode != eNOERROR) break;
    }

  if (errcode != eEXIT)
    {
      printf("Runtime error 0x%02x -- Execution Stopped\n", errcode);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * pashello_main
 ****************************************************************************/

int pashello_main(int argc, FAR char *argv[])
{
  FAR struct pexec_s *st;

  /* Register the /dev/hello driver */

  hello_register();

  /* Load the POFF file */

  st = pload("/dev/hello", CONFIG_PASHELLO_VARSTACKSIZE, CONFIG_PASHELLO_STRSTACKSIZE);
  if (!st)
    {
      fprintf(stderr, "pashello_main: ERROR: Could not load /dev/hello\n");
      exit(1);
    }
  printf("pashello_main: /dev/hello Loaded\n");
  printf("pashello_main: Interpreter started:\n");

  /* And start program execution */

  prun(st);

  /* Clean up resources used by the interpreter */

  printf("pashello_main: Interpreter terminated");
  pexec_release(st);
  return 0;
}
