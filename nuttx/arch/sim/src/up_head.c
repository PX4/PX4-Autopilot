/****************************************************************************
 * up_head.c
 *
 *   Copyright (C) 2007-2009, 2011-2112 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <nuttx/power/pm.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static jmp_buf sim_abort;

/****************************************************************************
 * Global Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  /* Power management should be initialized early in the (simulated) boot
   * sequence.
   */

#ifdef CONFIG_PM
  pm_initialize();
#endif

  /* Then start NuttX */

  if (setjmp(sim_abort) == 0)
    {
      os_start();
    }
  return 0;
}

void up_assert(const uint8_t *filename, int line)
{
  fprintf(stderr, "Assertion failed at file:%s line: %d\n", filename, line);
  longjmp(sim_abort, 1);
}

void up_assert_code(const uint8_t *filename, int line, int code)
{
  fprintf(stderr, "Assertion failed at file:%s line: %d error code: %d\n", filename, line, code);
  longjmp(sim_abort, 1);
}
