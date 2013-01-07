/****************************************************************************
 * examples/ostest/vfork.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "ostest.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_VFORK) && !defined(CONFIG_DISABLE_SIGNALS)
static volatile bool g_vforkchild;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int vfork_test(void)
{
#if defined(CONFIG_ARCH_HAVE_VFORK) && !defined(CONFIG_DISABLE_SIGNALS)
  pid_t pid;

  g_vforkchild = false;
  pid = vfork();
  if (pid == 0)
    {
      /* There is not very much that the child is permitted to do.  Perhaps
       * it can just set g_vforkchild.
       */

      g_vforkchild = true;
      exit(0);
    }
  else if (pid < 0)
    {
      printf("vfork_test: vfork() failed: %d\n", errno);
      return -1;
    }
  else
    {
      sleep(1);
      if (g_vforkchild)
        {
          printf("vfork_test: Child %d ran successfully\n", pid);
        }
      else
        {
          printf("vfork_test: ERROR Child %d did not run\n", pid);
          return -1;
        }
    }
#endif

  return 0;
}
