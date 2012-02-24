/***********************************************************************
 * apps/examples/ostest/fpu.c
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
 ***********************************************************************/

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <nuttx/config.h>
#include <sys/wait.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>

#include "ostest.h"

/***********************************************************************
 * Pre-processor definitions
 ***********************************************************************/
/* Configuration *******************************************************/

#undef HAVE_FPU
#ifdef CONFIG_ARCH_FPU
#  if defined(CONFIG_EXAMPLES_OSTEST_FPUSIZE) && defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_DISABLE_SIGNALS)
#    define HAVE_FPU 1
#  else
#    ifndef CONFIG_EXAMPLES_OSTEST_FPUSIZE
#      warning "FPU test not built; CONFIG_EXAMPLES_OSTEST_FPUSIZE not defined"
#    endif
#    ifndef CONFIG_SCHED_WAITPID
#      warning "FPU test not built; CONFIG_SCHED_WAITPID not defined"
#    endif
#    ifdef CONFIG_DISABLE_SIGNALS
#      warning "FPU test not built; CONFIG_DISABLE_SIGNALS defined"
#    endif
#  endif
#endif

#ifdef HAVE_FPU

#ifndef CONFIG_EXAMPLES_OSTEST_FPULOOPS
#  define CONFIG_EXAMPLES_OSTEST_FPULOOPS 16
#endif

#ifndef CONFIG_EXAMPLES_OSTEST_FPUMSDELAY
#  define CONFIG_EXAMPLES_OSTEST_FPUMSDELAY 750
#endif

#ifndef CONFIG_EXAMPLES_OSTEST_FPUPRIORITY
#  define CONFIG_EXAMPLES_OSTEST_FPUPRIORITY SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_OSTEST_FPUSTACKSIZE
#  define CONFIG_EXAMPLES_OSTEST_FPUSTACKSIZE 2048
#endif

/* Other defintions ****************************************************/

#ifndef NULL
# define NULL (void*)0
#endif

/***********************************************************************
 * External Dependencies
 ***********************************************************************/
/* This test is very dependent on support provided by the chip/board-
 * layer logic.  In particular, it expects the following functions
 * to be provided:
 */

/* Given a uint8_t array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE, this
 * function will return the current FPU registers.
 */

extern void arch_getfpu(FAR uint8_t *fpusave);

/* Given a uint8_t array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE, this
 * function will set the current FPU regisers to match the provided
 * register save set.
 */

extern void arch_setfpu(FAR const uint8_t *fpusave);

/* Given a uint8_t array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE and a
 * seed value, this function will set the FPU registers to a known
 * values for testing purposes.  The contents of the FPU registers
 * must be uniqe for each sed value.
 */

extern void arch_initfpu(FAR uint8_t *fpusave, int seed);

/* Given two uint8_t arrays of size CONFIG_EXAMPLES_OSTEST_FPUSIZE this
 * function will compare then an return true if they are identical.
 */

extern bool arch_cmpfpu(FAR const uint8_t *fpusave1, FAR const uint8_t *fpusave2);

/***********************************************************************
 * Private Data
 ***********************************************************************/

static uint8_t g_fpuno;

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void fpu_dump(FAR uint8_t *buffer, FAR const char *msg)
{
  int i, j, k;

  printf("%s (%p):\n", msg, buffer);
  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_FPUSIZE; i += 32)
    {
      printf("%04x: ", i);
      for (j = 0; j < 32; j++)
        {
          k = i + j;

          if (k < CONFIG_EXAMPLES_OSTEST_FPUSIZE)
            {
              printf("%02x ", buffer[k]);
            }
          else
            {
              break;
            }
        }
      printf("\n");
   }
}

static int fpu_task(int argc, char *argv[])
{
  uint8_t fpusave1[CONFIG_EXAMPLES_OSTEST_FPUSIZE];
  uint8_t fpusave2[CONFIG_EXAMPLES_OSTEST_FPUSIZE];
  int id;
  int seed;
  int incr;
  int i;

  /* Which are we? */

  sched_lock();
  id = (int)(++g_fpuno);
  sched_unlock();

  seed = id << 24 | id << 16 | id << 8 | id;
  incr = seed;

  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_FPULOOPS; i++)
    {
      /* Initialize the FPU registers to a known value */

      sched_lock();
      arch_initfpu(fpusave1, seed);
      arch_setfpu(fpusave1);

      /* Check that the FPU registers were set as we expected */

      arch_getfpu(fpusave2);
      if (!arch_cmpfpu(fpusave1, fpusave2))
        {
          printf("ERROR FPU#%d: fpusave1 and fpusave2 do not match (BEFORE waiting)\n");
          fpu_dump(fpusave1, "Values written:");
          fpu_dump(fpusave2, "Values read:");
          sched_unlock();
          return EXIT_FAILURE;
        }

      /* Now unlock and sleep for a while */

      sched_unlock();
      usleep(CONFIG_EXAMPLES_OSTEST_FPUMSDELAY * 1000);

      /* Several context switches should have occurred.  Now verify that the floating
       * point registers are still correctly set.
       */

      sched_lock();
      arch_getfpu(fpusave2);
      if (!arch_cmpfpu(fpusave1, fpusave2))
        {
          printf("ERROR FPU#%d: fpusave1 and fpusave2 do not match (AFTER waiting)\n");
          fpu_dump(fpusave1, "Values written:");
          fpu_dump(fpusave2, "Values read:");
          sched_unlock();
          return EXIT_FAILURE;
        }

      seed += incr;
    }

  printf("FPU#%d: Succeeded\n");
  return EXIT_SUCCESS;
}
#endif /* HAVE_FPU */

/***********************************************************************
 * Private Functions
 ***********************************************************************/

void fpu_test(void)
{
#ifdef HAVE_FPU
  pid_t task1;
  pid_t task2;
  int statloc;

  /* Start two two tasks */

  g_fpuno = 0;
  printf("Starting task FPU#1\n");
  task1 = TASK_CREATE("FPU#1", CONFIG_EXAMPLES_OSTEST_FPUPRIORITY, CONFIG_EXAMPLES_OSTEST_FPUSTACKSIZE, fpu_task, NULL);
  if (task1 < 0)
    {
      printf("fpu_test: ERROR Failed to start task FPU#1\n");
    }
  else
    {
      printf("fpu_test: Started task FPU#1 at PID=%d\n", task1);
    }
  usleep(250);

  printf("Starting task FPU#2\n");
  task2 = TASK_CREATE("FPU#2", CONFIG_EXAMPLES_OSTEST_FPUPRIORITY, CONFIG_EXAMPLES_OSTEST_FPUSTACKSIZE, fpu_task, NULL);
  if (task2 < 0)
    {
      printf("fpu_test: ERROR Failed to start task FPU#1\n");
    }
  else
    {
      printf("fpu_test: Started task FPU#2 at PID=%d\n", task2);
    }

  /* Wait for each task to complete */

  (void)waitpid(task1, &statloc, 0);
  (void)waitpid(task2, &statloc, 0);

#else
  printf("fpu_test: ERROR: The FPU test is not properly configured\n");
#endif
  printf("fpu_test: Returning\n");
}
