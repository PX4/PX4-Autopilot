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
#include <string.h>
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
/* We'll keep all data using 32-bit values only to force 32-bit alignment.
 * This logic has no real notion of the underlying representation.
 */

#define FPU_WORDSIZE ((CONFIG_EXAMPLES_OSTEST_FPUSIZE+3)>>2)
#define FPU_NTHREADS  2

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

/* Given an array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE, this function
 * will return the current FPU registers.
 */

extern void arch_getfpu(FAR uint32_t *fpusave);

/* Given two arrays of size CONFIG_EXAMPLES_OSTEST_FPUSIZE this
 * function will compare them and return true if they are identical.
 */

extern bool arch_cmpfpu(FAR const uint32_t *fpusave1,
                        FAR const uint32_t *fpusave2);

/***********************************************************************
 * Private Types
 ***********************************************************************/

struct fpu_threaddata_s
{
  uint32_t save1[FPU_WORDSIZE];
  uint32_t save2[FPU_WORDSIZE];

  /* These are just dummy values to force the compiler to do the
   * requested floating point computations without the nonsense
   * computations being optimized away.
   */

  volatile float sp1;
  volatile float sp2;
  volatile float sp3;
  volatile float sp4;

  volatile float dp1;
  volatile float dp2;
  volatile float dp3;
  volatile float dp4;
};

/***********************************************************************
 * Private Data
 ***********************************************************************/

static uint8_t g_fpuno;
/* static */ struct fpu_threaddata_s g_fputhread[FPU_NTHREADS];

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void fpu_dump(FAR uint32_t *buffer, FAR const char *msg)
{
  int i, j, k;

  printf("%s (%p):\n", msg, buffer);
  for (i = 0; i < FPU_WORDSIZE; i += 8)
    {
      printf("    %04x: ", i);
      for (j = 0; j < 8; j++)
        {
          k = i + j;

          if (k < FPU_WORDSIZE)
            {
              printf("%08x ", buffer[k]);
            }
          else
            {
              printf("\n");
              break;
            }
        }
      printf("\n");
   }
}

static int fpu_task(int argc, char *argv[])
{
  FAR struct fpu_threaddata_s *fpu;
  register float sp1;
  register float sp2;
  register float sp3;
  register float sp4;
  register double dp1;
  register double dp2;
  register double dp3;
  register double dp4;

  int id;
  int i;

  /* Which are we? */

  sched_lock();
  fpu = &g_fputhread[g_fpuno];
  id  = (int)(++g_fpuno);
  sched_unlock();

  /* Seed the flowing point values */

  sp1 = (float)id;
  dp1 = (double)id;

  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_FPULOOPS; i++)
    {
      printf("FPU#%d: pass %d\n", id, i+1);
      fflush(stdout);

      /* Set the FPU register save arrays to a known-but-illogical values so 
       * that we can verify that reading of the registers actually occurs.
       */

      memset(fpu->save1, 0xff, FPU_WORDSIZE * sizeof(uint32_t));
      memset(fpu->save2, 0xff, FPU_WORDSIZE * sizeof(uint32_t));

      /* Prevent context switches while we set up some stuff */

      sched_lock();

      /* Do some trivial floating point operations that should cause some
       * changes to floating point registers.  First, some single preceision
       * nonsense.
       */

      sp4 = (float)3.14159 * sp1;    /* Multiple by Pi */
      sp3 = sp4 + (float)1.61803;    /* Add the golden ratio */
      sp2 = sp3 / (float)2.71828;    /* Divide by Euler's constant */
      sp1 = sp2 + (float)1.0;        /* Plus one */

      fpu->sp1 = sp1;                /* Make the compiler believe that somebody cares about the result */
      fpu->sp2 = sp2;
      fpu->sp3 = sp3;
      fpu->sp4 = sp4;

      /* Again using double precision */

      dp4 = (double)3.14159 * dp1;   /* Multiple by Pi */
      dp3 = dp4 + (double)1.61803;   /* Add the golden ratio */
      dp2 = dp3 / (double)2.71828;   /* Divide by Euler's constant */
      dp1 = dp2 + (double)1.0;       /* Plus one */

      fpu->dp1 = dp1;                /* Make the compiler believe that somebody cares about the result */
      fpu->dp2 = dp2;
      fpu->dp3 = dp3;
      fpu->dp4 = dp4;

      /* Sample the floating point registers */

      arch_getfpu(fpu->save1);

      /* Re-read and verify the FPU registers consistently without corruption */

      arch_getfpu(fpu->save2);
      if (!arch_cmpfpu(fpu->save1, fpu->save2))
        {
          printf("ERROR FPU#%d: save1 and save2 do not match\n", id);
          fpu_dump(fpu->save1, "Values after math operations (save1)");
          fpu_dump(fpu->save2, "Values after verify re-read (save2)");
          return EXIT_FAILURE;
        }

      /* Now unlock and sleep for a while -- this should result in some context switches */

      sched_unlock();
      usleep(CONFIG_EXAMPLES_OSTEST_FPUMSDELAY * 1000);

      /* Several context switches should have occurred.  Now verify that the floating
       * point registers are still correctly set.
       */

      arch_getfpu(fpu->save2);
      if (!arch_cmpfpu(fpu->save1, fpu->save2))
        {
          printf("ERROR FPU#%d: save1 and save2 do not match\n", id);
          fpu_dump(fpu->save1, "Values before waiting (save1)");
          fpu_dump(fpu->save2, "Values after waiting (save2)");
          return EXIT_FAILURE;
        }
    }

  printf("FPU#%d: Succeeded\n", id);
  fflush(stdout);
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
  fflush(stdout);
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

  fflush(stdout);
  (void)waitpid(task1, &statloc, 0);
  (void)waitpid(task2, &statloc, 0);

#else
  printf("fpu_test: ERROR: The FPU test is not properly configured\n");
#endif
  printf("fpu_test: Returning\n");
}
