/****************************************************************************
 * apps/examples/ostest/ostest_main.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <sched.h>
#include <nuttx/init.h>

#include "ostest.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define PRIORITY         100
#define NARGS              4
#define HALF_SECOND_USEC 500000L

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char arg1[] = "Arg1";
static const char arg2[] = "Arg2";
static const char arg3[] = "Arg3";
static const char arg4[] = "Arg4";

#if CONFIG_NFILE_DESCRIPTORS > 0
static const char write_data1[] = "stdio_test: write fd=1\n";
static const char write_data2[] = "stdio_test: write fd=2\n";
#endif

#ifdef SDCC
/* I am not yet certain why SDCC does not like the following
 * initializer.  It involves some issues with 2- vs 3-byte
 * pointer types.
 */

static const char *g_argv[NARGS+1];
#else
static const char *g_argv[NARGS+1] = { arg1, arg2, arg3, arg4, NULL };
#endif

#ifndef CONFIG_DISABLE_SIGNALS
static struct mallinfo g_mmbefore;
static struct mallinfo g_mmprevious;
static struct mallinfo g_mmafter;
#endif

#ifndef CONFIG_DISABLE_ENVIRON
const char g_var1_name[]    = "Variable1";
const char g_var1_value[]   = "GoodValue1";
const char g_var2_name[]    = "Variable2";
const char g_var2_value[]   = "GoodValue2";
const char g_var3_name[]    = "Variable3";
const char g_var3_value[]   = "GoodValue3";

const char g_bad_value1[]   = "BadValue1";
const char g_bad_value2[]   = "BadValue2";

const char g_putenv_value[] = "Variable1=BadValue3";

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_memory_usage
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
static void show_memory_usage(struct mallinfo *mmbefore,
                              struct mallinfo *mmafter)
{
  printf("VARIABLE  BEFORE   AFTER\n");
  printf("======== ======== ========\n");
  printf("arena    %8x %8x\n", mmbefore->arena,    mmafter->arena);
  printf("ordblks  %8d %8d\n", mmbefore->ordblks,  mmafter->ordblks);
  printf("mxordblk %8x %8x\n", mmbefore->mxordblk, mmafter->mxordblk);
  printf("uordblks %8x %8x\n", mmbefore->uordblks, mmafter->uordblks);
  printf("fordblks %8x %8x\n", mmbefore->fordblks, mmafter->fordblks);
}
#else
# define show_memory_usage(mm1, mm2)
#endif

/****************************************************************************
 * Name: check_test_memory_usage
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
static void check_test_memory_usage(void)
{
  /* Wait a little bit to let any threads terminate */

  usleep(HALF_SECOND_USEC);

  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_mmafter = mallinfo();
#else
  (void)mallinfo(&g_mmafter);
#endif

  /* Show the change from the previous time */

  printf("\nEnd of test memory usage:\n");
  show_memory_usage(&g_mmprevious, &g_mmafter);

  /* Set up for the next test */

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_mmprevious = g_mmafter;
#else
  memcpy(&g_mmprevious, &g_mmafter, sizeof(struct mallinfo));
#endif

  /* If so enabled, show the use of priority inheritance resources */

  dump_nfreeholders("user_main:");
}
#else
# define check_test_memory_usage()
#endif

/****************************************************************************
 * Name: show_variable
 ****************************************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
static void show_variable(const char *var_name, const char *exptd_value, bool var_valid)
{
  char *actual_value = getenv(var_name);
  if (actual_value)
    {
      if (var_valid)
        {
          if (strcmp(actual_value, exptd_value) == 0)
            {
              printf("show_variable: Variable=%s has value=%s\n", var_name, exptd_value);
            }
          else
            {
              printf("show_variable: ERROR Variable=%s has the wrong value\n", var_name);
              printf("show_variable:       found=%s expected=%s\n", actual_value, exptd_value);
            }
        }
      else
        {
          printf("show_variable: ERROR Variable=%s has a value when it should not\n", var_name);
          printf("show_variable:       value=%s\n", actual_value);
        }
    }
  else if (var_valid)
    {
      printf("show_variable: ERROR Variable=%s has no value\n", var_name);
      printf("show_variable:       Should have had value=%s\n", exptd_value);
    }
  else
    {
      printf("show_variable: Variable=%s has no value\n", var_name);
    }
}

static void show_environment(bool var1_valid, bool var2_valid, bool var3_valid)
{
  show_variable(g_var1_name, g_var1_value, var1_valid);
  show_variable(g_var2_name, g_var2_value, var2_valid);
  show_variable(g_var3_name, g_var3_value, var3_valid);
}
#else
# define show_environment()
#endif

/****************************************************************************
 * Name: user_main
 ****************************************************************************/

static int user_main(int argc, char *argv[])
{
  int i;

  /* Sample the memory usage now */

#ifndef CONFIG_DISABLE_SIGNALS
  usleep(HALF_SECOND_USEC);

#ifdef CONFIG_CAN_PASS_STRUCTS
  g_mmbefore = mallinfo();
  g_mmprevious = g_mmbefore;
#else
  (void)mallinfo(&g_mmbefore);
  memcpy(&g_mmprevious, &g_mmbefore, sizeof(struct mallinfo));
#endif
#endif

  printf("\nuser_main: Begin argument test\n");
  printf("user_main: Started with argc=%d\n", argc);

  /* Verify passed arguments */

  if (argc != NARGS + 1)
    {
      printf("user_main: Error expected argc=%d got argc=%d\n",
             NARGS+1, argc);
    }

  for (i = 0; i <= NARGS; i++)
    {
      printf("user_main: argv[%d]=\"%s\"\n", i, argv[i]);
    }

  for (i = 1; i <= NARGS; i++)
    {
      if (strcmp(argv[i], g_argv[i-1]) != 0)
        {
          printf("user_main: ERROR argv[%d]:  Expected \"%s\" found \"%s\"\n",
                 i, g_argv[i-1], argv[i]);
        }
    }
  check_test_memory_usage();

  /* Check environment variables */
#ifndef CONFIG_DISABLE_ENVIRON
  show_environment(true, true, true);

  unsetenv(g_var1_name);
  show_environment(false, true, true);
  check_test_memory_usage();

  clearenv();
  show_environment(false, false, false);
  check_test_memory_usage();
#endif

  /* Top of test loop */
  
#if CONFIG_EXAMPLES_OSTEST_LOOPS > 1
  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_LOOPS; i++)
#elif CONFIG_EXAMPLES_OSTEST_LOOPS == 0
  for (;;)
#endif
    {
#if CONFIG_NFILE_DESCRIPTORS > 0
      /* Checkout /dev/null */

      printf("\nuser_main: /dev/null test\n");
      dev_null();
      check_test_memory_usage();
#endif

#ifdef  CONFIG_ARCH_FPU
  /* Check that the FPU is properly supported during context switching */

      printf("\nuser_main: FPU test\n");
      fpu_test();
      check_test_memory_usage();
#endif

#ifdef CONFIG_SCHED_WAITPID
      /* Check waitpid() and friends */

      printf("\nuser_main: waitpid test\n");
      waitpid_test();
      check_test_memory_usage();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
      /* Verify pthreads and pthread mutex */

      printf("\nuser_main: mutex test\n");
      mutex_test();
      check_test_memory_usage();
#endif

#if !defined(CONFIG_DISABLE_PTHREAD) && defined(CONFIG_MUTEX_TYPES)
      /* Verify recursive mutexes */

      printf("\nuser_main: recursive mutex test\n");
      recursive_mutex_test();
      check_test_memory_usage();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
      /* Verify pthread cancellation */

      printf("\nuser_main: cancel test\n");
      cancel_test();
      check_test_memory_usage();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
      /* Verify pthreads and semaphores */

      printf("\nuser_main: semaphore test\n");
      sem_test();
      check_test_memory_usage();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
    /* Verify pthreads and condition variables */

      printf("\nuser_main: condition variable test\n");
#ifdef CONFIG_PRIORITY_INHERITANCE
      printf("\n           Skipping, Test logic incompatible with priority inheritance\n");
#else
      cond_test();
      check_test_memory_usage();
#endif
#endif

#if !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD) && !defined(CONFIG_DISABLE_CLOCK)
      /* Verify pthreads and condition variable timed waits */

      printf("\nuser_main: timed wait test\n");
      timedwait_test();
      check_test_memory_usage();
#endif

#if !defined(CONFIG_DISABLE_MQUEUE) && !defined(CONFIG_DISABLE_PTHREAD)
      /* Verify pthreads and message queues */

      printf("\nuser_main: message queue test\n");
      mqueue_test();
      check_test_memory_usage();
#endif

#if !defined(CONFIG_DISABLE_MQUEUE) && !defined(CONFIG_DISABLE_PTHREAD) && !defined(CONFIG_DISABLE_CLOCK)
      /* Verify pthreads and message queues */

      printf("\nuser_main: timed message queue test\n");
      timedmqueue_test();
      check_test_memory_usage();
#endif

#ifndef CONFIG_DISABLE_SIGNALS
      /* Verify signal handlers */

      printf("\nuser_main: signal handler test\n");
      sighand_test();
      check_test_memory_usage();
#endif

#if !defined(CONFIG_DISABLE_POSIX_TIMERS) && !defined(CONFIG_DISABLE_SIGNALS)
      /* Verify posix timers */

      printf("\nuser_main: POSIX timer test\n");
      timer_test();
      check_test_memory_usage();
#endif

#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_RR_INTERVAL > 0
      /* Verify round robin scheduling */

      printf("\nuser_main: round-robin scheduler test\n");
      rr_test();
      check_test_memory_usage();
#endif

#ifndef CONFIG_DISABLE_PTHREAD
      /* Verify pthread barriers */

      printf("\nuser_main: barrier test\n");
      barrier_test();
      check_test_memory_usage();
#endif

#if defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
      /* Verify priority inheritance */

      printf("\nuser_main: priority inheritance test\n");
      priority_inheritance();
      check_test_memory_usage();
#endif /* CONFIG_PRIORITY_INHERITANCE && !CONFIG_DISABLE_SIGNALS && !CONFIG_DISABLE_PTHREAD */

#ifdef CONFIG_ARCH_HAVE_VFORK
      printf("\nuser_main: vfork() test\n");
      vfork_test();
#endif

      /* Compare memory usage at time ostest_main started until
       * user_main exits.  These should not be identical, but should
       * be similar enough that we can detect any serious OS memory
       * leaks.
       */

#ifndef CONFIG_DISABLE_SIGNALS
      usleep(HALF_SECOND_USEC);

#ifdef CONFIG_CAN_PASS_STRUCTS
      g_mmafter = mallinfo();
#else
      (void)mallinfo(&g_mmafter);
#endif

      printf("\nFinal memory usage:\n");
      show_memory_usage(&g_mmbefore, &g_mmafter);
#endif
    }

  printf("user_main: Exitting\n");
  return 0;
}

/****************************************************************************
 * Name: stdio_test
 ****************************************************************************/

static void stdio_test(void)
{
  /* Verify that we can communicate */

#if CONFIG_NFILE_DESCRIPTORS > 0
  write(1, write_data1, sizeof(write_data1)-1);
#endif
  printf("stdio_test: Standard I/O Check: printf\n");

#if CONFIG_NFILE_DESCRIPTORS > 1
  write(2, write_data2, sizeof(write_data2)-1);
#endif
#if CONFIG_NFILE_STREAMS > 0
  fprintf(stderr, "stdio_test: Standard I/O Check: fprintf to stderr\n");
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * ostest_main
 ****************************************************************************/

int ostest_main(int argc, char *argv[])
{
  int result;

  /* Verify that stdio works first */

  stdio_test();

#ifdef SDCC
  /* I am not yet certain why SDCC does not like the following initilizers.
   * It involves some issues with 2- vs 3-byte pointer types.
   */

  g_argv[0] = arg1;
  g_argv[1] = arg2;
  g_argv[2] = arg3;
  g_argv[3] = arg4;
  g_argv[4] = NULL;
#endif

  /* Set up some environment variables */

#ifndef CONFIG_DISABLE_ENVIRON
  printf("ostest_main: putenv(%s)\n", g_putenv_value);
  putenv(g_putenv_value);                   /* Varaible1=BadValue3 */
  printf("ostest_main: setenv(%s, %s, TRUE)\n", g_var1_name, g_var1_value);
  setenv(g_var1_name, g_var1_value, TRUE);  /* Variable1=GoodValue1 */

  printf("ostest_main: setenv(%s, %s, FALSE)\n", g_var2_name, g_bad_value1);
  setenv(g_var2_name, g_bad_value1, FALSE); /* Variable2=BadValue1 */
  printf("ostest_main: setenv(%s, %s, TRUE)\n", g_var2_name, g_var2_value);
  setenv(g_var2_name, g_var2_value, TRUE);  /* Variable2=GoodValue2 */

  printf("ostest_main: setenv(%s, %s, FALSE)\n", g_var3_name, g_var3_name);
  setenv(g_var3_name, g_var3_value, FALSE); /* Variable3=GoodValue3 */
  printf("ostest_main: setenv(%s, %s, FALSE)\n", g_var3_name, g_var3_name);
  setenv(g_var3_name, g_bad_value2, FALSE); /* Variable3=GoodValue3 */
  show_environment(true, true, true);
#endif

  /* Verify that we can spawn a new task */

#ifndef CONFIG_CUSTOM_STACK
  result = task_create("ostest", PRIORITY, STACKSIZE, user_main, g_argv);
#else
  result = task_create("ostest", PRIORITY, user_main, g_argv);
#endif
  if (result == ERROR)
    {
      printf("ostest_main: ERROR Failed to start user_main\n");
    }
  else
    {
      printf("ostest_main: Started user_main at PID=%d\n", result);
    }

  printf("ostest_main: Exitting\n");
  return 0;
}
