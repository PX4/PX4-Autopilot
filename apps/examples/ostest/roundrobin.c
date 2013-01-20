/********************************************************************************
 * examples/ostest/roundrobin.c
 *
 *   Copyright (C) 2007, 2008, 2012 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include "ostest.h"

#if CONFIG_RR_INTERVAL > 0

/********************************************************************************
 * Definitions
 ********************************************************************************/

/* This numbers should be tuned for different processor speeds via .config file.
 * With default values the test takes about 30s on Cortex-M3 @ 24MHz. With 32767
 * range and 10 runs it takes ~320s. */

#ifndef CONFIG_EXAMPLES_OSTEST_RR_RANGE
#  define CONFIG_EXAMPLES_OSTEST_RR_RANGE 10000
#  warning "CONFIG_EXAMPLES_OSTEST_RR_RANGE undefined, using default value = 10000"
#elif (CONFIG_EXAMPLES_OSTEST_RR_RANGE < 1) || (CONFIG_EXAMPLES_OSTEST_RR_RANGE > 32767)
#  define CONFIG_EXAMPLES_OSTEST_RR_RANGE 10000
#  warning "Invalid value of CONFIG_EXAMPLES_OSTEST_RR_RANGE, using default value = 10000"
#endif

#ifndef CONFIG_EXAMPLES_OSTEST_RR_RUNS
#  define CONFIG_EXAMPLES_OSTEST_RR_RUNS 10
#  warning "CONFIG_EXAMPLES_OSTEST_RR_RUNS undefined, using default value = 10"
#elif (CONFIG_EXAMPLES_OSTEST_RR_RUNS < 1) || (CONFIG_EXAMPLES_OSTEST_RR_RUNS > 32767)
#  define CONFIG_EXAMPLES_OSTEST_RR_RUNS 10
#  warning "Invalid value of CONFIG_EXAMPLES_OSTEST_RR_RUNS, using default value = 10"
#endif

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Name: get_primes
 *
 * Description
 *   This function searches for prime numbers in the most primitive way possible.
 ********************************************************************************/

static void get_primes(int *count, int *last)
{
  int number;
  int local_count = 0;
  *last = 0;    // to make compiler happy

  for (number = 1; number < CONFIG_EXAMPLES_OSTEST_RR_RANGE; number++)
  {
    int div;
    bool is_prime = true;

    for (div = 2; div <= number / 2; div++)
      if (number % div == 0)
      {
        is_prime = false;
        break;
      }

    if (is_prime)
    {
      local_count++;
      *last = number;
#if 0 /* We don't really care what the numbers are */
      printf(" Prime %d: %d\n", local_count, number);
#endif
    }
  }

  *count = local_count;
}

/********************************************************************************
 * Name: get_primes_thread
 ********************************************************************************/

static void *get_primes_thread(void *parameter)
{
  int id = (int)parameter;
  int i, count, last;

  printf("get_primes_thread id=%d started, looking for primes < %d, doing %d run(s)\n",
         id, CONFIG_EXAMPLES_OSTEST_RR_RANGE, CONFIG_EXAMPLES_OSTEST_RR_RUNS);

  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_RR_RUNS; i++)
    {
      get_primes(&count, &last);
    }

  printf("get_primes_thread id=%d finished, found %d primes, last one was %d\n",
         id, count, last);

  pthread_exit(NULL);
  return NULL; /* To keep some compilers happy */
}

/********************************************************************************
 * Public Functions
 ********************************************************************************/

/********************************************************************************
 * Name: rr_test
 ********************************************************************************/

void rr_test(void)
{
  pthread_t get_primes1_thread;
  pthread_t get_primes2_thread;
  struct sched_param sparam;
  pthread_attr_t attr;
  pthread_addr_t result;
  int status;

  status = pthread_attr_init(&attr);
  if (status != OK)
    {
      printf("rr_test: pthread_attr_init failed, status=%d\n",  status);
    }

  sparam.sched_priority = sched_get_priority_min(SCHED_FIFO);
  status = pthread_attr_setschedparam(&attr, &sparam);
  if (status != OK)
    {
      printf("rr_test: pthread_attr_setschedparam failed, status=%d\n",  status);
    }
  else
    {
      printf("rr_test: Set thread priority to %d\n",  sparam.sched_priority);
    }

  status = pthread_attr_setschedpolicy(&attr, SCHED_RR);
  if (status != OK)
    {
      printf("rr_test: pthread_attr_setschedpolicy failed, status=%d\n",  status);
    }
  else
    {
      printf("rr_test: Set thread policy to SCHED_RR\n");
    }

  printf("rr_test: Starting first get_primes_thread\n");

  status = pthread_create(&get_primes1_thread, &attr, get_primes_thread, (void*)1);
  if (status != 0)
    {
      printf("rr_test: Error in thread 1 creation, status=%d\n",  status);
    }

  printf("rr_test: Starting second get_primes_thread\n");

  status = pthread_create(&get_primes2_thread, &attr, get_primes_thread, (void*)2);
  if (status != 0)
    {
      printf("rr_test: Error in thread 2 creation, status=%d\n",  status);
    }

  printf("rr_test: Waiting for threads to complete -- this should take awhile\n");
  printf("rr_test: If RR scheduling is working, they should start and complete at\n");
  printf("rr_test: about the same time\n");

  pthread_join(get_primes2_thread, &result);
  pthread_join(get_primes1_thread, &result);
  printf("rr_test: Done\n");
}

#endif /* CONFIG_RR_INTERVAL */
