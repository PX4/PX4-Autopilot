/****************************************************************************
 * examples/ostest/barrier.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <pthread.h>

#include "ostest.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define HALF_SECOND 500000L

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_barrier_t barrier;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: barrier_func
 ****************************************************************************/

static void *barrier_func(void *parameter)
{
  int id = (int)parameter;
  int status;

  printf("barrier_func: Thread %d started\n",  id);
#ifndef CONFIG_DISABLE_SIGNALS
  usleep(HALF_SECOND);
#endif

  /* Wait at the barrier until all threads are synchronized. */

  printf("barrier_func: Thread %d calling pthread_barrier_wait()\n",
         id);
  FFLUSH();
  status = pthread_barrier_wait(&barrier);
  if (status == 0)
    {
      printf("barrier_func: Thread %d, back with "
             "status=0 (I am not special)\n",
             id, status);
    }
  else if (status == PTHREAD_BARRIER_SERIAL_THREAD)
    {
      printf("barrier_func: Thread %d, back with "
             "status=PTHREAD_BARRIER_SERIAL_THREAD (I AM SPECIAL)\n",
             id, status);
    }
  else
    {
      printf("barrier_func: ERROR thread %d could not get semaphore value\n",
             id);
    }
  FFLUSH();

#ifndef CONFIG_DISABLE_SIGNALS
  usleep(HALF_SECOND);
#endif
  printf("barrier_func: Thread %d done\n",  id);
  FFLUSH();
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: barrier_test
 ****************************************************************************/

void barrier_test(void)
{
  pthread_t barrier_thread[CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS];
  pthread_addr_t result;
  pthread_attr_t attr;
  pthread_barrierattr_t barrierattr;
  int status;
  int i;

  printf("barrier_test: Initializing barrier\n");

  status = pthread_barrierattr_init(&barrierattr);
  if (status != OK)
    {
      printf("barrier_test: pthread_barrierattr_init failed, status=%d\n",
             status);
    }

  status = pthread_barrier_init(&barrier, &barrierattr,
                                CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS);
  if (status != OK)
    {
      printf("barrier_test: pthread_barrierattr_init failed, status=%d\n",
             status);
    }

  /* Create the barrier */

  status = pthread_barrierattr_init(&barrierattr);

  /* Start CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS thread instances */

  status = pthread_attr_init(&attr);
  if (status != OK)
    {
      printf("barrier_test: pthread_attr_init failed, status=%d\n",
              status);
    }

  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS; i++)
    {
      status = pthread_create(&barrier_thread[i], &attr, barrier_func,
                              (pthread_addr_t)i);
      if (status != 0)
        {
          printf("barrier_test: Error in thread %d create, status=%d\n",
                 i, status);
          printf("barrier_test: Test aborted with waiting threads\n");
          goto abort_test;
        }
      else
        {
          printf("barrier_test: Thread %d created\n", i);
        }
    }
  FFLUSH();

  /* Wait for all thread instances to complete */

  for (i = 0; i < CONFIG_EXAMPLES_OSTEST_NBARRIER_THREADS; i++)
    {
      status = pthread_join(barrier_thread[i], &result);
      if (status != 0)
        {
          printf("barrier_test: Error in thread %d join, status=%d\n",
                 i, status);
        }
      else
        {
          printf("barrier_test: Thread %d completed with result=%p\n",
                 i, result);
        }
    }

  /* Destroy the barrier */

abort_test:
  status = pthread_barrier_destroy(&barrier);
  if (status != OK)
    {
      printf("barrier_test: pthread_barrier_destroy failed, status=%d\n",
             status);
    }

  status = pthread_barrierattr_destroy(&barrierattr);
  if (status != OK)
    {
      printf("barrier_test: pthread_barrierattr_destroy failed, status=%d\n",
             status);
    }
  FFLUSH();
}
