/***********************************************************************
 * mutex.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <pthread.h>
#include "ostest.h"

#ifndef NULL
# define NULL (void*)0
#endif

#define NLOOPS 32

static pthread_mutex_t mut;
static volatile int my_mutex = 0;
static unsigned long nloops[2] = {0, 0};
static unsigned long nerrors[2] = {0, 0};

static void *thread_func(void *parameter)
{
  int id  = (int)parameter;
  int ndx = id - 1;
  int i;

  for (nloops[ndx] = 0; nloops[ndx] < NLOOPS; nloops[ndx]++)
    {
      int status = pthread_mutex_lock(&mut);
      if (status != 0)
        {
          printf("ERROR thread %d: pthread_mutex_lock failed, status=%d\n",
                  id, status);
        }

      if (my_mutex == 1)
        {
          printf("ERROR thread=%d: "
                 "my_mutex should be zero, instead my_mutex=%d\n",
                  id, my_mutex);
          nerrors[ndx]++;
        }

      my_mutex = 1;	
      for (i = 0; i < 10; i++)
        {
          pthread_yield();
        }
      my_mutex = 0;

      status = pthread_mutex_unlock(&mut);
      if (status != 0)
        {
          printf("ERROR thread %d: pthread_mutex_unlock failed, status=%d\n",
                 id, status);
        }
    }
  pthread_exit(NULL);
  return NULL; /* Non-reachable -- needed for some compilers */
}

void mutex_test(void)
{
  pthread_t thread1, thread2;
#ifdef SDCC
  pthread_addr_t result1, result2;
  pthread_attr_t attr;
#endif
  int status;

  /* Initialize the mutex */

  printf("Initializing mutex\n");
  pthread_mutex_init(&mut, NULL);

  /* Start two thread instances */

  printf("Starting thread 1\n");
#ifdef SDCC
  (void)pthread_attr_init(&attr);
  status = pthread_create(&thread1, &attr, thread_func, (pthread_addr_t)1);
#else
  status = pthread_create(&thread1, NULL, thread_func, (pthread_addr_t)1);
#endif
  if (status != 0)
    {
      printf("Error in thread#1 creation\n");
    }

  printf("Starting thread 2\n");
#ifdef SDCC
  status = pthread_create(&thread2, &attr, thread_func, (pthread_addr_t)2);
#else
  status = pthread_create(&thread2, NULL, thread_func, (pthread_addr_t)2);
#endif
  if (status != 0)
    {
      printf("Error in thread#2 creation\n");
    }

#ifdef SDCC
  pthread_join(thread1, &result1);
  pthread_join(thread2, &result2);
#else
  pthread_join(thread1, NULL);
  pthread_join(thread2, NULL);
#endif

  printf("\t\tThread1\tThread2\n");
  printf("\tLoops\t%ld\t%ld\n", nloops[0], nloops[1]);
  printf("\tErrors\t%ld\t%ld\n", nerrors[0], nerrors[1]);
}
