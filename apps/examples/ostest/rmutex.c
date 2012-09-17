/***********************************************************************
 * rmutex.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#define NTHREADS    3
#define NLOOPS      3
#define NRECURSIONS 3

static pthread_mutex_t mut;

static void thread_inner(int id, int level)
{
  int status;
  if (level < NRECURSIONS)
    {
      /* Take the mutex */

      printf("thread_inner[%d, %d]: Locking\n", id, level);
      status = pthread_mutex_lock(&mut);
      if (status != 0)
        {
          printf("thread_inner[%d, %d]: ERROR pthread_mutex_lock failed: %d\n",
                  id, level, status);
        }
      printf("thread_inner[%d, %d]: Locked\n", id, level);

      /* Give the other threads a chance */
      
      pthread_yield();
      thread_inner(id, level+1);
      pthread_yield();

      /* Unlock the mutex */

      printf("thread_inner[%d, %d]: Unlocking\n", id, level);
      status = pthread_mutex_unlock(&mut);
      if (status != 0)
        {
          printf("thread_inner[%d, %d]: ERROR pthread_mutex_unlock failed: %d\n",
                 id, level, status);
        }
      printf("thread_inner[%d, %d]: Unlocked\n", id, level);
      pthread_yield();
    }
}

static void *thread_outer(void *parameter)
{
  int i;
  printf("thread_outer[%d]: Started\n", (int)parameter);
  for (i = 0; i < NLOOPS; i++)
    {
      printf("thread_outer[%d]: Loop %d\n", (int)parameter, i);
      thread_inner((int)parameter, 0);
    }
  printf("thread_outer[%d]: Exitting\n", (int)parameter);
  pthread_exit(NULL);
  return NULL; /* Non-reachable -- needed for some compilers */
}

void recursive_mutex_test(void)
{
  pthread_t thread[NTHREADS];
#ifdef SDCC
  pthread_addr_t result[NTHREADS];
  pthread_attr_t attr;
#endif
  pthread_mutexattr_t mattr;
  int type;
  int status;
  int i;

  /* Initialize the mutex attributes */

  pthread_mutexattr_init(&mattr);
  status = pthread_mutexattr_settype(&mattr, PTHREAD_MUTEX_RECURSIVE);
  if (status != 0)
    {
      printf("recursive_mutex_test: ERROR pthread_mutexattr_settype failed, status=%d\n", status);
    }

  status = pthread_mutexattr_gettype(&mattr, &type);
  if (status != 0)
    {
      printf("recursive_mutex_test: ERROR pthread_mutexattr_gettype failed, status=%d\n", status);
    }
  if (type != PTHREAD_MUTEX_RECURSIVE)
    {
      printf("recursive_mutex_test: ERROR pthread_mutexattr_gettype return type=%d\n", type);
    }
  
  /* Initialize the mutex */

  printf("recursive_mutex_test: Initializing mutex\n");
  pthread_mutex_init(&mut, &mattr);

  /* Start the threads -- all at the same, default priority */

  for (i = 0; i < NTHREADS; i++)
    {
      printf("recursive_mutex_test: Starting thread %d\n", i+1);
#ifdef SDCC
      (void)pthread_attr_init(&attr);
      status = pthread_create(&thread[i], &attr, thread_outer, (pthread_addr_t)i+1);
#else
      status = pthread_create(&thread[i], NULL, thread_outer, (pthread_addr_t)i+1);
#endif
      if (status != 0)
        {
          printf("recursive_mutex_test: ERRROR thread#%d creation: %d\n", i+1, status);
        }
    }

  /* Wait for all; of the threads to complete */

  for (i = 0; i < NTHREADS; i++)
    {
      printf("recursive_mutex_test: Waiting for thread %d\n", i+1);
#ifdef SDCC
      pthread_join(thread[i], &result1);
#else
      pthread_join(thread[i], NULL);
#endif
    }

  printf("recursive_mutex_test: Complete\n");
}
