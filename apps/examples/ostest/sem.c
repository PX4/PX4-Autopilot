/***********************************************************************
 * sem.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <sched.h>
#include "ostest.h"

#ifndef NULL
# define NULL (void*)0
#endif

static sem_t sem;

static void *waiter_func(void *parameter)
{
  int id = (int)parameter;
  int status;
  int value;

  printf("waiter_func: Thread %d Started\n",  id);

  /* Take the semaphore */

  status = sem_getvalue(&sem, &value);
  if (status < 0)
    {
      printf("waiter_func: ERROR thread %d could not get semaphore value\n",  id);
    }
  else
    {
      printf("waiter_func: Thread %d initial semaphore value = %d\n",  id, value);
    }

  printf("waiter_func: Thread %d waiting on semaphore\n",  id);
  status = sem_wait(&sem);
  if (status != 0)
    {
      printf("waiter_func: ERROR thread %d sem_wait failed\n",  id);
    }
  printf("waiter_func: Thread %d awakened\n",  id);

  status = sem_getvalue(&sem, &value);
  if (status < 0)
    {
      printf("waiter_func: ERROR thread %d could not get semaphore value\n",  id);
    }
  else
    {
      printf("waiter_func: Thread %d new semaphore value = %d\n",  id, value);
    }

  printf("waiter_func: Thread %d done\n",  id);
  return NULL;
}

static void *poster_func(void *parameter)
{
  int id = (int)parameter;
  int status;
  int value;

  printf("poster_func: Thread %d started\n",  id);

  /* Take the semaphore */

  do
    {
      status = sem_getvalue(&sem, &value);
      if (status < 0)
        {
          printf("poster_func: ERROR thread %d could not get semaphore value\n",  id);
        }
      else
        {
          printf("poster_func: Thread %d semaphore value = %d\n",  id, value);
        }

      if (value < 0)
        {
          printf("poster_func: Thread %d posting semaphore\n",  id);
          status = sem_post(&sem);
          if (status != 0)
            {
              printf("poster_func: ERROR thread %d sem_wait failed\n",  id);
            }

          pthread_yield();

          status = sem_getvalue(&sem, &value);
          if (status < 0)
            {
              printf("poster_func: ERROR thread %d could not get semaphore value\n",  id);
            }
          else
            {
              printf("poster_func: Thread %d new semaphore value = %d\n",  id, value);
            }
        }
    }
  while (value < 0);

  printf("poster_func: Thread %d done\n",  id);
  return NULL;

}

void sem_test(void)
{
  pthread_t waiter_thread1;
  pthread_t waiter_thread2;
  pthread_t poster_thread;
#ifdef SDCC
  pthread_addr_t result;
#endif
  struct sched_param sparam;
  int prio_min;
  int prio_max;
  int prio_mid;
  pthread_attr_t attr;
  int status;

  printf("sem_test: Initializing semaphore to 0\n");
  sem_init(&sem, 0, 0);

  /* Start two waiter thread instances */

  printf("sem_test: Starting waiter thread 1\n");
  status = pthread_attr_init(&attr);
  if (status != OK)
    {
      printf("sem_test: pthread_attr_init failed, status=%d\n",  status);
    }

  prio_min = sched_get_priority_min(SCHED_FIFO);
  prio_max = sched_get_priority_max(SCHED_FIFO);
  prio_mid = (prio_min + prio_max) / 2;

  sparam.sched_priority = (prio_mid + prio_max) / 2;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("sem_test: pthread_attr_setschedparam failed, status=%d\n",  status);
    }
  else
    {
      printf("sem_test: Set thread 1 priority to %d\n",  sparam.sched_priority);
    }

  status = pthread_create(&waiter_thread1, &attr, waiter_func, (pthread_addr_t)1);
  if (status != 0)
    {
      printf("sem_test: Error in thread 1 creation, status=%d\n",  status);
    }

  printf("sem_test: Starting waiter thread 2\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("sem_test: pthread_attr_init failed, status=%d\n",  status);
    }

  sparam.sched_priority = prio_mid;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("sem_test: pthread_attr_setschedparam failed, status=%d\n",  status);
    }
  else
    {
      printf("sem_test: Set thread 2 priority to %d\n",  sparam.sched_priority);
    }

  status = pthread_create(&waiter_thread2, &attr, waiter_func, (pthread_addr_t)2);
  if (status != 0)
    {
      printf("sem_test: Error in thread 2 creation, status=%d\n",  status);
    }

  printf("sem_test: Starting poster thread 3\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("sem_test: pthread_attr_init failed, status=%d\n",  status);
    }

  sparam.sched_priority = (prio_min + prio_mid) / 2;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("sem_test: pthread_attr_setschedparam failed, status=%d\n",  status);
    }
  else
    {
      printf("sem_test: Set thread 3 priority to %d\n",  sparam.sched_priority);
    }

  status = pthread_create(&poster_thread, &attr, poster_func, (pthread_addr_t)3);
  if (status != 0)
    {
      printf("sem_test: Error in thread 3 creation, status=%d\n",  status);
    }

#ifdef SDCC
  pthread_join(waiter_thread1, &result);
  pthread_join(waiter_thread2, &result);
  pthread_join(poster_thread, &result);
#else
  pthread_join(waiter_thread1, NULL);
  pthread_join(waiter_thread2, NULL);
  pthread_join(poster_thread, NULL);
#endif
}
