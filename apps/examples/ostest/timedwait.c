/***********************************************************************
 * examples/ostest/timedwait.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>

#include "ostest.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Data
 **************************************************************************/

static pthread_mutex_t mutex;
static pthread_cond_t  cond;

/**************************************************************************
 * Private Functions
 **************************************************************************/

static void *thread_waiter(void *parameter)
{
  struct timespec ts;
  int status;

  /* Take the mutex */

  printf("thread_waiter: Taking mutex\n");
  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
      printf("thread_waiter: ERROR pthread_mutex_lock failed, status=%d\n", status);
    }

  printf("thread_waiter: Starting 5 second wait for condition\n");

  status = clock_gettime(CLOCK_REALTIME, &ts);
  if (status != 0)
    {
      printf("thread_waiter: ERROR clock_gettime failed\n");
    }
  ts.tv_sec += 5;

  /* The wait -- no-one is ever going to awaken us */

  status = pthread_cond_timedwait(&cond, &mutex, &ts);
  if (status != 0)
    {
      if (status == ETIMEDOUT)
        {
          printf("thread_waiter: pthread_cond_timedwait timed out\n");
        }
      else
        {
          printf("thread_waiter: ERROR pthread_cond_timedwait failed, status=%d\n", status);
        }
    }
  else
    {
      printf("thread_waiter: ERROR pthread_cond_timedwait returned without timeout, status=%d\n", status);
    }

  /* Release the mutex */

  printf("thread_waiter: Releasing mutex\n");
  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("thread_waiter: ERROR pthread_mutex_unlock failed, status=%d\n", status);
    }

  printf("thread_waiter: Exit with status 0x12345678\n");
  pthread_exit((pthread_addr_t)0x12345678);
  return NULL;
}

/**************************************************************************
 * Public Definitions
 **************************************************************************/

void timedwait_test(void)
{
  pthread_t waiter;
  pthread_attr_t attr;
  struct sched_param sparam;
  void *result;
  int prio_max;
  int status;

  /* Initialize the mutex */

  printf("thread_waiter: Initializing mutex\n");
  status = pthread_mutex_init(&mutex, NULL);
  if (status != 0)
    {
      printf("timedwait_test: ERROR pthread_mutex_init failed, status=%d\n", status);
    }

  /* Initialize the condition variable */

  printf("timedwait_test: Initializing cond\n");
  status = pthread_cond_init(&cond, NULL);
  if (status != 0)
    {
      printf("timedwait_test: ERROR pthread_condinit failed, status=%d\n", status);
    }

  /* Start the waiter thread at higher priority */

  printf("timedwait_test: Starting waiter\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("timedwait_test: pthread_attr_init failed, status=%d\n", status);
    }

  prio_max = sched_get_priority_max(SCHED_FIFO);
  status = sched_getparam (getpid(), &sparam);
  if (status != 0)
    {
      printf("timedwait_test: sched_getparam failed\n");
      sparam.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }

  sparam.sched_priority = (prio_max + sparam.sched_priority) / 2;
  status = pthread_attr_setschedparam(&attr,&sparam);
  if (status != OK)
    {
      printf("timedwait_test: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("timedwait_test: Set thread 2 priority to %d\n", sparam.sched_priority);
    }

  status = pthread_create(&waiter, &attr, thread_waiter, NULL);
  if (status != 0)
    {
      printf("timedwait_test: pthread_create failed, status=%d\n", status);
    }

  printf("timedwait_test: Joining\n");
  FFLUSH();
  status = pthread_join(waiter, &result);
  if (status != 0)
    {
      printf("timedwait_test: ERROR pthread_join failed, status=%d\n", status);
    }
  else
    {
      printf("timedwait_test: waiter exited with result=%p\n", result);
    }
}
