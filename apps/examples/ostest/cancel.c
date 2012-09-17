/***********************************************************************
 * examples/ostest/cancel.c
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
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include "ostest.h"

static pthread_mutex_t mutex;
static pthread_cond_t  cond;

static void *thread_waiter(void *parameter)
{
  int status;

  /* Take the mutex */

  printf("thread_waiter: Taking mutex\n");
  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
       printf("thread_waiter: ERROR pthread_mutex_lock failed, status=%d\n", status);
    }

  printf("thread_waiter: Starting wait for condition\n");

  /* Are we a non-cancelable thread?   Yes, set the non-cancelable state */

  if (!parameter)
    {
      printf("thread_waiter: Setting non-cancelable\n");
      status = pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
      if (status != 0)
        {
           printf("thread_waiter: ERROR pthread_setcancelstate failed, status=%d\n", status);
        }
    }

  /* The wait -- we will never awaken from this. */

  status = pthread_cond_wait(&cond, &mutex);
  if (status != 0)
    {
      printf("thread_waiter: ERROR pthread_cond_wait failed, status=%d\n", status);
    }

  /* Release the mutex */

  printf("thread_waiter: Releasing mutex\n");
  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("thread_waiter: ERROR pthread_mutex_unlock failed, status=%d\n", status);
    }

  /* Set the cancelable state */

  printf("thread_waiter: Setting cancelable\n");
  status = pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  if (status != 0)
    {
      printf("thread_waiter: ERROR pthread_setcancelstate failed, status=%d\n", status);
    }

  printf("thread_waiter: Exit with status 0x12345678\n");
  pthread_exit((pthread_addr_t)0x12345678);
  return NULL;
}

static void start_thread(pthread_t *waiter, int cancelable)
{
  pthread_attr_t attr;
  int status;

  /* Initialize the mutex */

  printf("start_thread: Initializing mutex\n");
  status = pthread_mutex_init(&mutex, NULL);
  if (status != 0)
    {
      printf("start_thread: ERROR pthread_mutex_init failed, status=%d\n", status);
    }

  /* Initialize the condition variable */

  printf("start_thread: Initializing cond\n");
  status = pthread_cond_init(&cond, NULL);
  if (status != 0)
    {
      printf("start_thread: ERROR pthread_cond_init failed, status=%d\n", status);
    }

  /* Set up attributes */

  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("start_thread: pthread_attr_init failed, status=%d\n", status);
    }

  status = pthread_attr_setstacksize(&attr, STACKSIZE);
  if (status != 0)
    {
      printf("start_thread: pthread_attr_setstacksize failed, status=%d\n", status);
    }

  /* Start the waiter thread  */

  printf("start_thread: Starting thread\n");
  status = pthread_create(waiter, &attr, thread_waiter, (pthread_addr_t)cancelable);
  if (status != 0)
    {
      printf("start_thread: ERROR pthread_create failed, status=%d\n", status);
    }

  /* Make sure that the waiter thread gets a chance to run */

  printf("start_thread: Yielding\n");
  pthread_yield();
}

static void restart_thread(pthread_t *waiter, int cancelable)
{
  int status;

  /* Destroy the condition variable */

  printf("restart_thread: Destroying cond\n");
  status = pthread_cond_destroy(&cond);
  if (status != 0)
    {
      printf("restart_thread: ERROR pthread_cond_destroy failed, status=%d\n", status);
    }

  /* Destroy the mutex */

  printf("restart_thread: Destroying mutex\n");
  status = pthread_cond_destroy(&cond);
  if (status != 0)
    {
      printf("restart_thread: ERROR pthread_mutex_destroy failed, status=%d\n", status);
    }

  /* Then restart the thread */

  printf("restart_thread: Re-starting thread\n");
  start_thread(waiter, cancelable);
}

void cancel_test(void)
{
  pthread_t waiter;
  void *result;
  int status;

  /* Test 1: Normal Cancel *********************************************/
  /* Start the waiter thread  */

  printf("cancel_test: Test 1: Normal Cancelation\n");
  printf("cancel_test: Starting thread\n");
  start_thread(&waiter, 1);

  /* Then cancel it.  It should be in the pthread_cond_wait now */

  printf("cancel_test: Canceling thread\n");
  status = pthread_cancel(waiter);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_cancel failed, status=%d\n", status);
    }

  /* Then join to the thread to pick up the result (if we don't do
   * we will have a memory leak!)
   */

  printf("cancel_test: Joining\n");
  status = pthread_join(waiter, &result);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_join failed, status=%d\n", status);
    }
  else
    {
      printf("cancel_test: waiter exited with result=%p\n", result);
      if (result != PTHREAD_CANCELED)
        {
          printf("cancel_test: ERROR expected result=%p\n", PTHREAD_CANCELED);
        }
      else
        {
          printf("cancel_test: PASS thread terminated with PTHREAD_CANCELED\n");
        }
    }

  /* Test 2: Cancel Detached Thread ************************************/

  printf("cancel_test: Test 2: Cancelation of detached thread\n");
  printf("cancel_test: Re-starting thread\n");
  restart_thread(&waiter, 1);

  /* Detach the thread */

  status = pthread_detach(waiter);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_detach, status=%d\n", status);
    }

  /* Then cancel it.  It should be in the pthread_cond_wait now */

  printf("cancel_test: Canceling thread\n");
  status = pthread_cancel(waiter);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_cancel failed, status=%d\n", status);
    }

  /* Join should now fail */

  printf("cancel_test: Joining\n");
  status = pthread_join(waiter, &result);
  if (status == 0)
    {
      printf("cancel_test: ERROR pthread_join succeeded\n");
    }
  else if (status != ESRCH)
    {
      printf("cancel_test: ERROR pthread_join failed but with wrong status=%d\n", status);
    }
  else
    {
      printf("cancel_test: PASS pthread_join failed with status=ESRCH\n");
    }

  /* Test 3: Non-cancelable threads ************************************/

  printf("cancel_test: Test 3: Non-cancelable threads\n");
  printf("cancel_test: Re-starting thread (non-cancelable)\n");
  restart_thread(&waiter, 0);

  /* Then cancel it.  It should be in the pthread_cond_wait now.  The
   * behavior here is non-standard:  when the thread is at a cancelation
   * point, it should be cancelable, even when cancelation is disable.
   *
   * The cancelation should succeed, because the cancelation is pending.
   */

  printf("cancel_test: Canceling thread\n");
  status = pthread_cancel(waiter);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_cancel failed, status=%d\n", status);
    }

  /* Signal the thread.  It should wake up and restore the cancelable state.
   * When the cancelable state is re-enabled, the thread should be canceled.
   */

  status = pthread_mutex_lock(&mutex);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_mutex_lock failed, status=%d\n", status);
    }

  status = pthread_cond_signal(&cond);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_cond_signal failed, status=%d\n", status);
    }

  status = pthread_mutex_unlock(&mutex);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_mutex_unlock failed, status=%d\n", status);
    }

  /* Then join to the thread to pick up the result (if we don't do
   * we will have a memory leak!)
   */

  printf("cancel_test: Joining\n");
  status = pthread_join(waiter, &result);
  if (status != 0)
    {
      printf("cancel_test: ERROR pthread_join failed, status=%d\n", status);
    }
  else
    {
      printf("cancel_test: waiter exited with result=%p\n", result);
      if (result != PTHREAD_CANCELED)
        {
          printf("cancel_test: ERROR expected result=%p\n", PTHREAD_CANCELED);
        }
      else
        {
          printf("cancel_test: PASS thread terminated with PTHREAD_CANCELED\n");
        }
    }

}
