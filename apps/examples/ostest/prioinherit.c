/****************************************************************************
 * examples/ostest/prioinherit.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <pthread.h>
#include <errno.h>

#ifdef CONFIG_ARCH_SIM
#  include <nuttx/arch.h>
#endif

#include "ostest.h"

#if defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef CONFIG_SEM_PREALLOCHOLDERS
#  define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

/* If resources were configured for lots of holders, then run 3 low priority
 * threads.  Otherwise, just one.
 */

#if CONFIG_SEM_PREALLOCHOLDERS > 3
#  define NLOWPRI_THREADS 3
#else
#  define NLOWPRI_THREADS 1
#endif

#ifndef CONFIG_SEM_NNESTPRIO
#  define CONFIG_SEM_NNESTPRIO 0
#endif

/* Where resources configured for lots of waiters?  If so then run 3 high
 * priority threads.  Otherwise, just one.
 */

#if CONFIG_SEM_NNESTPRIO > 3
#  define NHIGHPRI_THREADS 3
#else
#  define NHIGHPRI_THREADS 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

enum thstate_e
{
  NOTSTARTED = 0,
  RUNNING,
  WAITING,
  DONE
};

static sem_t g_sem;
static volatile enum thstate_e g_middlestate;
static volatile enum thstate_e g_highstate[NHIGHPRI_THREADS];
static volatile enum thstate_e g_lowstate[NLOWPRI_THREADS];
static int g_highpri;
static int g_medpri;
static int g_lowpri;

/****************************************************************************
 * Name: nhighpri_waiting
 ****************************************************************************/

static int nhighpri_waiting(void)
{
   int n = 0;
   int i;
 
   for (i = 0; i < NHIGHPRI_THREADS; i++)
     {
       if (g_highstate[i] == WAITING)
         {
           n++;
         }
     }
   return n;
}

/****************************************************************************
 * Name: nhighpri_running
 ****************************************************************************/

static int nhighpri_running(void)
{
   int n = 0;
   int i;
 
   for (i = 0; i < NHIGHPRI_THREADS; i++)
     {
       if (g_highstate[i] != DONE)
         {
           n++;
         }
     }
   return n;
}

/****************************************************************************
 * Name: highpri_thread
 ****************************************************************************/

static void *highpri_thread(void *parameter)
{
  int threadno = (int)parameter;
  int ret;

  g_highstate[threadno-1] = RUNNING;

  printf("highpri_thread-%d: Started\n", threadno);
  FFLUSH();
  sleep(1);

  printf("highpri_thread-%d: Calling sem_wait()\n", threadno);
  g_highstate[threadno-1] = WAITING;
  ret                     = sem_wait(&g_sem);
  g_highstate[threadno-1] = DONE;

  if (ret != 0)
    {
      printf("highpri_thread-%d: sem_take failed: %d\n", threadno, ret);
    }
  else if (g_middlestate == RUNNING)
    {
      printf("highpri_thread-%d: SUCCESS midpri_thread is still running!\n", threadno);
    }
  else
    {
      printf("highpri_thread-%d: ERROR --  midpri_thread has already exited!\n", threadno);
    }

  sem_post(&g_sem);
  printf("highpri_thread-%d: Okay... I'm done!\n", threadno);
  FFLUSH();
  return NULL;
}

/****************************************************************************
 * Name: hog_cpu
 ****************************************************************************/

static inline void hog_cpu(void)
{
#ifdef CONFIG_ARCH_SIM
  /* The simulator doesn't have any mechanism to do asynchronous pre-emption
   * (basically because it doesn't have any interupts/asynchronous events).
   * The simulator does "fake" a timer interrupt in up_idle() -- the idle
   * thread that only executes when nothing else is running.  In the simulator,
   * we cannot suspend the middle priority task, or we wouldn't have the
   * test that we want.  So, we have no option but to pump the fake clock
   * here by calling up_idle().  Sigh!
   */

  up_idle();
#else
  /* On real platforms with a real timer interrupt, we really can hog the
   * CPU.  When the sleep() goes off in priority_inheritance(), it will
   * wake up and start the high priority thread.
   */

  volatile int i;
  for (i = 0; i < INT_MAX; i++);
#endif
}

/****************************************************************************
 * Name: medpri_thread
 ****************************************************************************/

static void *medpri_thread(void *parameter)
{
  printf("medpri_thread: Started ... I won't let go of the CPU!\n");
  g_middlestate = RUNNING;
  FFLUSH();

  /* The following loop will completely block lowpri_thread from running.
   * UNLESS priority inheritance is working.  In that case, its priority
   * will be boosted.
   */

  while (nhighpri_running() > 0)
    {
      hog_cpu();
    }

  printf("medpri_thread: Okay... I'm done!\n");
  FFLUSH();
  g_middlestate = DONE;
  return NULL;
}

/****************************************************************************
 * Name: lowpri_thread
 ****************************************************************************/

static void *lowpri_thread(void *parameter)
{
  void *retval = (void*)-1;
  struct sched_param sparam;
  int threadno = (int)parameter;
  int expected;
  int count;
  int policy;
  int ret;
  int nwaiting;
  int i;

  g_lowstate[threadno-1] = RUNNING;
  printf("lowpri_thread-%d: Started\n", threadno);

  ret = pthread_getschedparam(pthread_self(), &policy, &sparam);
  if (ret != 0)
    {
      printf("lowpri_thread-%d: ERROR pthread_getschedparam failed: %d\n", threadno, ret);
    }
  else
    {
      printf("lowpri_thread-%d: initial priority: %d\n", threadno, sparam.sched_priority);
      if (sparam.sched_priority != g_lowpri)
        {
          printf("               ERROR should have been %d\n", g_lowpri);
        } 
    }

  g_lowstate[threadno-1] = WAITING;
  ret = sem_wait(&g_sem);
  if (ret != 0)
    {
      printf("lowpri_thread-%d: sem_take failed: %d\n", threadno, ret);
    }
  else
    {
      /* Hang on to the thread until the middle priority thread runs */

      while (g_middlestate == NOTSTARTED && nhighpri_waiting() < NHIGHPRI_THREADS)
        {
          printf("lowpri_thread-%d: Waiting for the midle pri task to run\n", threadno);
          printf("    g_middlestate:  %d\n", (int)g_middlestate);
          for (i = 0; i < NHIGHPRI_THREADS; i++)
            {
              printf("    g_highstate[%d]: %d\n", i, (int)g_highstate[i]);
            }
          printf("    I still have a count on the semaphore\n");
          sem_enumholders(&g_sem);
          FFLUSH();
          sleep(1);
        }

      /* Account for all of the semaphore counts.  At any given time if there are 'n'
       * running hight prioity tasks, then the semaphore count should be '-n'
       */

      sched_lock(); /* Needs to be atomic */
      ret      = sem_getvalue(&g_sem, &count);
      nwaiting = nhighpri_waiting();
      sched_unlock();

      if (ret < 0)
        {
          printf("lowpri_thread-%d: ERROR sem_getvalue failed: %d\n", threadno, errno);
        }
      printf("lowpri_thread-%d: Sem count: %d, No. highpri thread: %d\n", threadno, count, nwaiting);

      /* The middle priority task is running, let go of the semaphore */

      if (g_middlestate == RUNNING && nwaiting == -count)
        {
          /* Good.. the middle priority task is still running and the counts are okay. */

          retval = NULL;
        }
      else
        {
          /* If the sem count is positive, then there all of the higher priority threads
           * should have already completed.
           */

          printf("lowpri_thread-%d: %s the middle priority task has already exitted!\n",
                 threadno, count >= 0 ? "SUCCESS" : "ERROR" );
          printf("               g_middlestate:  %d sem count=%d\n", (int)g_middlestate, count);
          for (i = 0; i < NHIGHPRI_THREADS; i++)
            {
              printf("               g_highstate[%d]: %d\n", i, (int)g_highstate[i]);
            }
        }
    }

  ret = pthread_getschedparam(pthread_self(), &policy, &sparam);
  sem_enumholders(&g_sem);
  sem_post(&g_sem);
  if (ret != 0)
    {
      printf("lowpri_thread-%d: ERROR pthread_getschedparam failed: %d\n", threadno, ret);
    }
  else
    {
      if (nwaiting > 0)
        {
          expected = g_highpri;
        }
      else
        {
          expected = g_lowpri;
        }

      printf("lowpri_thread-%d: %s priority before sem_post: %d\n",
             threadno,
             sparam.sched_priority != expected ? "ERROR" : "SUCCESS",
             sparam.sched_priority);

      if (sparam.sched_priority != expected)
        {
          printf("               ERROR should have been %d\n", expected);
        }
    }

  ret = pthread_getschedparam(pthread_self(), &policy, &sparam);
  if (ret != 0)
    {
      printf("lowpri_thread-%d: ERROR pthread_getschedparam failed: %d\n", threadno, ret);
    }
  else
    {
      printf("lowpri_thread-%d: %s final priority: %d\n",
             threadno,
             sparam.sched_priority != g_lowpri ? "ERROR" : "SUCCESS",
             sparam.sched_priority);

      if (sparam.sched_priority != g_lowpri)
        {
          printf("               ERROR should have been %d\n", g_lowpri);
        } 
    }
  sem_enumholders(&g_sem);

  printf("lowpri_thread-%d: Okay... I'm done!\n", threadno);
  FFLUSH();
  g_lowstate[threadno-1] = DONE;
  return retval;  
}
#endif /* CONFIG_PRIORITY_INHERITANCE && !CONFIG_DISABLE_SIGNALS && !CONFIG_DISABLE_PTHREAD */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: priority_inheritance
 ****************************************************************************/

void priority_inheritance(void)
{
#if defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_DISABLE_SIGNALS) && !defined(CONFIG_DISABLE_PTHREAD)
  pthread_t lowpri[NLOWPRI_THREADS];
  pthread_t medpri;
  pthread_t highpri[NHIGHPRI_THREADS];
  pthread_addr_t result;
  pthread_attr_t attr;
  struct sched_param sparam;
  int my_pri;
  int status;
  int i;

  printf("priority_inheritance: Started\n");

  g_middlestate = NOTSTARTED;
  for (i = 0; i < NHIGHPRI_THREADS; i++) g_highstate[i] = NOTSTARTED;
  for (i = 0; i < NLOWPRI_THREADS; i++)  g_lowstate[i]  = NOTSTARTED;

  status = sched_getparam (getpid(), &sparam);
  if (status != 0)
    {
      printf("priority_inheritance: sched_getparam failed\n");
      sparam.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }
  my_pri  = sparam.sched_priority;

  g_highpri = sched_get_priority_max(SCHED_FIFO);
  g_lowpri = sched_get_priority_min(SCHED_FIFO);
  g_medpri = my_pri - 1;

  sem_init(&g_sem, 0, NLOWPRI_THREADS);
  dump_nfreeholders("priority_inheritance:");

  /* Start the low priority threads */

  for (i = 0; i < NLOWPRI_THREADS; i++)
    {
      int threadno = i+1;
      printf("priority_inheritance: Starting lowpri_thread-%d (of %d) at %d\n",
             threadno, NLOWPRI_THREADS, g_lowpri);
      status = pthread_attr_init(&attr);
      if (status != 0)
        {
          printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
        }
      sparam.sched_priority = g_lowpri;
      status = pthread_attr_setschedparam(&attr,& sparam);
      if (status != OK)
        {
          printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
        }
      else
        {
          printf("priority_inheritance: Set lowpri_thread-%d priority to %d\n",
                 threadno, sparam.sched_priority);
        }

      status = pthread_create(&lowpri[i], &attr, lowpri_thread, (void*)threadno);
      if (status != 0)
        {
          printf("priority_inheritance: pthread_create failed, status=%d\n", status);
        }
    }
  printf("priority_inheritance: Waiting...\n");
  sleep(2);
  dump_nfreeholders("priority_inheritance:");

  /* Start the medium priority thread */

  printf("priority_inheritance: Starting medpri_thread at %d\n", g_medpri);
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
    }

  sparam.sched_priority = g_medpri;
  status = pthread_attr_setschedparam(&attr,& sparam);
  if (status != OK)
    {
      printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
    }
  else
    {
      printf("priority_inheritance: Set medpri_thread priority to %d\n", sparam.sched_priority);
    }
  FFLUSH();

  status = pthread_create(&medpri, &attr, medpri_thread, NULL);
  if (status != 0)
    {
      printf("priority_inheritance: pthread_create failed, status=%d\n", status);
    }
  printf("priority_inheritance: Waiting...\n");
  sleep(1);
  dump_nfreeholders("priority_inheritance:");

  /* Start the high priority threads */

  for (i = 0; i < NHIGHPRI_THREADS; i++)
    {
      int threadno = i+1;
      printf("priority_inheritance: Starting highpri_thread-%d (of %d) at %d\n",
             threadno, NHIGHPRI_THREADS, g_highpri);
      status = pthread_attr_init(&attr);
      if (status != 0)
        {
          printf("priority_inheritance: pthread_attr_init failed, status=%d\n", status);
        }

      sparam.sched_priority = g_highpri - i;
      status = pthread_attr_setschedparam(&attr,& sparam);
      if (status != OK)
        {
          printf("priority_inheritance: pthread_attr_setschedparam failed, status=%d\n", status);
        }
      else
        {
          printf("priority_inheritance: Set highpri_thread-%d priority to %d\n",
                 threadno, sparam.sched_priority);
        }
      FFLUSH();

      status = pthread_create(&highpri[i], &attr, highpri_thread, (void*)threadno);
      if (status != 0)
        {
          printf("priority_inheritance: pthread_create failed, status=%d\n", status);
        }
    }
  dump_nfreeholders("priority_inheritance:");
  FFLUSH();

  /* Wait for all thread instances to complete */

  for (i = 0; i < NHIGHPRI_THREADS; i++)
    {
      printf("priority_inheritance: Waiting for highpri_thread-%d to complete\n", i+1);
      FFLUSH();
      (void)pthread_join(highpri[i], &result);
      dump_nfreeholders("priority_inheritance:");
  }
  printf("priority_inheritance: Waiting for medpri_thread to complete\n");
  FFLUSH();
  (void)pthread_join(medpri, &result);
  dump_nfreeholders("priority_inheritance:");
  for (i = 0; i < NLOWPRI_THREADS; i++)
    {
      printf("priority_inheritance: Waiting for lowpri_thread-%d to complete\n", i+1);
      FFLUSH();
      (void)pthread_join(lowpri[i], &result);
      dump_nfreeholders("priority_inheritance:");
    }

  printf("priority_inheritance: Finished\n");
  sem_destroy(&g_sem);
  dump_nfreeholders("priority_inheritance:");
  FFLUSH();
#endif /* CONFIG_PRIORITY_INHERITANCE && !CONFIG_DISABLE_SIGNALS && !CONFIG_DISABLE_PTHREAD */
}
