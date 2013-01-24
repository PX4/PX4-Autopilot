/***********************************************************************
 * apps/examples/ostest/sighand.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include "ostest.h"

#ifndef NULL
# define NULL (void*)0
#endif

#define WAKEUP_SIGNAL 17
#define SIGVALUE_INT  42

static sem_t sem;
static bool sigreceived = false;
static bool threadexited = false;

#ifdef CONFIG_SCHED_HAVE_PARENT
static void death_of_child(int signo, siginfo_t *info, void *ucontext)
{
  /* Use of printf in a signal handler is NOT safe! It can cause deadlocks!
   * Also, signals are not queued by NuttX.  As a consequence, some
   * notifications will get lost (or the info data can be overwrittedn)! 
   * Because POSIX  does not require signals to be queued, I do not think
   * that this is a bug (the overwriting is a bug, however).
   */

  if (info)
    {
      printf("death_of_child: PID %d received signal=%d code=%d pid=%d status=%d\n",
             getpid(), signo, info->si_code, info->si_pid, info->si_status);
    }
  else
    {
      printf("death_of_child: PID %d received signal=%d (no info?)\n",
             getpid(), signo);
    }
}
#endif

static void wakeup_action(int signo, siginfo_t *info, void *ucontext)
{
  sigset_t oldset;
  sigset_t allsigs;
  int status;

  /* Use of printf in a signal handler is NOT safe! It can cause deadlocks! */

  printf("wakeup_action: Received signal %d\n" , signo);

  sigreceived = true;

  /* Check signo */

  if (signo != WAKEUP_SIGNAL)
    {
      printf("wakeup_action: ERROR expected signo=%d\n" , WAKEUP_SIGNAL);
    }

  /* Check siginfo */

  if (info->si_value.sival_int != SIGVALUE_INT)
    {
      printf("wakeup_action: ERROR sival_int=%d expected %d\n",
              info->si_value.sival_int, SIGVALUE_INT);
    }
  else
    {
      printf("wakeup_action: sival_int=%d\n" , info->si_value.sival_int);
    }

  if (info->si_signo != WAKEUP_SIGNAL)
    {
      printf("wakeup_action: ERROR expected si_signo=%d, got=%d\n",
               WAKEUP_SIGNAL, info->si_signo);
    }

  printf("wakeup_action: si_code=%d\n" , info->si_code);

  /* Check ucontext_t */

  printf("wakeup_action: ucontext=%p\n" , ucontext);

  /* Check sigprocmask */

  (void)sigfillset(&allsigs);
  status = sigprocmask(SIG_SETMASK, NULL, &oldset);
  if (status != OK)
    {
      printf("wakeup_action: ERROR sigprocmask failed, status=%d\n",
              status);
    }

  if (oldset != allsigs)
    {
      printf("wakeup_action: ERROR sigprocmask=%x expected=%x\n",
              oldset, allsigs);
    }
}

static int waiter_main(int argc, char *argv[])
{
  sigset_t sigset;
  struct sigaction act;
  struct sigaction oact;
  int status;

  printf("waiter_main: Waiter started\n" );

  printf("waiter_main: Unmasking signal %d\n" , WAKEUP_SIGNAL);
  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, WAKEUP_SIGNAL);
  status = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  if (status != OK)
    {
      printf("waiter_main: ERROR sigprocmask failed, status=%d\n",
              status);
    }

  printf("waiter_main: Registering signal handler\n" );
  act.sa_sigaction = wakeup_action;
  act.sa_flags  = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, WAKEUP_SIGNAL);

  status = sigaction(WAKEUP_SIGNAL, &act, &oact);
  if (status != OK)
    {
      printf("waiter_main: ERROR sigaction failed, status=%d\n" , status);
    }

#ifndef SDCC
  printf("waiter_main: oact.sigaction=%p oact.sa_flags=%x oact.sa_mask=%x\n",
          oact.sa_sigaction, oact.sa_flags, oact.sa_mask);
#endif

  /* Take the semaphore */

  printf("waiter_main: Waiting on semaphore\n" );
  FFLUSH();

  status = sem_wait(&sem);
  if (status != 0)
    {
      int error = errno;
      if (error == EINTR)
        {
          printf("waiter_main: sem_wait() successfully interrupted by signal\n" );
        }
      else
        {
          printf("waiter_main: ERROR sem_wait failed, errno=%d\n" , error);
        }
    }
  else
    {
      printf("waiter_main: ERROR awakened with no error!\n" );
    }

  /* Detach the signal handler */

  act.sa_sigaction = SIG_DFL;
  status = sigaction(WAKEUP_SIGNAL, &act, &oact);

  printf("waiter_main: done\n" );
  FFLUSH();

  threadexited = true;
  return 0;
}

void sighand_test(void)
{
#ifdef CONFIG_SCHED_HAVE_PARENT
  struct sigaction act;
  struct sigaction oact;
  sigset_t sigset;
#endif
  struct sched_param param;
  union sigval sigvalue;
  pid_t waiterpid;
  int policy;
  int status;

  printf("sighand_test: Initializing semaphore to 0\n" );
  sem_init(&sem, 0, 0);

#ifdef CONFIG_SCHED_HAVE_PARENT
  printf("sighand_test: Unmasking SIGCHLD\n");

  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, SIGCHLD);
  status = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  if (status != OK)
    {
      printf("sighand_test: ERROR sigprocmask failed, status=%d\n",
              status);
    }

  printf("sighand_test: Registering SIGCHLD handler\n" );
  act.sa_sigaction = death_of_child;
  act.sa_flags  = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, SIGCHLD);

  status = sigaction(SIGCHLD, &act, &oact);
  if (status != OK)
    {
      printf("waiter_main: ERROR sigaction failed, status=%d\n" , status);
    }
#endif

  /* Start waiter thread  */

  printf("sighand_test: Starting waiter task\n" );
  status = sched_getparam (0, &param);
  if (status != OK)
    {
      printf("sighand_test: ERROR sched_getparam() failed\n" );
      param.sched_priority = PTHREAD_DEFAULT_PRIORITY;
    }

  policy = sched_getscheduler(0);
  if (policy == ERROR)
    {
      printf("sighand_test: ERROR sched_getscheduler() failed\n" );
      policy = SCHED_FIFO;
    }

  waiterpid = task_create("waiter", param.sched_priority,
                           PTHREAD_STACK_DEFAULT, waiter_main, NULL);
  if (waiterpid == ERROR)
    {
      printf("sighand_test: ERROR failed to start waiter_main\n" );
    }
  else
    {
      printf("sighand_test: Started waiter_main pid=%d\n", waiterpid);
    }

  /* Wait a bit */

  FFLUSH();
  sleep(2);

  /* Then signal the waiter thread. */

  printf("sighand_test: Signaling pid=%d with signo=%d sigvalue=%d\n",
         waiterpid, WAKEUP_SIGNAL, SIGVALUE_INT);

  sigvalue.sival_int = SIGVALUE_INT;
#ifdef CONFIG_CAN_PASS_STRUCTS
  status = sigqueue(waiterpid, WAKEUP_SIGNAL, sigvalue);
#else
  status = sigqueue(waiterpid, WAKEUP_SIGNAL, sigvalue.sival_ptr);
#endif
  if (status != OK)
    {
      printf("sighand_test: ERROR sigqueue failed\n" );
      task_delete(waiterpid);
    }

  /* Wait a bit */

  FFLUSH();
  sleep(2);

  /* Then check the result */

  if (!threadexited)
    {
      printf("sighand_test: ERROR waiter task did not exit\n" );
    }

  if (!sigreceived)
    {
      printf("sighand_test: ERROR signal handler did not run\n" );
    }

  /* Detach the signal handler */

#ifdef CONFIG_SCHED_HAVE_PARENT
  act.sa_sigaction = SIG_DFL;
  status = sigaction(SIGCHLD, &act, &oact);
#endif

  printf("sighand_test: done\n" );
  FFLUSH();
}
