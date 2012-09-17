/***********************************************************************
 * examples/ostest/posixtimer.c
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
 ***********************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include "ostest.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

#ifndef NULL
# define NULL (void*)0
#endif

#define MY_TIMER_SIGNAL 17
#define SIGVALUE_INT  42

/**************************************************************************
 * Private Data
 **************************************************************************/

static sem_t sem;
static int g_nsigreceived = 0;

/**************************************************************************
 * Private Functions
 **************************************************************************/

static void timer_expiration(int signo, siginfo_t *info, void *ucontext)
{
  sigset_t oldset;
  sigset_t allsigs;
  int status;

  printf("timer_expiration: Received signal %d\n" , signo);

  g_nsigreceived++;

  /* Check signo */

  if (signo != MY_TIMER_SIGNAL)
    {
      printf("timer_expiration: ERROR expected signo=%d\n" , MY_TIMER_SIGNAL);
    }

  /* Check siginfo */

  if (info->si_value.sival_int != SIGVALUE_INT)
    {
      printf("timer_expiration: ERROR sival_int=%d expected %d\n",
              info->si_value.sival_int, SIGVALUE_INT);
    }
  else
    {
      printf("timer_expiration: sival_int=%d\n" , info->si_value.sival_int);
    }

  if (info->si_signo != MY_TIMER_SIGNAL)
    {
      printf("timer_expiration: ERROR expected si_signo=%d, got=%d\n",
               MY_TIMER_SIGNAL, info->si_signo);
    }

  if (info->si_code == SI_TIMER)
    {
      printf("timer_expiration: si_code=%d (SI_TIMER)\n" , info->si_code);
    }
  else
    {
      printf("timer_expiration: ERROR si_code=%d, expected SI_TIMER=%d\n",
             info->si_code, SI_TIMER);
    }

  /* Check ucontext_t */

  printf("timer_expiration: ucontext=%p\n" , ucontext);

  /* Check sigprocmask */

  (void)sigfillset(&allsigs);
  status = sigprocmask(SIG_SETMASK, NULL, &oldset);
  if (status != OK)
    {
      printf("timer_expiration: ERROR sigprocmask failed, status=%d\n",
              status);
    }

  if (oldset != allsigs)
    {
      printf("timer_expiration: ERROR sigprocmask=%x expected=%x\n",
              oldset, allsigs);
    }

}

/**************************************************************************
 * Public Functions
 **************************************************************************/

void timer_test(void)
{
  sigset_t           sigset;
  struct sigaction   act;
  struct sigaction   oact;
  struct sigevent    notify;
  struct itimerspec  timer;
  timer_t            timerid;
  int                status;
  int                i;

  printf("timer_test: Initializing semaphore to 0\n" );
  sem_init(&sem, 0, 0);

  /* Start waiter thread  */

  printf("timer_test: Unmasking signal %d\n" , MY_TIMER_SIGNAL);

  (void)sigemptyset(&sigset);
  (void)sigaddset(&sigset, MY_TIMER_SIGNAL);
  status = sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  if (status != OK)
    {
      printf("timer_test: ERROR sigprocmask failed, status=%d\n",
              status);
    }

  printf("timer_test: Registering signal handler\n" );
  act.sa_sigaction = timer_expiration;
  act.sa_flags  = SA_SIGINFO;

  (void)sigfillset(&act.sa_mask);
  (void)sigdelset(&act.sa_mask, MY_TIMER_SIGNAL);

  status = sigaction(MY_TIMER_SIGNAL, &act, &oact);
  if (status != OK)
    {
      printf("timer_test: ERROR sigaction failed, status=%d\n" , status);
    }

#ifndef SDCC
  printf("timer_test: oact.sigaction=%p oact.sa_flags=%x oact.sa_mask=%x\n",
          oact.sa_sigaction, oact.sa_flags, oact.sa_mask);
#endif

  /* Create the POSIX timer */

  printf("timer_test: Creating timer\n" );

  notify.sigev_notify          = SIGEV_SIGNAL;
  notify.sigev_signo           = MY_TIMER_SIGNAL;
  notify.sigev_value.sival_int = SIGVALUE_INT;

  status = timer_create(CLOCK_REALTIME, &notify, &timerid);
  if (status != OK)
    {
      printf("timer_test: timer_create failed, errno=%d\n", errno);
      goto errorout;
    }

  /* Start the POSIX timer */

  printf("timer_test: Starting timer\n" );

  timer.it_value.tv_sec     = 2;
  timer.it_value.tv_nsec    = 0;
  timer.it_interval.tv_sec  = 2;
  timer.it_interval.tv_nsec = 0;

  status = timer_settime(timerid, 0, &timer, NULL);
  if (status != OK)
    {
      printf("timer_test: timer_settime failed, errno=%d\n", errno);
      goto errorout;
    }

  /* Take the semaphore */

  for (i = 0; i < 5; i++)
    {
      printf("timer_test: Waiting on semaphore\n" );
      FFLUSH();
      status = sem_wait(&sem);
      if (status != 0)
        {
          int error = errno;
          if (error == EINTR)
            {
              printf("timer_test: sem_wait() successfully interrupted by signal\n" );
            }
          else
            {
              printf("timer_test: ERROR sem_wait failed, errno=%d\n" , error);
            }
        }
      else
        {
          printf("timer_test: ERROR awakened with no error!\n" );
        }
      printf("timer_test: g_nsigreceived=%d\n", g_nsigreceived);
    }

errorout:
  sem_destroy(&sem);

  /* Then delete the timer */

  printf("timer_test: Deleting timer\n" );
  status = timer_delete(timerid);
  if (status != OK)
    {
      printf("timer_test: timer_create failed, errno=%d\n", errno);
    }

  /* Detach the signal handler */

  act.sa_sigaction = SIG_DFL;
  status = sigaction(MY_TIMER_SIGNAL, &act, &oact);

  printf("timer_test: done\n" );
  FFLUSH();
}
