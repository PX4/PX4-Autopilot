/********************************************************************************
 * examples/ostest/roundrobin.c
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include "ostest.h"

#if CONFIG_RR_INTERVAL > 0

/********************************************************************************
 * Definitions
 ********************************************************************************/

/* This number may need to be tuned for different processor speeds.  Since these
 * arrays must be large to very correct SCHED_RR behavior, this test may require
 * too much memory on many targets.
 */

/* #define CONFIG_NINTEGERS 32768 Takes forever on 60Mhz ARM7 */

#define CONFIG_NINTEGERS 2048

/********************************************************************************
 * Private Data
 ********************************************************************************/

static int prime1[CONFIG_NINTEGERS];
static int prime2[CONFIG_NINTEGERS];

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Name: dosieve
 *
 * Description
 *   This implements a "sieve of aristophanes" algorithm for finding prime number.
 *   Credit for this belongs to someone, but I am not sure who anymore.  Anyway,
 *   the only purpose here is that we need some algorithm that takes a long period 
 *   of time to execute.
 *
 ********************************************************************************/

static void dosieve(int *prime)
{
  int a,d;
  int i;
  int j;

  a = 2;
  d = a;

  for (i = 0; i < CONFIG_NINTEGERS; i++)
    {
      prime[i] = i+2;
    }

  for (i = 1; i < 10; i++)
    {
      for (j = 0; j < CONFIG_NINTEGERS; j++)
        {
          d = a + d;
          if (d < CONFIG_NINTEGERS)
            {
              prime[d]=0;
            }
        }
      a++;
      d = a;
      i++;
    }

#if 0 /* We don't really care what the numbers are */
  for (i = 0, j= 0; i < CONFIG_NINTEGERS; i++)
    {
      if (prime[i] != 0)
       {
         printf(" Prime %d: %d\n", j, prime[i]);
         j++;
       }
    }
#endif
}

/********************************************************************************
 * Name: sieve1
 ********************************************************************************/

static void *sieve1(void *parameter)
{
  int i;

  printf("sieve1 started\n");

  for (i = 0; i < 1000; i++)
    {
      dosieve(prime1);
    }

  printf("sieve1 finished\n");

  pthread_exit(NULL);
  return NULL; /* To keep some compilers happy */
}

/********************************************************************************
 * Name: sieve2
 ********************************************************************************/

static void *sieve2(void *parameter)
{
  int i;

  printf("sieve2 started\n");

  for (i = 0; i < 1000; i++)
    {
      dosieve(prime2);
    }

  printf("sieve2 finished\n");

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
  pthread_t sieve1_thread;
  pthread_t sieve2_thread;
  struct sched_param sparam;
  pthread_attr_t attr;
  pthread_addr_t result;
  int status;

  printf("rr_test: Starting sieve1 thread \n");
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
      printf("rr_test: Set thread policty to SCHED_RR\n");
    }

  status = pthread_create(&sieve1_thread, &attr, sieve1, NULL);
  if (status != 0)
    {
      printf("rr_test: Error in thread 1 creation, status=%d\n",  status);
    }

  printf("rr_test: Starting sieve1 thread \n");

  status = pthread_create(&sieve2_thread, &attr, sieve2, NULL);
  if (status != 0)
    {
      printf("rr_test: Error in thread 2 creation, status=%d\n",  status);
    }

  printf("rr_test: Waiting for sieves to complete -- this should take awhile\n");
  printf("rr_test: If RR scheduling is working, they should start and complete at\n");
  printf("rr_test: about the same time\n");

  pthread_join(sieve2_thread, &result);
  pthread_join(sieve1_thread, &result);
  printf("rr_test: Done\n");
}

#endif /* CONFIG_RR_INTERVAL */
