/****************************************************************************
 * netutils/thttpd/timers.c
 * Simple Timer Routines
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995,1998,2000 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/time.h>

#include <stdlib.h>
#include <stdio.h>
#include <debug.h>

#include "thttpd_alloc.h"
#include "timers.h"

/****************************************************************************
 * Pre-Processor Definitons
 ****************************************************************************/

#define HASH_SIZE 67

/****************************************************************************
 * Private Data
 ****************************************************************************/

static Timer *timers[HASH_SIZE];
static Timer *free_timers;

/****************************************************************************
 * Public Data
 ****************************************************************************/

ClientData JunkClientData;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned int hash(Timer *tmr)
{
  /* We can hash on the trigger time, even though it can change over the
   * life of a timer via the periodic bit.
   * This is because both of those guys call l_resort(), which  * recomputes
   * the hash and moves the timer to the appropriate list.
   */

  return ((unsigned int)tmr->time.tv_sec ^
          (unsigned int)tmr->time.tv_usec) % HASH_SIZE;
}

static void l_add(Timer *tmr)
{
  int h = tmr->hash;
  register Timer *tmr2;
  register Timer *tmr2prev;

  tmr2 = timers[h];
  if (tmr2 == NULL)
    {
      /* The list is empty. */
      timers[h] = tmr;
      tmr->prev = tmr->next = NULL;
    }
  else
    {
      if (tmr->time.tv_sec < tmr2->time.tv_sec ||
          (tmr->time.tv_sec == tmr2->time.tv_sec &&
           tmr->time.tv_usec <= tmr2->time.tv_usec))
        {
          /* The new timer goes at the head of the list. */

          timers[h] = tmr;
          tmr->prev = NULL;
          tmr->next = tmr2;
          tmr2->prev = tmr;
        }
      else
        {
          /* Walk the list to find the insertion point. */

          for (tmr2prev = tmr2, tmr2 = tmr2->next; tmr2 != NULL;
               tmr2prev = tmr2, tmr2 = tmr2->next)
            {
              if (tmr->time.tv_sec < tmr2->time.tv_sec ||
                  (tmr->time.tv_sec == tmr2->time.tv_sec &&
                   tmr->time.tv_usec <= tmr2->time.tv_usec))
                {
                  /* Found it. */
                  tmr2prev->next = tmr;
                  tmr->prev = tmr2prev;
                  tmr->next = tmr2;
                  tmr2->prev = tmr;
                  return;
                }
            }

          /* Oops, got to the end of the list.  Add to tail. */

          tmr2prev->next = tmr;
          tmr->prev = tmr2prev;
          tmr->next = NULL;
        }
    }
}

static void l_remove(Timer *tmr)
{
  int h = tmr->hash;

  if (tmr->prev == NULL)
    {
      timers[h] = tmr->next;
    }
  else
    {
      tmr->prev->next = tmr->next;
    }

  if (tmr->next != NULL)
    {
      tmr->next->prev = tmr->prev;
    }
}

static void l_resort(Timer *tmr)
{
  /* Remove the timer from its old list. */

  l_remove(tmr);

  /* Recompute the hash. */

  tmr->hash = hash(tmr);

  /* And add it back in to its new list, sorted correctly. */

  l_add(tmr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tmr_init(void)
{
  int h;

  for (h = 0; h < HASH_SIZE; ++h)
    {
      timers[h] = NULL;
    }

  free_timers = NULL;
}

Timer *tmr_create(struct timeval *now, TimerProc *timer_proc,
                  ClientData client_data, long msecs, int periodic)
{
  Timer *tmr;

  if (free_timers != NULL)
    {
      tmr = free_timers;
      free_timers = tmr->next;
    }
  else
    {
      tmr = (Timer*)httpd_malloc(sizeof(Timer));
      if (!tmr)
        {
          return NULL;
        }
    }

  tmr->timer_proc  = timer_proc;
  tmr->client_data = client_data;
  tmr->msecs       = msecs;
  tmr->periodic    = periodic;

  if (now != NULL)
    {
      tmr->time = *now;
    }
  else
    {
      (void)gettimeofday(&tmr->time, NULL);
    }

  tmr->time.tv_sec  += msecs / 1000L;
  tmr->time.tv_usec += (msecs % 1000L) * 1000L;
  if (tmr->time.tv_usec >= 1000000L)
    {
      tmr->time.tv_sec  += tmr->time.tv_usec / 1000000L;
      tmr->time.tv_usec %= 1000000L;
    }
  tmr->hash = hash(tmr);

  /* Add the new timer to the proper active list. */

  l_add(tmr);
  return tmr;
}

long tmr_mstimeout(struct timeval *now)
{
  int h;
  int gotone;
  long msecs, m;
  register Timer *tmr;

  gotone = 0;
  msecs  = 0;

  /* Since the lists are sorted, we only need to look at the  * first timer on
   * each one.
   */

  for (h = 0; h < HASH_SIZE; ++h)
    {
      tmr = timers[h];
      if (tmr != NULL)
        {
          m = (tmr->time.tv_sec - now->tv_sec) * 1000L +
            (tmr->time.tv_usec - now->tv_usec) / 1000L;
          if (!gotone)
            {
              msecs = m;
              gotone = 1;
            }
          else if (m < msecs)
            {
              msecs = m;
            }
        }
    }

  if (!gotone)
    {
      return INFTIM;
    }

  if (msecs <= 0)
    {
      msecs = 0;
    }

  return msecs;
}

void tmr_run(struct timeval *now)
{
  int h;
  Timer *tmr;
  Timer *next;

  for (h = 0; h < HASH_SIZE; ++h)
    {
      for (tmr = timers[h]; tmr != NULL; tmr = next)
        {
          next = tmr->next;

          /* Since the lists are sorted, as soon as we find a timer  * that isn'tmr 
           * ready yet, we can go on to the next list
           */

          if (tmr->time.tv_sec > now->tv_sec ||
              (tmr->time.tv_sec == now->tv_sec && tmr->time.tv_usec > now->tv_usec))
            {
              break;
            }

          (tmr->timer_proc)(tmr->client_data, now);
          if (tmr->periodic)
            {
              /* Reschedule. */

              tmr->time.tv_sec += tmr->msecs / 1000L;
              tmr->time.tv_usec += (tmr->msecs % 1000L) * 1000L;
              if (tmr->time.tv_usec >= 1000000L)
                {
                  tmr->time.tv_sec += tmr->time.tv_usec / 1000000L;
                  tmr->time.tv_usec %= 1000000L;
                }
              l_resort(tmr);
            }
          else
            {
              tmr_cancel(tmr);
            }
        }
    }
}

void tmr_cancel(Timer *tmr)
{
  /* Remove it from its active list. */

  l_remove(tmr);

  /* And put it on the free list. */

  tmr->next   = free_timers;
  free_timers = tmr;
  tmr->prev   = NULL;
}

void tmr_cleanup(void)
{
  Timer *tmr;

  while (free_timers != NULL)
    {
      tmr = free_timers;
      free_timers = tmr->next;
      httpd_free((void*)tmr);
    }
}

void tmr_destroy(void)
{
  int h;

  for (h = 0; h < HASH_SIZE; ++h)
    {
      while (timers[h] != NULL)
        {
          tmr_cancel(timers[h]);
        }
    }
  tmr_cleanup();
}
