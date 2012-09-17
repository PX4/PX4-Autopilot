/****************************************************************************
 * net/uip/uip_lock.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <unistd.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/net/uip/uip.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_NOINTS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NO_HOLDER (pid_t)-1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t        g_uipsem;
static pid_t        g_holder  = NO_HOLDER;
static unsigned int g_count   = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_takesem
 *
 * Description:
 *   Take the semaphore
 *
 ****************************************************************************/

static void uip_takesem(void)
{
  while (sem_wait(&g_uipsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_lockinit
 *
 * Description:
 *   Initialize the locking facility
 *
 ****************************************************************************/

void uip_lockinit(void)
{
  sem_init(&g_uipsem, 0, 1);
}

/****************************************************************************
 * Function: uip_lock
 *
 * Description:
 *   Take the lock
 *
 ****************************************************************************/

uip_lock_t uip_lock(void)
{
  pid_t me = getpid();

  /* Does this thread already hold the semaphore? */

  if (g_holder == me)
    {
      /* Yes.. just increment the reference count */

      g_count++;
    }
  else
    {
      /* No.. take the semaphore (perhaps waiting) */

      uip_takesem();

      /* Now this thread holds the semaphore */

      g_holder = me;
      g_count  = 1;
    }

  return 0;
}

/****************************************************************************
 * Function: uip_unlock
 *
 * Description:
 *   Release the lock.
 *
 ****************************************************************************/

void uip_unlock(uip_lock_t flags)
{
  DEBUGASSERT(g_holder == getpid() && g_count > 0);

  /* If the count would go to zero, then release the semaphore */

  if (g_count == 1)
    {
      /* We no longer hold the semaphore */

      g_holder = NO_HOLDER;
      g_count  = 0;
      sem_post(&g_uipsem);
    }
  else
    {
      /* We still hold the semaphore. Just decrement the count */

      g_count--;
    }
}

/****************************************************************************
 * Function: uip_lockedwait
 *
 * Description:
 *   Atomically wait for sem while temporarily releasing g_uipsem.
 *
 ****************************************************************************/

int uip_lockedwait(sem_t *sem)
{
  pid_t        me = getpid();
  unsigned int count;
  irqstate_t   flags;
  int          ret;

  flags = irqsave(); /* No interrupts */
  sched_lock();      /* No context switches */
  if (g_holder == me)
    {
      /* Release the uIP semaphore, remembering the count */
 
      count    = g_count;
      g_holder = NO_HOLDER;
      g_count  = 0;
      sem_post(&g_uipsem);

      /* Now take the semaphore */

      ret = sem_wait(sem);

      /* Recover the uIP semaphore at the proper count */

      uip_takesem();
      g_holder = me;
      g_count  = count;
    }
  else
    {
      ret = sem_wait(sem);
    }

  sched_unlock();
  irqrestore(flags);
  return ret;
 }

#endif /* CONFIG_NET */
