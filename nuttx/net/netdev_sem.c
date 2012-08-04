/****************************************************************************
 * net/netdev_sem.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/types.h>

#include <unistd.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include "net_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define NO_HOLDER (pid_t)-1

/****************************************************************************
 * Priviate Types
 ****************************************************************************/

/* There is at least on context in which recursive semaphores are required:
 * When netdev_foreach is used with a telnet client, we will deadlock if we
 * do not provide this capability.
 */

struct netdev_sem_s
{
  sem_t        sem;
  pid_t        holder;
  unsigned int count;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

static struct netdev_sem_s g_devlock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_seminit
 *
 * Description:
 *   Initialize the network device semaphore.
 *
 ****************************************************************************/

void netdev_seminit(void)
{
  sem_init(&g_devlock.sem, 0, 1);
  g_devlock.holder = NO_HOLDER;
  g_devlock.count  = 0;
}

/****************************************************************************
 * Function: netdev_semtake
 *
 * Description:
 *   Get exclusive access to the network device list.
 *
 ****************************************************************************/

void netdev_semtake(void)
{
  pid_t me = getpid();

  /* Does this thread already hold the semaphore? */

  if (g_devlock.holder == me)
    {
      /* Yes.. just increment the reference count */

      g_devlock.count++;
    }
  else
    {
      /* No.. take the semaphore (perhaps waiting) */

      while (uip_lockedwait(&g_devlock.sem) != 0)
        {
          /* The only case that an error should occur here is if
           * the wait was awakened by a signal.
           */

          ASSERT(errno == EINTR);
        }

      /* Now this thread holds the semaphore */

      g_devlock.holder = me;
      g_devlock.count  = 1;
    }
}

/****************************************************************************
 * Function: netdev_semtake
 *
 * Description:
 *   Release exclusive access to the network device list
 *
 ****************************************************************************/

void netdev_semgive(void)
{
  DEBUGASSERT(g_devlock.holder == getpid() && g_devlock.count > 0);

  /* If the count would go to zero, then release the semaphore */

  if (g_devlock.count == 1)
    {
      /* We no longer hold the semaphore */

      g_devlock.holder = NO_HOLDER;
      g_devlock.count  = 0;
      sem_post(&g_devlock.sem);
    }
  else
    {
      /* We still hold the semaphore. Just decrement the count */

      g_devlock.count--;
    }
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
