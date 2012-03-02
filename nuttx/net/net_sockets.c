/****************************************************************************
 * net/net_sockets.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <net/uip/uip.h>
#include <nuttx/net.h>
#include <nuttx/kmalloc.h>

#include "net_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
static void _net_semtake(FAR struct socketlist *list)
{
  /* Take the semaphore (perhaps waiting) */

  while (uip_lockedwait(&list->sl_sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

# define _net_semgive(list) sem_post(&list->sl_sem)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* This is called from the initialization logic to configure the socket layer */

void net_initialize(void)
{
  /* Initialize the uIP layer */

  uip_initialize();

  /* Initialize the socket layer */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  sem_init(&g_netdev_sem, 0, 1);
#endif

  /* Initialize the periodic ARP timer */

  arptimer_init();
}

#if CONFIG_NSOCKET_DESCRIPTORS > 0

/* Allocate a list of files for a new task */

FAR struct socketlist *net_alloclist(void)
{
  FAR struct socketlist *list;
  list = (FAR struct socketlist*)kzalloc(sizeof(struct socketlist));
  if (list)
    {
       /* Start with a reference count of one */

       list->sl_crefs = 1;

       /* Initialize the list access mutex */

      (void)sem_init(&list->sl_sem, 0, 1);
    }
  return list;
}

/* Increase the reference count on a file list */

int net_addreflist(FAR struct socketlist *list)
{
  if (list)
    {
       /* Increment the reference count on the list.
        * NOTE: that we disable interrupts to do this
        * (vs. taking the list semaphore).  We do this
        * because file cleanup operations often must be
        * done from the IDLE task which cannot wait
        * on semaphores.
        */

       register uip_lock_t flags = uip_lock();
       list->sl_crefs++;
       uip_unlock(flags);
    }
  return OK;
}

/* Release a reference to the file list */

int net_releaselist(FAR struct socketlist *list)
{
  int crefs;
  int ndx;

  if (list)
    {
       /* Decrement the reference count on the list.
        * NOTE: that we disable interrupts to do this
        * (vs. taking the list semaphore).  We do this
        * because file cleanup operations often must be
        * done from the IDLE task which cannot wait
        * on semaphores.
        */

       uip_lock_t flags = uip_lock();
       crefs = --(list->sl_crefs);
       uip_unlock(flags);

       /* If the count decrements to zero, then there is no reference
        * to the structure and it should be deallocated.  Since there
        * are references, it would be an error if any task still held
        * a reference to the list's semaphore.
        */

       if (crefs <= 0)
         {
           /* Close each open socket in the list
            * REVISIT:  psock_close() will attempt to use semaphores.
            * If we actually are in the IDLE thread, then could this cause
            * problems?  Probably not, if the task has exited and crefs is
            * zero, then there probably could not be a contender for the
            * semaphore.
            */

           for (ndx = 0; ndx < CONFIG_NSOCKET_DESCRIPTORS; ndx++)
             {
               FAR struct socket *psock = &list->sl_sockets[ndx];
               if (psock->s_crefs > 0)
                 {
                   (void)psock_close(psock);
                 }
             }

             /* Destroy the semaphore and release the filelist */

             (void)sem_destroy(&list->sl_sem);
             sched_free(list);
         }
    }
  return OK;
}

int sockfd_allocate(int minsd)
{
  FAR struct socketlist *list;
  int i;

  /* Get the socket list for this task/thread */

  list = sched_getsockets();
  if (list)
    {
      /* Search for a socket structure with no references */

      _net_semtake(list);
      for (i = minsd; i < CONFIG_NSOCKET_DESCRIPTORS; i++)
        {
          /* Are there references on this socket? */

          if (!list->sl_sockets[i].s_crefs)
            {
              /* No take the reference and return the index + an offset
               * as the socket descriptor.
               */

               memset(&list->sl_sockets[i], 0, sizeof(struct socket));
               list->sl_sockets[i].s_crefs = 1;
               _net_semgive(list);
               return i + __SOCKFD_OFFSET;
            }
        }
      _net_semgive(list);
    }
 
  return ERROR;
}

void sock_release(FAR struct socket *psock)
{
#if CONFIG_DEBUG
  if (psock)
#endif
    {
      /* Take the list semaphore so that there will be no accesses
       * to this socket structure.
       */

      FAR struct socketlist *list = sched_getsockets();
      if (list)
        {
          /* Decrement the count if there the socket will persist
           * after this.
           */

          _net_semtake(list);
          if (psock && psock->s_crefs > 1)
            {
              psock->s_crefs--;
            }
          else
            {
              /* The socket will not persist... reset it */

              memset(psock, 0, sizeof(struct socket));
            }
          _net_semgive(list);
        }
    }
}

void sockfd_release(int sockfd)
{
  /* Get the socket structure for this sockfd */

  FAR struct socket *psock = sockfd_socket(sockfd);

  /* Get the socket structure for this sockfd */

  if (psock)
    {
      sock_release(psock);
    }
}

FAR struct socket *sockfd_socket(int sockfd)
{
  FAR struct socketlist *list;
  int ndx = sockfd - __SOCKFD_OFFSET;

  if (ndx >=0 && ndx < CONFIG_NSOCKET_DESCRIPTORS)
    {
      list = sched_getsockets();
      if (list)
        {
          return &list->sl_sockets[ndx];
        }
    }
  return NULL;
}

#endif /* CONFIG_NSOCKET_DESCRIPTORS */
#endif /* CONFIG_NET */
