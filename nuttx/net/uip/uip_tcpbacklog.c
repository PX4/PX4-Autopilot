/****************************************************************************
 * net/uip/uip_tcpbacklog.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/uip/uipopt.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCPBACKLOG)

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-tcp.h>

#include "uip_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: uip_backlogcreate
 *
 * Description:
 *   Called from the listen() logic to setup the backlog as specified in the
 *   the listen arguments.
 *
 * Assumptions:
 *   Called from normal user code. Interrupts may be disabled.
 *
 ****************************************************************************/

int uip_backlogcreate(FAR struct uip_conn *conn, int nblg)
{
  FAR struct uip_backlog_s     *bls = NULL;
  FAR struct uip_blcontainer_s *blc;
  uip_lock_t flags;
  int size;
  int offset;
  int i;

  nllvdbg("conn=%p nblg=%d\n", conn, nblg);

#ifdef CONFIG_DEBUG
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  /* Then allocate the backlog as requested */

  if (nblg > 0)
    {
      /* Align the list of backlog structures to 32-bit boundaries.  This
       * may be excessive on 24-16-bit address machines; and insufficient
       * on 64-bit address machines -- REVISIT
       */

      offset = (sizeof(struct uip_backlog_s) + 3) & ~3;

      /* Then determine the full size of the allocation include the
       * uip_backlog_s, a pre-allocated array of struct uip_blcontainer_s
       * and alignement padding
       */

      size = offset + nblg * sizeof(struct uip_blcontainer_s);

      /* Then allocate that much */

      bls = (FAR struct uip_backlog_s *)zalloc(size);
      if (!bls)
        {
          nlldbg("Failed to allocate backlog\n");
          return -ENOMEM;
        }

      /* Then add all of the pre-allocated containers to the free list */

      blc = (FAR struct uip_blcontainer_s*)(((FAR uint8_t*)bls) + offset);
      for (i = 0; i < nblg; i++)
        {
          sq_addfirst(&blc->bc_node, &bls->bl_free);
        }
    }

  /* Destroy any existing backlog (shouldn't be any) */

  flags = uip_lock();
  uip_backlogdestroy(conn);

  /* Now install the backlog tear-off in the connection.  NOTE that bls may
   * actually be NULL if nblg is <= 0;  In that case, we are disabling backlog
   * support.  Since interrupts are disabled, destroying the old backlog and
   * replace it with the new is an atomic operation
   */

  conn->backlog = bls;
  uip_unlock(flags);
  return OK;
}

/****************************************************************************
 * Function: uip_backlogdestroy
 *
 * Description:
 *   (1) Called from uip_tcpfree() whenever a connection is freed.
 *   (2) Called from uip_backlogcreate() to destroy any old backlog
 *
 *   NOTE: This function may re-enter uip_tcpfree when a connection that
 *   is freed that has pending connections.
 *
 * Assumptions:
 *   The caller has disabled interrupts so that there can be no conflict
 *   with ongoing, interrupt driven activity
 *
 ****************************************************************************/

int uip_backlogdestroy(FAR struct uip_conn *conn)
{
  FAR struct uip_backlog_s     *blg;
  FAR struct uip_blcontainer_s *blc;
  FAR struct uip_conn          *blconn;

  nllvdbg("conn=%p\n", conn);

#ifdef CONFIG_DEBUG
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  /* Make sure that the connection has a backlog to be destroyed */

  if (conn->backlog)
    {
       /* Remove the backlog structure reference from the connection */

       blg           = conn->backlog;
       conn->backlog = NULL;

       /* Handle any pending connections in the backlog */

       while ((blc = (FAR struct uip_blcontainer_s*)sq_remfirst(&blg->bl_pending)) != NULL)
         {
           blconn = blc->bc_conn;
           if (blconn)
             {
               /* REVISIT -- such connections really need to be gracefully closed */

               blconn->blparent = NULL;
               blconn->backlog  = NULL;
               blconn->crefs    = 0;
               uip_tcpfree(blconn);
             }
         }

       /* Then free the entire backlog structure */

       free(blg);
    }

  return OK;
}

/****************************************************************************
 * Function: uip_backlogadd
 *
 * Description:
 *  Called uip_listen when a new connection is made with a listener socket
 *  but when there is no accept() in place to receive the connection.  This
 *  function adds the new connection to the backlog.
 *
 * Assumptions:
 *   Called from the interrupt level with interrupts disabled
 *
 ****************************************************************************/

int uip_backlogadd(FAR struct uip_conn *conn, FAR struct uip_conn *blconn)
{
  FAR struct uip_backlog_s     *bls;
  FAR struct uip_blcontainer_s *blc;
  int ret = -EINVAL;

  nllvdbg("conn=%p blconn=%p\n", conn, blconn);

#ifdef CONFIG_DEBUG
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  bls = conn->backlog;
  if (bls && blconn)
    {
       /* Allocate a container for the connection from the free list */

       blc = (FAR struct uip_blcontainer_s *)sq_remfirst(&bls->bl_free);
       if (!blc)
         {
           nlldbg("Failed to allocate container\n");
           ret = -ENOMEM;
         }
       else
         {
           /* Save the connection reference in the container and put the
            * container at the end of the pending connection list (FIFO).
            */

           blc->bc_conn = blconn;
           sq_addlast(&blc->bc_node, &bls->bl_pending);
           ret = OK;
         }
    }
  return ret;
}

/****************************************************************************
 * Function: uip_backlogremove
 *
 * Description:
 *  Called from poll().  Before waiting for a new connection, poll will
 *  call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   Called from normal user code, but with interrupts disabled,
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
bool uip_backlogavailable(FAR struct uip_conn *conn)
{
  return (conn && conn->backlog && !sq_empty(&conn->backlog->bl_pending));
}
#endif

/****************************************************************************
 * Function: uip_backlogremove
 *
 * Description:
 *  Called from accept().  Before waiting for a new connection, accept will
 *  call this API to see if there are pending connections in the backlog.
 *
 * Assumptions:
 *   Called from normal user code, but with interrupts disabled,
 *
 ****************************************************************************/

struct uip_conn *uip_backlogremove(FAR struct uip_conn *conn)
{
  FAR struct uip_backlog_s     *bls;
  FAR struct uip_blcontainer_s *blc;
  FAR struct uip_conn          *blconn = NULL;

#ifdef CONFIG_DEBUG
  if (!conn)
    {
      return NULL;
    }
#endif

  bls = conn->backlog;
  if (bls)
    {
       /* Remove the a container at the head of the pending connection list
        * (FIFO)
        */

       blc = (FAR struct uip_blcontainer_s *)sq_remfirst(&bls->bl_pending);
       if (blc)
         {
           /* Extract the connection reference from the container and put
            * container in the free list
            */

           blconn       = blc->bc_conn;
           blc->bc_conn = NULL;
           sq_addlast(&blc->bc_node, &bls->bl_free);
         }
    }

  nllvdbg("conn=%p, returning %p\n", conn, blconn);
  return blconn;
}

/****************************************************************************
 * Function: uip_backlogdelete
 *
 * Description:
 *  Called from uip_tcpfree() when a connection is freed that this also
 *  retained in the pending connectino list of a listener.  We simply need
 *  to remove the defunct connecton from the list.
 *
 * Assumptions:
 *   Called from the interrupt level with interrupts disabled
 *
 ****************************************************************************/

int uip_backlogdelete(FAR struct uip_conn *conn, FAR struct uip_conn *blconn)
{
  FAR struct uip_backlog_s     *bls;
  FAR struct uip_blcontainer_s *blc;
  FAR struct uip_blcontainer_s *prev;

  nllvdbg("conn=%p blconn=%p\n", conn, blconn);

#ifdef CONFIG_DEBUG
  if (!conn)
    {
      return -EINVAL;
    }
#endif

  bls = conn->backlog;
  if (bls)
    {
       /* Find the container hold the connection */

       for (blc = (FAR struct uip_blcontainer_s *)sq_peek(&bls->bl_pending), prev = NULL;
            blc;
            prev = blc, blc = (FAR struct uip_blcontainer_s *)sq_next(&blc->bc_node))
         {
            if (blc->bc_conn == blconn)
              {
                if (prev)
                  {
                    /* Remove the a container from the middle of the list of
                     * pending connections
                      */

                    (void)sq_remafter(&prev->bc_node, &bls->bl_pending);
                  }
                else
                  {
                    /* Remove the a container from the head of the list of
                     * pending connections
                     */

                    (void)sq_remfirst(&bls->bl_pending);
                  }

                /* Put container in the free list */

                blc->bc_conn = NULL;
                sq_addlast(&blc->bc_node, &bls->bl_free);
                return OK;
              }
          }

        nlldbg("Failed to find pending connection\n");
        return -EINVAL;
    }
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP && CONFIG_NET_TCPBACKLOG */
