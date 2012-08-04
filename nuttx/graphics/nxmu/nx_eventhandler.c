/****************************************************************************
 * graphics/nxmu/nx_eventhandler.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <mqueue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include "nxfe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_connected
 *
 * Description:
 *   The server has completed the connection and is ready.
 *
 ****************************************************************************/

static inline void nx_connected(FAR struct nxfe_conn_s *conn)
{
  DEBUGASSERT(conn->state == NX_CLISTATE_NOTCONNECTED);
  conn->state = NX_CLISTATE_CONNECTED;
}

/****************************************************************************
 * Name: nx_disconnected
 ****************************************************************************/

static inline void nx_disconnected(FAR struct nxfe_conn_s *conn)
{
  /* Close the server and client MQs */

  (void)mq_close(conn->cwrmq);
  (void)mq_close(conn->crdmq);

  /* And free the client structure */

  free(conn);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_eventhandler
 *
 * Description:
 *   The client code must call this function periodically to process
 *   incoming messages from the server.  If CONFIG_NX_BLOCKING is defined,
 *   then this function not return until a server message is received.
 *
 *   When CONFIG_NX_BLOCKING is not defined, the client must exercise
 *   caution in the looping to assure that it does not eat up all of
 *   the CPU bandwidth calling nx_eventhandler repeatedly.  nx_eventnotify
 *   may be called to get a signal event whenever a new incoming server
 *   event is avaiable.
 *
 * Input Parameters:
 *   handle - the handle returned by nx_connect
 *
 * Return:
 *     OK: No errors occurred.  If CONFIG_NX_BLOCKING is defined, then
 *         one or more server message was processed.
 *  ERROR: An error occurred and errno has been set appropriately.  Of
 *         particular interest, it will return errno == EHOSTDOWN when the
 *         server is disconnected.  After that event, the handle can no
 *         longer be used.
 *
 ****************************************************************************/

int nx_eventhandler(NXHANDLE handle)
{
  FAR struct nxfe_conn_s *conn = (FAR struct nxfe_conn_s *)handle;
  struct nxsvrmsg_s      *msg;
  struct nxbe_window_s   *wnd;
  uint8_t                 buffer[NX_MXCLIMSGLEN];
  int                     nbytes;

  /* Get the next message from our incoming message queue */

  do
    {
      nbytes = mq_receive(conn->crdmq, buffer, NX_MXCLIMSGLEN, 0);
      if (nbytes < 0)
        {
          /* EINTR is not an error.  The wait was interrupted by a signal and
           * we just need to try reading again.
           */

          if (errno != EINTR)
            {
              if (errno == EAGAIN)
                {
                  /* EAGAIN is not an error.  It occurs because the MQ is opened with
                   * O_NONBLOCK and there is no message available now.
                   */

                  return OK;
                }
              else
                {
                  gdbg("mq_receive failed: %d\n", errno);
                  return ERROR;
                }
            }
        }
    }
  while (nbytes < 0);

  DEBUGASSERT(nbytes >= sizeof(struct nxclimsg_s));

  /* Dispatch the message appropriately */

  msg = (struct nxsvrmsg_s *)buffer;
  gvdbg("Received msgid=%d\n", msg->msgid);
  switch (msg->msgid)
    {
    case NX_CLIMSG_CONNECTED:
      nx_connected(conn);
      break;

    case NX_CLIMSG_DISCONNECTED:
      nx_disconnected(conn);
      errno = EHOSTDOWN;
      return ERROR;

    case NX_CLIMSG_REDRAW:
      {
        FAR struct nxclimsg_redraw_s *redraw = (FAR struct nxclimsg_redraw_s *)buffer;
        wnd = redraw->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->redraw)
          {
            wnd->cb->redraw((NXWINDOW)wnd, &redraw->rect, redraw->more, wnd->arg);
          }
      }
      break;

    case NX_CLIMSG_NEWPOSITION:
      {
        FAR struct nxclimsg_newposition_s *postn = (FAR struct nxclimsg_newposition_s *)buffer;
        wnd = postn->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->position)
          {
            wnd->cb->position((NXWINDOW)wnd, &postn->size, &postn->pos, &postn->bounds, wnd->arg);
          }
      }
      break;

#ifdef CONFIG_NX_MOUSE
    case NX_CLIMSG_MOUSEIN:
      {
        FAR struct nxclimsg_mousein_s *mouse = (FAR struct nxclimsg_mousein_s *)buffer;
        wnd = mouse->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->mousein)
          {
            wnd->cb->mousein((NXWINDOW)wnd, &mouse->pos, mouse->buttons, wnd->arg);
          }
        }
      break;
#endif

#ifdef CONFIG_NX_KBD
    case NX_CLIMSG_KBDIN:
      {
        FAR struct nxclimsg_kbdin_s *kbd = (FAR struct nxclimsg_kbdin_s *)buffer;
        wnd = kbd->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->kbdin)
          {
            wnd->cb->kbdin((NXWINDOW)wnd, kbd->nch, kbd->ch, wnd->arg);
          }
        }
      break;
#endif

    case NX_CLIMSG_BLOCKED:
      {
        FAR struct nxclimsg_blocked_s *blocked = (FAR struct nxclimsg_blocked_s *)buffer;
        wnd = blocked->wnd;
        DEBUGASSERT(wnd);
        if (wnd->cb->blocked)
          {
            wnd->cb->blocked((NXWINDOW)wnd, wnd->arg, blocked->arg);
          }
        }
      break;

    default:
      gdbg("Unrecognized message opcode: %d\n", ((FAR struct nxsvrmsg_s *)buffer)->msgid);
      break;
    }

  return OK;
}

