/****************************************************************************
 * net/net_monitor.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void connection_event(struct uip_conn *conn, uint16_t flags);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: connection_event
 *
 * Description:
 *   Some connection related event has occurred
 *
 * Parameters:
 *   conn     The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static void connection_event(FAR struct uip_conn *conn, uint16_t flags)
{
  FAR struct socket *psock = (FAR struct socket *)conn->connection_private;

  if (psock)
    {
      nllvdbg("flags: %04x s_flags: %02x\n", flags, psock->s_flags);

      /* UIP_CLOSE, UIP_ABORT, or UIP_TIMEDOUT: Loss-of-connection events */

      if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
        {
          net_lostconnection(psock, flags);
        }

      /* UIP_CONNECTED: The socket is successfully connected */

      else if ((flags & UIP_CONNECTED) != 0)
        {
          /* Indicate that the socket is now connected */

          psock->s_flags |= _SF_CONNECTED;
          psock->s_flags &= ~_SF_CLOSED;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: net_startmonitor
 *
 * Description:
 *   Set up to receive TCP connection state changes for a given socket
 *
 * Input Parameters:
 *   psock - The socket of interest
 *
 * Returned Value:
 *   For now, this function always returns OK.
 *
 ****************************************************************************/

int net_startmonitor(FAR struct socket *psock)
{
  FAR struct uip_conn *conn = psock->s_conn;

  DEBUGASSERT(psock && conn);

  /* Set up to receive callbacks on connection-related events */

  conn->connection_private = (void*)psock;
  conn->connection_event   = connection_event;
  return OK;
}

/****************************************************************************
 * Name: net_stopmonitor
 *
 * Description:
 *   Stop monitoring TCP connection changes for a given socket
 *
 * Input Parameters:
 *   conn - The TCP connection of interest
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_stopmonitor(FAR struct uip_conn *conn)
{
  DEBUGASSERT(conn);

  conn->connection_private = NULL;
  conn->connection_event   = NULL;
}

/****************************************************************************
 * Name: net_lostconnection
 *
 * Description:
 *   Called when a loss-of-connection event has occurred.
 *
 * Parameters:
 *   psock    The TCP socket structure associated.
 *   flags    Set of connection events events
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

void net_lostconnection(FAR struct socket *psock, uint16_t flags)
{
  DEBUGASSERT(psock)

  /* These loss-of-connection events may be reported:
   *
   *   UIP_CLOSE: The remote host has closed the connection
   *   UIP_ABORT: The remote host has aborted the connection
   *   UIP_TIMEDOUT: Connection aborted due to too many retransmissions.
   *
   * And we need to set these two socket status bits appropriately:
   *
   *  _SF_CONNECTED==1 && _SF_CLOSED==0 - the socket is connected
   *  _SF_CONNECTED==0 && _SF_CLOSED==1 - the socket was gracefully disconnected
   *  _SF_CONNECTED==0 && _SF_CLOSED==0 - the socket was rudely disconnected
   */

  if ((flags & UIP_CLOSE) != 0)
    {
      /* The peer gracefully closed the connection.  Marking the
       * connection as disconnected will suppress some subsequent
       * ENOTCONN errors from receive.  A graceful disconnection is
       * not handle as an error but as an "end-of-file"
       */

      psock->s_flags &= ~_SF_CONNECTED;
      psock->s_flags |= _SF_CLOSED;
    }
  else if ((flags & (UIP_ABORT|UIP_TIMEDOUT)) != 0)
    {
      /* The loss of connection was less than graceful.  This will (eventually)
       * be reported as an ENOTCONN error.
       */

      psock->s_flags &= ~(_SF_CONNECTED |_SF_CLOSED);
    }
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
