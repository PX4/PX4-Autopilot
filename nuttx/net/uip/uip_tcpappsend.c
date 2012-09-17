/****************************************************************************
 * net/uip/uip_tcpappsend.c
 *
 *   Copyright (C) 2007-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>

#include "uip_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpappsend
 *
 * Description:
 *   Handle application or TCP protocol response.  If this function is called
 *   with dev->d_sndlen > 0, then this is an application attempting to send
 *   packet.
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpappsend(struct uip_driver_s *dev, struct uip_conn *conn,
                    uint16_t result)
{
  /* Handle the result based on the application response */

  nllvdbg("result: %04x d_sndlen: %d conn->unacked: %d\n",
          result, dev->d_sndlen, conn->unacked);

  /* Check for connection aborted */

  if ((result & UIP_ABORT) != 0)
    {
      dev->d_sndlen = 0;
      conn->tcpstateflags = UIP_CLOSED;
      nllvdbg("TCP state: UIP_CLOSED\n");

      uip_tcpsend(dev, conn, TCP_RST | TCP_ACK, UIP_IPTCPH_LEN);
    }

  /* Check for connection closed */

  else if ((result & UIP_CLOSE) != 0)
    {
      conn->tcpstateflags = UIP_FIN_WAIT_1;
      conn->unacked  = 1;
      conn->nrtx     = 0;
      nllvdbg("TCP state: UIP_FIN_WAIT_1\n");

      dev->d_sndlen  = 0;
      uip_tcpsend(dev, conn, TCP_FIN | TCP_ACK, UIP_IPTCPH_LEN);
    }

  /* None of the above */

  else
    {
      /* If d_sndlen > 0, the application has data to be sent. */

      if (dev->d_sndlen > 0)
        {
          /* Remember how much data we send out now so that we know
           * when everything has been acknowledged.  Just increment the amount
           * of data sent.  This will be needed in sequence number calculations
           * and we know that this is not a re-tranmission.  Retransmissions
           * do not go through this path.
           */

          conn->unacked += dev->d_sndlen;

          /* The application cannot send more than what is allowed by the
           * MSS (the minumum of the MSS and the available window).
           */

          DEBUGASSERT(dev->d_sndlen <= conn->mss);
        }

      /* Then handle the rest of the operation just as for the rexmit case */

      conn->nrtx = 0;
      uip_tcprexmit(dev, conn, result);
    }
}

/****************************************************************************
 * Name: uip_tcprexmit
 *
 * Description:
 *   Handle application retransmission
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   result - App result event sent
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcprexmit(struct uip_driver_s *dev, struct uip_conn *conn,
                   uint16_t result)
{
  nllvdbg("result: %04x d_sndlen: %d conn->unacked: %d\n",
          result, dev->d_sndlen, conn->unacked);

  dev->d_appdata = dev->d_snddata;

  /* If the application has data to be sent, or if the incoming packet had
   * new data in it, we must send out a packet.
   */

  if (dev->d_sndlen > 0 && conn->unacked > 0)
    {
      /* We always set the ACK flag in response packets adding the length of
       * the IP and TCP headers.
       */

      uip_tcpsend(dev, conn, TCP_ACK | TCP_PSH, dev->d_sndlen + UIP_TCPIP_HLEN);
    }

  /* If there is no data to send, just send out a pure ACK if one is requested`. */

  else if ((result & UIP_SNDACK) != 0)
    {
      uip_tcpsend(dev, conn, TCP_ACK, UIP_TCPIP_HLEN);
    }

  /* There is nothing to do -- drop the packet */

  else
    {
      dev->d_len = 0;
    }
}
#endif /* CONFIG_NET && CONFIG_NET_TCP */
