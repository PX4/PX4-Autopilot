/****************************************************************************
 * net/recvfrom.c
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/clock.h>
#include <nuttx/net/uip/uip-arch.h>

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define UDPBUF ((struct uip_udpip_hdr *)&dev->d_buf[UIP_LLH_LEN])
#define TCPBUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
struct recvfrom_s
{
  FAR struct socket         *rf_sock;      /* The parent socket structure */
#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  uint32_t                   rf_starttime; /* rcv start time for determining timeout */
#endif
  FAR struct uip_callback_s *rf_cb;        /* Reference to callback instance */
  sem_t                      rf_sem;       /* Semaphore signals recv completion */
  size_t                     rf_buflen;    /* Length of receive buffer */
  char                      *rf_buffer;    /* Pointer to receive buffer */
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6   *rf_from;      /* Address of sender */
#else
  FAR struct sockaddr_in    *rf_from;      /* Address of sender */
#endif
  size_t                     rf_recvlen;   /* The received length */
  int                        rf_result;    /* Success:OK, failure:negated errno */
};
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   The number of bytes taken from the packet.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
static size_t recvfrom_newdata(FAR struct uip_driver_s *dev,
                               FAR struct recvfrom_s *pstate)
{
  size_t recvlen;

  /* Get the length of the data to return */

  if (dev->d_len > pstate->rf_buflen)
    {
      recvlen = pstate->rf_buflen;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new appdata into the user buffer */

  memcpy(pstate->rf_buffer, dev->d_appdata, recvlen);
  nllvdbg("Received %d bytes (of %d)\n", (int)recvlen, (int)dev->d_len);

  /* Update the accumulated size of the data read */

  pstate->rf_recvlen += recvlen;
  pstate->rf_buffer  += recvlen;
  pstate->rf_buflen  -= recvlen;

  return recvlen;
}
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_newtcpdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void recvfrom_newtcpdata(FAR struct uip_driver_s *dev,
                                       FAR struct recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  size_t recvlen = recvfrom_newdata(dev, pstate);

  /* If there is more data left in the packet that we could not buffer, than
   * add it to the read-ahead buffers.
   */

 if (recvlen < dev->d_len)
   {
#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
      FAR struct uip_conn *conn   = (FAR struct uip_conn *)pstate->rf_sock->s_conn;
      FAR uint8_t         *buffer = (FAR uint8_t *)dev->d_appdata + recvlen;
      uint16_t             buflen = dev->d_len - recvlen;
      uint16_t             nsaved;

      nsaved = uip_datahandler(conn, buffer, buflen);

      /* There are complicated buffering issues that are not addressed fully
       * here.  For example, what if up_datahandler() cannot buffer the
       * remainder of the packet?  In that case, the data will be dropped but
       * still ACKed.  Therefore it would not be resent.
       * 
       * This is probably not an issue here because we only get here if the
       * read-ahead buffers are empty and there would have to be something
       * serioulsy wrong with the configuration not to be able to buffer a
       * partial packet in this context.
       */

#ifdef CONFIG_DEBUG_NET
      if (nsaved < buflen)
        {
          ndbg("ERROR: packet data not saved (%d bytes)\n", buflen - nsaved);
        }
#endif
#else
      ndbg("ERROR: packet data lost (%d bytes)\n", dev->d_len - recvlen);
#endif
   }

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_newudpdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static inline void recvfrom_newudpdata(FAR struct uip_driver_s *dev,
                                       FAR struct recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  (void)recvfrom_newdata(dev, pstate);

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_readahead
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_TCP) && CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
static inline void recvfrom_readahead(struct recvfrom_s *pstate)
{
  FAR struct uip_conn        *conn = (FAR struct uip_conn *)pstate->rf_sock->s_conn;
  FAR struct uip_readahead_s *readahead;
  size_t                      recvlen;

  /* Check there is any TCP data already buffered in a read-ahead
   * buffer.
   */

  do
    {
      /* Get the read-ahead buffer at the head of the list (if any) */

      readahead = (struct uip_readahead_s *)sq_remfirst(&conn->readahead);
      if (readahead)
        {
          /* We have a new buffer... transfer that buffered data into
           * the user buffer.
           *
           * First, get the length of the data to transfer.
           */

          if (readahead->rh_nbytes > pstate->rf_buflen)
            {
              recvlen = pstate->rf_buflen;
            }
          else
            {
              recvlen = readahead->rh_nbytes;
            }

          if (recvlen > 0)
            {
              /* Copy the read-ahead data into the user buffer */

              memcpy(pstate->rf_buffer, readahead->rh_buffer, recvlen);
              nllvdbg("Received %d bytes (of %d)\n", recvlen, readahead->rh_nbytes);

              /* Update the accumulated size of the data read */

              pstate->rf_recvlen += recvlen;
              pstate->rf_buffer  += recvlen;
              pstate->rf_buflen  -= recvlen;
            }

          /* If the read-ahead buffer is empty, then release it.  If not, then
           * we will have to move the data down and return the buffer to the
           * front of the list.
           */

          if (recvlen < readahead->rh_nbytes)
            {
              readahead->rh_nbytes -= recvlen;
              memcpy(readahead->rh_buffer, &readahead->rh_buffer[recvlen],
                     readahead->rh_nbytes);
              sq_addfirst(&readahead->rh_node, &conn->readahead);
            }
          else
            {
              uip_tcpreadaheadrelease(readahead);
            }
        }
    }
  while (readahead && pstate->rf_buflen > 0);
}
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_timeout
 *
 * Description:
 *   Check for recvfrom timeout.
 *
 * Parameters:
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
static int recvfrom_timeout(struct recvfrom_s *pstate)
{
  FAR struct socket *psock = 0;
  socktimeo_t        timeo = 0;

  /* Check for a timeout configured via setsockopts(SO_RCVTIMEO). If none...
   * we well let the read hang forever (except for the special case below).
   */

  /* Get the socket reference from the private data */

  psock = pstate->rf_sock;
  if (psock)
    {
      /* Recover the timeout value (zero if no timeout) */

      timeo = psock->s_rcvtimeo;
    }

  /* Use a fixed, configurable delay under the following circumstances:
   *
   * 1) This delay function has been enabled with CONFIG_NET_TCP_RECVDELAY > 0
   * 2) Some data has already been received from the socket.  Since this can
   *    only be true for a TCP/IP socket, this logic applies only to TCP/IP
   *    sockets.  And either
   * 3) There is no configured receive timeout, or
   * 4) The configured receive timeout is greater than than the delay
   */

#if CONFIG_NET_TCP_RECVDELAY > 0
  if ((timeo == 0 || timeo > CONFIG_NET_TCP_RECVDELAY) &&
      pstate->rf_recvlen > 0)
    {
      /* Use the configured timeout */

      timeo = CONFIG_NET_TCP_RECVDELAY;
    }
#endif

  /* Is there an effective timeout? */

  if (timeo)
    {
      /* Yes.. Check if the timeout has elapsed */

      return net_timeo(pstate->rf_starttime, timeo);
    }

  /* No timeout -- hang forever waiting for data. */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_tcpsender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void recvfrom_tcpsender(struct uip_driver_s *dev, struct recvfrom_s *pstate)
{
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *infrom = pstate->rf_from;
#else
  FAR struct sockaddr_in *infrom  = pstate->rf_from;
#endif

  if (infrom)
    {
      infrom->sin_family = AF_INET;
      infrom->sin_port   = TCPBUF->srcport;

#ifdef CONFIG_NET_IPv6
      uip_ipaddr_copy(infrom->sin6_addr.s6_addr, TCPBUF->srcipaddr);
#else
      uip_ipaddr_copy(infrom->sin_addr.s_addr, uip_ip4addr_conv(TCPBUF->srcipaddr));
#endif
    }
}
#endif

/****************************************************************************
 * Function: recvfrom_tcpinterrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   TCP receive operation via by the uIP layer.
 *
 * Parameters:
 *   dev      The structure of the network driver that caused the interrupt
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

#ifdef CONFIG_NET_TCP
static uint16_t recvfrom_tcpinterrupt(FAR struct uip_driver_s *dev,
                                      FAR void *conn, FAR void *pvpriv,
                                      uint16_t flags)
{
  FAR struct recvfrom_s *pstate = (struct recvfrom_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If new data is available, then complete the read action. */

      if ((flags & UIP_NEWDATA) != 0)
        {
          /* Copy the data from the packet (saving any unused bytes from the
           * packet in the read-ahead buffer).
           */

          recvfrom_newtcpdata(dev, pstate);

          /* Save the sender's address in the caller's 'from' location */

          recvfrom_tcpsender(dev, pstate);

          /* Indicate that the data has been consumed and that an ACK
           * should be sent.
           */

          flags = (flags & ~UIP_NEWDATA) | UIP_SNDACK;

          /* Check for transfer complete.  We will consider the transfer
           * complete in own of two different ways, depending on the setting
           * of CONFIG_NET_TCP_RECVDELAY.
           *
           * 1) If CONFIG_NET_TCP_RECVDELAY == 0 then we will consider the
           *    TCP/IP transfer complete as soon as any data has been received.
           *    This is safe because if any additional data is received, it
           *    will be retained inthe TCP/IP read-ahead buffer until the
           *    next receive is performed.
           * 2) CONFIG_NET_TCP_RECVDELAY > 0 may be set to wait a little
           *    bit to determine if more data will be received.  You might
           *    do this if read-ahead buffereing is disabled and we want to
           *    minimize the loss of back-to-back packets.  In this case,
           *    the transfer is complete when either a) the entire user buffer 
           *    is full or 2) when the receive timeout occurs (below).
           */

#if CONFIG_NET_TCP_RECVDELAY > 0
          if (pstate->rf_buflen == 0)
#else
          if (pstate->rf_recvlen > 0)
#endif
            {
              nllvdbg("TCP resume\n");

              /* The TCP receive buffer is full.  Return now and don't allow
               * any further TCP call backs.
               */

              pstate->rf_cb->flags   = 0;
              pstate->rf_cb->priv    = NULL;
              pstate->rf_cb->event   = NULL;

              /* Wake up the waiting thread, returning the number of bytes
               * actually read.
               */

              sem_post(&pstate->rf_sem);
            }

            /* Reset the timeout.  We will want a short timeout to terminate
             * the TCP receive.
             */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
            pstate->rf_starttime = clock_systimer();
#endif
        }

      /* Check for a loss of connection.
       *
       * UIP_CLOSE: The remote host has closed the connection
       * UIP_ABORT: The remote host has aborted the connection
       * UIP_TIMEDOUT: Connection aborted due to too many retransmissions.
       */

      else if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
        {
          nllvdbg("Lost connection\n");

          /* Stop further callbacks */

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;

          /* Handle loss-of-connection event */

          net_lostconnection(pstate->rf_sock, flags);

          /* Check if the peer gracefully closed the connection. */

          if ((flags & UIP_CLOSE) != 0)
            {
              /* This case should always return success (zero)! The value of
               * rf_recvlen, if zero, will indicate that the connection was
               * gracefully closed.
               */

              pstate->rf_result = 0;
            }
          else
            {
              /* If no data has been received, then return ENOTCONN.
               * Otherwise, let this return success.  The failure will
               * be reported the next time that recv[from]() is called.
               */

#if CONFIG_NET_TCP_RECVDELAY > 0
              if (pstate->rf_recvlen > 0)
                {
                  pstate->rf_result = 0;
                }
              else
                {
                  pstate->rf_result = -ENOTCONN;
                }
#else
              pstate->rf_result = -ENOTCONN;
#endif
            }

          /* Wake up the waiting thread */

          sem_post(&pstate->rf_sem);
        }

      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
      else if (recvfrom_timeout(pstate))
        {
          /* Yes.. the timeout has elapsed... do not allow any further
           * callbacks
           */

          nllvdbg("TCP timeout\n");

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;

          /* Report an error only if no data has been received. (If 
           * CONFIG_NET_TCP_RECVDELAY then rf_recvlen should always be
           * zero).
           */

#if CONFIG_NET_TCP_RECVDELAY > 0
          if (pstate->rf_recvlen == 0)
#endif
            {
              /* Report the timeout error */

              pstate->rf_result = -EAGAIN;
            }

          /* Wake up the waiting thread, returning either the error -EAGAIN
           * that signals the timeout event or the data received up to
           * the point tht the timeout occured (no error).
           */

          sem_post(&pstate->rf_sem);
        }
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */
    }
  return flags;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_udpsender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static inline void recvfrom_udpsender(struct uip_driver_s *dev, struct recvfrom_s *pstate)
{
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *infrom = pstate->rf_from;
#else
  FAR struct sockaddr_in *infrom  = pstate->rf_from;
#endif

  if (infrom)
    {
      infrom->sin_family = AF_INET;
      infrom->sin_port   = UDPBUF->srcport;

#ifdef CONFIG_NET_IPv6
      uip_ipaddr_copy(infrom->sin6_addr.s6_addr, UDPBUF->srcipaddr);
#else
      uip_ipaddr_copy(infrom->sin_addr.s_addr, uip_ip4addr_conv(UDPBUF->srcipaddr));
#endif
    }
}
#endif

/****************************************************************************
 * Function: recvfrom_udpinterrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   UDP receive operation via by the uIP layer.
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
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

#ifdef CONFIG_NET_UDP
static uint16_t recvfrom_udpinterrupt(struct uip_driver_s *dev, void *pvconn,
                                      void *pvpriv, uint16_t flags)
{
  struct recvfrom_s *pstate = (struct recvfrom_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If new data is available, then complete the read action. */

      if ((flags & UIP_NEWDATA) != 0)
        {
          /* Copy the data from the packet */

          recvfrom_newudpdata(dev, pstate);

          /* We are finished. */

          nllvdbg("UDP done\n");

          /* Don't allow any further UDP call backs. */

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;

          /* Save the sender's address in the caller's 'from' location */

          recvfrom_udpsender(dev, pstate);

         /* Indicate that the data has been consumed */

          flags &= ~UIP_NEWDATA;

           /* Wake up the waiting thread, returning the number of bytes
           * actually read.
           */

          sem_post(&pstate->rf_sem);
        }

      /* No data has been received -- this is some other event... probably a
       * poll -- check for a timeout.
       */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
      else if (recvfrom_timeout(pstate))
        {
          /* Yes.. the timeout has elapsed... do not allow any further
           * callbacks
           */

          nllvdbg("UDP timeout\n");

          /* Stop further callbacks */

          pstate->rf_cb->flags   = 0;
          pstate->rf_cb->priv    = NULL;
          pstate->rf_cb->event   = NULL;

          /* Report a timeout error */

          pstate->rf_result = -EAGAIN;

          /* Wake up the waiting thread */

          sem_post(&pstate->rf_sem);
        }
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */
    }
  return flags;
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: recvfrom_init
 *
 * Description:
 *   Initialize the state structure
 *
 * Parameters:
 *   psock    Pointer to the socket structure for the socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
static void recvfrom_init(FAR struct socket *psock, FAR void *buf, size_t len,
#ifdef CONFIG_NET_IPv6
                          FAR struct sockaddr_in6 *infrom,
#else
                          FAR struct sockaddr_in *infrom,
#endif
                          struct recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct recvfrom_s));
  (void)sem_init(&pstate->rf_sem, 0, 0); /* Doesn't really fail */
  pstate->rf_buflen    = len;
  pstate->rf_buffer    = buf;
  pstate->rf_from      = infrom;

  /* Set up the start time for the timeout */

  pstate->rf_sock      = psock;
#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  pstate->rf_starttime = clock_systimer();
#endif
}

/* The only uninitialization that has to be performed is destroying the
 * semaphore.
 */

#define recvfrom_uninit(s) sem_destroy(&(s)->rf_sem)

#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Parameters:
 *   result   The result of the uip_lockedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
static ssize_t recvfrom_result(int result, struct recvfrom_s *pstate)
{
  int save_errno = errno; /* In case something we do changes it */

  /* Check for a error/timeout detected by the interrupt handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->rf_result < 0)
    {
      /* This might return EAGAIN on a timeout or ENOTCONN on loss of
       * connection (TCP only)
       */

      return pstate->rf_result;
    }

  /* If uip_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, uip_lockedwait will have set errno appropriately.
   */

  if (result < 0)
    {
      return -save_errno;
    }

  return pstate->rf_recvlen;
}
#endif /* CONFIG_NET_UDP || CONFIG_NET_TCP */

/****************************************************************************
 * Function: udp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a UDP SOCK_DGRAM
 *
 * Parameters:
 *   psock    Pointer to the socket structure for the SOCK_DRAM socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   infrom   INET ddress of source (may be NULL)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
#ifdef CONFIG_NET_IPv6
static ssize_t udp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr_in6 *infrom )
#else
static ssize_t udp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr_in *infrom )
#endif
{
  struct uip_udp_conn *conn = (struct uip_udp_conn *)psock->s_conn;
  struct recvfrom_s    state;
  uip_lock_t           save;
  int                  ret;

  /* Perform the UDP recvfrom() operation */

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = uip_lock();
  recvfrom_init(psock, buf, len, infrom, &state);

  /* Setup the UDP remote connection */

  ret = uip_udpconnect(conn, NULL);
  if (ret < 0)
    {
      goto errout_with_state;
    }

  /* Set up the callback in the connection */

  state.rf_cb = uip_udpcallbackalloc(conn);
  if (state.rf_cb)
    {
      /* Set up the callback in the connection */

      state.rf_cb->flags   = UIP_NEWDATA|UIP_POLL;
      state.rf_cb->priv    = (void*)&state;
      state.rf_cb->event   = recvfrom_udpinterrupt;

      /* Enable the UDP socket */

      uip_udpenable(conn);

      /* Wait for either the receive to complete or for an error/timeout to occur.
       * NOTES:  (1) uip_lockedwait will also terminate if a signal is received, (2)
       * interrupts are disabled!  They will be re-enabled while the task sleeps
       * and automatically re-enabled when the task restarts.
       */

      ret = uip_lockedwait(&state. rf_sem);

      /* Make sure that no further interrupts are processed */

      uip_udpdisable(conn);
      uip_udpcallbackfree(conn, state.rf_cb);
      ret = recvfrom_result(ret, &state);
    }
  else
    {
      ret = -EBUSY;
    }

errout_with_state:
  uip_unlock(save);
  recvfrom_uninit(&state);
  return ret;
}
#endif /* CONFIG_NET_UDP */

/****************************************************************************
 * Function: tcp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a TCP/IP SOCK_STREAM
 *
 * Parameters:
 *   psock    Pointer to the socket structure for the SOCK_DRAM socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   infrom   INET ddress of source (may be NULL)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_IPv6
static ssize_t tcp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr_in6 *infrom )
#else
static ssize_t tcp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                            FAR struct sockaddr_in *infrom )
#endif
{
  struct recvfrom_s       state;
  uip_lock_t              save;
  int                     ret;

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save = uip_lock();
  recvfrom_init(psock, buf, len, infrom, &state);

  /* Handle any any TCP data already buffered in a read-ahead buffer.  NOTE
   * that there may be read-ahead data to be retrieved even after the
   * socket has been disconnected.
   */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  recvfrom_readahead(&state);

  /* The default return value is the number of bytes that we just copied into
   * the user buffer.  We will return this if the socket has become disconnected
   * or if the user request was completely satisfied with data from the readahead
   * buffers.
   */
   
  ret = state.rf_recvlen;

#else
  /* Otherwise, the default return value of zero is used (only for the case
   * where len == state.rf_buflen is zero).
   */

  ret = 0;
#endif

  /* Verify that the SOCK_STREAM has been and still is connected */

  if (!_SS_ISCONNECTED(psock->s_flags))
    {
      /* Was any data transferred from the readahead buffer after we were
       * disconnected?  If so, then return the number of bytes received.  We
       * will wait to return end disconnection indications the next time that
       * recvfrom() is called.
       *
       * If no data was received (i.e.,  ret == 0  -- it will not be negative)
       * and the connection was gracefully closed by the remote peer, then return
       * success.  If rf_recvlen is zero, the caller of recvfrom() will get an
       * end-of-file indication.
       */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
      if (ret <= 0 && !_SS_ISCLOSED(psock->s_flags))
#else
      if (!_SS_ISCLOSED(psock->s_flags))
#endif
        {
          /* Nothing was previously received from the readahead buffers.
           * The SOCK_STREAM must be (re-)connected in order to receive any
           * additional data.
           */

          ret = -ENOTCONN;
        }
    }

  /* In general, this uIP-based implementation will not support non-blocking
   * socket operations... except in a few cases:  Here for TCP receive with read-ahead
   * enabled.  If this socket is configured as non-blocking then return EAGAIN
   * if no data was obtained from the read-ahead buffers.
   */

  else
#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret <= 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
        }
    }

  /* It is okay to block if we need to.  If there is space to receive anything
   * more, then we will wait to receive the data.  Otherwise return the number
   * of bytes read from the read-ahead buffer (already in 'ret').
   */

  else
#endif

  /* We get here when we we decide that we need to setup the wait for incoming
   * TCP/IP data.  Just a few more conditions to check:
   *
   * 1) Make sure thet there is buffer space to receive additional data
   *    (state.rf_buflen > 0).  This could be zero, for example, if read-ahead
   *    buffering was enabled and we filled the user buffer with data from
   *    the read-ahead buffers.  Aand
   * 2) if read-ahead buffering is enabled (CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0)
   *    and delay logic is disabled (CONFIG_NET_TCP_RECVDELAY == 0), then we
   *    not want to wait if we already obtained some data from the read-ahead
   *    buffer.  In that case, return now with what we have (don't want for more
   *    because there may be no timeout).
   */

#if CONFIG_NET_TCP_RECVDELAY == 0 && CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  if (state.rf_recvlen == 0 && state.rf_buflen > 0)
#else
  if (state.rf_buflen > 0)
#endif
    {
      struct uip_conn *conn = (struct uip_conn *)psock->s_conn;

      /* Set up the callback in the connection */

      state.rf_cb = uip_tcpcallbackalloc(conn);
      if (state.rf_cb)
        {
          state.rf_cb->flags   = UIP_NEWDATA|UIP_POLL|UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT;
          state.rf_cb->priv    = (void*)&state;
          state.rf_cb->event   = recvfrom_tcpinterrupt;

          /* Wait for either the receive to complete or for an error/timeout to occur.
           * NOTES:  (1) uip_lockedwait will also terminate if a signal is received, (2)
           * interrupts may be disabled!  They will be re-enabled while the task sleeps
           * and automatically re-enabled when the task restarts.
           */

          ret = uip_lockedwait(&state.rf_sem);

          /* Make sure that no further interrupts are processed */

          uip_tcpcallbackfree(conn, state.rf_cb);
          ret = recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  uip_unlock(save);
  recvfrom_uninit(&state);
  return (ssize_t)ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Othwerwise, on errors, -1 is returned, and errno
 *   is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would block,
 *     or a receive timeout had been set and the timeout expired before data
 *     was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically because
 *     it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data were
 *     available.
 *   EINVAL
 *     Invalid argument passed.
 *   ENOMEM
 *     Could not allocate memory.
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                       int flags,FAR struct sockaddr *from,
                       FAR socklen_t *fromlen)
{
#if defined(CONFIG_NET_UDP) || defined(CONFIG_NET_TCP)
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *infrom = (struct sockaddr_in6 *)from;
#else
  FAR struct sockaddr_in *infrom = (struct sockaddr_in *)from;
#endif
#endif

  ssize_t ret;
  int err;

  /* Verify that non-NULL pointers were passed */

#ifdef CONFIG_DEBUG
  if (!buf)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* If a 'from' address has been provided, verify that it is large
   * enough to hold this address family.
   */

  if (from)
    {
#ifdef CONFIG_NET_IPv6
      if (*fromlen < sizeof(struct sockaddr_in6))
#else
      if (*fromlen < sizeof(struct sockaddr_in))
#endif
        {
          err = EINVAL;
          goto errout;
        }
    }

  /* Set the socket state to receiving */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_RECV);

  /* Perform the TCP/IP or UDP recv() operation */

#if defined(CONFIG_NET_UDP) && defined(CONFIG_NET_TCP)
  if (psock->s_type == SOCK_STREAM)
    {
      ret = tcp_recvfrom(psock, buf, len, infrom);
    }
  else
    {
      ret = udp_recvfrom(psock, buf, len, infrom);
    }
#elif defined(CONFIG_NET_TCP)
  ret = tcp_recvfrom(psock, buf, len, infrom);
#elif defined(CONFIG_NET_UDP)
  ret = udp_recvfrom(psock, buf, len, infrom);
#else
  ret = -ENOSYS;
#endif

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Handle returned errors */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Success return */

  return ret;

errout:
  errno = err;
  return ERROR;
}

/****************************************************************************
 * Function: recvfrom
 *
 * Description:
 *   recvfrom() receives messages from a socket, and may be used to receive
 *   data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN
 *     The socket is marked non-blocking and the receive operation would block,
 *     or a receive timeout had been set and the timeout expired before data
 *     was received.
 *   EBADF
 *     The argument sockfd is an invalid descriptor.
 *   ECONNREFUSED
 *     A remote host refused to allow the network connection (typically because
 *     it is not running the requested service).
 *   EFAULT
 *     The receive buffer pointer(s) point outside the process's address space.
 *   EINTR
 *     The receive was interrupted by delivery of a signal before any data were
 *     available.
 *   EINVAL
 *     Invalid argument passed.
 *   ENOMEM
 *     Could not allocate memory.
 *   ENOTCONN
 *     The socket is associated with a connection-oriented protocol and has
 *     not been connected.
 *   ENOTSOCK
 *     The argument sockfd does not refer to a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t recvfrom(int sockfd, FAR void *buf, size_t len, int flags,
                 FAR struct sockaddr *from, FAR socklen_t *fromlen)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* Then let psock_recvfrom() do all of the work */

  return psock_recvfrom(psock, buf, len, flags, from, fromlen);
}

#endif /* CONFIG_NET */
