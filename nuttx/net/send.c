/****************************************************************************
 * net/send.c
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

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/clock.h>
#include <nuttx/net/uip/uip-arch.h>

#ifdef CONFIG_NET_ARP_IPIN
#  include <nuttx/net/uip/uip-arp.h>
#endif

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#if defined(CONFIG_NET_TCP_SPLIT) && !defined(CONFIG_NET_TCP_SPLIT_SIZE)
#  define CONFIG_NET_TCP_SPLIT_SIZE 40
#endif

#define TCPBUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure holds the state of the send operation until it can be
 * operated upon from the interrupt level.
 */

struct send_s
{
  FAR struct socket         *snd_sock;    /* Points to the parent socket structure */
  FAR struct uip_callback_s *snd_cb;      /* Reference to callback instance */
  sem_t                      snd_sem;     /* Used to wake up the waiting thread */
  FAR const uint8_t         *snd_buffer;  /* Points to the buffer of data to send */
  size_t                     snd_buflen;  /* Number of bytes in the buffer to send */
  ssize_t                    snd_sent;    /* The number of bytes sent */
  uint32_t                   snd_isn;     /* Initial sequence number */
  uint32_t                   snd_acked;   /* The number of bytes acked */
#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  uint32_t                   snd_time;    /* last send time for determining timeout */
#endif
#if defined(CONFIG_NET_TCP_SPLIT)
  bool                       snd_odd;     /* True: Odd packet in pair transaction */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: send_timeout
 *
 * Description:
 *   Check for send timeout.
 *
 * Parameters:
 *   pstate   send state structure
 *
 * Returned Value:
 *   TRUE:timeout FALSE:no timeout
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
static inline int send_timeout(struct send_s *pstate)
{
  FAR struct socket *psock = 0;

  /* Check for a timeout configured via setsockopts(SO_SNDTIMEO).
   * If none... we well let the send wait forever.
   */

  psock = pstate->snd_sock;
  if (psock && psock->s_sndtimeo != 0)
    {
      /* Check if the configured timeout has elapsed */

      return net_timeo(pstate->snd_time, psock->s_sndtimeo);
    }

  /* No timeout */

  return FALSE;
}
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */

/****************************************************************************
 * Function: send_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the uIP layer.
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

static uint16_t send_interrupt(struct uip_driver_s *dev, void *pvconn,
                               void *pvpriv, uint16_t flags)
{
  struct uip_conn *conn = (struct uip_conn*)pvconn;
  struct send_s *pstate = (struct send_s *)pvpriv;

  nllvdbg("flags: %04x acked: %d sent: %d\n",
          flags, pstate->snd_acked, pstate->snd_sent);

  /* If this packet contains an acknowledgement, then update the count of
   * acknowldged bytes.
   */

  if ((flags & UIP_ACKDATA) != 0)
    {
      /* The current acknowledgement number number is the (relative) offset
       * of the of the next byte needed by the receiver.  The snd_isn is the
       * offset of the first byte to send to the receiver.  The difference
       * is the number of bytes to be acknowledged.
       */

      pstate->snd_acked = uip_tcpgetsequence(TCPBUF->ackno) - pstate->snd_isn;
      nllvdbg("ACK: acked=%d sent=%d buflen=%d\n",
              pstate->snd_acked, pstate->snd_sent, pstate->snd_buflen);

      /* Have all of the bytes in the buffer been sent and acknowledged? */

      if (pstate->snd_acked >= pstate->snd_buflen)
        {
          /* Yes.  Then pstate->snd_buflen should hold the number of bytes
           * actually sent.
           */

          goto end_wait;
        }

      /* No.. fall through to send more data if necessary */
    }

  /* Check if we are being asked to retransmit data */

  else if ((flags & UIP_REXMIT) != 0)
    {
      /* Yes.. in this case, reset the number of bytes that have been sent
       * to the number of bytes that have been ACKed.
       */

      pstate->snd_sent = pstate->snd_acked;

#if defined(CONFIG_NET_TCP_SPLIT)
      /* Reset the the even/odd indicator to even since we need to
       * retransmit.
       */

      pstate->snd_odd = false;
#endif

      /* Fall through to re-send data from the last that was ACKed */
    }

 /* Check for a loss of connection */

  else if ((flags & (UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT)) != 0)
    {
      /* Report not connected */

      nllvdbg("Lost connection\n");

      net_lostconnection(pstate->snd_sock, flags);
      pstate->snd_sent = -ENOTCONN;
      goto end_wait;
    }

   /* Check if the outgoing packet is available (it may have been claimed
    * by a sendto interrupt serving a different thread).
    */

#if 0 /* We can't really support multiple senders on the same TCP socket */
   else if (dev->d_sndlen > 0)
     {
       /* Another thread has beat us sending data, wait for the next poll */

         return flags;
      }
#endif

  /* We get here if (1) not all of the data has been ACKed, (2) we have been
   * asked to retransmit data, (3) the connection is still healthy, and (4)
   * the outgoing packet is available for our use.  In this case, we are
   * now free to send more data to receiver -- UNLESS the buffer contains
   * unprocessing incoming data.  In that event, we will have to wait for the
   * next polling cycle.
   */

  if ((flags & UIP_NEWDATA) == 0 && pstate->snd_sent < pstate->snd_buflen)
    {
      uint32_t seqno;

      /* Get the amount of data that we can send in the next packet */

      uint32_t sndlen = pstate->snd_buflen - pstate->snd_sent;


#if defined(CONFIG_NET_TCP_SPLIT)

      /* RFC 1122 states that a host may delay ACKing for up to 500ms but
       * must respond to every second  segment).  This logic here will trick
       * the RFC 1122 recipient into responding sooner.  This logic will be
       * activated if:
       *
       *   1. An even number of packets has been send (where zero is an even
       *      number),
       *   2. There is more data be sent (more than or equal to
       *      CONFIG_NET_TCP_SPLIT_SIZE), but
       *   3. Not enough data for two packets.
       *
       * Then we will split the remaining, single packet into two partial
       * packets.  This will stimulate the RFC 1122 peer to ACK sooner.
       *
       * Don't try to split very small packets (less than CONFIG_NET_TCP_SPLIT_SIZE).
       * Only the first even packet and the last odd packets could have
       * sndlen less than CONFIG_NET_TCP_SPLIT_SIZE.  The value of sndlen on
       * the last even packet is guaranteed to be at least MSS/2 by the
       * logic below.
       */

      if (sndlen >= CONFIG_NET_TCP_SPLIT_SIZE)
        {
          /* sndlen is the number of bytes remaining to be sent.
           * uip_mss(conn) will return the number of bytes that can sent
           * in one packet.  The difference, then, is the number of bytes
           * that would be sent in the next packet after this one.
           */

          int32_t next_sndlen = sndlen - uip_mss(conn);

          /*  Is this the even packet in the packet pair transaction? */

          if (!pstate->snd_odd)
            {
              /* next_sndlen <= 0 means that the entire remaining data
               * could fit into this single packet.  This is condition
               * in which we must do the split.
               */

              if (next_sndlen <= 0)
                {
                  /* Split so that there will be an odd packet.  Here
                   * we know that 0 < sndlen <= MSS
                   */

                  sndlen = (sndlen / 2) + 1;
                }
            }

          /* No... this is the odd packet in the packet pair transaction */

          else
            {
              /* Will there be another (even) packet afer this one?
               * (next_sndlen > 0)  Will the split conidition occur on that
               * next, even packet? ((next_sndlen - uip_mss(conn)) < 0) If
               * so, then perform the split now to avoid the case where the
               * byte count is less than CONFIG_NET_TCP_SPLIT_SIZE on the
               * next pair.
               */

              if (next_sndlen > 0 && (next_sndlen - uip_mss(conn)) < 0)
                {
                  /* Here, we know that sndlen must be MSS < sndlen <= 2*MSS
                   * and so (sndlen / 2) is <= MSS.
                   */

                  sndlen /= 2;
                }
            }
        }

      /* Toggle the even/odd indicator */

      pstate->snd_odd ^= true;

#endif /* CONFIG_NET_TCP_SPLIT */

      if (sndlen > uip_mss(conn))
        {
          sndlen = uip_mss(conn);
        }

      /* Set the sequence number for this packet.  NOTE:  uIP updates
       * sndseq on recept of ACK *before* this function is called.  In that
       * case sndseq will point to the next unacknowledge byte (which might
       * have already been sent).  We will overwrite the value of sndseq
       * here before the packet is sent.
       */

      seqno = pstate->snd_sent + pstate->snd_isn;
      nllvdbg("SEND: sndseq %08x->%08x\n", conn->sndseq, seqno);
      uip_tcpsetsequence(conn->sndseq, seqno);

      /* Then set-up to send that amount of data. (this won't actually
       * happen until the polling cycle completes).
       */

      uip_send(dev, &pstate->snd_buffer[pstate->snd_sent], sndlen);

      /* Check if the destination IP address is in the ARP table.  If not,
       * then the send won't actually make it out... it will be replaced with
       * an ARP request.
       *
       * NOTE 1: This could be an expensive check if there are a lot of entries
       * in the ARP table.  Hence, we only check on the first packet -- when
       * snd_sent is zero.
       *
       * NOTE 2: If we are actually harvesting IP addresses on incomming IP
       * packets, then this check should not be necessary; the MAC mapping
       * should already be in the ARP table.
       */

#if defined(CONFIG_NET_ETHERNET) && defined (CONFIG_NET_ARP_IPIN)
      if (pstate->snd_sent != 0 || uip_arp_find(conn->ripaddr) != NULL)
#endif
        {
          /* Update the amount of data sent (but not necessarily ACKed) */

          pstate->snd_sent += sndlen;
          nllvdbg("SEND: acked=%d sent=%d buflen=%d\n",
                  pstate->snd_acked, pstate->snd_sent, pstate->snd_buflen);

          /* Update the send time */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
          pstate->snd_time = clock_systimer();
#endif
        }
    }

  /* All data has been send and we are just waiting for ACK or re-transmit
   * indications to complete the send.  Check for a timeout.
   */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
  else if (send_timeout(pstate))
    {
      /* Yes.. report the timeout */

      nlldbg("SEND timeout\n");
      pstate->snd_sent = -ETIMEDOUT;
      goto end_wait;
    }
#endif /* CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */

  /* Continue waiting */

  return flags;

end_wait:
  /* Do not allow any further callbacks */

  pstate->snd_cb->flags   = 0;
  pstate->snd_cb->priv    = NULL;
  pstate->snd_cb->event   = NULL;

  /* There are no outstanding, unacknowledged bytes */

  conn->unacked           = 0;

  /* Wake up the waiting thread */

  sem_post(&pstate->snd_sem);
  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_send
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   psock    And instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_send(FAR struct socket *psock, const void *buf, size_t len, int flags)
{
  struct send_s state;
  uip_lock_t save;
  int err;
  int ret = OK;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* If this is an un-connected socket, then return ENOTCONN */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      err = ENOTCONN;
      goto errout;
    }

  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Perform the TCP send operation */

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save                = uip_lock();
  memset(&state, 0, sizeof(struct send_s));
  (void)sem_init(&state. snd_sem, 0, 0); /* Doesn't really fail */
  state.snd_sock      = psock;             /* Socket descriptor to use */
  state.snd_buflen    = len;               /* Number of bytes to send */
  state.snd_buffer    = buf;               /* Buffer to send from */

  if (len > 0)
    {
      struct uip_conn *conn = (struct uip_conn*)psock->s_conn;

      /* Allocate resources to receive a callback */

      state.snd_cb = uip_tcpcallbackalloc(conn);
      if (state.snd_cb)
        {
          /* Get the initial sequence number that will be used */

          state.snd_isn         = uip_tcpgetsequence(conn->sndseq);

          /* There is no outstanding, unacknowledged data after this
           * initial sequence number.
           */

          conn->unacked         = 0;

          /* Update the initial time for calculating timeouts */

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
          state.snd_time        = clock_systimer();
#endif
          /* Set up the callback in the connection */

          state.snd_cb->flags   = UIP_ACKDATA|UIP_REXMIT|UIP_POLL|UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT;
          state.snd_cb->priv    = (void*)&state;
          state.snd_cb->event   = send_interrupt;

          /* Notify the device driver of the availaibilty of TX data */

          netdev_txnotify(&conn->ripaddr);

          /* Wait for the send to complete or an error to occur:  NOTES: (1)
           * uip_lockedwait will also terminate if a signal is received, (2) interrupts
           * may be disabled!  They will be re-enabled while the task sleeps and
           * automatically re-enabled when the task restarts.
           */

          ret = uip_lockedwait(&state. snd_sem);

          /* Make sure that no further interrupts are processed */

          uip_tcpcallbackfree(conn, state.snd_cb);
        }
    }

  sem_destroy(&state. snd_sem);
  uip_unlock(save);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for a errors.  Errors are signaled by negative errno values
   * for the send length
   */

  if (state.snd_sent < 0)
    {
      err = state.snd_sent;
      goto errout;
    }

  /* If uip_lockedwait failed, then we were probably reawakened by a signal. In
   * this case, uip_lockedwait will have set errno appropriately.
   */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  /* Return the number of bytes actually sent */

  return state.snd_sent;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

/****************************************************************************
 * Function: send
 *
 * Description:
 *   The send() call may be used only when the socket is in a connected state
 *   (so that the intended recipient is known). The only difference between
 *   send() and write() is the presence of flags. With zero flags parameter,
 *   send() is equivalent to write(). Also, send(sockfd,buf,len,flags) is
 *   equivalent to sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EOPNOTSUPP
 *     Some bit in the flags argument is inappropriate for the socket
 *     type.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t send(int sockfd, const void *buf, size_t len, int flags)
{
  return psock_send(sockfd_socket(sockfd), buf, len, flags);
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
