/****************************************************************************
 * net/sendto.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <arch/irq.h>
#include <net/uip/uip-arch.h>

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sendto_s
{
  FAR struct uip_callback_s *st_cb; /* Reference to callback instance */
  sem_t       st_sem;        /* Semaphore signals sendto completion */
  uint16_t    st_buflen;     /* Length of send buffer (error if <0) */
  const char *st_buffer;     /* Pointer to send buffer */
  int         st_sndlen;     /* Result of the send (length sent or negated errno) */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: sendto_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   send operation when polled by the uIP layer.
 *
 * Parameters:
 *   dev        The sructure of the network driver that caused the interrupt
 *   conn       An instance of the UDP connection structure cast to void *
 *   pvpriv     An instance of struct sendto_s cast to void*
 *   flags      Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   Modified value of the input flags
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static uint16_t sendto_interrupt(struct uip_driver_s *dev, void *conn,
                                 void *pvpriv, uint16_t flags)
{
  struct sendto_s *pstate = (struct sendto_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);
  if (pstate)
    {
      /* Check if the outgoing packet is available (it may have been claimed
       * by a sendto interrupt serving a different thread -OR- if the output
       * buffer currently contains unprocessed incoming data.  In these cases
       * we will just have to wait for the next polling cycle.
       */

      if (dev->d_sndlen > 0 || (flags & UIP_NEWDATA) != 0)
        {
           /* Another thread has beat us sending data or the buffer is busy,
            * wait for the next polling cycle
            */

           return flags;
        }

      /* It looks like we are good to send the data */

      else
        {
          /* Copy the user data into d_snddata and send it */

          uip_send(dev, pstate->st_buffer, pstate->st_buflen);
          pstate->st_sndlen = pstate->st_buflen;
        }

      /* Don't allow any further call backs. */

      pstate->st_cb->flags   = 0;
      pstate->st_cb->priv    = NULL;
      pstate->st_cb->event   = NULL;

      /* Wake up the waiting thread */

      sem_post(&pstate->st_sem);
    }

  return flags;
}
#endif

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
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

ssize_t psock_sendto(FAR struct socket *psock, FAR const void *buf,
                     size_t len, int flags, FAR const struct sockaddr *to,
                     socklen_t tolen)
{
#ifdef CONFIG_NET_UDP
  FAR struct uip_udp_conn *conn;
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *into = (const struct sockaddr_in6 *)to;
#else
  FAR const struct sockaddr_in *into = (const struct sockaddr_in *)to;
#endif
  struct sendto_s state;
  uip_lock_t save;
  int ret;
#endif
  int err;

  /* If to is NULL or tolen is zero, then this function is same as send (for
   * connected socket types)
   */

  if (!to || !tolen)
    {
#ifdef CONFIG_NET_TCP
      return psock_send(psock, buf, len, flags);
#else
      err = EINVAL;
      goto errout;
#endif
    }

  /* Verify that a valid address has been provided */

#ifdef CONFIG_NET_IPv6
  if (to->sa_family != AF_INET6 || tolen < sizeof(struct sockaddr_in6))
#else
  if (to->sa_family != AF_INET || tolen < sizeof(struct sockaddr_in))
#endif
  {
      err = EBADF;
      goto errout;
  }

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* If this is a connected socket, then return EISCONN */

  if (psock->s_type != SOCK_DGRAM)
    {
      err = EISCONN;
      goto errout;
    }

  /* Perform the UDP sendto operation */

#ifdef CONFIG_NET_UDP
  /* Set the socket state to sending */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_SEND);

  /* Initialize the state structure.  This is done with interrupts
   * disabled because we don't want anything to happen until we
   * are ready.
   */

  save            = uip_lock();
  memset(&state, 0, sizeof(struct sendto_s));
  sem_init(&state.st_sem, 0, 0);
  state.st_buflen = len;
  state.st_buffer = buf;

  /* Setup the UDP socket */

  conn = (struct uip_udp_conn *)psock->s_conn;
  ret = uip_udpconnect(conn, into);
  if (ret < 0)
    {
      uip_unlock(save);
      err = -ret;
      goto errout;
    }

  /* Set up the callback in the connection */

  state.st_cb = uip_udpcallbackalloc(conn);
  if (state.st_cb)
    {
      state.st_cb->flags   = UIP_POLL;
      state.st_cb->priv    = (void*)&state;
      state.st_cb->event   = sendto_interrupt;

      /* Enable the UDP socket */

      uip_udpenable(conn);

      /* Notify the device driver of the availabilty of TX data */

      netdev_txnotify(&conn->ripaddr);

      /* Wait for either the receive to complete or for an error/timeout to occur.
       * NOTES:  (1) uip_lockedwait will also terminate if a signal is received, (2)
       * interrupts may be disabled!  They will be re-enabled while the task sleeps
       * and automatically re-enabled when the task restarts.
       */

      uip_lockedwait(&state.st_sem);

      /* Make sure that no further interrupts are processed */

      uip_udpdisable(conn);
      uip_udpcallbackfree(conn, state.st_cb);
    }
  uip_unlock(save);

  sem_destroy(&state.st_sem);

  /* Set the socket state to idle */

  psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

  /* Check for errors */

  if (state.st_sndlen < 0)
    {
      err = -state.st_sndlen;
      goto errout;
    }

  /* Sucess */

  return state.st_sndlen;
#else
  err = ENOSYS;
#endif

errout:
  errno = err;
  return ERROR;
}

/****************************************************************************
 * Function: sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
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

ssize_t sendto(int sockfd, FAR const void *buf, size_t len, int flags,
               FAR const struct sockaddr *to, socklen_t tolen)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */

  psock = sockfd_socket(sockfd);

  /* And let psock_sendto do all of the work */

  return psock_sendto(psock, buf, len, flags, to, tolen);
}

#endif /* CONFIG_NET */
