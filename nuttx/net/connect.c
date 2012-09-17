/****************************************************************************
 * net/connect.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <nuttx/net/uip/uip-arch.h>

#include "net_internal.h"
#include "uip/uip_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
struct tcp_connect_s
{
  FAR struct uip_conn       *tc_conn;    /* Reference to TCP connection structure */
  FAR struct uip_callback_s *tc_cb;      /* Reference to callback instance */
  sem_t                      tc_sem;     /* Semaphore signals recv completion */
  int                        tc_result;  /* OK on success, otherwise a negated errno. */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline int tcp_setup_callbacks(FAR struct socket *psock,
                                      FAR struct tcp_connect_s *pstate);
static inline void tcp_teardown_callbacks(struct tcp_connect_s *pstate, int status);
static uint16_t tcp_connect_interrupt(struct uip_driver_s *dev, void *pvconn,
                                    void *pvpriv, uint16_t flags);
#ifdef CONFIG_NET_IPv6
static inline int tcp_connect(FAR struct socket *psock, const struct sockaddr_in6 *inaddr);
#else
static inline int tcp_connect(FAR struct socket *psock, const struct sockaddr_in *inaddr);
#endif
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: tcp_setup_callbacks
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline int tcp_setup_callbacks(FAR struct socket *psock,
                                      FAR struct tcp_connect_s *pstate)
{
  FAR struct uip_conn *conn = psock->s_conn;
  int ret = -EBUSY;

  /* Initialize the TCP state structure */

  (void)sem_init(&pstate->tc_sem, 0, 0); /* Doesn't really fail */
  pstate->tc_conn   = conn;
  pstate->tc_result = -EAGAIN;

  /* Set up the callbacks in the connection */

  pstate->tc_cb = uip_tcpcallbackalloc(conn);
  if (pstate->tc_cb)
    {
      /* Set up the connection "interrupt" handler */

      pstate->tc_cb->flags   = UIP_NEWDATA|UIP_CLOSE|UIP_ABORT|UIP_TIMEDOUT|UIP_CONNECTED;
      pstate->tc_cb->priv    = (void*)pstate;
      pstate->tc_cb->event   = tcp_connect_interrupt;

      /* Set up the connection event monitor */

      net_startmonitor(psock);
      ret = OK;
    }
  return ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: tcp_teardown_callbacks
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static inline void tcp_teardown_callbacks(struct tcp_connect_s *pstate,
                                          int status)
{
  FAR struct uip_conn *conn = pstate->tc_conn;

  /* Make sure that no further interrupts are processed */

  uip_tcpcallbackfree(conn, pstate->tc_cb);

  /* If we successfully connected, we will continue to monitor the connection
   * state via callbacks.
   */

  if (status < 0)
    {
      /* Failed to connect. Stop the connection event monitor */

      net_stopmonitor(conn);
    }
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: tcp_connect_interrupt
 *
 * Description:
 *   This function is called from the interrupt level to perform the actual
 *   connection operation via by the uIP layer.
 *
 * Parameters:
 *   dev      The sructure of the network driver that caused the interrupt
 *   pvconn   The connection structure associated with the socket
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   The new flags setting
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static uint16_t tcp_connect_interrupt(struct uip_driver_s *dev, void *pvconn,
                                      void *pvpriv, uint16_t flags)
{
  struct tcp_connect_s *pstate = (struct tcp_connect_s *)pvpriv;

  nllvdbg("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* The following errors should be detected here (someday)
       *
       *     ECONNREFUSED
       *       No one listening on the remote address.
       *     ENETUNREACH
       *       Network is unreachable.
       *     ETIMEDOUT
       *       Timeout while attempting connection. The server may be too busy
       *       to accept new connections.
       */

      /* UIP_CLOSE: The remote host has closed the connection
       * UIP_ABORT: The remote host has aborted the connection
       */

      if ((flags & (UIP_CLOSE|UIP_ABORT)) != 0)
        {
          /* Indicate that remote host refused the connection */

          pstate->tc_result = -ECONNREFUSED;
        }

      /* UIP_TIMEDOUT: Connection aborted due to too many retransmissions. */

      else if ((flags & UIP_TIMEDOUT) != 0)
        {
          /* Indicate that the remote host is unreachable (or should this be timedout?) */

          pstate->tc_result = -ETIMEDOUT;
        }

      /* UIP_CONNECTED: The socket is successfully connected */

      else if ((flags & UIP_CONNECTED) != 0)
        {
          /* Indicate that the socket is no longer connected */

          pstate->tc_result = OK;
        }

      /* Otherwise, it is not an event of importance to us at the moment */

      else
        {
          /* Drop data received in this state */

          dev->d_len = 0;
          return flags & ~UIP_NEWDATA;
        }

      nllvdbg("Resuming: %d\n", pstate->tc_result);

      /* Stop further callbacks */

      tcp_teardown_callbacks(pstate, pstate->tc_result);

      /* Wake up the waiting thread */

      sem_post(&pstate->tc_sem);
    }

  return flags;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Name: tcp_connect
 *
 * Description:
 *   Perform a TCP connection
 *
 * Parameters:
 *   psock    A reference to the socket structure of the socket to be connected
 *   inaddr   The address of the remote server to connect to
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
#ifdef CONFIG_NET_IPv6
static inline int tcp_connect(FAR struct socket *psock, const struct sockaddr_in6 *inaddr)
#else
static inline int tcp_connect(FAR struct socket *psock, const struct sockaddr_in *inaddr)
#endif
{
  struct tcp_connect_s state;
  uip_lock_t           flags;
  int                  ret = OK;

  /* Interrupts must be disabled through all of the following because
   * we cannot allow the network callback to occur until we are completely
   * setup.
   */

  flags = uip_lock();

  /* Get the connection reference from the socket */

  if (!psock->s_conn) /* Should always be non-NULL */
    {
      ret = -EINVAL;
    }
  else
    {
      /* Perform the uIP connection operation */

      ret = uip_tcpconnect(psock->s_conn, inaddr);
    }

  if (ret >= 0)
    {
      /* Set up the callbacks in the connection */

      ret = tcp_setup_callbacks(psock, &state);
      if (ret >= 0)
        {
          /* Wait for either the connect to complete or for an error/timeout
           * to occur. NOTES:  (1) uip_lockedwait will also terminate if a signal
           * is received, (2) interrupts may be disabled!  They will be re-
           * enabled while the task sleeps and automatically re-disabled
           * when the task restarts.
           */

          ret = uip_lockedwait(&state.tc_sem);

          /* Uninitialize the state structure */

          (void)sem_destroy(&state.tc_sem);

          /* If uip_lockedwait failed, recover the negated error (probably -EINTR) */

          if (ret < 0)
            {
              ret = -errno;
            }
          else
            {
              /* If the wait succeeded, then get the new error value from
               * the state structure
               */

              ret = state.tc_result;
            }

          /* Make sure that no further interrupts are processed */

          tcp_teardown_callbacks(&state, ret);
        }

      /* Mark the connection bound and connected */

      if (ret >= 0)
        {
          psock->s_flags |= (_SF_BOUND|_SF_CONNECTED);
        }
    }

    uip_unlock(flags);
    return ret;
}
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: psock_connect
 *
 * Description:
 *   connect() connects the socket referred to by the structure 'psock'
 *   to the address specified by 'addr'. The addrlen argument specifies
 *   the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'psock'.
 *
 *   If the socket 'psock' is of type SOCK_DGRAM then 'addr' is the address
 *   to which datagrams are sent by default, and the only address from which
 *   datagrams are received. If the socket is of type SOCK_STREAM or
 *   SOCK_SEQPACKET, this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully connect()
 *   only once; connectionless protocol sockets may use connect() multiple
 *   times to change their association.  Connectionless sockets may dissolve
 *   the association by connecting to an address with the sa_family member of
 *   sockaddr set to AF_UNSPEC.
 *
 * Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *     EACCES, EPERM
 *       The user tried to connect to a broadcast address without having the
 *       socket broadcast flag enabled or the connection request failed
 *       because of a local firewall rule.
 *     EADDRINUSE
 *       Local address is already in use.
 *     EAFNOSUPPORT
 *       The passed address didn't have the correct address family in its
 *       sa_family field.
 *     EAGAIN
 *       No more free local ports or insufficient entries in the routing
 *       cache.
 *     EALREADY
 *       The socket is non-blocking and a previous connection attempt has
 *       not yet been completed.
 *     EBADF
 *       The file descriptor is not a valid index in the descriptor table.
 *     ECONNREFUSED
 *       No one listening on the remote address.
 *     EFAULT
 *       The socket structure address is outside the user's address space.
 *     EINPROGRESS
 *       The socket is non-blocking and the connection cannot be completed
 *       immediately.
 *     EINTR
 *       The system call was interrupted by a signal that was caught.
 *     EISCONN
 *       The socket is already connected.
 *     ENETUNREACH
 *       Network is unreachable.
 *     ENOTSOCK
 *       The file descriptor is not associated with a socket.
 *     ETIMEDOUT
 *       Timeout while attempting connection. The server may be too busy
 *       to accept new connections.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_connect(FAR struct socket *psock, FAR const struct sockaddr *addr,
                  socklen_t addrlen)
{
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *inaddr = (const struct sockaddr_in6 *)addr;
#else
  FAR const struct sockaddr_in *inaddr = (const struct sockaddr_in *)addr;
#endif
  int ret;
#endif

  int err;

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Verify that a valid address has been provided */

#ifdef CONFIG_NET_IPv6
  if (addr->sa_family != AF_INET6 || addrlen < sizeof(struct sockaddr_in6))
#else
  if (addr->sa_family != AF_INET || addrlen < sizeof(struct sockaddr_in))
#endif
  {
      err = EBADF;
      goto errout;
  }

  /* Perform the connection depending on the protocol type */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
          /* Verify that the socket is not already connected */

          if (_SS_ISCONNECTED(psock->s_flags))
            {
              err = EISCONN;
              goto errout;
            }

          /* Its not ... connect it */

          ret = tcp_connect(psock, inaddr);
          if (ret < 0)
            {
              err = -ret;
              goto errout;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
          ret = uip_udpconnect(psock->s_conn, inaddr);
          if (ret < 0)
            {
              err = -ret;
              goto errout;
            }
        }
        break;
#endif

      default:
        err = EBADF;
        goto errout;
    }

  return OK;

errout:
   errno = err;
  return ERROR;
}

/****************************************************************************
 * Name: connect
 *
 * Description:
 *   connect() connects the socket referred to by the file descriptor 'sockfd'
 *   to the address specified by 'addr'. The addrlen argument specifies
 *   the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'sockfd'.
 *
 *   If the socket 'sockfd' is of type SOCK_DGRAM then 'addr' is the address
 *   to which datagrams are sent by default, and the only address from which
 *   datagrams are received. If the socket is of type SOCK_STREAM or
 *   SOCK_SEQPACKET, this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully connect()
 *   only once; connectionless protocol sockets may use connect() multiple
 *   times to change their association.  Connectionless sockets may dissolve
 *   the association by connecting to an address with the sa_family member of
 *   sockaddr set to AF_UNSPEC.
 *
 * Parameters:
 *   sockfd    Socket descriptor returned by socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *     EACCES, EPERM
 *       The user tried to connect to a broadcast address without having the
 *       socket broadcast flag enabled or the connection request failed
 *       because of a local firewall rule.
 *     EADDRINUSE
 *       Local address is already in use.
 *     EAFNOSUPPORT
 *       The passed address didn't have the correct address family in its
 *       sa_family field.
 *     EAGAIN
 *       No more free local ports or insufficient entries in the routing
 *       cache.
 *     EALREADY
 *       The socket is non-blocking and a previous connection attempt has
 *       not yet been completed.
 *     EBADF
 *       The file descriptor is not a valid index in the descriptor table.
 *     ECONNREFUSED
 *       No one listening on the remote address.
 *     EFAULT
 *       The socket structure address is outside the user's address space.
 *     EINPROGRESS
 *       The socket is non-blocking and the connection cannot be completed
 *       immediately.
 *     EINTR
 *       The system call was interrupted by a signal that was caught.
 *     EISCONN
 *       The socket is already connected.
 *     ENETUNREACH
 *       Network is unreachable.
 *     ENOTSOCK
 *       The file descriptor is not associated with a socket.
 *     ETIMEDOUT
 *       Timeout while attempting connection. The server may be too busy
 *       to accept new connections.
 *
 * Assumptions:
 *
 ****************************************************************************/

int connect(int sockfd, FAR const struct sockaddr *addr, socklen_t addrlen)
{
  /* Get the underlying socket structure */

  FAR struct socket *psock = sockfd_socket(sockfd);

  /* Then let psock_connect() do all of the work */

  return psock_connect(psock, addr, addrlen);
}

#endif /* CONFIG_NET */
