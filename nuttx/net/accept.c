/****************************************************************************
 * net/accept.c
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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET_TCP)

#include <sys/types.h>
#include <sys/socket.h>

#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "net_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct accept_s
{
  sem_t                    acpt_sem;        /* Wait for interrupt event */
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *acpt_addr;       /* Return connection adress */
#else
  FAR struct sockaddr_in  *acpt_addr;       /* Return connection adress */
#endif
  FAR struct uip_conn     *acpt_newconn;    /* The accepted connection */
  int                      acpt_result;     /* The result of the wait */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: accept_tcpsender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Parameters:
 *   conn   - The newly accepted TCP connection
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
#ifdef CONFIG_NET_IPv6
static inline void accept_tcpsender(FAR struct uip_conn *conn,
                                    FAR struct sockaddr_in6 *addr)
{
  if (addr)
    {
      addr->sin_family = AF_INET6;
      addr->sin_port   = conn->rport;
      uip_ipaddr_copy(addr->sin6_addr.s6_addr, conn->ripaddr);
    }
}
#else
static inline void accept_tcpsender(FAR struct uip_conn *conn,
                                    FAR struct sockaddr_in *addr)
{
  if (addr)
    {
      addr->sin_family = AF_INET;
      addr->sin_port   = conn->rport;
      uip_ipaddr_copy(addr->sin_addr.s_addr, conn->ripaddr);
    }
}
#endif /* CONFIG_NET_IPv6 */
#endif /* CONFIG_NET_TCP */

/****************************************************************************
 * Function: accept_interrupt
 *
 * Description:
 *   Receive interrupt level callbacks when connections occur
 *
 * Parameters:
 *   listener The conection stucture of the listener
 *   conn     The connection stucture that was just accepted
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Running at the interrupt level
 *
 ****************************************************************************/

static int accept_interrupt(struct uip_conn *listener, struct uip_conn *conn)
{
  struct accept_s *pstate = (struct accept_s *)listener->accept_private;
  int ret = -EINVAL;

  if (pstate)
    {
      /* Get the connection address */

      accept_tcpsender(conn, pstate->acpt_addr);

      /* Save the connection structure */

      pstate->acpt_newconn     = conn;
      pstate->acpt_result      = OK;

      /* There should be a reference of one on the new connection */

      DEBUGASSERT(conn->crefs == 1);

      /* Wake-up the waiting caller thread */

      sem_post(&pstate->acpt_sem);

      /* Stop any further callbacks */

      listener->accept_private = NULL;
      listener->accept         = NULL;
      ret                      = OK;
  }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: accept
 *
 * Description:
 *   The accept function is used with connection-based socket types
 *   (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, accept blocks the caller until a connection
 *   is present. If the socket is marked non-blocking and no pending
 *   connections are present on the queue, accept returns EAGAIN.
 *
 * Parameters:
 *   sockfd   The listening socket descriptior
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *
 * Returned Value:
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 *
 * EAGAIN or EWOULDBLOCK
 *   The socket is marked non-blocking and no connections are present to
 *   be accepted.
 * EBADF
 *   The descriptor is invalid.
 * ENOTSOCK
 *  The descriptor references a file, not a socket.
 * EOPNOTSUPP
 *   The referenced socket is not of type SOCK_STREAM.
 * EINTR
 *   The system call was interrupted by a signal that was caught before
 *   a valid connection arrived.
 * ECONNABORTED
 *   A connection has been aborted.
 * EINVAL
 *   Socket is not listening for connections.
 * EMFILE
 *   The per-process limit of open file descriptors has been reached.
 * ENFILE
 *   The system maximum for file descriptors has been reached.
 * EFAULT
 *   The addr parameter is not in a writable part of the user address
 *   space.
 * ENOBUFS or ENOMEM
 *   Not enough free memory.
 * EPROTO
 *   Protocol error.
 * EPERM
 *   Firewall rules forbid connection.
 *
 * Assumptions:
 *
 ****************************************************************************/

int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  FAR struct socket *pnewsock;
  FAR struct uip_conn *conn;
  struct accept_s state;
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *inaddr = (struct sockaddr_in6 *)addr;
#else
  FAR struct sockaddr_in *inaddr = (struct sockaddr_in *)addr;
#endif
  uip_lock_t save;
  int newfd;
  int err;
  int ret;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      /* It is not a valid socket description.  Distinguish between the cases
       * where sockfd is a just valid and when it is a valid file descriptor used
       * in the wrong context.
       */

#if CONFIG_NFILE_DESCRIPTORS > 0
      if ((unsigned int)sockfd < CONFIG_NFILE_DESCRIPTORS)
        {
          err = ENOTSOCK;
        }
      else
#endif
        {
          err = EBADF;
        }
      goto errout;
    }

  /* We have a socket descriptor, but it is a stream? */

  if (psock->s_type != SOCK_STREAM)
    {
      err = EOPNOTSUPP;
      goto errout;
    }

  /* Is the socket listening for a connection? */

  if (!_SS_ISLISTENING(psock->s_flags))
    {
      err = EINVAL;
      goto errout;
    }

  /* Verify that a valid memory block has been provided to receive the
   * address
   */

  if (addr)
    {
#ifdef CONFIG_NET_IPv6
      if (*addrlen < sizeof(struct sockaddr_in6))
#else
      if (*addrlen < sizeof(struct sockaddr_in))
#endif
        {
          err = EBADF;
          goto errout;
        }
    }

  /* Allocate a socket descriptor for the new connection now (so that it
   * cannot fail later)
   */

  newfd = sockfd_allocate(0);
  if (newfd < 0)
    {
      err = ENFILE;
      goto errout;
    }

  pnewsock = sockfd_socket(newfd);
  if (!pnewsock)
    {
      err = ENFILE;
      goto errout_with_socket;
    }

  /* Check the backlog to see if there is a connection already pending for
   * this listener.
   */

  save = uip_lock();
  conn = (struct uip_conn *)psock->s_conn;

#ifdef CONFIG_NET_TCPBACKLOG
  state.acpt_newconn = uip_backlogremove(conn);
  if (state.acpt_newconn)
    {
      /* Yes... get the address of the connected client */

      nvdbg("Pending conn=%p\n", state.acpt_newconn);
      accept_tcpsender(state.acpt_newconn, inaddr);
    }

  /* In general, this uIP-based implementation will not support non-blocking
   * socket operations... except in a few cases:  Here for TCP accept with backlog
   * enabled.  If this socket is configured as non-blocking then return EAGAIN
   * if there is no pending connection in the backlog.
   */

  else if (_SS_ISNONBLOCK(psock->s_flags))
    {
      err = EAGAIN;
      goto errout_with_lock;
    }
  else
#endif
    {
      /* Set the socket state to accepting */

      psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_ACCEPT);

      /* Perform the TCP accept operation */

      /* Initialize the state structure.  This is done with interrupts
       * disabled because we don't want anything to happen until we
       * are ready.
       */

      state.acpt_addr       = inaddr;
      state.acpt_newconn    = NULL;
      state.acpt_result     = OK;
      sem_init(&state.acpt_sem, 0, 0);

      /* Set up the callback in the connection */

      conn->accept_private  = (void*)&state;
      conn->accept          = accept_interrupt;

      /* Wait for the send to complete or an error to occur:  NOTES: (1)
       * uip_lockedwait will also terminate if a signal is received, (2) interrupts
       * may be disabled!  They will be re-enabled while the task sleeps and
       * automatically re-enabled when the task restarts.
       */

      ret = uip_lockedwait(&state.acpt_sem);

      /* Make sure that no further interrupts are processed */

      conn->accept_private = NULL;
      conn->accept         = NULL;

      sem_destroy(&state. acpt_sem);

      /* Set the socket state to idle */

      psock->s_flags = _SS_SETSTATE(psock->s_flags, _SF_IDLE);

      /* Check for a errors.  Errors are signaled by negative errno values
       * for the send length.
       */

      if (state.acpt_result != 0)
        {
          err = state.acpt_result;
          goto errout_with_lock;
        }

      /* If uip_lockedwait failed, then we were probably reawakened by a signal. In
       * this case, uip_lockedwait will have set errno appropriately.
       */

      if (ret < 0)
        {
          err = -ret;
          goto errout_with_lock;
        }
    }

  /* Initialize the socket structure and mark the socket as connected.
   * (The reference count on the new connection structure was set in the
   * interrupt handler).
   */

  pnewsock->s_type   = SOCK_STREAM;
  pnewsock->s_conn   = state.acpt_newconn;
  pnewsock->s_flags |= _SF_CONNECTED;
  pnewsock->s_flags &= ~_SF_CLOSED;

  /* Begin monitoring for TCP connection events on the newly connected socket */

  net_startmonitor(pnewsock);
  uip_unlock(save);
  return newfd;

errout_with_lock:
  uip_unlock(save);

errout_with_socket:
  sockfd_release(newfd);

errout:
  errno = err;
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS && CONFIG_NET_TCP */
