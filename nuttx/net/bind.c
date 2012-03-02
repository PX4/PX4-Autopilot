/****************************************************************************
 * net/bind.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include "net_internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_bind
 *
 * Description:
 *   bind() gives the socket 'psock' the local address 'addr'. 'addr' is
 *   'addrlen' bytes long. Traditionally, this is called "assigning a name to
 *   a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     psock is a descriptor for a file, not a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_bind(FAR struct socket *psock, const struct sockaddr *addr,
              socklen_t addrlen)
{
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
#ifdef CONFIG_NET_IPv6
  FAR const struct sockaddr_in6 *inaddr = (const struct sockaddr_in6 *)addr;
#else
  FAR const struct sockaddr_in *inaddr = (const struct sockaddr_in *)addr;
#endif
#endif

  int err;
  int ret;

  /* Verify that the psock corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = ENOTSOCK;
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

  /* Perform the binding depending on the protocol type */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        ret = uip_tcpbind(psock->s_conn, inaddr);
        psock->s_flags |= _SF_BOUND;
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        ret = uip_udpbind(psock->s_conn, inaddr);
        break;
#endif

      default:
        err = EBADF;
        goto errout;
    }

  /* Was the bind successful */

  if (ret < 0)
    {
      err = -ret;
      goto errout;
    }

  return OK;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

/****************************************************************************
 * Function: bind
 *
 * Description:
 *   bind() gives the socket 'sockfd' the local address 'addr'. 'addr' is
 *   'addrlen' bytes long. Traditionally, this is called "assigning a name to
 *   a socket." When a socket is created with socket, it exists in a name
 *   space (address family) but has no name assigned.
 *
 * Parameters:
 *   sockfd   Socket descriptor of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 *   EACCES
 *     The address is protected, and the user is not the superuser.
 *   EADDRINUSE
 *     The given address is already in use.
 *   EBADF
 *     sockfd is not a valid descriptor.
 *   EINVAL
 *     The socket is already bound to an address.
 *   ENOTSOCK
 *     sockfd is a descriptor for a file, not a socket.
 *
 * Assumptions:
 *
 ****************************************************************************/

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
  /* Make the socket descriptor to the underlying socket structure */

  FAR struct socket *psock = sockfd_socket(sockfd);

  /* Then let psock_bind do all of the work */

  return psock_bind(psock, addr, addrlen);
}

#endif /* CONFIG_NET */
