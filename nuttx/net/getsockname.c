/****************************************************************************
 * net/getsockname.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/socket.h>

#include <string.h>
#include <errno.h>

#include <nuttx/net/net.h>
#include <nuttx/net/uip/uip-arch.h>

#include "net_internal.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: getsockname
 *
 * Description:
 *   The getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *   Possible errno values that may be returned include:
 *
 *   EBADF - The socket argument is not a valid file descriptor.
 *   EOPNOTSUPP - The operation is not supported for this socket's protocol.
 *   EINVAL - The socket has been shut down.
 *
 * Assumptions:
 *
 ****************************************************************************/

int getsockname(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  FAR struct uip_driver_s *dev;

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *outaddr = (FAR struct sockaddr_in6 *)addr;
#else
  FAR struct sockaddr_in *outaddr = (FAR struct sockaddr_in *)addr;
#endif
#endif

  int err;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Some sanity checking... Shouldn't need this on a buckled up embedded
   * system (?)
   */

#ifdef CONFIG_DEBUG
  if (!addr || !addrlen)
    {
      err = EINVAL;
      goto errout;
    }
#endif

  /* Check if enough space has been provided for the full address */

#ifdef CONFIG_NET_IPv6
  if (*addrlen < sizeof(struct sockaddr_in6))
#else
  if (*addrlen < sizeof(struct sockaddr_in))
#endif
  {
    /* This function is supposed to return the partial address if
     * a smaller buffer has been provided.  This support has not
     * been implemented.
     */

    err = ENOSYS;
    goto errout;
  }

  /* Set the port number */

  switch (psock->s_type)
    {
#ifdef CONFIG_NET_TCP
      case SOCK_STREAM:
        {
          struct uip_conn *tcp_conn = (struct uip_conn *)psock->s_conn;
          outaddr->sin_port = tcp_conn->lport; /* Already in network byte order */
        }
        break;
#endif

#ifdef CONFIG_NET_UDP
      case SOCK_DGRAM:
        {
          struct uip_udp_conn *udp_conn = (struct uip_udp_conn *)psock->s_conn;
          outaddr->sin_port = udp_conn->lport; /* Already in network byte order */
        }
        break;
#endif

      default:
        err = EOPNOTSUPP;
        goto errout;
    }

  /* ISSUE: As of this writing, the socket/connection does not know its IP
   * address.  This is because the uIP design is only intended to support
   * a single network device and, therefore, only the network device knows
   * the IP address.
   *
   * Right now, we can just pick the first network device.  But that may
   * not work in the future.
   */

  netdev_semtake();
  dev = g_netdevices;
  if (!dev)
    {
      netdev_semgive();
      err = EINVAL;
      goto errout;
    }

  /* Set the address family and the IP address */

#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
#ifdef CONFIG_NET_IPv6
  outaddr->sin_family = AF_INET6;
  memcpy(outaddr->sin6_addr.in6_u.u6_addr8, dev->d_ipaddr, 16);
  *addrlen = sizeof(struct sockaddr_in6);
#else
  outaddr->sin_family = AF_INET;
  outaddr->sin_addr.s_addr = dev->d_ipaddr;
  *addrlen = sizeof(struct sockaddr_in);
#endif
#endif
  netdev_semgive();

  /* Return success */

  return OK;

errout:
  set_errno(err);
  return ERROR;
}

#endif /* CONFIG_NET */
