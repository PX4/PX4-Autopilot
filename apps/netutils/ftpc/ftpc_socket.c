/****************************************************************************
 * apps/netutils/ftpc/ftpc_socket.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include "ftpc_config.h"
#include <sys/socket.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "ftpc_internal.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpc_sockinit
 *
 * Description:
 *   Initialize a socket.  Create the socket and "wrap" it as C standard
 *   incoming and outgoing streams.
 *
 ****************************************************************************/

int ftpc_sockinit(FAR struct ftpc_socket_s *sock)
{
  /* Initialize the socket structure */

  memset(sock, 0, sizeof(struct ftpc_socket_s));

  /* Create a socket descriptor */

  sock->sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock->sd < 0)
    {
      ndbg("socket() failed: %d\n", errno);
      goto errout;
    }

  /* Call fdopen to "wrap" the socket descriptor as an input stream using C
   * buffered I/O.
   */

  sock->instream = fdopen(sock->sd, "r");
  if (!sock->instream)
    {
      ndbg("fdopen() failed: %d\n", errno);
      goto errout_with_sd;
    }

  /* Call fdopen to "wrap" the socket descriptor as an output stream using C
   * buffered I/O.
   */

  sock->outstream = fdopen(sock->sd, "w");
  if (!sock->outstream)
    {
      ndbg("fdopen() failed: %d\n", errno);
      goto errout_with_instream;
    }

  return OK;

/* Close the instream.  NOTE:  Since the underlying socket descriptor is
 * *not* dup'ed, the following close should fail harmlessly.
 */

errout_with_instream:
  fclose(sock->instream);
  sock->instream = NULL;
errout_with_sd:
  close(sock->sd);
  sock->sd = -1;
errout:
  return ERROR;
}

/****************************************************************************
 * Name: ftpc_sockclose
 *
 * Description:
 *   Close a socket
 *
 ****************************************************************************/

void ftpc_sockclose(struct ftpc_socket_s *sock)
{
  /* Note that the same underlying socket descriptor is used for both streams.
   * There should be harmless failures on the second fclose and the close.
   */

  fclose(sock->instream);
  fclose(sock->outstream);
  close(sock->sd);
  memset(sock, 0, sizeof(struct ftpc_socket_s));
  sock->sd = -1;
}

/****************************************************************************
 * Name: ftpc_sockconnect
 *
 * Description:
 *   Connect the socket to the host.  On a failure, the caller should call.
 *   ftpc_sockclose() to clean up.
 *
 ****************************************************************************/

int ftpc_sockconnect(struct ftpc_socket_s *sock, struct sockaddr_in *addr)
{
  int ret;

  /* Connect to the server */

  ret = connect(sock->sd, (struct sockaddr *)addr, sizeof(struct sockaddr));
  if (ret < 0)
    {
      ndbg("connect() failed: %d\n", errno);
      return ERROR;
    }

  /* Get the local address of the socket */

  ret = ftpc_sockgetsockname(sock, &sock->laddr);
  if (ret < 0)
    {
      ndbg("ftpc_sockgetsockname() failed: %d\n", errno);
      return ERROR;
    }

  sock->connected = true;
  return OK;
}

/****************************************************************************
 * Name: ftpc_sockcopy
 *
 * Description:
 *   Copy the socket state from one location to another.
 *
 ****************************************************************************/

void ftpc_sockcopy(FAR struct ftpc_socket_s *dest,
                   FAR const struct ftpc_socket_s *src)
{
  memcpy(&dest->laddr, &src->laddr, sizeof(struct sockaddr_in));
  dest->connected = ftpc_sockconnected(src);
}

/****************************************************************************
 * Name: ftpc_sockaccept
 *
 * Description:
 *   Accept a connection on the data socket.  This function is onlly used
 *   in active mode. 
 *
 *   In active mode FTP the client connects from a random port (N>1023) to the
 *   FTP server's command port, port 21. Then, the client starts listening to
 *   port N+1 and sends the FTP command PORT N+1 to the FTP server. The server
 *   will then connect back to the client's specified data port from its local
 *   data port, which is port 20. In passive mode FTP the client initiates
 *   both connections to the server, solving the problem of firewalls filtering 
 *   the incoming data port connection to the client from the server. When
 *   opening an FTP connection, the client opens two random ports locally
 *   (N>1023 and N+1). The first port contacts the server on port 21, but
 *   instead of then issuing a PORT command and allowing the server to connect
 *   back to its data port, the client will issue the PASV command. The result
 *   of this is that the server then opens a random unprivileged port (P >
 *   1023) and sends the PORT P command back to the client. The client then
 *   initiates the connection from port N+1 to port P on the server to transfer 
 *   data.
 *
 ****************************************************************************/

int ftpc_sockaccept(FAR struct ftpc_socket_s *sock)
{
  struct sockaddr addr;
  socklen_t addrlen;

  /* Any previous socket should have been uninitialized (0) or explicitly
   * closed (-1).  But the path to this function may include a call to
   * ftpc_sockinit().  If so... close that socket and call accept to
   * get a new one.
   */

  if (sock->sd > 0)
    {
      ftpc_sockclose(sock);
    }

  addrlen  = sizeof(addr);
  sock->sd = accept(sock->sd, &addr, &addrlen);
  if (sock->sd == -1)
    {
      ndbg("accept() failed: %d\n", errno);
      return ERROR;
    }

  memcpy(&sock->laddr, &addr, sizeof(sock->laddr));

  /* Create in/out C buffer I/O streams on the data channel.  First,
   * create the incoming buffered stream.
   */

  sock->instream = fdopen(sock->sd, "r");
  if (!sock->instream)
    {
      ndbg("fdopen() failed: %d\n", errno);
      goto errout_with_sd;
    }

  /* Create the outgoing stream */

  sock->outstream = fdopen(sock->sd, "w");
  if (!sock->outstream)
    {
      ndbg("fdopen() failed: %d\n", errno);
      goto errout_with_instream;
    }

  return OK;

/* Close the instream.  NOTE:  Since the underlying socket descriptor is
 * *not* dup'ed, the following close should fail harmlessly.
 */

errout_with_instream:
  fclose(sock->instream);
  sock->instream = NULL;
errout_with_sd:
  close(sock->sd);
  sock->sd = -1;
  return ERROR;
}

/****************************************************************************
 * Name: ftpc_socklisten
 *
 * Description:
 *   Bind the socket to local address and wait for connection from the server.
 *
 ****************************************************************************/

int ftpc_socklisten(struct ftpc_socket_s *sock)
{
  unsigned int addrlen = sizeof(sock->laddr);
  int ret;

  /* Bind the local socket to the local address */

  sock->laddr.sin_port = 0;
  ret = bind(sock->sd, (struct sockaddr *)&sock->laddr, addrlen);
  if (ret < 0)
    {
      ndbg("bind() failed: %d\n", errno);
      return ERROR;
    }

  /* Wait for the connection to the server */

  if (listen(sock->sd, 1) == -1)
    {
      return ERROR;
    }

  /* Then get the local address selected by NuttX */

  ret = ftpc_sockgetsockname(sock, &sock->laddr);
  return ret;
}

/****************************************************************************
 * Name: ftpc_sockprintf
 *
 * Description:
 *   printf to a socket stream
 *
 ****************************************************************************/

int ftpc_sockprintf(struct ftpc_socket_s *sock, const char *fmt, ...)
{
  va_list ap;
  int r;

  va_start(ap, fmt);
  r = vfprintf(sock->outstream, fmt, ap);
  va_end(ap);
  return r;
}

/****************************************************************************
 * Name: ftpc_sockgetsockname
 *
 * Description:
 *   Get the address of the local socket
 *
 ****************************************************************************/

int ftpc_sockgetsockname(FAR struct ftpc_socket_s *sock,
                         FAR struct sockaddr_in *addr)
{
  socklen_t len = sizeof(struct sockaddr_in);
  int ret;

  ret = getsockname(sock->sd, (FAR struct sockaddr *)addr, &len);
  if (ret < 0)
    {
      ndbg("getsockname failed: %d\n", errno);
      return ERROR;
    }
  return OK;
}
