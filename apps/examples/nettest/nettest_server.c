/****************************************************************************
 * examples/nettest/nettest-server.c
 *
 *   Copyright (C) 2007, 2011-2012 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "nettest.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void recv_server(void)
{
  struct sockaddr_in myaddr;
#ifdef NETTEST_HAVE_SOLINGER
  struct linger ling;
#endif
  char *buffer;
  int listensd;
  int acceptsd;
  socklen_t addrlen;
  int nbytesread;
#ifndef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  int totalbytesread;
  int nbytessent;
  int ch;
  int i;
#endif
  int optval;

  /* Allocate a BIG buffer */

  buffer = (char*)malloc(2*SENDSIZE);
  if (!buffer)
    {
      message("server: failed to allocate buffer\n");
      exit(1);
    }


  /* Create a new TCP socket */

  listensd = socket(PF_INET, SOCK_STREAM, 0);
  if (listensd < 0)
    {
      message("server: socket failure: %d\n", errno);
      goto errout_with_buffer;
    }

  /* Set socket to reuse address */

  optval = 1;
  if (setsockopt(listensd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int)) < 0)
    {
      message("server: setsockopt SO_REUSEADDR failure: %d\n", errno);
      goto errout_with_listensd;
    }

  /* Bind the socket to a local address */

  myaddr.sin_family      = AF_INET;
  myaddr.sin_port        = HTONS(PORTNO);
  myaddr.sin_addr.s_addr = INADDR_ANY;

  if (bind(listensd, (struct sockaddr*)&myaddr, sizeof(struct sockaddr_in)) < 0)
    {
      message("server: bind failure: %d\n", errno);
      goto errout_with_listensd;
    }

  /* Listen for connections on the bound TCP socket */

  if (listen(listensd, 5) < 0)
    {
      message("server: listen failure %d\n", errno);
      goto errout_with_listensd;
    }

  /* Accept only one connection */

  message("server: Accepting connections on port %d\n", PORTNO);
  addrlen = sizeof(struct sockaddr_in);
  acceptsd = accept(listensd, (struct sockaddr*)&myaddr, &addrlen);
  if (acceptsd < 0)
    {
      message("server: accept failure: %d\n", errno);
      goto errout_with_listensd;
    }
  message("server: Connection accepted -- receiving\n");

  /* Configure to "linger" until all data is sent when the socket is closed */

#ifdef NETTEST_HAVE_SOLINGER
    ling.l_onoff  = 1;
    ling.l_linger = 30;     /* timeout is seconds */
    if (setsockopt(acceptsd, SOL_SOCKET, SO_LINGER, &ling, sizeof(struct linger)) < 0)
      {
      message("server: setsockopt SO_LINGER failure: %d\n", errno);
      goto errout_with_acceptsd;
    }
#endif

#ifdef CONFIG_EXAMPLES_NETTEST_PERFORMANCE
  /* Then receive data forever */

  for (;;)
    {
      nbytesread = recv(acceptsd, buffer, 2*SENDSIZE, 0);
      if (nbytesread < 0)
        {
          message("server: recv failed: %d\n", errno);
          goto errout_with_acceptsd;
        }
      else if (nbytesread == 0)
        {
          message("server: The client broke the connection\n");
          goto errout_with_acceptsd;
        }
      message("Received %d bytes\n", nbytesread);
    }
#else
  /* Receive canned message */

  totalbytesread = 0;
  while (totalbytesread < SENDSIZE)
    {
      message("server: Reading...\n");
      nbytesread = recv(acceptsd, &buffer[totalbytesread], 2*SENDSIZE - totalbytesread, 0);
      if (nbytesread < 0)
        {
          message("server: recv failed: %d\n", errno);
          goto errout_with_acceptsd;
        }
      else if (nbytesread == 0)
        {
          message("server: The client broke the connection\n");
          goto errout_with_acceptsd;
        }

      totalbytesread += nbytesread;
      message("server: Received %d of %d bytes\n", totalbytesread, SENDSIZE);
    }

  /* Verify the message */

  if (totalbytesread != SENDSIZE)
    {
      message("server: Received %d / Expected %d bytes\n", totalbytesread, SENDSIZE);
      goto errout_with_acceptsd;
    }

  ch = 0x20;
  for (i = 0; i < SENDSIZE; i++ )
    {
      if (buffer[i] != ch)
        {
          message("server: Byte %d is %02x / Expected %02x\n", i, buffer[i], ch);
          goto errout_with_acceptsd;
        }

      if (++ch > 0x7e)
        {
          ch = 0x20;
        }
    }

  /* Then send the same data back to the client */

  message("server: Sending %d bytes\n", totalbytesread);
  nbytessent = send(acceptsd, buffer, totalbytesread, 0);
  if (nbytessent <= 0)
    {
      message("server: send failed: %d\n", errno);
      goto errout_with_acceptsd;
    }
  message("server: Sent %d bytes\n", nbytessent);

  /* If this platform only does abortive disconnects, then wait a bit to get the
   * client side a change to receive the data.
   */

#if 1 /* Do it for all platforms */
  message("server: Wait before closing\n");
  sleep(60);
#endif

  close(listensd);
  close(acceptsd);
  free(buffer);
  return;
#endif

errout_with_acceptsd:
  close(acceptsd);

errout_with_listensd:
  close(listensd);

errout_with_buffer:
  free(buffer);
  exit(1);
}
