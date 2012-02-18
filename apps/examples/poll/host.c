/****************************************************************************
 * examples/poll/host.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#define pthread_addr_t void *
#include "poll_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef TARGETIP
#  error TARGETIP not defined
#endif

#define IOBUFFER_SIZE 80

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct sockaddr_in myaddr;
  char outbuf[IOBUFFER_SIZE];
  char inbuf[IOBUFFER_SIZE];
  int sockfd;
  int len;
  int nbytessent;
  int nbytesrecvd;
  int i;

  /* Create a new TCP socket */

  sockfd = socket(PF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    {
      message("client socket failure %d\n", errno);
      goto errout_with_outbufs;
    }

  /* Connect the socket to the server */

  myaddr.sin_family      = AF_INET;
  myaddr.sin_port        = htons(LISTENER_PORT);
  myaddr.sin_addr.s_addr = inet_addr(TARGETIP);

  message("client: Connecting to %s...\n", TARGETIP);
  if (connect( sockfd, (struct sockaddr*)&myaddr, sizeof(struct sockaddr_in)) < 0)
    {
      message("client: connect failure: %d\n", errno);
      goto errout_with_socket;
    }
  message("client: Connected\n");

  /* Then send and receive messages */

  for (i = 0; ; i++)
    {
      sprintf(outbuf, "Remote message %d", i);
      len = strlen(outbuf);

      message("client: Sending '%s' (%d bytes)\n", outbuf, len);
      nbytessent = send(sockfd, outbuf, len, 0);
      message("client: Sent %d bytes\n", nbytessent);

      if (nbytessent < 0)
        {
          message("client: send failed: %d\n", errno);
          goto errout_with_socket;
        }
      else if (nbytessent != len)
        {
          message("client: Bad send length: %d Expected: %d\n", nbytessent, len);
          goto errout_with_socket;
        }

      message("client: Receiving...\n");
      nbytesrecvd = recv(sockfd, inbuf, IOBUFFER_SIZE, 0);

      if (nbytesrecvd < 0)
        {
          message("client: recv failed: %d\n", errno);
          goto errout_with_socket;
        }
      else if (nbytesrecvd == 0)
        {
          message("client: The server broke the connections\n");
          goto errout_with_socket;
        }

      inbuf[nbytesrecvd] = '\0';
      message("client: Received '%s' (%d bytes)\n", inbuf, nbytesrecvd);

      if (nbytesrecvd != len)
        {
          message("client: Bad recv length: %d Expected: %d\n", nbytesrecvd, len);
          goto errout_with_socket;
        }
      else if (memcmp(inbuf, outbuf, len) != 0)
        {
          message("client: Received outbuf does not match sent outbuf\n");
          goto errout_with_socket;
        }

      message("client: Sleeping\n");
      sleep(8);
    }

  close(sockfd);
  return 0;

errout_with_socket:
  close(sockfd);
errout_with_outbufs:
  exit(1);
}
