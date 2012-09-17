/****************************************************************************
 * examples/udp/udp-server.c
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "udp-internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int check_buffer(unsigned char *buf)
{
  int ret = 1;
  int offset;
  int ch;
  int j;

  offset = buf[0];
  for (ch = 0x20, j = offset + 1; ch < 0x7f; ch++, j++)
    {
      if (j >= SENDSIZE)
        {
          j = 1;
        }
      if (buf[j] != ch)
        {
          message("server: Buffer content error for offset=%d, index=%d\n", offset, j);
          ret = 0;
        }
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void recv_server(void)
{
  struct sockaddr_in server;
  struct sockaddr_in client;
  in_addr_t tmpaddr;
  unsigned char inbuf[1024];
  int sockfd;
  int nbytes;
  int optval;
  int offset;
  socklen_t addrlen;

  /* Create a new UDP socket */

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      message("server: socket failure: %d\n", errno);
      exit(1);
    }

  /* Set socket to reuse address */

  optval = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval, sizeof(int)) < 0)
    {
      message("server: setsockopt SO_REUSEADDR failure: %d\n", errno);
      exit(1);
    }

  /* Bind the socket to a local address */

  server.sin_family      = AF_INET;
  server.sin_port        = HTONS(PORTNO);
  server.sin_addr.s_addr = HTONL(INADDR_ANY);

  if (bind(sockfd, (struct sockaddr*)&server, sizeof(struct sockaddr_in)) < 0)
    {
      message("server: bind failure: %d\n", errno);
      exit(1);
    }

  /* Then receive up to 256 packets of data */

  for (offset = 0; offset < 256; offset++)
    {
      message("server: %d. Receiving up 1024 bytes\n", offset);
      addrlen = sizeof(struct sockaddr_in);
      nbytes = recvfrom(sockfd, inbuf, 1024, 0, 
                        (struct sockaddr*)&client, &addrlen);

      tmpaddr = ntohl(client.sin_addr.s_addr);
      message("server: %d. Received %d bytes from %d.%d.%d.%d:%d\n",
              offset, nbytes, 
              tmpaddr >> 24, (tmpaddr >> 16) & 0xff, 
              (tmpaddr >> 8) & 0xff, tmpaddr & 0xff, 
              ntohs(client.sin_port));

      if (nbytes < 0)
        {
          message("server: %d. recv failed: %d\n", offset, errno);
          close(sockfd);
          exit(-1);
        }

      if (nbytes != SENDSIZE)
        {
          message("server: %d. recv size incorrect: %d vs %d\n", offset, nbytes, SENDSIZE);
          close(sockfd);
          exit(-1);
        }

      if (offset < inbuf[0])
        {
          message("server: %d. %d packets lost, resetting offset\n", offset, inbuf[0] - offset);
          offset = inbuf[0];
        }
      else if (offset > inbuf[0])
        {
          message("server: %d. Bad offset in buffer: %d\n", offset, inbuf[0]);
          close(sockfd);
          exit(-1);
        }

      if (!check_buffer(inbuf))
        {
          message("server: %d. Bad buffer contents\n", offset);
          close(sockfd);
          exit(-1);
        }
    }
  close(sockfd);
}
