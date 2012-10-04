/****************************************************************************
 * examples/udp/udp-client.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>

#include "udp-internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void fill_buffer(unsigned char *buf, int offset)
{
  int ch;
  int j;

  buf[0] = offset;
  for (ch = 0x20, j = offset + 1; ch < 0x7f; ch++, j++)
    {
      if (j >= SENDSIZE)
        {
          j = 1;
        }
      buf[j] = ch;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void send_client(void)
{
  struct sockaddr_in server;
  unsigned char outbuf[SENDSIZE];
  int sockfd;
  int nbytes;
  int offset;

  /* Create a new TCP socket */

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      message("client socket failure %d\n", errno);
      exit(1);
    }

  /* Then send and receive 256 messages */

  for (offset = 0; offset < 256; offset++)
    {
      /* Set up the output buffer */

      fill_buffer(outbuf, offset);

      /* Send the message */

      server.sin_family      = AF_INET;
      server.sin_port        = HTONS(PORTNO);
      server.sin_addr.s_addr = HTONL(CONFIG_EXAMPLES_UDP_SERVERIP);

      message("client: %d. Sending %d bytes\n", offset, SENDSIZE);
      nbytes = sendto(sockfd, outbuf, SENDSIZE, 0,
                      (struct sockaddr*)&server, sizeof(struct sockaddr_in));
      message("client: %d. Sent %d bytes\n", offset, nbytes);

      if (nbytes < 0)
        {
          message("client: %d. sendto failed: %d\n", offset, errno);
          close(sockfd);
          exit(-1);
        }
      else if (nbytes != SENDSIZE)
        {
          message("client: %d. Bad send length: %d Expected: %d\n",
                  offset, nbytes, SENDSIZE);
          close(sockfd);
          exit(-1);
        }

      /* Now, sleep a bit.  No packets should be dropped due to overrunning
       * the server.
       */

      sleep(2);
    }
  close(sockfd);
}
