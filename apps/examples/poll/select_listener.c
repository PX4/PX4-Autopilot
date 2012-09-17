/****************************************************************************
 * examples/poll/select_listener.c
 *
 *   Copyright (C) 2008, 2009 Gregory Nutt. All rights reserved.
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
#include <sys/stat.h>
#include <sys/select.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "poll_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: select_listener
 ****************************************************************************/

void *select_listener(pthread_addr_t pvarg)
{
  fd_set rfds;
  struct timeval tv;
  char buffer[64];
  ssize_t nbytes;
  bool timeout;
  bool ready;
  int fd;
  int ret;

  /* Open the FIFO for non-blocking read */

  message("select_listener: Opening %s for non-blocking read\n", FIFO_PATH2);
  fd = open(FIFO_PATH2, O_RDONLY|O_NONBLOCK);
  if (fd < 0)
    {
      message("select_listener: ERROR Failed to open FIFO %s: %d\n",
              FIFO_PATH2, errno);
      (void)close(fd);
      return (void*)-1;
    }

  /* Loop forever */

  for (;;)
    {
      message("select_listener: Calling select()\n");

      FD_ZERO(&rfds);
      FD_SET(fd, &rfds);

      tv.tv_sec  = SELECT_LISTENER_DELAY;
      tv.tv_usec = 0;

      timeout    = false;
      ready      = false;

      ret = select(fd+1, (FAR fd_set*)&rfds, (FAR fd_set*)NULL, (FAR fd_set*)NULL, &tv);
      message("\nselect_listener: select returned: %d\n", ret);

      if (ret < 0)
        {
          message("select_listener: ERROR select failed: %d\n");
        }
      else if (ret == 0)
        {
          message("select_listener: Timeout\n");
          timeout = true;
        }
      else
        {
          if (ret != 1)
            {
              message("select_listener: ERROR poll reported: %d\n");
            }
          else
            {
              ready = true;
            }

          if (!FD_ISSET(fd, rfds))
            {
              message("select_listener: ERROR fd=%d not in fd_set\n");
            }
        }

      /* In any event, read until the pipe is empty */

      do
        {
          nbytes = read(fd, buffer, 63);
          if (nbytes <= 0)
            {
              if (nbytes == 0 || errno == EAGAIN)
                {
                  if (ready)
                    {
                      message("select_listener: ERROR no read data\n");
                    }
                }
              else if (errno != EINTR)
                {
                  message("select_listener: read failed: %d\n", errno);
                }
              nbytes = 0;
            }
          else
            {
              if (timeout)
                {
                  message("select_listener: ERROR? Poll timeout, but data read\n");
                  message("               (might just be a race condition)\n");
                }

              buffer[nbytes] = '\0';
              message("select_listener: Read '%s' (%d bytes)\n", buffer, nbytes);
            }

          timeout = false;
          ready   = false;
        }
      while (nbytes > 0);

      /* Make sure that everything is displayed */

      msgflush();
    }

  /* Won't get here */

  (void)close(fd);
  return NULL;
}
