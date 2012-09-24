/****************************************************************************
 * examples/poll/poll_listener.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include "poll_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#if defined(CONFIG_DEV_CONSOLE) && !defined(CONFIG_DEV_LOWCONSOLE)
#   define HAVE_CONSOLE
#   define NPOLLFDS 2
#   define CONSNDX  0
#   define FIFONDX  1
#else
#   undef  HAVE_CONSOLE
#   define NPOLLFDS 1
#   define FIFONDX  0
#endif

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
 * Name: poll_listener
 ****************************************************************************/

void *poll_listener(pthread_addr_t pvarg)
{
  struct pollfd fds[NPOLLFDS];
  char buffer[64];
  ssize_t nbytes;
  bool timeout;
  bool pollin;
  int nevents;
  int fd;
  int ret;
  int i;

  /* Open the FIFO for non-blocking read */

  message("poll_listener: Opening %s for non-blocking read\n", FIFO_PATH1);
  fd = open(FIFO_PATH1, O_RDONLY|O_NONBLOCK);
  if (fd < 0)
    {
      message("poll_listener: ERROR Failed to open FIFO %s: %d\n",
              FIFO_PATH1, errno);
      (void)close(fd);
      return (void*)-1;
    }

  /* Loop forever */

  for (;;)
    {
      message("poll_listener: Calling poll()\n");

      memset(fds, 0, sizeof(struct pollfd)*NPOLLFDS);
#ifdef HAVE_CONSOLE
      fds[CONSNDX].fd      = 0;
      fds[CONSNDX].events  = POLLIN;
      fds[CONSNDX].revents = 0;
#endif
      fds[FIFONDX].fd      = fd;
      fds[FIFONDX].events  = POLLIN;
      fds[FIFONDX].revents = 0;

      timeout              = false;
      pollin               = false;

      ret = poll(fds, NPOLLFDS, POLL_LISTENER_DELAY);

      message("\npoll_listener: poll returned: %d\n", ret);
      if (ret < 0)
        {
          message("poll_listener: ERROR poll failed: %d\n", errno);
        }
      else if (ret == 0)
        {
          message("poll_listener: Timeout\n");
          timeout = true;
        }
      else if (ret > NPOLLFDS)
        {
          message("poll_listener: ERROR poll reported: %d\n");
        }
      else
        {
          pollin = true;
        }

      nevents = 0;
      for (i = 0; i < NPOLLFDS; i++)
        {
          message("poll_listener: FIFO revents[%d]=%02x\n", i, fds[i].revents);
          if (timeout)
            {
              if (fds[i].revents != 0)
                {
                  message("poll_listener: ERROR? expected revents=00, received revents[%d]=%02x\n",
                          fds[i].revents, i);
                }
            }
          else if (pollin)
            {
               if (fds[i].revents == POLLIN)
                 {
                   nevents++;
                 }
               else if (fds[i].revents != 0)
                {
                   message("poll_listener: ERROR unexpected revents[i]=%02x\n",
                           i, fds[i].revents);
                 }
            }
        }

      if (pollin && nevents != ret)
        {
           message("poll_listener: ERROR found %d events, poll reported %d\n", nevents, ret);
        }

      /* In any event, read until the pipe/serial  is empty */

      for (i = 0; i < NPOLLFDS; i++)
        {
          do
            {
#ifdef HAVE_CONSOLE
              /* Hack to work around the fact that the console driver on the
               * simulator is always non-blocking.
               */

              if (i == CONSNDX)
                {
                  if ((fds[CONSNDX].revents & POLLIN) != 0)
                    {
                      buffer[0] = getchar();
                      nbytes = 1;
                    }
                  else
                    {
                      nbytes = 0;
                    }
                }
              else
#endif
                {
                  /* The pipe works differently, it returns whatever data
                   * it has available without blocking.
                   */

                  nbytes = read(fds[i].fd, buffer, 63);
                }

              if (nbytes <= 0)
                {
                  if (nbytes == 0 || errno == EAGAIN)
                    {
                      if ((fds[i].revents & POLLIN) != 0)
                        {
                          message("poll_listener: ERROR no read data[%d]\n", i);
                        }
                    }
                  else if (errno != EINTR)
                    {
                      message("poll_listener: read[%d] failed: %d\n", i, errno);
                    }
                  nbytes = 0;
                }
              else
                {
                  if (timeout)
                    {
                      message("poll_listener: ERROR? Poll timeout, but data read[%d]\n", i);
                      message("               (might just be a race condition)\n");
                    }

                  buffer[nbytes] = '\0';
                  message("poll_listener: Read[%d] '%s' (%d bytes)\n", i, buffer, nbytes);
                }

              /* Suppress error report if no read data on the next time through */

              fds[i].revents = 0;
            }
          while (nbytes > 0);
        }
 
      /* Make sure that everything is displayed */

      msgflush();
    }

  /* Won't get here */

  (void)close(fd);
  return NULL;
}
