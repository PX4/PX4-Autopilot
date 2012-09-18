/****************************************************************************
 * examples/poll/poll_main.c
 *
 *   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>
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
 * Name: poll_main
 ****************************************************************************/

int poll_main(int argc, char *argv[])
{
  char buffer[64];
  ssize_t nbytes;
  pthread_t tid1;
  pthread_t tid2;
#ifdef HAVE_NETPOLL
  pthread_t tid3;
#endif
  int count;
  int fd1 = -1;
  int fd2 = -1;
  int ret;
  int exitcode = 0;

  /* Open FIFOs */

  message("\npoll_main: Creating FIFO %s\n", FIFO_PATH1);
  ret = mkfifo(FIFO_PATH1, 0666);
  if (ret < 0)
    {
      message("poll_main: mkfifo failed: %d\n", errno);
      exitcode = 1;
      goto errout;
    }

  message("\npoll_main: Creating FIFO %s\n", FIFO_PATH2);
  ret = mkfifo(FIFO_PATH2, 0666);
  if (ret < 0)
    {
      message("poll_main: mkfifo failed: %d\n", errno);
      exitcode = 2;
      goto errout;
    }

  /* Open the FIFOs for blocking, write */

  fd1 = open(FIFO_PATH1, O_WRONLY);
  if (fd1 < 0)
    {
      message("poll_main: Failed to open FIFO %s for writing, errno=%d\n",
              FIFO_PATH1, errno);
      exitcode = 3;
      goto errout;
    }

  fd2 = open(FIFO_PATH2, O_WRONLY);
  if (fd2 < 0)
    {
      message("poll_main: Failed to open FIFO %s for writing, errno=%d\n",
              FIFO_PATH2, errno);
      exitcode = 4;
      goto errout;
    }

  /* Start the listeners */

  message("poll_main: Starting poll_listener thread\n");

  ret = pthread_create(&tid1, NULL, poll_listener, NULL);
  if (ret != 0)
    {
      message("poll_main: Failed to create poll_listener thread: %d\n", ret);
      exitcode = 5;
      goto errout;
    }

  message("poll_main: Starting select_listener thread\n");

  ret = pthread_create(&tid2, NULL, select_listener, NULL);
  if (ret != 0)
    {
      message("poll_main: Failed to create select_listener thread: %d\n", ret);
      exitcode = 6;
      goto errout;
    }

#ifdef HAVE_NETPOLL
#ifdef CONFIG_NET_TCPBACKLOG
  message("poll_main: Starting net_listener thread\n");

  ret = pthread_create(&tid3, NULL, net_listener, NULL);
#else
  message("poll_main: Starting net_reader thread\n");

  ret = pthread_create(&tid3, NULL, net_reader, NULL);
#endif
  if (ret != 0)
    {
      message("poll_main: Failed to create net_listener thread: %d\n", ret);
    }
#endif

  /* Loop forever */

  for (count = 0; ; count++)
    {
      /* Send a message to the listener... this should wake the listener
       * from the poll.
       */

      sprintf(buffer, "Message %d", count);
      nbytes = write(fd1, buffer, strlen(buffer));
      if (nbytes < 0)
        {
          message("poll_main: Write to fd1 failed: %d\n", errno);
          exitcode = 7;
          goto errout;
        }

      nbytes = write(fd2, buffer, strlen(buffer));
      if (nbytes < 0)
        {
          message("poll_main: Write fd2 failed: %d\n", errno);
          exitcode = 8;
          goto errout;
        }

      message("\npoll_main: Sent '%s' (%d bytes)\n", buffer, nbytes);
      msgflush();

      /* Wait awhile.  This delay should be long enough that the
       * listener will timeout.
       */

      sleep(WRITER_DELAY);
    }

errout:
  if (fd1 >= 0)
    {
      close(fd1);
    }

  if (fd2 >= 0)
    {
      close(fd2);
    }

  fflush(stdout);
  return exitcode;
}
