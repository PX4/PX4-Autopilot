/****************************************************************************
 * fs/fs_select.c
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

#include <nuttx/config.h>

#include <sys/select.h>

#include <string.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs.h>

#include "fs_internal.h"

#ifndef CONFIG_DISABLE_POLL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: select
 *
 * Description:
 *   select() allows a program to monitor multiple file descriptors, waiting
 *   until one or more of the file descriptors become "ready" for some class
 *   of I/O operation (e.g., input possible).  A file descriptor is
 *   considered  ready if it is possible to perform the corresponding I/O
 *   operation (e.g., read(2)) without blocking.
 *
 *   NOTE: poll() is the fundamental API for performing such monitoring
 *   operation under NuttX.  select() is provided for compatibility and
 *   is simply a layer of added logic on top of poll().  As such, select()
 *   is more wasteful of resources and poll() is the recommended API to be
 *   used.
 *
 * Input parameters:
 *   nfds - the maximum fd number (+1) of any descriptor in any of the
 *     three sets.
 *   readfds - the set of descriptions to monitor for read-ready events
 *   writefds - the set of descriptions to monitor for write-ready events
 *   exceptfds - the set of descriptions to monitor for error events
 *   timeout - Return at this time if none of these events of interest
 *     occur.
 *
 *  Return:
 *   0: Timer expired
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int select(int nfds, FAR fd_set *readfds, FAR fd_set *writefds,
           FAR fd_set *exceptfds, FAR struct timeval *timeout)
{
  struct pollfd *pollset;
  int fd;
  int npfds;
  int msec;
  int ndx;
  int ret;

  /* Allocate the descriptor list for poll() */

  pollset = (struct pollfd *)kzalloc(nfds * sizeof(struct pollfd));
  if (!pollset)
    {
      errno = ENOMEM;
      return ERROR;
    }

  /* Initialize the descriptor list for poll() */

  for (fd = 0, npfds = 0; fd < nfds; fd++)
    {
      int incr = 0;

      /* The readfs set holds the set of FDs that the caller can be assured
       * of reading from without blocking.  Note that POLLHUP is included as
       * a read-able condition.  POLLHUP will be reported at the end-of-file
       * or when a connection is lost.  In either case, the read() can then
       * be performed without blocking.
       */

      if (readfds && FD_ISSET(fd, readfds))
        {
           pollset[npfds].fd      = fd;
           pollset[npfds].events |= POLLIN;
           incr                   = 1;
        }

      /* The writefds set holds the set of FDs that the caller can be assured
       * of writing to without blocking.
       */

      if (writefds && FD_ISSET(fd, writefds))
        {
           pollset[npfds].fd      = fd;
           pollset[npfds].events |= POLLOUT;
           incr                   = 1;
        }

      /* The exceptfds set holds the set of FDs that are watched for exceptions */

      if (exceptfds && FD_ISSET(fd, exceptfds))
        {
           pollset[npfds].fd      = fd;
           incr                   = 1;
        }

      npfds += incr;
    }

  /* Convert the timeout to milliseconds */

  msec = timeout->tv_sec * 1000 + timeout->tv_usec / 1000;

  /* Then let poll do all of the real work. */

  ret = poll(pollset, npfds, msec);

  /* Now set up the return values */

  if (readfds)
    {
      memset(readfds, 0, sizeof(fd_set));
    }

  if (writefds)
    {
      memset(writefds, 0, sizeof(fd_set));
    }

  if (exceptfds)
    {
      memset(exceptfds, 0, sizeof(fd_set));
    }

  /* Convert the poll descriptor list back into selects 3 bitsets */

  if (ret > 0)
    {
      ret = 0;
      for (ndx = 0; ndx < npfds; ndx++)
        {
          /* Check for read conditions.  Note that POLLHUP is included as a
           * read condition.  POLLHUP will be reported when no more data will
           * be available (such as when a connection is lost).  In either
           * case, the read() can then be performed without blocking.
           */

          if (readfds)
            {
              if (pollset[ndx].revents & (POLLIN|POLLHUP))
                {
                  FD_SET(pollset[ndx].fd, readfds);
                  ret++;
                }
            }

          /* Check for write conditions */

          if (writefds)
            {
              if (pollset[ndx].revents & POLLOUT)
                {
                  FD_SET(pollset[ndx].fd, writefds);
                  ret++;
                }
            }

          /* Check for exceptions */

          if (exceptfds)
            {
              if (pollset[ndx].revents & POLLERR)
                {
                  FD_SET(pollset[ndx].fd, exceptfds);
                  ret++;
                }
            }
        }
    }

  kfree(pollset);
  return ret;
}

#endif /* CONFIG_DISABLE_POLL */

