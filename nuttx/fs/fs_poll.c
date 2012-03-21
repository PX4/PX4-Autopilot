/****************************************************************************
 * fs/fs_poll.c
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

#include <stdint.h>
#include <stdbool.h>
#include <poll.h>
#include <wdog.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>

#include "fs_internal.h"

#ifndef CONFIG_DISABLE_POLL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define poll_semgive(sem) sem_post(sem)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll_semtake
 ****************************************************************************/

static void poll_semtake(FAR sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: poll_fdsetup
 *
 * Description:
 *   Configure (or unconfigure) one file/socket descriptor for the poll
 *   operation.  If fds and sem are non-null, then the poll is being setup.
 *   if fds and sem are NULL, then the poll is being torn down.
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int poll_fdsetup(int fd, FAR struct pollfd *fds, bool setup)
{
  FAR struct filelist *list;
  FAR struct file     *this_file;
  FAR struct inode    *inode;
  int                  ret = -ENOSYS;

  /* Check for a valid file descriptor */

  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Perform the socket ioctl */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
      if ((unsigned int)fd < (CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS))
        {
          return net_poll(fd, fds, setup);
        }
      else
#endif
        {
          return -EBADF;
        }
    }

  /* Get the thread-specific file list */

  list = sched_getfiles();
  if (!list)
    {
      return -EMFILE;
    }

  /* Is a driver registered? Does it support the poll method?
   * If not, return -ENOSYS
   */

  this_file = &list->fl_files[fd];
  inode     = this_file->f_inode;

  if (inode && inode->u.i_ops && inode->u.i_ops->poll)
    {
      /* Yes, then setup the poll */

      ret = (int)inode->u.i_ops->poll(this_file, fds, setup);
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: poll_setup
 *
 * Description:
 *   Setup the poll operation for each descriptor in the list.
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static inline int poll_setup(FAR struct pollfd *fds, nfds_t nfds, sem_t *sem)
{
  int ret;
  int i;

  /* Process each descriptor in the list */

  for (i = 0; i < nfds; i++)
    {
      /* Setup the poll descriptor */

      fds[i].sem     = sem;
      fds[i].revents = 0;
      fds[i].priv    = NULL;

      /* Set up the poll */

      ret = poll_fdsetup(fds[i].fd, &fds[i], true);
      if (ret < 0)
        {
          return ret;
        }
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: poll_teardown
 *
 * Description:
 *   Teardown the poll operation for each descriptor in the list and return
 *   the count of non-zero poll events.
 *
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static inline int poll_teardown(FAR struct pollfd *fds, nfds_t nfds, int *count)
{
  int status;
  int ret = OK;
  int i;

  /* Process each descriptor in the list */

  *count = 0;
  for (i = 0; i < nfds; i++)
    {
      /* Teardown the poll */

      status = poll_fdsetup(fds[i].fd, &fds[i], false);
      if (status < 0)
        {
          ret = status;
        }

      /* Check if any events were posted */

      if (fds[i].revents != 0)
        {
          (*count)++;
        }

      /* Un-initialize the poll structure */

      fds[i].sem = NULL;
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: poll_timeout
 *
 * Description:
 *   The wdog expired before any other events were received.
 *
 ****************************************************************************/

static void poll_timeout(int argc, uint32_t isem, ...)
{
  /* Wake up the poller */

  FAR sem_t *sem = (FAR sem_t *)isem;
  poll_semgive(sem);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: poll
 *
 * Description:
 *   poll() waits for one of a set of file descriptors to become ready to
 *   perform I/O.  If none of the events requested (and no error) has
 *   occurred for any of  the  file  descriptors,  then  poll() blocks until
 *   one of the events occurs.
 *
 * Inputs:
 *   fds  - List of structures describing file descriptors to be monitored
 *   nfds - The number of entries in the list
 *   timeout - Specifies an upper limit on the time for which poll() will
 *     block in milliseconds.  A negative value of timeout means an infinite
 *     timeout.
 *
 * Return:
 *   On success, the number of structures that have nonzero revents fields.
 *   A value of 0 indicates that the call timed out and no file descriptors
 *   were ready.  On error, -1 is returned, and errno is set appropriately:
 *
 *   EBADF - An invalid file descriptor was given in one of the sets.
 *   EFAULT - The fds address is invalid
 *   EINTR - A signal occurred before any requested event.
 *   EINVAL - The nfds value exceeds a system limit.
 *   ENOMEM - There was no space to allocate internal data structures.
 *   ENOSYS - One or more of the drivers supporting the file descriptor
 *     does not support the poll method.
 *
 ****************************************************************************/

int poll(FAR struct pollfd *fds, nfds_t nfds, int timeout)
{
  WDOG_ID wdog;
  sem_t sem;
  int count = 0;
  int ret;

  sem_init(&sem, 0, 0);
  ret = poll_setup(fds, nfds, &sem);
  if (ret >= 0)
    {
      if (timeout >= 0)
        {
          /* Wait for the poll event with a timeout.  Note that the
           * millisecond timeout has to be converted to system clock
           * ticks for wd_start
           */

          wdog = wd_create();
          wd_start(wdog,  MSEC2TICK(timeout), poll_timeout, 1, (uint32_t)&sem);
          poll_semtake(&sem);
          wd_delete(wdog);
        }
      else
        {
          /* Wait for the poll event with no timeout */

          poll_semtake(&sem);
        }

      /* Teardown the poll operation and get the count of events */

      ret = poll_teardown(fds, nfds, &count);
    }
  sem_destroy(&sem);

  /* Check for errors */

  if (ret < 0)
    {
      errno = -ret;
      return ERROR;
    }

  return count;
}

#endif /* CONFIG_DISABLE_POLL */

