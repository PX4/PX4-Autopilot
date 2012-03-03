/****************************************************************************
 * net/net_vfcntl.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <stdint.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/irq.h>
#include <nuttx/net/net.h>
#include "net_internal.h"

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

int net_vfcntl(int sockfd, int cmd, va_list ap)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  uip_lock_t flags;
  int err = 0;
  int ret = 0;

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Interrupts must be disabled in order to perform operations on socket structures */

  flags = uip_lock();
  switch (cmd)
    {
      case F_DUPFD:
        /* Return a new file descriptor which shall be the lowest numbered
         * available (that is, not already open) file descriptor greater than
         * or equal to the third argument, arg, taken as an integer of type
         * int. The new file descriptor shall refer to the same open file
         * description as the original file descriptor, and shall share any
         * locks.  The FD_CLOEXEC flag associated  with the new file descriptor
         * shall be cleared to keep the file open across calls to one of the
         * exec functions.
         */

        {
          ret = net_dup(sockfd, va_arg(ap, int));
        }
        break;

      case F_GETFD:
        /* Get the file descriptor flags defined in <fcntl.h> that are associated
         * with the file descriptor fildes.  File descriptor flags are associated
         * with a single file descriptor and do not affect other file descriptors
         * that refer to the same file.
         */

      case F_SETFD:
        /* Set the file descriptor flags defined in <fcntl.h>, that are associated
         * with fildes, to the third argument, arg, taken as type int. If the
         * FD_CLOEXEC flag in the third argument is 0, the file shall remain open
         * across the exec functions; otherwise, the file shall be closed upon
         * successful execution of one  of  the  exec  functions.
         */

         err = ENOSYS; /* F_GETFD and F_SETFD not implemented */
         break;

      case F_GETFL:
        /* Get the file status flags and file access modes, defined in <fcntl.h>,
         * for the file description associated with fildes. The file access modes
         * can be extracted from the return value using the mask O_ACCMODE, which is
         * defined  in <fcntl.h>. File status flags and file access modes are associated
         * with the file description and do not affect other file descriptors that
         * refer to the same file with different open file descriptions.
         */

        {
          /* This summarizes the behavior of the NuttX/uIP sockets */

          ret = O_RDWR | O_SYNC | O_RSYNC;

          /* TCP/IP sockets may also be non-blocking if read-ahead is enabled */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
          if (psock->s_type == SOCK_STREAM && _SS_ISNONBLOCK(psock->s_flags))
            {
              ret |= O_NONBLOCK;
            }
#endif
        }
        break;

      case F_SETFL:
        /* Set the file status flags, defined in <fcntl.h>, for the file description
         * associated with fildes from the corresponding  bits in the third argument,
         * arg, taken as type int. Bits corresponding to the file access mode and
         * the file creation flags, as defined in <fcntl.h>, that are set in arg shall
         * be ignored. If any bits in arg other than those mentioned here are changed
         * by the application, the result is unspecified.
         */

        {
           /* Non-blocking is the only configurable option.  And it applies only to
            * read operations on TCP/IP sockets when read-ahead is enabled.
            */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
          int mode =  va_arg(ap, int);
          if (psock->s_type == SOCK_STREAM)
            {
               if ((mode & O_NONBLOCK) != 0)
                 {
                   psock->s_flags |= _SF_NONBLOCK;
                 }
               else
                 {
                   psock->s_flags &= ~_SF_NONBLOCK;
                 }
            }
#endif
        }
        break;

      case F_GETOWN:
        /* If fildes refers to a socket, get the process or process group ID specified
         * to receive SIGURG signals when out-of-band data is available. Positive values
         * indicate a process ID; negative values, other than -1, indicate a process group
         * ID. If fildes does not refer to a socket, the results are unspecified.
         */

      case F_SETOWN:
        /* If fildes refers to a socket, set the process or process group ID specified
         * to receive SIGURG signals when out-of-band data is available, using the value
         * of the third argument, arg, taken as type int. Positive values indicate a 
         * process ID; negative values, other than -1, indicate a process group ID. If
         * fildes does not refer to a socket, the results are unspecified.
         */

      case F_GETLK:
        /* Get the first lock which blocks the lock description pointed to by the third
         * argument, arg, taken as a pointer to type struct flock, defined in <fcntl.h>.
         * The information retrieved shall overwrite the information passed to fcntl() in
         * the structure flock. If no lock is found that would prevent this lock from being
         * created, then the structure shall be left unchanged except for the lock type
         * which shall be set to F_UNLCK.
         */

      case F_SETLK:
        /* Set or clear a file segment lock according to the lock description pointed to
         * by the third argument, arg, taken as a pointer to type struct flock, defined in
         * <fcntl.h>. F_SETLK can establish shared (or read) locks (F_RDLCK) or exclusive
         * (or write) locks (F_WRLCK), as well  as  to  remove  either  type  of  lock  (F_UNLCK).
         * F_RDLCK, F_WRLCK, and F_UNLCK are defined in <fcntl.h>. If a shared or exclusive
         * lock cannot be set, fcntl() shall return immediately with a return value of -1.
         */

      case F_SETLKW:
        /* This command shall be equivalent to F_SETLK except that if a shared or exclusive
         * lock is blocked by other locks, the thread shall wait until the request can be
         * satisfied. If a signal that is to be caught is received while fcntl() is waiting
         * for a region, fcntl() shall be interrupted. Upon return from the signal handler,
         * fcntl() shall return -1 with errno set to [EINTR], and the lock operation shall
         * not be done.
         */

         err = ENOSYS; /* F_GETOWN, F_SETOWN, F_GETLK, F_SETLK, F_SETLKW */
         break;

      default:
         err = EINVAL;
         break;
  }

  uip_unlock(flags);

errout:
  if (err != 0)
    {
      errno = err;
      return ERROR;
    }
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS > 0 */

