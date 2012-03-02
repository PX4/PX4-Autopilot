/****************************************************************************
 * net/setsockopt.c
 *
 *   Copyright (C) 2007, 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_SOCKOPTS)

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <arch/irq.h>

#include "net_internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_setsockopt
 *
 * Description:
 *   psock_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket on the 'psock' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Parameters:
 *   psock     Socket structure of socket to operate on
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *  0 on success; -1 on failure
 *
 *  EDOM
 *    The send and receive timeout values are too big to fit into the
 *    timeout fields in the socket structure.
 *  EINVAL
 *    The specified option is invalid at the specified socket 'level' or the
 *    socket has been shut down.
 *  EISCONN
 *    The socket is already connected, and a specified option cannot be set
 *    while the socket is connected.
 *  ENOPROTOOPT
 *    The 'option' is not supported by the protocol.
 *  ENOTSOCK
 *    The 'sockfd' argument does not refer to a socket.
 *  ENOMEM
 *    There was insufficient memory available for the operation to complete.
 *  ENOBUFS
 *    Insufficient resources are available in the system to complete the
 *    call.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_setsockopt(FAR struct socket *psock, int level, int option,
                     FAR const void *value, socklen_t value_len)
{
  uip_lock_t flags;
  int err;

  /* Verify that the socket option if valid (but might not be supported ) */

  if (!_SO_SETVALID(option) || !value)
    {
      err = EINVAL;
      goto errout;
    }

  /* Process the option */
  switch (option)
    {
      /* The following options take a point to an integer boolean value.
       * We will blindly set the bit here although the implementation
       * is outside of the scope of setsockopt.
       */

      case SO_DEBUG:      /* Enables recording of debugging information */
      case SO_BROADCAST:  /* Permits sending of broadcast messages */
      case SO_REUSEADDR:  /* Allow reuse of local addresses */
      case SO_KEEPALIVE:  /* Keeps connections active by enabling the
                           * periodic transmission */
      case SO_OOBINLINE:  /* Leaves received out-of-band data inline */
      case SO_DONTROUTE:  /* Requests outgoing messages bypass standard routing */
        {
          int setting;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (value_len != sizeof(int))
            {
              err = EINVAL;
              goto errout;
           }

          /* Get the value.  Is the option being set or cleared? */

          setting = *(int*)value;

          /* Disable interrupts so that there is no conflict with interrupt
           * level access to options.
           */

           flags = uip_lock();

          /* Set or clear the option bit */

          if (setting)
            {
              _SO_SETOPT(psock->s_options, option);
            }
          else
            {
              _SO_CLROPT(psock->s_options, option);
            }
          uip_unlock(flags);
        }
        break;

      /* The following are valid only if the OS CLOCK feature is enabled */

      case SO_RCVTIMEO:
      case SO_SNDTIMEO:
#ifndef CONFIG_DISABLE_CLOCK
        {
          socktimeo_t timeo;

          /* Verify that option is the size of an 'struct timeval'. */

          if (value_len != sizeof(struct timeval))
            {
              err = EINVAL;
              goto errout;
           }

          /* Get the timeout value */

          timeo = net_timeval2dsec((struct timeval *)value);

          /* Save the timeout value */

          if (option == SO_RCVTIMEO)
            {
              psock->s_rcvtimeo = timeo;
            }
          else
            {
              psock->s_sndtimeo = timeo;
            }

          /* Set/clear the corresponding enable/disable bit */

          if (timeo)
            {
              _SO_CLROPT(psock->s_options, option);
            }
          else
            {
              _SO_SETOPT(psock->s_options, option);
            }
        }
        break;
#endif

      /* The following are not yet implemented */

      case SO_LINGER:
      case SO_SNDBUF:     /* Sets send buffer size */
      case SO_RCVBUF:     /* Sets receive buffer size */
      case SO_RCVLOWAT:   /* Sets the minimum number of bytes to input */
      case SO_SNDLOWAT:   /* Sets the minimum number of bytes to output */

      /* There options are only valid when used with getopt */

      case SO_ACCEPTCONN: /* Reports whether socket listening is enabled */
      case SO_ERROR:      /* Reports and clears error status. */
      case SO_TYPE:       /* Reports the socket type */

      default:
        err = ENOPROTOOPT;
        goto errout;
    }
  return OK;

errout:
  set_errno(err);
  return ERROR;
}

/****************************************************************************
 * Function: setsockopt
 *
 * Description:
 *   setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket associated with the
 *   file descriptor specified by the 'sockfd' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument as SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *  0 on success; -1 on failure
 *
 *  EBADF
 *    The 'sockfd' argument is not a valid socket descriptor.
 *  EDOM
 *    The send and receive timeout values are too big to fit into the
 *    timeout fields in the socket structure.
 *  EINVAL
 *    The specified option is invalid at the specified socket 'level' or the
 *    socket has been shut down.
 *  EISCONN
 *    The socket is already connected, and a specified option cannot be set
 *    while the socket is connected.
 *  ENOPROTOOPT
 *    The 'option' is not supported by the protocol.
 *  ENOTSOCK
 *    The 'sockfd' argument does not refer to a socket.
 *  ENOMEM
 *    There was insufficient memory available for the operation to complete.
 *  ENOBUFS
 *    Insufficient resources are available in the system to complete the
 *    call.
 *
 * Assumptions:
 *
 ****************************************************************************/

int setsockopt(int sockfd, int level, int option, const void *value, socklen_t value_len)
{
  FAR struct socket *psock;

  /* Get the underlying socket structure */
  /* Verify that the sockfd corresponds to valid, allocated socket */

  psock = sockfd_socket(sockfd);
  if (!psock || psock->s_crefs <= 0)
    {
      set_errno(EBADF);
      return ERROR;
    }

  /* Then let psock_setockopt() do all of the work */

  return psock_setsockopt(psock, level, option, value, value_len);
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS */
