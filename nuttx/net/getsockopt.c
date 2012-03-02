/****************************************************************************
 * net/getsockopt.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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

#include "net_internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: psock_getsockopt
 *
 * Description:
 *   getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *
 *  EINVAL
 *    The specified option is invalid at the specified socket 'level' or the
 *    socket has been shutdown.
 *  ENOPROTOOPT
 *    The 'option' is not supported by the protocol.
 *  ENOTSOCK
 *    The 'psock' argument does not refer to a socket.
 *  ENOBUFS
 *    Insufficient resources are available in the system to complete the
 *    call.
 *
 * Assumptions:
 *
 ****************************************************************************/

int psock_getsockopt(FAR struct socket *psock, int level, int option,
                     FAR void *value, FAR socklen_t *value_len)
{
  int err;

  /* Verify that the socket option if valid (but might not be supported ) */

  if (!_SO_GETVALID(option) || !value || !value_len)
    {
      err = EINVAL;
      goto errout;
    }

  /* Process the option */

  switch (option)
    {
      /* The following options take a point to an integer boolean value.
       * We will blindly report the bit here although the implementation
       * is outside of the scope of getsockopt.
       */

      case SO_DEBUG:      /* Enables recording of debugging information */
      case SO_BROADCAST:  /* Permits sending of broadcast messages */
      case SO_REUSEADDR:  /* Allow reuse of local addresses */
      case SO_KEEPALIVE:  /* Keeps connections active by enabling the
                           * periodic transmission */
      case SO_OOBINLINE:  /* Leaves received out-of-band data inline */
      case SO_DONTROUTE:  /* Requests outgoing messages bypass standard routing */
        {
          sockopt_t optionset;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (*value_len < sizeof(int))
            {
              err = EINVAL;
              goto errout;
           }

          /* Sample the current options.  This is atomic operation and so
           * should not require any special steps for thread safety. We
           * this outside of the macro because you can never be sure what
           * a macro will do.
           */

          optionset    = psock->s_options;
          *(int*)value = _SO_GETOPT(optionset, option);
          *value_len   = sizeof(int);
        }
        break;

      case SO_TYPE:       /* Reports the socket type */
        {
          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (*value_len < sizeof(int))
            {
              err = EINVAL;
              goto errout;
           }

          /* Return the socket type */

          *(int*)value = psock->s_type;
          *value_len   = sizeof(int);
        }
        break;

      /* The following are valid only if the OS CLOCK feature is enabled */

      case SO_RCVTIMEO:
      case SO_SNDTIMEO:
#ifndef CONFIG_DISABLE_CLOCK
        {
          socktimeo_t timeo;

          /* Verify that option is the size of an 'int'.  Should also check
           * that 'value' is properly aligned for an 'int'
           */

          if (*value_len < sizeof(struct timeval))
            {
              err = EINVAL;
              goto errout;
           }

          /* Get the timeout value.  This is a atomic operation and should
         * require no special operation.
         */

          if (option == SO_RCVTIMEO)
            {
              timeo = psock->s_rcvtimeo;
            }
          else
            {
              timeo = psock->s_sndtimeo;
            }

          /* Then return the timeout value to the caller */

          net_dsec2timeval(timeo, (struct timeval *)value);
          *value_len   = sizeof(struct timeval);
        }
        break;
#endif

      /* The following are not yet implemented */

      case SO_ACCEPTCONN: /* Reports whether socket listening is enabled */
      case SO_LINGER:
      case SO_SNDBUF:     /* Sets send buffer size */
      case SO_RCVBUF:     /* Sets receive buffer size */
      case SO_ERROR:      /* Reports and clears error status. */
      case SO_RCVLOWAT:   /* Sets the minimum number of bytes to input */
      case SO_SNDLOWAT:   /* Sets the minimum number of bytes to output */

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
 * Function: getsockopt
 *
 * Description:
 *   getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'sockfd' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET.
 *
 *   See <sys/socket.h> a complete list of values for the 'option' argument.
 *
 * Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *
 *  EBADF
 *    The 'sockfd' argument is not a valid socket descriptor.
 *  EINVAL
 *    The specified option is invalid at the specified socket 'level' or the
 *    socket has been shutdown.
 *  ENOPROTOOPT
 *    The 'option' is not supported by the protocol.
 *  ENOTSOCK
 *    The 'sockfd' argument does not refer to a socket.
 *  ENOBUFS
 *    Insufficient resources are available in the system to complete the
 *    call.
 *
 * Assumptions:
 *
 ****************************************************************************/

int getsockopt(int sockfd, int level, int option, void *value, socklen_t *value_len)
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

  /* Then let psock_getsockopt() do all of the work */

  return psock_getsockopt(psock, level, option, value, value_len);
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS */
