/****************************************************************************
 * net/net_clone.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#ifdef CONFIG_NET

#include <sys/socket.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "net_internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_clone
 *
 * Description:
 *   Performs the low level, common portion of net_dup() and net_dup2()
 *
 ****************************************************************************/

int net_clone(FAR struct socket *psock1, FAR struct socket *psock2)
{
  uip_lock_t flags;
  int ret = OK;

  /* Parts of this operation need to be atomic */

  flags = uip_lock();

  /* Duplicate the socket state */

  psock2->s_type     = psock1->s_type;      /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  psock2->s_flags    = psock1->s_flags;     /* See _SF_* definitions */
#ifdef CONFIG_NET_SOCKOPTS
  psock2->s_options  = psock1->s_options;   /* Selected socket options */
#endif
#ifndef CONFIG_DISABLE_CLOCK
  psock2->s_rcvtimeo = psock1->s_rcvtimeo;  /* Receive timeout value (in deciseconds) */
  psock2->s_sndtimeo = psock1->s_sndtimeo;  /* Send timeout value (in deciseconds) */
#endif
  psock2->s_conn     = psock1->s_conn;      /* UDP or TCP connection structure */

  /* Increment the reference count on the connection */

  DEBUGASSERT(psock2->s_conn);
  psock2->s_crefs    = 1;                   /* One reference on the new socket itself */

#ifdef CONFIG_NET_TCP
  if (psock2->s_type == SOCK_STREAM)
    {
      struct uip_conn *conn = psock2->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
#ifdef CONFIG_NET_UDP
  if (psock2->s_type == SOCK_DGRAM)
    {
      struct uip_udp_conn *conn = psock2->s_conn;
      DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
      conn->crefs++;
    }
  else
#endif
    {
      ndbg("Unsupported type: %d\n", psock2->s_type);
      ret = -EBADF;
    }

  uip_unlock(flags);
  return ret;
}

#endif /* CONFIG_NET */


