/****************************************************************************
 * net/net_internal.h
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#ifndef __NET_INTERNAL_H
#define __NET_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/types.h>
#include <stdint.h>
#include <time.h>

#include <nuttx/net/net.h>
#include <nuttx/net/uip/uip.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Definitions of 8-bit socket flags */

                                  /* Bits 0-2: Socket state */
#define _SF_IDLE            0x00  /* - There is no socket activity */
#define _SF_ACCEPT          0x01  /* - Socket is waiting to accept a connection */
#define _SF_RECV            0x02  /* - Waiting for recv action to complete */
#define _SF_SEND            0x03  /* - Waiting for send action to complete */
#define _SF_MASK            0x03  /* - Mask to isolate the above actions */

#define _SF_NONBLOCK        0x08  /* Bit 3: Don't block if no data (TCP/READ only) */
#define _SF_LISTENING       0x10  /* Bit 4: SOCK_STREAM is listening */
#define _SF_BOUND           0x20  /* Bit 5: SOCK_STREAM is bound to an address */
                                  /* Bits 6-7: Connection state */
#define _SF_CONNECTED       0x40  /* Bit 6: SOCK_STREAM is connected */
#define _SF_CLOSED          0x80  /* Bit 7: SOCK_STREAM was gracefully disconnected */

/* Connection state encoding:
 *
 *  _SF_CONNECTED==1 && _SF_CLOSED==0 - the socket is connected
 *  _SF_CONNECTED==0 && _SF_CLOSED==1 - the socket was gracefully disconnected
 *  _SF_CONNECTED==0 && _SF_CLOSED==0 - the socket was rudely disconnected
 */

/* Macro to manage the socket state and flags */

#define _SS_SETSTATE(s,f)   (((s) & ~_SF_MASK) | (f))
#define _SS_GETSTATE(s)     ((s) & _SF_MASK)
#define _SS_ISBUSY(s)       (_SS_GETSTATE(s) != _SF_IDLE)

#define _SS_ISNONBLOCK(s)   (((s) & _SF_NONBLOCK)  != 0)
#define _SS_ISLISTENING(s)  (((s) & _SF_LISTENING) != 0)
#define _SS_ISBOUND(s)      (((s) & _SF_CONNECTED) != 0)
#define _SS_ISCONNECTED(s)  (((s) & _SF_CONNECTED) != 0)
#define _SS_ISCLOSED(s)     (((s) & _SF_CLOSED) != 0)

/* This macro converts a socket option value into a bit setting */

#define _SO_BIT(o)       (1 << (o))

/* These define bit positions for each socket option (see sys/socket.h) */

#define _SO_DEBUG        _SO_BIT(SO_DEBUG)
#define _SO_ACCEPTCONN   _SO_BIT(SO_ACCEPTCONN)
#define _SO_BROADCAST    _SO_BIT(SO_BROADCAST)
#define _SO_REUSEADDR    _SO_BIT(SO_REUSEADDR)
#define _SO_KEEPALIVE    _SO_BIT(SO_KEEPALIVE)
#define _SO_LINGER       _SO_BIT(SO_LINGER)
#define _SO_OOBINLINE    _SO_BIT(SO_OOBINLINE)
#define _SO_SNDBUF       _SO_BIT(SO_SNDBUF)
#define _SO_RCVBUF       _SO_BIT(SO_RCVBUF)
#define _SO_ERROR        _SO_BIT(SO_ERROR)
#define _SO_TYPE         _SO_BIT(SO_TYPE)
#define _SO_DONTROUTE    _SO_BIT(SO_DONTROUTE)
#define _SO_RCVLOWAT     _SO_BIT(SO_RCVLOWAT)
#define _SO_RCVTIMEO     _SO_BIT(SO_RCVTIMEO)
#define _SO_SNDLOWAT     _SO_BIT(SO_SNDLOWAT)
#define _SO_SNDTIMEO     _SO_BIT(SO_SNDTIMEO)

/* This is the larget option value */

#define _SO_MAXOPT       (15)

/* Macros to set, test, clear options */

#define _SO_SETOPT(s,o)  ((s) |= _SO_BIT(o))
#define _SO_CLROPT(s,o)  ((s) &= ~_SO_BIT(o))
#define _SO_GETOPT(s,o)  (((s) & _SO_BIT(o)) != 0)

/* These are macros that can be used to determine if socket option code is
 * valid (in range) and supported by API.
 */

#define _SO_GETONLYSET   (_SO_ACCEPTCONN|_SO_ERROR|_SO_TYPE)
#define _SO_GETONLY(o)   ((_SO_BIT(o) & _SO_GETONLYSET) != 0)
#define _SO_GETVALID(o)  (((unsigned int)(o)) <= _SO_MAXOPT)
#define _SO_SETVALID(o)  ((((unsigned int)(o)) <= _SO_MAXOPT) && !_SO_GETONLY(o))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* List of registered ethernet device drivers */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
EXTERN struct uip_driver_s *g_netdevices;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* net_sockets.c *************************************************************/

int  sockfd_allocate(int minsd);
void sock_release(FAR struct socket *psock);
void sockfd_release(int sockfd);
FAR struct socket *sockfd_socket(int sockfd);

/* net_connect.c *************************************************************/

#ifdef CONFIG_NET_TCP
int net_startmonitor(FAR struct socket *psock);
void net_stopmonitor(FAR struct uip_conn *conn);
void net_lostconnection(FAR struct socket *psock, uint16_t flags);
#endif

/* net_close.c ***************************************************************/

int psock_close(FAR struct socket *psock);

/* sockopt support ***********************************************************/

#if defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)
int net_timeo(uint32_t start_time, socktimeo_t timeo);
socktimeo_t net_timeval2dsec(FAR struct timeval *tv);
void net_dsec2timeval(uint16_t dsec, FAR struct timeval *tv);
#endif

/* net_register.c ************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
void netdev_seminit(void);
void netdev_semtake(void);
void netdev_semgive(void);
#endif

/* net_findbyname.c **********************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct uip_driver_s *netdev_findbyname(FAR const char *ifname);
#endif

/* net_findbyaddr.c **********************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct uip_driver_s *netdev_findbyaddr(FAR const uip_ipaddr_t *raddr);
#endif

/* net_txnotify.c ************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
void netdev_txnotify(const uip_ipaddr_t *raddr);
#endif

/* net_count.c ***************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
int netdev_count(void);
#endif

/* net_arptimer.c ************************************************************/

#ifdef CONFIG_NET_ARP
void arptimer_init(void);
#else
# define arptimer_init()
#endif

/* send.c ********************************************************************/

ssize_t psock_send(FAR struct socket *psock, FAR const void *buf, size_t len,
                   int flags);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NET */
#endif /* __NET_INTERNAL_H */
