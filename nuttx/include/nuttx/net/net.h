/****************************************************************************
 * include/nuttx/net/net.h
 *
 *   Copyright (C) 2007, 2009-2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NET_NET_H
#define __INCLUDE_NUTTX_NET_NET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <semaphore.h>

#include <nuttx/net/uip/uip.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Socket descriptors are the index into the TCB sockets list, offset by the
 * following amount. This offset is used to distinquish file descriptors from
 * socket descriptors
 */

#ifdef CONFIG_NFILE_DESCRIPTORS
# define __SOCKFD_OFFSET CONFIG_NFILE_DESCRIPTORS
#else
# define __SOCKFD_OFFSET 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This defines a bitmap big enough for one bit for each socket option */

typedef uint16_t sockopt_t;

/* This defines the storage size of a timeout value.  This effects only
 * range of supported timeout values.  With an LSB in seciseconds, the
 * 16-bit maximum of 65535 corresponds to 1 hr 49 min 13.5 sec at decisecond
 * resolution.
 */

typedef uint16_t socktimeo_t;

/* This is the internal representation of a socket reference by a file
 * descriptor.
 */

struct socket
{
  int           s_crefs;     /* Reference count on the socket */
  uint8_t       s_type;      /* Protocol type: Only SOCK_STREAM or SOCK_DGRAM */
  uint8_t       s_flags;     /* See _SF_* definitions */
#ifdef CONFIG_NET_SOCKOPTS
  sockopt_t     s_options;   /* Selected socket options */
#ifndef CONFIG_DISABLE_CLOCK
  socktimeo_t   s_rcvtimeo;  /* Receive timeout value (in deciseconds) */
  socktimeo_t   s_sndtimeo;  /* Send timeout value (in deciseconds) */
#endif
#endif
  void         *s_conn;      /* Connection: struct uip_conn or uip_udp_conn */
};

/* This defines a list of sockets indexed by the socket descriptor */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
struct socketlist
{
  sem_t   sl_sem;            /* Manage access to the socket list */
  int16_t sl_crefs;          /* Reference count */
  struct socket sl_sockets[CONFIG_NSOCKET_DESCRIPTORS];
};
#endif

/* Callback from netdev_foreach() */

struct uip_driver_s; /* Forward reference.  See nuttx/net/uip/uip-arch.h */
typedef int (*netdev_callback_t)(FAR struct uip_driver_s *dev, void *arg);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* net_checksd.c *************************************************************/
/* Check if the socket descriptor is valid for the provided TCB and if it
 * supports the requested access.
 */

int net_checksd(int fd, int oflags);

/* net_sockets.c *************************************************************/
/* There interfaces are called only from OS scheduling and iniialization logic
 * under sched/
 */

void weak_function net_initialize(void);
FAR struct socketlist *net_alloclist(void);
int net_addreflist(FAR struct socketlist *list);
int net_releaselist(FAR struct socketlist *list);

/* Given a socket descriptor, return the underly NuttX-specific socket
 * structure.
 */

FAR struct socket *sockfd_socket(int sockfd);

/* socket.c ******************************************************************/
/* socket using underlying socket structure */

int psock_socket(int domain, int type, int protocol, FAR struct socket *psock);

/* net_close.c ***************************************************************/
/* The standard close() operation redirects operations on socket descriptors
 * to this function.
 */

int net_close(int sockfd);

/* Performs the close operation on a socket instance */

int psock_close(FAR struct socket *psock);

/* net_close.c ***************************************************************/
/* Performs the bind() operation on a socket instance */

int psock_bind(FAR struct socket *psock, FAR const struct sockaddr *addr,
               socklen_t addrlen);

/* connect.c *****************************************************************/
/* Performs the connect() operation on a socket instance */

int psock_connect(FAR struct socket *psock, FAR const struct sockaddr *addr,
                  socklen_t addrlen);

/* send.c ********************************************************************/
/* Send using underlying socket structure */

ssize_t psock_send(FAR struct socket *psock, const void *buf, size_t len,
                   int flags);

/* sendto.c ******************************************************************/
/* Sendto using underlying socket structure */

ssize_t psock_sendto(FAR struct socket *psock, FAR const void *buf,
                     size_t len, int flags, FAR const struct sockaddr *to,
                     socklen_t tolen);

/* recvfrom.c ****************************************************************/
/* recvfrom using the underlying socket structure */

ssize_t psock_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                       int flags,FAR struct sockaddr *from,
                       FAR socklen_t *fromlen);

/* recv using the underlying socket structure */

#define psock_recv(psock,buf,len,flags) \
  psock_recvfrom(psock,buf,len,flags,NULL,0)

/* getsockopt.c **************************************************************/
/* getsockopt using the underlying socket structure */

int psock_getsockopt(FAR struct socket *psock, int level, int option,
                     FAR void *value, FAR socklen_t *value_len);

/* setsockopt.c **************************************************************/
/* setsockopt using the underlying socket structure */

int psock_setsockopt(FAR struct socket *psock, int level, int option,
                     FAR const void *value, socklen_t value_len);

/* net_ioctl.c ***************************************************************/
/* The standard ioctl() operation redirects operations on socket descriptors
 * to this function.
 */

int netdev_ioctl(int sockfd, int cmd, unsigned long arg);

/* net_poll.c ****************************************************************/
/* The standard poll() operation redirects operations on socket descriptors
 * to this function.
 */

#ifndef CONFIG_DISABLE_POLL
struct pollfd; /* Forward reference -- see poll.h */

int psock_poll(FAR struct socket *psock, struct pollfd *fds, bool setup);
int net_poll(int sockfd, struct pollfd *fds, bool setup);
#endif

/* net_dup.c *****************************************************************/
/* The standard dup() operation redirects operations on socket descriptors to
 * this function
 */

int net_dup(int sockfd, int minsd);

/* net_dup2.c ****************************************************************/
/* The standard dup2() operation redirects operations on socket descriptors to
 * this function (when both file and socket descriptors are supported)
 */

#if CONFIG_NFILE_DESCRIPTORS > 0
int net_dup2(int sockfd1, int sockfd2);
#else
#  define net_dup2(sockfd1, sockfd2) dup2(sockfd1, sockfd2)
#endif

/* net_clone.c ***************************************************************/
/* Performs the low level, common portion of net_dup() and net_dup2() */

int net_clone(FAR struct socket *psock1, FAR struct socket *psock2);

/* net_vfcntl.c **************************************************************/
/* Performs fcntl operations on socket */

int net_vfcntl(int sockfd, int cmd, va_list ap);

/* netdev-register.c *********************************************************/
/* This function is called by network interface device drivers to inform the
 * socket layer of their existence.  This registration is necesary to support
 * ioctl() operations on network devices to, for example, set MAC and IP
 * addresses
 */

int netdev_register(FAR struct uip_driver_s *dev);

/* netdev-unregister.c *********************************************************/
/* Unregister a network device driver. */

int netdev_unregister(FAR struct uip_driver_s *dev);

/* net_foreach.c ************************************************************/
/* Enumerates all registered network devices */

int netdev_foreach(netdev_callback_t callback, void *arg);

/* drivers/net/slip.c ******************************************************/
/* Instantiate a SLIP network interface. */

#ifdef CONFIG_NET_SLIP
int slip_initialize(int intf, const char *devname);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* __INCLUDE_NUTTX_NET_NET_H */
