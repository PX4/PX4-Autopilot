/****************************************************************************
 * net/uip/uip_internal.h
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __UIP_INTERNAL_H
#define __UIP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <arch/irq.h>
#include <nuttx/net/uip/uip.h>

/****************************************************************************
 * Public Macro Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uip_ipaddr_t g_alloneaddr;
extern const uip_ipaddr_t g_allzeroaddr;

/* Increasing number used for the IP ID field. */

extern uint16_t g_ipid;

/* Reassembly timer (units: deci-seconds) */

#if UIP_REASSEMBLY && !defined(CONFIG_NET_IPv6)
extern uint8_t uip_reasstmr;
#endif

/* List of applications waiting for ICMP ECHO REPLY */

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING)
extern struct uip_callback_s *g_echocallback;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Defined in uip_callback.c ************************************************/

EXTERN void uip_callbackinit(void);
EXTERN FAR struct uip_callback_s *uip_callbackalloc(struct uip_callback_s **list);
EXTERN void uip_callbackfree(FAR struct uip_callback_s *cb, struct uip_callback_s **list);
EXTERN uint16_t uip_callbackexecute(FAR struct uip_driver_s *dev, void *pvconn,
                                  uint16_t flags, FAR struct uip_callback_s *list);

#ifdef CONFIG_NET_TCP
/* Defined in uip_tcpconn.c *************************************************/

EXTERN void uip_tcpinit(void);
EXTERN struct uip_conn *uip_tcpactive(struct uip_tcpip_hdr *buf);
EXTERN struct uip_conn *uip_nexttcpconn(struct uip_conn *conn);
EXTERN struct uip_conn *uip_tcplistener(uint16_t portno);
EXTERN struct uip_conn *uip_tcpaccept(struct uip_tcpip_hdr *buf);

/* Defined in uip_tcpseqno.c ************************************************/

EXTERN void uip_tcpsetsequence(FAR uint8_t *seqno, uint32_t value);
EXTERN uint32_t uip_tcpgetsequence(FAR uint8_t *seqno);
EXTERN uint32_t uip_tcpaddsequence(FAR uint8_t *seqno, uint16_t len);
EXTERN void uip_tcpinitsequence(FAR uint8_t *seqno);
EXTERN void uip_tcpnextsequence(void);

/* Defined in uip_tcppoll.c *************************************************/

EXTERN void uip_tcppoll(struct uip_driver_s *dev, struct uip_conn *conn);

/* Defined in uip_udptimer.c ************************************************/

EXTERN void uip_tcptimer(struct uip_driver_s *dev, struct uip_conn *conn, int hsec);

/* Defined in uip_listen.c **************************************************/

EXTERN void uip_listeninit(void);
EXTERN bool uip_islistener(uint16_t port);
EXTERN int uip_accept(struct uip_driver_s *dev, struct uip_conn *conn, uint16_t portno);

/* Defined in uip_tcpsend.c *************************************************/

EXTERN void uip_tcpsend(struct uip_driver_s *dev, struct uip_conn *conn,
                        uint16_t flags, uint16_t len);
EXTERN void uip_tcpreset(struct uip_driver_s *dev);
EXTERN void uip_tcpack(struct uip_driver_s *dev, struct uip_conn *conn,
                       uint8_t ack);

/* Defined in uip_tcpappsend.c **********************************************/

EXTERN void uip_tcpappsend(struct uip_driver_s *dev, struct uip_conn *conn,
                           uint16_t result);
EXTERN void uip_tcprexmit(struct uip_driver_s *dev, struct uip_conn *conn,
                          uint16_t result);

/* Defined in uip_tcpinput.c ************************************************/

EXTERN void uip_tcpinput(struct uip_driver_s *dev);

/* Defined in uip_tcpcallback.c *********************************************/

EXTERN uint16_t uip_tcpcallback(FAR struct uip_driver_s *dev,
                                FAR struct uip_conn *conn, uint16_t flags);
#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
EXTERN uint16_t uip_datahandler(FAR struct uip_conn *conn,
                                FAR uint8_t *buffer, uint16_t nbytes);
#endif

/* Defined in uip_tcpreadahead.c ********************************************/

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
EXTERN void uip_tcpreadaheadinit(void);
EXTERN struct uip_readahead_s *uip_tcpreadaheadalloc(void);
EXTERN void uip_tcpreadaheadrelease(struct uip_readahead_s *buf);
#endif /* CONFIG_NET_NTCP_READAHEAD_BUFFERS */

#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
/* Defined in uip_udpconn.c *************************************************/

EXTERN void uip_udpinit(void);
EXTERN struct uip_udp_conn *uip_udpactive(struct uip_udpip_hdr *buf);
EXTERN struct uip_udp_conn *uip_nextudpconn(struct uip_udp_conn *conn);

/* Defined in uip_udppoll.c *************************************************/

EXTERN void uip_udppoll(struct uip_driver_s *dev, struct uip_udp_conn *conn);

/* Defined in uip_udpsend.c *************************************************/

EXTERN void uip_udpsend(struct uip_driver_s *dev, struct uip_udp_conn *conn);

/* Defined in uip_udpinput.c ************************************************/

EXTERN void uip_udpinput(struct uip_driver_s *dev);

/* Defined in uip_udpcallback.c *********************************************/

EXTERN void uip_udpcallback(struct uip_driver_s *dev,
                            struct uip_udp_conn *conn, uint16_t flags);
#endif /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_ICMP
/* Defined in uip_icmpinput.c ***********************************************/

EXTERN void uip_icmpinput(struct uip_driver_s *dev);

#ifdef CONFIG_NET_ICMP_PING
/* Defined in uip_icmpoll.c *************************************************/

EXTERN void uip_icmppoll(struct uip_driver_s *dev);

/* Defined in uip_icmsend.c *************************************************/

EXTERN void uip_icmpsend(struct uip_driver_s *dev, uip_ipaddr_t *destaddr);

#endif /* CONFIG_NET_ICMP_PING */
#endif /* CONFIG_NET_ICMP */

#ifdef CONFIG_NET_IGMP
/* Defined in uip_igmpinit.c ************************************************/

EXTERN void uip_igmpinit(void);

/* Defined in uip_igmpinput.c ***********************************************/

EXTERN void uip_igmpinput(struct uip_driver_s *dev);

/* Defined in uip_igmpgroup.c ***********************************************/

EXTERN void uip_grpinit(void);
EXTERN FAR struct igmp_group_s *uip_grpalloc(FAR struct uip_driver_s *dev,
                                             FAR const uip_ipaddr_t *addr);
EXTERN FAR struct igmp_group_s *uip_grpfind(FAR struct uip_driver_s *dev,
                                            FAR const uip_ipaddr_t *addr);
EXTERN FAR struct igmp_group_s *uip_grpallocfind(FAR struct uip_driver_s *dev,
                                                 FAR const uip_ipaddr_t *addr);
EXTERN void uip_grpfree(FAR struct uip_driver_s *dev,
                        FAR struct igmp_group_s *group);

/* Defined in uip_igmpmsg.c **************************************************/

EXTERN void uip_igmpschedmsg(FAR struct igmp_group_s *group, uint8_t msgid);
EXTERN void uip_igmpwaitmsg(FAR struct igmp_group_s *group, uint8_t msgid);

/* Defined in uip_igmppoll.c *************************************************/

EXTERN void uip_igmppoll(FAR struct uip_driver_s *dev);

/* Defined in up_igmpsend.c **************************************************/

EXTERN void uip_igmpsend(FAR struct uip_driver_s *dev,
                         FAR struct igmp_group_s *group,
                         FAR uip_ipaddr_t *dest);

/* Defined in uip_igmptimer.c ************************************************/

EXTERN int uip_decisec2tick(int decisecs);
EXTERN void uip_igmpstartticks(FAR struct igmp_group_s *group, int ticks);
EXTERN void uip_igmpstarttimer(FAR struct igmp_group_s *group, uint8_t decisecs);
EXTERN bool uip_igmpcmptimer(FAR struct igmp_group_s *group, int maxticks);

/* Defined in uip_mcastmac ***************************************************/

EXTERN void uip_addmcastmac(FAR struct uip_driver_s *dev, FAR uip_ipaddr_t *ip);
EXTERN void uip_removemcastmac(FAR struct uip_driver_s *dev, FAR uip_ipaddr_t *ip);

#endif /* CONFIG_NET_IGMP */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET */
#endif /* __UIP_INTERNAL_H */
