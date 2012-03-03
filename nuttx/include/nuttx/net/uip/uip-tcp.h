/****************************************************************************
 * include/nuttx/net/uip/uip-tcp.h
 * Header file for the uIP TCP/IP stack.
 *
 * The uIP TCP/IP stack header file contains definitions for a number
 * of C macros that are used by uIP programs as well as internal uIP
 * structures, TCP/IP header structures and function declarations.
 *
 *   Copyright (C) 2007, 2009-2010, 2012 Gregory Nutt. All rights reserved.
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
 *
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

#ifndef __INCLUDE_NUTTX_NET_UIP_UIP_TCP_H
#define __INCLUDE_NUTTX_NET_UIP_UIP_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET_TCP

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/net/uip/uipopt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TCP definitions */

#define TCP_FIN 0x01
#define TCP_SYN 0x02
#define TCP_RST 0x04
#define TCP_PSH 0x08
#define TCP_ACK 0x10
#define TCP_URG 0x20
#define TCP_CTL 0x3f

#define TCP_OPT_END     0   /* End of TCP options list */
#define TCP_OPT_NOOP    1   /* "No-operation" TCP option */
#define TCP_OPT_MSS     2   /* Maximum segment size TCP option */

#define TCP_OPT_MSS_LEN 4   /* Length of TCP MSS option. */

/* The TCP states used in the struct uip_conn tcpstateflags field */

#define UIP_TS_MASK     0x0f /* Bits 0-3: TCP state */
#define UIP_CLOSED      0x00 /* The connection is not in use and available */
#define UIP_ALLOCATED   0x01 /* The connection is allocated, but not yet initialized */
#define UIP_SYN_RCVD    0x02
#define UIP_SYN_SENT    0x03
#define UIP_ESTABLISHED 0x04
#define UIP_FIN_WAIT_1  0x05
#define UIP_FIN_WAIT_2  0x06
#define UIP_CLOSING     0x07
#define UIP_TIME_WAIT   0x08
#define UIP_LAST_ACK    0x09
#define UIP_STOPPED     0x10 /* Bit 4: stopped */
                             /* Bit 5-7: Unused, but not available */

/* Flag bits in 16-bit flags+ipoffset IPv4 TCP header field */

#define UIP_TCPFLAG_RESERVED  0x8000
#define UIP_TCPFLAG_DONTFRAG  0x4000
#define UIP_TCPFLAG_MOREFRAGS 0x2000

/* TCP header sizes */

#define UIP_TCPH_LEN    20    /* Size of TCP header */
#define UIP_IPTCPH_LEN (UIP_TCPH_LEN + UIP_IPH_LEN)    /* Size of IP + TCP header */
#define UIP_TCPIP_HLEN UIP_IPTCPH_LEN

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Representation of a uIP TCP connection.
 *
 * The uip_conn structure is used for identifying a connection. All
 * but one field in the structure are to be considered read-only by an
 * application. The only exception is the "private: field whos purpose
 * is to let the application store application-specific state (e.g.,
 * file pointers) for the connection.
 */

struct uip_driver_s;      /* Forward reference */
struct uip_callback_s;    /* Forward reference */
struct uip_backlog_s;     /* Forward reference */

struct uip_conn
{
  dq_entry_t node;        /* Implements a doubly linked list */
  uip_ipaddr_t ripaddr;   /* The IP address of the remote host */
  uint16_t lport;         /* The local TCP port, in network byte order */
  uint16_t rport;         /* The remoteTCP port, in network byte order */
  uint8_t  rcvseq[4];     /* The sequence number that we expect to
                           * receive next */
  uint8_t  sndseq[4];     /* The sequence number that was last sent by us */
  uint16_t unacked;       /* Number bytes sent but not yet ACKed */
  uint16_t mss;           /* Current maximum segment size for the
                           * connection */
  uint16_t initialmss;    /* Initial maximum segment size for the
                           * connection */
  uint8_t  crefs;         /* Reference counts on this instance */
  uint8_t  sa;            /* Retransmission time-out calculation state
                           * variable */
  uint8_t  sv;            /* Retransmission time-out calculation state
                           * variable */
  uint8_t  rto;           /* Retransmission time-out */
  uint8_t  tcpstateflags; /* TCP state and flags */
  uint8_t  timer;         /* The retransmission timer (units: half-seconds) */
  uint8_t  nrtx;          /* The number of retransmissions for the last
                           * segment sent */

  /* Read-ahead buffering.
   *
   * readahead - A singly linked list of type struct uip_readahead_s
   *   where the TCP/IP read-ahead data is retained.
   */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  sq_queue_t readahead;   /* Read-ahead buffering */
#endif

  /* Listen backlog support
   *
   *   blparent - The backlog parent.  If this connection is backlogged,
   *     this field will be non-null and will refer to the TCP connection
   *     structure in which this connection is backlogged.
   *   backlog - The pending connection backlog.  If this connection is
   *     configured as a listener with backlog, then this refers to the
   *     struct uip_backlog_s tear-off structure that manages that backlog.
   */

#ifdef CONFIG_NET_TCPBACKLOG
  struct uip_conn      *blparent;
  struct uip_backlog_s *backlog;
#endif

  /* Application callbacks:
   *
   * Data transfer events are retained in 'list'.  Event handlers in 'list'
   * are called for events specified in the flags set within struct
   * uip_callback_s
   *
   * When an callback is executed from 'list', the input flags are normally
   * returned, however, the implementation may set one of the following:
   *
   *   UIP_CLOSE   - Gracefully close the current connection
   *   UIP_ABORT   - Abort (reset) the current connection on an error that
   *                 prevents UIP_CLOSE from working.
   *
   * And/Or set/clear the following:
   *
   *   UIP_NEWDATA - May be cleared to indicate that the data was consumed
   *                 and that no further process of the new data should be
   *                 attempted.
   *   UIP_SNDACK  - If UIP_NEWDATA is cleared, then UIP_SNDACK may be set
   *                 to indicate that an ACK should be included in the response.
   *                 (In UIP_NEWDATA is cleared bu UIP_SNDACK is not set, then
   *                 dev->d_len should also be cleared).
   */

  struct uip_callback_s *list;

  /* accept() is called when the TCP logic has created a connection */

  FAR void *accept_private;
  int (*accept)(FAR struct uip_conn *listener, struct uip_conn *conn);

  /* connection_event() is called on any of the subset of connection-related events */

  FAR void *connection_private;
  void (*connection_event)(FAR struct uip_conn *conn, uint16_t flags);
};

/* The following structure is used to handle read-ahead buffering for TCP
 * connection.  When incoming TCP data is received while no application is
 * listening for the data, that data will be retained in these read-ahead
 * buffers so that no data is lost.
 */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
struct uip_readahead_s
{
  sq_entry_t rh_node;      /* Supports a singly linked list */
  uint16_t rh_nbytes;      /* Number of bytes available in this buffer */
  uint8_t  rh_buffer[CONFIG_NET_TCP_READAHEAD_BUFSIZE];
};
#endif

/* Support for listen backlog:
 *
 *   struct uip_blcontainer_s describes one backlogged connection
 *   struct uip_backlog_s is a "tear-off" describing all backlog for a
 *      listener connection
 */

#ifdef CONFIG_NET_TCPBACKLOG
struct uip_blcontainer_s
{
  sq_entry_t           bc_node;    /* Implements a singly linked list */
  FAR struct uip_conn *bc_conn;    /* Holds reference to the new connection structure */
};

struct uip_backlog_s
{
  sq_queue_t           bl_free;    /* Implements a singly-linked list of free containers */
  sq_queue_t           bl_pending; /* Implements a singly-linked list of pending connections */
};
#endif

/* The structure holding the TCP/IP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct uip_tcp_stats_s
{
  uip_stats_t drop;       /* Number of dropped TCP segments */
  uip_stats_t recv;       /* Number of received TCP segments */
  uip_stats_t sent;       /* Number of sent TCP segments */
  uip_stats_t chkerr;     /* Number of TCP segments with a bad checksum */
  uip_stats_t ackerr;     /* Number of TCP segments with a bad ACK number */
  uip_stats_t rst;        /* Number of received TCP RST (reset) segments */
  uip_stats_t rexmit;     /* Number of retransmitted TCP segments */
  uip_stats_t syndrop;    /* Number of dropped SYNs due to too few
                             available connections */
  uip_stats_t synrst;     /* Number of SYNs for closed ports triggering a RST */
};
#endif

/* The TCP and IP headers */

struct uip_tcpip_hdr
{
#ifdef CONFIG_NET_IPv6

  /* IPv6 Ip header */

  uint8_t  vtc;             /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  uint8_t  tcf;             /* Bits 0-3: traffic class (LS), 4-bits: flow label (MS) */
  uint16_t flow;            /* 16-bit flow label (LS) */
  uint8_t  len[2];          /* 16-bit Payload length */
  uint8_t  proto;           /*  8-bit Next header (same as IPv4 protocol field) */
  uint8_t  ttl;             /*  8-bit Hop limit (like IPv4 TTL field) */
  uip_ip6addr_t srcipaddr;  /* 128-bit Source address */
  uip_ip6addr_t destipaddr; /* 128-bit Destination address */

#else /* CONFIG_NET_IPv6 */

  /* IPv4 IP header */

  uint8_t  vhl;             /*  8-bit Version (4) and header length (5 or 6) */
  uint8_t  tos;             /*  8-bit Type of service (e.g., 6=TCP) */
  uint8_t  len[2];          /* 16-bit Total length */
  uint8_t  ipid[2];         /* 16-bit Identification */
  uint8_t  ipoffset[2];     /* 16-bit IP flags + fragment offset */
  uint8_t  ttl;             /*  8-bit Time to Live */
  uint8_t  proto;           /*  8-bit Protocol */
  uint16_t ipchksum;        /* 16-bit Header checksum */
  uint16_t srcipaddr[2];    /* 32-bit Source IP address */
  uint16_t destipaddr[2];   /* 32-bit Destination IP address */

#endif /* CONFIG_NET_IPv6 */

  /* TCP header */

  uint16_t srcport;
  uint16_t destport;
  uint8_t  seqno[4];
  uint8_t  ackno[4];
  uint8_t  tcpoffset;
  uint8_t  flags;
  uint8_t  wnd[2];
  uint16_t tcpchksum;
  uint8_t  urgp[2];
  uint8_t  optdata[4];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* uIP application functions
 *
 * Functions used by an application running of top of uIP. This includes
 * functions for opening and closing connections, sending and receiving
 * data, etc.
 *
 * Find a free connection structure and allocate it for use. This is
 * normally something done by the implementation of the socket() API
 */

extern struct uip_conn *uip_tcpalloc(void);

/* Allocate a new TCP data callback */

#define uip_tcpcallbackalloc(conn)   uip_callbackalloc(&conn->list)
#define uip_tcpcallbackfree(conn,cb) uip_callbackfree(cb, &conn->list)

/* Free a connection structure that is no longer in use. This should
 * be done by the implementation of close()
 */

extern void uip_tcpfree(struct uip_conn *conn);

/* Bind a TCP connection to a local address */

#ifdef CONFIG_NET_IPv6
extern int uip_tcpbind(struct uip_conn *conn, const struct sockaddr_in6 *addr);
#else
extern int uip_tcpbind(struct uip_conn *conn, const struct sockaddr_in *addr);
#endif

/* This function implements the UIP specific parts of the standard
 * TCP connect() operation:  It connects to a remote host using TCP.
 *
 * This function is used to start a new connection to the specified
 * port on the specied host. It uses the connection structure that was
 * allocated by a preceding socket() call.  It sets the connection to
 * the SYN_SENT state and sets the retransmission timer to 0. This will
 * cause a TCP SYN segment to be sent out the next time this connection
 * is periodically processed, which usually is done within 0.5 seconds
 * after the call to uip_tcpconnect().
 *
 * This function is called from normal user level code.
 */

#ifdef CONFIG_NET_IPv6
extern int uip_tcpconnect(struct uip_conn *conn, const struct sockaddr_in6 *addr);
#else
extern int uip_tcpconnect(struct uip_conn *conn, const struct sockaddr_in *addr);
#endif

/* Start listening to the port bound to the specified TCP connection */

extern int uip_listen(struct uip_conn *conn);

/* Stop listening to the port bound to the specified TCP connection */

extern int uip_unlisten(struct uip_conn *conn);

/* Access to TCP read-ahead buffers */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
extern struct uip_readahead_s *uip_tcpreadaheadalloc(void);
extern void uip_tcpreadaheadrelease(struct uip_readahead_s *buf);
#endif /* CONFIG_NET_NTCP_READAHEAD_BUFFERS */

/* Backlog support */

#ifdef CONFIG_NET_TCPBACKLOG
/* APIs to create and terminate TCP backlog support */

extern int uip_backlogcreate(FAR struct uip_conn *conn, int nblg);
extern int uip_backlogdestroy(FAR struct uip_conn *conn);

/* APIs to manage individual backlog actions */

extern int uip_backlogadd(FAR struct uip_conn *conn, FAR struct uip_conn *blconn);
#ifndef CONFIG_DISABLE_POLL
extern bool uip_backlogavailable(FAR struct uip_conn *conn);
#else
#  define uip_backlogavailable(conn)   (false);
#endif
extern FAR struct uip_conn *uip_backlogremove(FAR struct uip_conn *conn);
extern int uip_backlogdelete(FAR struct uip_conn *conn, FAR struct uip_conn *blconn);

#else
#  define uip_backlogcreate(conn,nblg) (-ENOSYS)
#  define uip_backlogdestroy(conn)     (-ENOSYS)
#  define uip_backlogadd(conn,blconn)  (-ENOSYS)
#  define uip_backlogavailable(conn)   (false);
#  define uip_backlogremove(conn)      (NULL)
#endif

/* Tell the sending host to stop sending data.
 *
 * This function will close our receiver's window so that we stop
 * receiving data for the current connection.
 */

#define uip_stop(conn) ((conn)->tcpstateflags |= UIP_STOPPED)

/* Find out if the current connection has been previously stopped with
 * uip_stop().
 */

#define uip_stopped(conn) ((conn)->tcpstateflags & UIP_STOPPED)

/* Restart the current connection, if is has previously been stopped
 * with uip_stop().
 *
 * This function will open the receiver's window again so that we start
 * receiving data for the current connection.
 */

#define uip_restart(conn,f) \
  do { \
    (f) |= UIP_NEWDATA; \
    (conn)->tcpstateflags &= ~UIP_STOPPED; \
  } while(0)

/* Get the initial maxium segment size (MSS) of the current
 * connection.
 */

#define uip_initialmss(conn) ((conn)->initialmss)

/* Get the current maximum segment size that can be sent on the current
 * connection.
 *
 * The current maxiumum segment size that can be sent on the connection is
 * computed from the receiver's window and the MSS of the connection (which
 * also is available by calling uip_initialmss()).
 */

#define uip_mss(conn) ((conn)->mss)

#endif /* CONFIG_NET_TCP */
#endif /* __INCLUDE_NUTTX_NET_UIP_UIP_TCP_H */
