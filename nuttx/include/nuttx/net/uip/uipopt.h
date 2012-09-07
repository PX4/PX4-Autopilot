/****************************************************************************
 * include/nuttx/net/uip/uipopt.h
 * Configuration options for uIP.
 *
 * This file is used for tweaking various configuration options for
 * uIP. You should make a copy of this file into one of your project's
 * directories instead of editing this example "uipopt.h" file that
 * comes with the uIP distribution.
 *
 * uIP is configured using the per-project configuration file
 * uipopt.h. This file contains all compile-time options for uIP and
 * should be tweaked to match each specific project. The uIP
 * distribution contains a documented example "uipopt.h" that can be
 * copied and modified for each project.
 *
 * Note: Most of the configuration options in the uipopt.h should not
 * be changed, but rather the per-project defconfig file.
 *
 *   Copyright (C) 2007, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
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

#ifndef __INCLUDE_NUTTX_NET_UIP_UIPOPT_H
#define __INCLUDE_NUTTX_NET_UIP_UIPOPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/config.h>

/****************************************************************************
 * Public Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Layer 2 Configuration Options ********************************************/

/* The default data link layer for uIP is Ethernet.  If CONFIG_NET_SLIP is
 * defined in the NuttX header file, then SLIP will be supported.  The basic
 * differences between the SLIP and Ethernet configurations is that when SLIP
 * is selected:
 *
 * - The link level header (that comes before the IP header) is omitted.
 * - All MAC address processing is suppressed.
 * - ARP is disabled.
 *
 * If CONFIG_NET_SLIP is not supported, then Ethernet will be used (there is
 * no need to define anything special in the configuration file to use
 * Ethernet -- it is the default).
 *
 * The "link level header" is the offset into the d_buf where the IP header
 * can be found. For Ethernet, this should be set to 14. For SLIP, this
 * should be set to 0.
 */

#undef CONFIG_NET_ETHERNET
#undef CONFIG_NET_ARP

#ifdef CONFIG_NET_SLIP
#  ifdef CONFIG_NET_IPv6
#    error "SLIP is not implemented for IPv6"
#  endif
#  define UIP_LLH_LEN         0
#else
#  define CONFIG_NET_ETHERNET 1
#  define CONFIG_NET_ARP      1
#  define UIP_LLH_LEN         14
#endif

/* Layer 3/4 Configuration Options ******************************************/

/* IP configuration options */

/* The IP TTL (time to live) of IP packets sent by uIP.
 *
 * This should normally not be changed.
 */

#define UIP_TTL 64

/* Turn on support for IP packet reassembly.
 *
 * uIP supports reassembly of fragmented IP packets. This features
 * requires an additonal amount of RAM to hold the reassembly buffer
 * and the reassembly code size is approximately 700 bytes.  The
 * reassembly buffer is of the same size as the d_buf buffer
 * (configured by CONFIG_NET_BUFSIZE).
 *
 * Note: IP packet reassembly is not heavily tested.
 */

#define UIP_REASSEMBLY 0

/* The maximum time an IP fragment should wait in the reassembly
 * buffer before it is dropped.  Units are deci-seconds, the range
 * of the timer is 8-bits.
 */

#define UIP_REASS_MAXAGE (20*10) /* 20 seconds */

/* Network drivers often receive packets with garbage at the end
 * and are longer than the size of packet in the TCP header.  The
 * following "fudge" factor increases the size of the I/O buffering
 * by a small amount to allocate slightly oversize packets.  After
 * receipt, the packet size will be chopped down to the size indicated
 * in the TCP header.
 */

#ifndef CONFIG_NET_GUARDSIZE
#  define CONFIG_NET_GUARDSIZE 2
#endif

/* ICMP configuration options */

#if !defined(CONFIG_NET_ICMP) || defined(CONFIG_DISABLE_CLOCK)
#  undef CONFIG_NET_ICMP_PING
#endif

/* UDP configuration options */

/* The maximum amount of concurrent UDP connection, Default: 10 */

#ifndef CONFIG_NET_UDP_CONNS
# ifdef CONFIG_NET_UDP
#  define CONFIG_NET_UDP_CONNS 10
# else
#  define CONFIG_NET_UDP_CONNS  0
# endif
#endif

/* The UDP maximum packet size. This is should not be to set to more
 * than CONFIG_NET_BUFSIZE - UIP_LLH_LEN - UIP_IPUDPH_LEN.
 */

#define UIP_UDP_MSS (CONFIG_NET_BUFSIZE - UIP_LLH_LEN - UIP_IPUDPH_LEN)

/* TCP configuration options */

/* The maximum number of simultaneously open TCP connections.
 *
 * Since the TCP connections are statically allocated, turning this
 * configuration knob down results in less RAM used. Each TCP
 * connection requires approximatly 30 bytes of memory.
 */

#ifndef CONFIG_NET_TCP_CONNS
# ifdef CONFIG_NET_TCP
#  define CONFIG_NET_TCP_CONNS 10
# else
#  define CONFIG_NET_TCP_CONNS  0
# endif
#endif

/* The maximum number of simultaneously listening TCP ports.
 *
 * Each listening TCP port requires 2 bytes of memory.
 */

#ifndef CONFIG_NET_MAX_LISTENPORTS
# define CONFIG_NET_MAX_LISTENPORTS 20
#endif

/* Define the maximum number of concurrently active UDP and TCP
 * ports.  This number must be greater than the number of open
 * sockets in order to support multi-threaded read/write operations.
 */

#ifndef CONFIG_NET_NACTIVESOCKETS
# define CONFIG_NET_NACTIVESOCKETS (CONFIG_NET_TCP_CONNS + CONFIG_NET_UDP_CONNS)
#endif

/* The initial retransmission timeout counted in timer pulses.
 *
 * This should not be changed.
 */

#define UIP_RTO 3

/* The maximum number of times a segment should be retransmitted
 * before the connection should be aborted.
 *
 * This should not be changed.
 */

#define UIP_MAXRTX  8

/* The maximum number of times a SYN segment should be retransmitted
 * before a connection request should be deemed to have been
 * unsuccessful.
 *
 * This should not need to be changed.
 */

#define UIP_MAXSYNRTX 5

/* The TCP maximum segment size. This is should not be set to more
 * than CONFIG_NET_BUFSIZE - UIP_LLH_LEN - UIP_TCPIP_HLEN.
 */

#define UIP_TCP_MSS (CONFIG_NET_BUFSIZE - UIP_LLH_LEN - UIP_TCPIP_HLEN)

/* The size of the advertised receiver's window.
 *
 * Should be set low (i.e., to the size of the d_buf buffer) is the
 * application is slow to process incoming data, or high (32768 bytes)
 * if the application processes data quickly.
 */

#ifndef CONFIG_NET_RECEIVE_WINDOW
# define CONFIG_NET_RECEIVE_WINDOW UIP_TCP_MSS
#endif

/* How long a connection should stay in the TIME_WAIT state.
 *
 * This configiration option has no real implication, and it should be
 * left untouched. Units: half second.
 */

#define UIP_TIME_WAIT_TIMEOUT (60*2)

/* ARP configuration options */

/* The size of the ARP table.
 *
 * This option should be set to a larger value if this uIP node will
 * have many connections from the local network.
 */

#ifndef CONFIG_NET_ARPTAB_SIZE
# define CONFIG_NET_ARPTAB_SIZE 8
#endif

/* The maxium age of ARP table entries measured in 10ths of seconds.
 *
 * An UIP_ARP_MAXAGE of 120 corresponds to 20 minutes (BSD
 * default).
 */

#define UIP_ARP_MAXAGE 120

/* General configuration options */

/* The size of the uIP packet buffer.
 *
 * The uIP packet buffer should not be smaller than 60 bytes, and does
 * not need to be larger than 1500 bytes. Lower size results in lower
 * TCP throughput, larger size results in higher TCP throughput.
 */

#ifndef CONFIG_NET_BUFSIZE
# define CONFIG_NET_BUFSIZE 400
#endif

/* Number of TCP read-ahead buffers (may be zero) */

#ifndef CONFIG_NET_NTCP_READAHEAD_BUFFERS
# define CONFIG_NET_NTCP_READAHEAD_BUFFERS 4
#endif

/* The size of the TCP read buffer size */

#ifndef CONFIG_NET_TCP_READAHEAD_BUFSIZE
#  if CONFIG_NET_NTCP_READAHEAD_BUFFERS < 1
#    define CONFIG_NET_TCP_READAHEAD_BUFSIZE 0
#  else
#    define CONFIG_NET_TCP_READAHEAD_BUFSIZE UIP_TCP_MSS
#  endif
#endif

/* Delay after receive to catch a following packet.  No delay should be
 * required if TCP/IP read-ahead buffering is enabled.
 */

#ifndef CONFIG_NET_TCP_RECVDELAY
#  if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
#    define CONFIG_NET_TCP_RECVDELAY 0
#  else
#    define CONFIG_NET_TCP_RECVDELAY 5
#  endif
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Statistics datatype
 *
 * This typedef defines the dataype used for keeping statistics in
 * uIP.
 */

typedef uint16_t uip_stats_t;

#endif /* __INCLUDE_NUTTX_NET_UIP_UIPOPT_H */
