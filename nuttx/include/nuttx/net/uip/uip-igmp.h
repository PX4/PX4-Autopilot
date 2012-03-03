/****************************************************************************
 * include/nuttx/net/uip/uip-igmp.h
 * The definitions in this header file are intended only for internal use
 * by the NuttX port of the uIP stack.
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The NuttX implementation of IGMP was inspired by the IGMP add-on for the
 * lwIP TCP/IP stack by Steve Reynolds:
 *
 *   Copyright (c) 2002 CITEL Technologies Ltd.
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
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_UIP_UIP_IGMP_H
#define __INCLUDE_NUTTX_NET_UIP_UIP_IGMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <netinet/in.h>

#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
#  error "IGMP for IPv6 not supported"
#endif

/* IGMP packet types */

#define IGMP_MEMBERSHIP_QUERY    0x11    /* Membership Query */
#define IGMPv1_MEMBERSHIP_REPORT 0x12    /* IGMP Ver. 1 Membership Report */
#define IGMPv2_MEMBERSHIP_REPORT 0x16    /* IGMP Ver. 2 Membership Report */
#define IGMPv3_MEMBERSHIP_REPORT 0x22    /* IGMP Ver. 3 Membership Report */
#define IGMP_LEAVE_GROUP         0x17    /* Leave Group */

/* Header sizes:
 *
 * UIP_IGMPH_LEN   - Size of IGMP header in bytes
 * UIP_IPIGMPH_LEN - Size of IP + Size of IGMP header + Size of router alert
 */

#define UIP_IGMPH_LEN            8
#define UIP_IPIGMPH_LEN          (UIP_IGMPH_LEN + UIP_IPH_LEN + 4)

/* Group flags */

#define IGMP_PREALLOCATED        (1 << 0)
#define IGMP_LASTREPORT          (1 << 1)
#define IGMP_IDLEMEMBER          (1 << 2)
#define IGMP_SCHEDMSG            (1 << 3)
#define IGMP_WAITMSG             (1 << 4)

#define SET_PREALLOCATED(f)      do { (f) |= IGMP_PREALLOCATED; } while (0)
#define SET_LASTREPORT(f)        do { (f) |= IGMP_LASTREPORT; } while (0)
#define SET_IDLEMEMBER(f)        do { (f) |= IGMP_IDLEMEMBER; } while (0)
#define SET_SCHEDMSG(f)          do { (f) |= IGMP_SCHEDMSG; } while (0)
#define SET_WAITMSG(f)           do { (f) |= IGMP_WAITMSG; } while (0)

#define CLR_PREALLOCATED(f)      do { (f) &= ~IGMP_PREALLOCATED; } while (0)
#define CLR_LASTREPORT(f)        do { (f) &= ~IGMP_LASTREPORT; } while (0)
#define CLR_IDLEMEMBER(f)        do { (f) &= ~IGMP_IDLEMEMBER; } while (0)
#define CLR_SCHEDMSG(f)          do { (f) &= ~IGMP_SCHEDMSG; } while (0)
#define CLR_WAITMSG(f)           do { (f) &= ~IGMP_WAITMSG; } while (0)

#define IS_PREALLOCATED(f)       (((f) & IGMP_PREALLOCATED) != 0)
#define IS_LASTREPORT(f)         (((f) & IGMP_LASTREPORT) != 0)
#define IS_IDLEMEMBER(f)         (((f) & IGMP_IDLEMEMBER) != 0)
#define IS_SCHEDMSG(f)           (((f) & IGMP_SCHEDMSG) != 0)
#define IS_WAITMSG(f)            (((f) & IGMP_WAITMSG) != 0)

#define IGMP_TTL                 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IGMPv2 packet structure as defined by RFC 2236.
 *
 * The Max Respone Time (maxresp) specifies the time limit for the
 * corresponding report. The field has a resolution of 100 miliseconds, the
 * value is taken directly. This field is meaningful only in Membership Query
 * (0x11); in other messages it is set to 0 and ignored by the receiver.
 */

struct uip_igmphdr_s
{
#ifdef CONFIG_NET_IPv6

  /* IPv6 Ip header */

  uint8_t  vtc;             /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  uint8_t  tcf;             /* Bits 0-3: traffic class (LS), bits 4-7: flow label (MS) */
  uint16_t flow;            /* 16-bit flow label (LS) */
  uint8_t  len[2];          /* 16-bit Payload length */
  uint8_t  proto;           /*  8-bit Next header (same as IPv4 protocol field) */
  uint8_t  ttl;             /*  8-bit Hop limit (like IPv4 TTL field) */
  uip_ip6addr_t srcipaddr;  /* 128-bit Source address */
  uip_ip6addr_t destipaddr; /* 128-bit Destination address */

#else /* CONFIG_NET_IPv6 */

  /* IPv4 IP header */

  uint8_t  vhl;             /*  8-bit Version (4) and header length (6 with Router Alert) */
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

  /* Router Alert IP header option */

  uint16_t ra[2];

  /* IGMPv2 header:
   *
   *  0                   1                   2                   3
   *  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * |      Type     | Max Resp Time |           Checksum            |
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   * |                         Group Address                         |
   * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   */

  uint8_t  type;           /* 8-bit IGMP packet type */
  uint8_t  maxresp;        /* 8-bit Max response time */
  uint16_t chksum;         /* 16-bit Checksum */
  uint16_t grpaddr[2];     /* 32-bit Group address */
};

#ifdef CONFIG_NET_STATISTICS
struct uip_igmp_stats_s
{
  uint32_t length_errors;
  uint32_t chksum_errors;
  uint32_t v1_received;
  uint32_t joins;
  uint32_t leaves;
  uint32_t leave_sched;
  uint32_t report_sched;
  uint32_t poll_send;
  uint32_t ucast_query;
  uint32_t query_received;
  uint32_t report_received;
};

# define IGMP_STATINCR(p) ((p)++)
#else
# define IGMP_STATINCR(p)
#endif

/* This structure represents one group member.  There is a list of groups
 * for each device interface structure.
 * 
 * There will be a group for the all systems group address but this 
 * will not run the state machine as it is used to kick off reports
 * from all the other groups
 */

typedef FAR struct wdog_s *WDOG_ID;
struct igmp_group_s
{
  struct igmp_group_s *next;    /* Implements a singly-linked list */
  uip_ipaddr_t         grpaddr; /* Group IP address */
  WDOG_ID              wdog;    /* WDOG used to detect timeouts */
  sem_t                sem;     /* Used to wait for message transmission */
  volatile uint8_t     flags;   /* See IGMP_ flags definitions */
  uint8_t              msgid;   /* Pending message ID (if non-zero) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN uip_ipaddr_t g_allsystems;
EXTERN uip_ipaddr_t g_allrouters;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmpdevinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for IGMP support.
 *
 ****************************************************************************/

EXTERN void uip_igmpdevinit(struct uip_driver_s *dev);
EXTERN int  igmp_joingroup(struct uip_driver_s *dev, FAR const struct in_addr *grpaddr);
EXTERN int  igmp_leavegroup(struct uip_driver_s *dev, FAR const struct in_addr *grpaddr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NET_IGMP */
#endif /* __INCLUDE_NUTTX_NET_UIP_UIP_IGMP_H */
