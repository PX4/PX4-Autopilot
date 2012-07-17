/************************************************************************************************************
 * include/nuttx/net/uip/uip-ipopt.h
 *
 * Defines values for the IP header options
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_UIP_UIP_IPOPT_H
#define __INCLUDE_NUTTX_NET_UIP_UIP_IPOPT_H

/************************************************************************************************************
 * Included Files
 ************************************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************/

/* IP Header Options:
 *
 * The basic 20-byte IP header can be extended by up to another 40 bytes
 * (in units of 4 bytes) to provide IP options.  The format of a single IP
 * option is as follows:
 *
 * Single byte options:
 *
 *   Type:         8-bits
 *
 * Multi-byte options:
 *
 *   Type:         8-bits
 *   Length:       8-bits
 *   Pointer:      8-bits:
 *   Option Data: (depends on Length)
 *
 * The IP Option Type byte consists of the following subfields:
 */
 
#define IPOPT_TYPE_OPTION_SHIFT       (0)      /* Bits 0-5: Option number*/
#define IPOPT_TYPE_OPTION_MASK        (0x1f << IPOPT_TYPE_OPTION_SHIFT)
#  define IPOPT_TYPE_OPTION_END       (0  << IPOPT_TYPE_OPTION_SHIFT) /* End of options list (RFC 791) */
#  define IPOPT_TYPE_OPTION_NOOP      (1  << IPOPT_TYPE_OPTION_SHIFT) /* No operation (RFC 791) */
#  define IPOPT_TYPE_OPTION_SEC       (2  << IPOPT_TYPE_OPTION_SHIFT) /* Security (RFC 791, 1108) */
#  define IPOPT_TYPE_OPTION_LSRR      (3  << IPOPT_TYPE_OPTION_SHIFT) /* Loose source and record route (RFC 791) */
#  define IPOPT_TYPE_OPTION_TIMESTAMP (4  << IPOPT_TYPE_OPTION_SHIFT) /* Timestamp (RFC 781, 791) */
#  define IPOPT_TYPE_OPTION_EXTSEC    (5  << IPOPT_TYPE_OPTION_SHIFT) /* Extended security (RFC 1108) */
#  define IPOPT_TYPE_OPTION_COMMSEC   (6  << IPOPT_TYPE_OPTION_SHIFT) /* Commercial security */
#  define IPOPT_TYPE_OPTION_RR        (7  << IPOPT_TYPE_OPTION_SHIFT) /* Record route (RFC 791) */
#  define IPOPT_TYPE_OPTION_SSID      (8  << IPOPT_TYPE_OPTION_SHIFT) /* Stream ID (RFC 791, 1122) */
#  define IPOPT_TYPE_OPTION_SSRR      (9  << IPOPT_TYPE_OPTION_SHIFT) /* Strict source and record route (RFC 791) */
#  define IPOPT_TYPE_OPTION_EXPMEAS   (10 << IPOPT_TYPE_OPTION_SHIFT) /* Experimental measurement */
#  define IPOPT_TYPE_OPTION_MTUPROBE  (11 << IPOPT_TYPE_OPTION_SHIFT) /* MTU Probe (RFC 1063) */
#  define IPOPT_TYPE_OPTION_MTUREPLY  (12 << IPOPT_TYPE_OPTION_SHIFT) /* MTU Reply (RFC 1063) */
#  define IPOPT_TYPE_OPTION_EXPFC     (13 << IPOPT_TYPE_OPTION_SHIFT) /* Experimental flow control */
#  define IPOPT_TYPE_OPTION_EXPAC     (14 << IPOPT_TYPE_OPTION_SHIFT) /* Experimental access control */
#  define IPOPT_TYPE_OPTION_ENCODE    (15 << IPOPT_TYPE_OPTION_SHIFT) /* ENCODE */
#  define IPOPT_TYPE_OPTION_IMITD     (16 << IPOPT_TYPE_OPTION_SHIFT) /* IMI traffic descriptor */
#  define IPOPT_TYPE_OPTION_EXTIP     (17 << IPOPT_TYPE_OPTION_SHIFT) /* Extended IP (RFC 1385) */
#  define IPOPT_TYPE_OPTION_TR        (18 << IPOPT_TYPE_OPTION_SHIFT) /* Traceroute (RFC 1393) */
#  define IPOPT_TYPE_OPTION_ADDREXT   (19 << IPOPT_TYPE_OPTION_SHIFT) /* Address extension (RFC 1475) */
#  define IPOPT_TYPE_OPTION_RA        (20 << IPOPT_TYPE_OPTION_SHIFT) /* Router alert (RFC 2113) */
#  define IPOPT_TYPE_OPTION_SDBM      (21 << IPOPT_TYPE_OPTION_SHIFT) /* Selective direct broadcast mode (RFC 1770) */
#  define IPOPT_TYPE_OPTION_DPS       (23 << IPOPT_TYPE_OPTION_SHIFT) /* Dynamic packet state */
#  define IPOPT_TYPE_OPTION_UMP       (24 << IPOPT_TYPE_OPTION_SHIFT) /* Upstream multicast packet */
#  define IPOPT_TYPE_OPTION_QS        (25 << IPOPT_TYPE_OPTION_SHIFT) /* Quick start (RFC 4782) */
#  define IPOPT_TYPE_OPTION_EXP3692   (30 << IPOPT_TYPE_OPTION_SHIFT) /* RFC 3692-style experiment (RFC 4782) */
#define IPOPT_TYPE_CLASS_SHIFT        (5)      /* Bits 5-6: Class */
#define IPOPT_TYPE_CLASS_MASK         (3 << IPOPT_TYPE_CLASS_SHIFT)
#  define IPOPT_TYPE_CLASS_CTRL       (0 << IPOPT_TYPE_CLASS_SHIFT)   /* Control */
#  define IPOPT_TYPE_CLASS_MEASURE    (2 << IPOPT_TYPE_CLASS_SHIFT)   /* Debugging and measurement */
#define IPOPT_TYPE_COPIED             (1 << 7) /* Bit 7: IP layer must copy option to each fragment */
#define IPOPT_TYPE_NOTCOPIED          (0)

/* IP Option encoding macros */

#define IPOPT_MKTYPE(copied,class,option) (copied|class|option)

#define IPOPT_MKOPTION8(copied,class,option) \
  ((uint8_t)IPOPT_MKTYPE(copied,class,option))
#define IPOPT_MKOPTION32(type,len,ptr,data) \
  ((uint32_t)(type) << 24 | (uint32_t)(len) << 16 | \
   (uint32_t)(ptr)   << 8 | (uint32_t)(data))

/* Option Copy Class Length    Description References
 * ------ ---- ----- --------- ------------------------------------------------
 *   0     0    0     1        End of options list (RFC 791)
 *   1     0    0     1        NOP (RFC 791 
 *   2     1    0     11       Security (RFC 791, RFC 1108)
 *   3     1    0     variable Loose Source Route (RFC 791)
 *   4     0    2     variable Time stamp (RFC 781, RFC 791)
 *   5     1    0     3-31     Extended Security (RFC 1108)
 *   6     1    0              Commercial Security
 *   7     0    0     variable Record Route (RFC 791)
 *   8     1    0     4        Stream Identifier (RFC 791, RFC 1122)
 *   9     1    0     variable Strict Source Route (RFC 791)
 *  10     0    0              Experimental Measurement
 *  11     0    0     4        MTU Probe (obsolete) (RFC 1063)
 *  12     0    0     4        MTU Reply (obsolete) (RFC 1063)
 *  13     1    2              Experimental Flow Control
 *  14     1    0              Experimental Access Control
 *  15     0    0              ENCODE
 *  16     1    0              IMI Traffic Descriptor
 *  17     1    0     variable Extended Internet Protocol (RFC 1385)
 *  18     0    2     12       Traceroute (RFC 1393)
 *  19     1    0     10       Address Extension (RFC 1475)
 *  20     1    0     4        Router Alert (RFC 2113)
 *  21     1    0     6-38     Selective Directed Broadcast Mode (RFC 1770)
 *  22     1    0        
 *  23     1    0              Dynamic Packet State
 *  24     1    0              Upstream Multicast Packet
 *  25     0    0              QS, Quick-Start (RFC 4782)
 *  26
 *  -
 *  29             
 *  30     0    0             EXP - RFC3692-style Experiment (RFC 4727)
 *  30     0    2             EXP - RFC3692-style Experiment (RFC 4727)
 *  30     1    0             EXP - RFC3692-style Experiment RFC 4727)
 *  30     1    2             EXP - RFC3692-style Experiment (RFC 4727)
 *  31             
 */

#define IPOTR_END_LEN 1
#define IPOPT_END_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_END)

#define IPOTR_NOOP_LEN 1
#define IPOPT_NOOP_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_NOOP)

#define IPOPT_SEC_LEN 11
#define IPOPT_SEC_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_SEC)

#define IPOPT_LSRR_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_LSRR)

#define IPOPT_TIMESTAMP_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_MEASURE, IPOPT_TYPE_OPTION_TIMESTAMP)

#define IPOPT_EXTSEC_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_EXTSEC)

#define IPOPT_COMMSEC_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_COMMSEC)

#define IPOPT_RR_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_RR)

#define IPOPT_SSID_LEN 4
#define IPOPT_SSID_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_SSID)

#define IPOPT_SSRR_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_SSRR)
#define IPOPT_EXPMEAS_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_EXPMEAS)

#define IPOPT_MTUPROBE_LEN 4
#define IPOPT_MTUPROBE_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_MTUPROBE)

#define IPOPT_MTUREPLY_LEN 4
#define IPOPT_MTUREPLY_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_MTUREPLY)

#define IPOPT_EXPFC_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_MEASURE, IPOPT_TYPE_OPTION_EXPFC)

#define IPOPT_EXPAC_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_EXPAC)

#define IPOPT_ENCODE_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_ENCODE)

#define IPOPT_IMITD_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_IMITD)

#define IPOPT_EXTIP_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_EXTIP)

#define IPOPT_TR_LEN 12
#define IPOPT_TR_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_MEASURE, IPOPT_TYPE_OPTION_TR)

#define IPOPT_ADDREXT_LEN 10
#define IPOPT_ADDREXT_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_ADDREXT)

#define IPOPT_RA_LEN 4
#define IPOPT_RA_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_RA)
#define IPOPT_RA \
  IPOPT_MKOPTION32(IPOPT_RA_TYPE, 4, 0, 0)

#define IPOPT_SDBM_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_SDBM)
  
#define IPOPT_DPS_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_DPS)
  
#define IPOPT_UMP_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_COPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_UMP)
  
#define IPOPT_QS_TYPE \
  IPOPT_MKTYPE(IPOPT_TYPE_NOTCOPIED, IPOPT_TYPE_CLASS_CTRL, IPOPT_TYPE_OPTION_QS)

/************************************************************************************************************
 * Public Type Definitions
 ************************************************************************************************************/

/************************************************************************************************************
 * Public Data
 ************************************************************************************************************/

/************************************************************************************************************
 * Public Function Prototypes
 ************************************************************************************************************/

#endif /* __INCLUDE_NUTTX_NET_UIP_UIP_IPOPT_H */
