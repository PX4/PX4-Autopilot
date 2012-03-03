/****************************************************************************
 * net/uip/uip_chksum.c
 *
 *   Copyright (C) 2007-2010, 2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>

#include "uip_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUF ((struct uip_ip_hdr *)&dev->d_buf[UIP_LLH_LEN])
#define ICMPBUF ((struct uip_icmpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !UIP_ARCH_CHKSUM
static uint16_t chksum(uint16_t sum, const uint8_t *data, uint16_t len)
{
  uint16_t t;
  const uint8_t *dataptr;
  const uint8_t *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while(dataptr < last_byte)
    {
      /* At least two more bytes */

      t = (dataptr[0] << 8) + dataptr[1];
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
      dataptr += 2;
    }

  if (dataptr == last_byte)
    {
      t = (dataptr[0] << 8) + 0;
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
    }

  /* Return sum in host byte order. */

  return sum;
}

static uint16_t upper_layer_chksum(struct uip_driver_s *dev, uint8_t proto)
{
  struct uip_ip_hdr *pbuf = BUF;
  uint16_t upper_layer_len;
  uint16_t sum;

#ifdef CONFIG_NET_IPv6
  upper_layer_len = (((uint16_t)(pbuf->len[0]) << 8) + pbuf->len[1]);
#else /* CONFIG_NET_IPv6 */
  upper_layer_len = (((uint16_t)(pbuf->len[0]) << 8) + pbuf->len[1]) - UIP_IPH_LEN;
#endif /* CONFIG_NET_IPv6 */

  /* First sum pseudoheader. */

  /* IP protocol and length fields. This addition cannot carry. */

  sum = upper_layer_len + proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (uint8_t *)&pbuf->srcipaddr, 2 * sizeof(uip_ipaddr_t));

  /* Sum TCP header and data. */

  sum = chksum(sum, &dev->d_buf[UIP_IPH_LEN + UIP_LLH_LEN], upper_layer_len);

  return (sum == 0) ? 0xffff : htons(sum);
}

#ifdef CONFIG_NET_IPv6
static uint16_t uip_icmp6chksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_ICMP6);
}
#endif /* CONFIG_NET_IPv6 */

#endif /* UIP_ARCH_CHKSUM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Calculate the Internet checksum over a buffer. */

#if !UIP_ARCH_ADD32
static inline void uip_carry32(uint8_t *sum, uint16_t op16)
{
  if (sum[2] < (op16 >> 8))
    {
      ++sum[1];
      if (sum[1] == 0)
        {
          ++sum[0];
        }
    }

  if (sum[3] < (op16 & 0xff))
    {
      ++sum[2];
      if (sum[2] == 0)
        {
          ++sum[1];
          if (sum[1] == 0)
            {
              ++sum[0];
            }
        }
    }
}

void uip_incr32(uint8_t *op32, uint16_t op16)
{
  op32[3] += (op16 & 0xff);
  op32[2] += (op16 >> 8);
  uip_carry32(op32, op16);
}

#endif /* UIP_ARCH_ADD32 */

#if !UIP_ARCH_CHKSUM
uint16_t uip_chksum(uint16_t *data, uint16_t len)
{
  return htons(chksum(0, (uint8_t *)data, len));
}

/* Calculate the IP header checksum of the packet header in d_buf. */

#ifndef UIP_ARCH_IPCHKSUM
uint16_t uip_ipchksum(struct uip_driver_s *dev)
{
  uint16_t sum;

  sum = chksum(0, &dev->d_buf[UIP_LLH_LEN], UIP_IPH_LEN);
  return (sum == 0) ? 0xffff : htons(sum);
}
#endif

/* Calculate the TCP checksum of the packet in d_buf and d_appdata. */

uint16_t uip_tcpchksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_TCP);
}

/* Calculate the UDP checksum of the packet in d_buf and d_appdata. */

#ifdef CONFIG_NET_UDP_CHECKSUMS
uint16_t uip_udpchksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_UDP);
}
#endif

/* Calculate the checksum of the ICMP message */

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_PING)
uint16_t uip_icmpchksum(struct uip_driver_s *dev, int len)
{
  struct uip_icmpip_hdr *picmp = ICMPBUF;
  return uip_chksum((uint16_t*)&picmp->type, len);
}
#endif

#endif /* UIP_ARCH_CHKSUM */

#endif /* CONFIG_NET */
