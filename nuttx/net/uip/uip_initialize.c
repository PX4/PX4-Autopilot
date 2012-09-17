/****************************************************************************
 * net/uip/uip_initialize.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <stdint.h>
#include <nuttx/net/uip/uip.h>

#include "uip_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* IP/TCP/UDP/ICMP statistics for all network interfaces */

#ifdef CONFIG_NET_STATISTICS
struct uip_stats uip_stat;
#endif

/* Increasing number used for the IP ID field. */

uint16_t g_ipid;

const uip_ipaddr_t g_alloneaddr =
#ifdef CONFIG_NET_IPv6
  {0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff};
#else
  0xffffffff;
#endif

const uip_ipaddr_t g_allzeroaddr =
#ifdef CONFIG_NET_IPv6
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
#else
  0x00000000;
#endif

/* Reassembly timer (units: deci-seconds) */

#if UIP_REASSEMBLY && !defined(CONFIG_NET_IPv6)
uint8_t uip_reasstmr;
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_initialize
 *
 * Description:
 *   Perform initialization of the uIP layer
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ****************************************************************************/

void uip_initialize(void)
{
  /* Initialize the locking facility */

  uip_lockinit();

  /* Initialize callback support */

  uip_callbackinit();

  /* Initialize the listening port structures */

#ifdef CONFIG_NET_TCP
  uip_listeninit();

  /* Initialize the TCP/IP connection structures */

  uip_tcpinit();

  /* Initialize the TCP/IP read-ahead buffering */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  uip_tcpreadaheadinit();
#endif
#endif /* CONFIG_NET_TCP */

  /* Initialize the UDP connection structures */

#ifdef CONFIG_NET_UDP
  uip_udpinit();
#endif

  /* Initialize IGMP support */

#ifdef CONFIG_NET_IGMP
  uip_igmpinit();
#endif
}
#endif /* CONFIG_NET */

