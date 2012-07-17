/****************************************************************************
 * include/nuttx/net/uip/uip-arch.h
 * Macros and definitions for the ARP module.
 *
 *   Copyright (C) 2007, 2009-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from uIP with has a similar BSD-styple license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
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

#ifndef __INCLUDE_NUTTX_NET_UIP_UIP_ARP_H
#define __INCLUDE_NUTTX_NET_UIP_UIP_ARP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <net/ethernet.h>
#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Recognized values of the type bytes in the Ethernet header */

#define UIP_ETHTYPE_ARP 0x0806
#define UIP_ETHTYPE_IP  0x0800
#define UIP_ETHTYPE_IP6 0x86dd

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The Ethernet header -- 14 bytes. The first two fields are type 'struct
 * ether_addr but are represented as a simple byte array here because
 * some compilers refuse to pack 6 byte structures.
 */

struct uip_eth_hdr
{
  uint8_t  dest[6]; /* Ethernet destination address (6 bytes) */
  uint8_t  src[6];  /* Ethernet source address (6 bytes) */
  uint16_t type;    /* Type code (2 bytes) */
};

/* One entry in the ARP table (volatile!) */

struct arp_entry
{
  in_addr_t         at_ipaddr;   /* IP address */
  struct ether_addr at_ethaddr;  /* Hardware address */
  uint8_t           at_time;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_ARP
/****************************************************************************
 * Name: uip_arp_init
 *
 * Description:
 *   Initialize the ARP module. This function must be called before any of
 *   the other ARP functions.
 *
 ****************************************************************************/

EXTERN void uip_arp_init(void);

/****************************************************************************
 * Name: uip_arp_ipin
 *
 * Description:
 *   The uip_arp_ipin() function should be called whenever an IP packet
 *   arrives from the Ethernet. This function refreshes the ARP table or
 *   inserts a new mapping if none exists. The function assumes that an
 *   IP packet with an Ethernet header is present in the d_buf buffer
 *   and that the length of the packet is in the d_len field.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_IPIN
EXTERN void uip_arp_ipin(struct uip_driver_s *dev);
#else
# define uip_arp_ipin(dev)
#endif

/****************************************************************************
 * Name: uip_arp_arpin
 *
 * Description:
 *   The uip_arp_arpin() should be called when an ARP packet is received
 *   by the Ethernet driver. This function also assumes that the
 *   Ethernet frame is present in the d_buf buffer. When the
 *   uip_arp_arpin() function returns, the contents of the d_buf
 *   buffer should be sent out on the Ethernet if the d_len field
 *   is > 0.
 *
 ****************************************************************************/

EXTERN void uip_arp_arpin(struct uip_driver_s *dev);

/****************************************************************************
 * Name: uip_arp_arpin
 *
 * Description:
 *   The uip_arp_out() function should be called when an IP packet
 *   should be sent out on the Ethernet. This function creates an
 *   Ethernet header before the IP header in the d_buf buffer. The
 *   Ethernet header will have the correct Ethernet MAC destination
 *   address filled in if an ARP table entry for the destination IP
 *   address (or the IP address of the default router) is present. If no
 *   such table entry is found, the IP packet is overwritten with an ARP
 *   request and we rely on TCP to retransmit the packet that was
 *   overwritten. In any case, the d_len field holds the length of
 *   the Ethernet frame that should be transmitted.
 *
 ****************************************************************************/

EXTERN void uip_arp_out(struct uip_driver_s *dev);

/****************************************************************************
 * Name: uip_arp_timer
 *
 * Description:
 *   This function performs periodic timer processing in the ARP module
 *   and should be called at regular intervals. The recommended interval
 *   is 10 seconds between the calls.  It is responsible for flushing old
 *   entries in the ARP table.
 *
 ****************************************************************************/

EXTERN void uip_arp_timer(void);

/****************************************************************************
 * Name: uip_arp_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input parameters:
 *   pipaddr - Refers to an IP address uint16_t[2] in network order
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Assumptions
 *   Interrupts are disabled
 *
 ****************************************************************************/

EXTERN void uip_arp_update(uint16_t *pipaddr, uint8_t *ethaddr);

/****************************************************************************
 * Name: uip_arp_find
 *
 * Description:
 *   Find the ARP entry corresponding to this IP address.
 *
 * Input parameters:
 *   ipaddr - Refers to an IP address in network order
 *
 * Assumptions
 *   Interrupts are disabled; Returned value will become unstable when
 *   interrupts are re-enabled or if any other uIP APIs are called.
 *
 ****************************************************************************/

EXTERN struct arp_entry *uip_arp_find(in_addr_t ipaddr);

/****************************************************************************
 * Name: uip_arp_delete
 *
 * Description:
 *   Remove an IP association from the ARP table
 *
 * Input parameters:
 *   ipaddr - Refers to an IP address in network order
 *
 * Assumptions
 *   Interrupts are disabled
 *
 ****************************************************************************/

#define uip_arp_delete(ipaddr) \
{ \
  struct arp_entry *tabptr = uip_arp_find(ipaddr); \
  if (tabptr) \
    { \
      tabptr->at_ipaddr = 0; \
    } \
}

#else /* CONFIG_NET_ARP */

/* If ARP is disabled, stub out all ARP interfaces */

# define uip_arp_init()
# define uip_arp_ipin(dev)
# define uip_arp_arpin(dev)
# define uip_arp_out(dev)
# define uip_arp_timer()
# define uip_arp_update(pipaddr,ethaddr)
# define uip_arp_find(ipaddr) NULL
# define uip_arp_delete(ipaddr)

#endif /* CONFIG_NET_ARP */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_UIP_UIP_ARP_H */
