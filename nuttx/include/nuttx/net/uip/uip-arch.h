/****************************************************************************
 * include/nuttx/net/uip/uip-arch.h
 * Defines architecture-specific device driver interfaces to uIP
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived largely from portions of uIP with has a similar BSD-styple license:
 *
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

#ifndef __INCLUDE_NUTTX_NET_UIP_UIP_ARCH_H
#define __INCLUDE_NUTTX_NET_UIP_UIP_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdint.h>
#include <net/if.h>

#include <nuttx/net/uip/uip.h>
#ifdef CONFIG_NET_IGMP
#  include <nuttx/net/uip/uip-igmp.h>
#endif

#include <nuttx/net/uip/uipopt.h>
#include <net/ethernet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure collects information that is specific to a specific network
 * interface driver.  If the hardware platform supports only a single instance
 * of this structure.
 */

struct uip_driver_s
{
  /* This link is used to maintain a single-linked list of ethernet drivers.
   * Must be the first field in the structure due to blink type casting.
   */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  FAR struct uip_driver_s *flink;

  /* This is the name of network device assigned when netdev_register was called.
   * This name is only used to support socket ioctl lookups by device name
   * Examples: "eth0"
   */

  char d_ifname[IFNAMSIZ];
#endif

  /* Drivers interface flags.  See IFF_* definitions in include/net/if.h */

  uint8_t d_flags;

  /* Ethernet device identity */

#ifdef CONFIG_NET_ETHERNET
  struct ether_addr d_mac;  /* Device MAC address */
#endif

  /* Network identity */

  uip_ipaddr_t d_ipaddr;  /* Host IP address assigned to the network interface */
  uip_ipaddr_t d_draddr;  /* Default router IP address */
  uip_ipaddr_t d_netmask; /* Network subnet mask */

  /* The d_buf array is used to hold incoming and outgoing packets. The device
   * driver should place incoming data into this buffer. When sending data,
   * the device driver should read the link level headers and the TCP/IP
   * headers from this buffer. The size of the link level headers is
   * configured by the UIP_LLH_LEN define.
   *
   * uIP will handle only a single buffer for both incoming and outgoing
   * packets.  However, the drive design may be concurrently send and
   * filling separate, break-off buffers if CONFIG_NET_MULTIBUFFER is
   * defined.  That buffer management must be controlled by the driver.
   */

#ifdef CONFIG_NET_MULTIBUFFER
  uint8_t *d_buf;
#else
  uint8_t d_buf[CONFIG_NET_BUFSIZE + CONFIG_NET_GUARDSIZE];
#endif

  /* d_appdata points to the location where application data can be read from
   * or written into a packet.
   */

  uint8_t *d_appdata;

  /* This is a pointer into d_buf where a user application may append
   * data to be sent.
   */

  uint8_t *d_snddata;

#ifdef CONFIG_NET_TCPURGDATA
  /* This pointer points to any urgent TCP data that has been received. Only
   * present if compiled with support for urgent data (CONFIG_NET_TCPURGDATA).
   */

  uint8_t *d_urgdata;

  /* Length of the (received) urgent data */

  uint16_t d_urglen;
#endif

/* The length of the packet in the d_buf buffer.
 *
 * Holds the length of the packet in the d_buf buffer.
 *
 * When the network device driver calls the uIP input function,
 * d_len should be set to the length of the packet in the d_buf
 * buffer.
 *
 * When sending packets, the device driver should use the contents of
 * the d_len variable to determine the length of the outgoing
 * packet.
 */

  uint16_t d_len;

  /* When d_buf contains outgoing xmit data, xmtlen is nonzero and represents
   * the amount of appllcation data after d_snddata
   */

  uint16_t d_sndlen;

  /* IGMP group list */

#ifdef CONFIG_NET_IGMP
  sq_queue_t grplist;
#endif

  /* Driver callbacks */

  int (*d_ifup)(struct uip_driver_s *dev);
  int (*d_ifdown)(struct uip_driver_s *dev);
  int (*d_txavail)(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
  int (*d_addmac)(struct uip_driver_s *dev, FAR const uint8_t *mac);
  int (*d_rmmac)(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

  /* Drivers may attached device-specific, private information */

  void *d_private;
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* uIP device driver functions
 *
 * These functions are used by a network device driver for interacting
 * with uIP.
 *
 * Process an incoming packet.
 *
 * This function should be called when the device driver has received
 * a packet from the network. The packet from the device driver must
 * be present in the d_buf buffer, and the length of the packet
 * should be placed in the d_len field.
 *
 * When the function returns, there may be an outbound packet placed
 * in the d_buf packet buffer. If so, the d_len field is set to
 * the length of the packet. If no packet is to be sent out, the
 * d_len field is set to 0.
 *
 * The usual way of calling the function is presented by the source
 * code below.
 *
 *     dev->d_len = devicedriver_poll();
 *     if(dev->d_len > 0) {
 *       uip_input(dev);
 *       if(dev->d_len > 0) {
 *         devicedriver_send();
 *       }
 *     }
 *
 * Note: If you are writing a uIP device driver that needs ARP
 * (Address Resolution Protocol), e.g., when running uIP over
 * Ethernet, you will need to call the uIP ARP code before calling
 * this function:
 *
 *     #define BUF ((struct uip_eth_hdr *)&dev->d_buf[0])
 *     dev->d_len = ethernet_devicedrver_poll();
 *     if(dev->d_len > 0) {
 *       if(BUF->type == HTONS(UIP_ETHTYPE_IP)) {
 *         uip_arp_ipin();
 *         uip_input(dev);
 *         if(dev->d_len > 0) {
 *           uip_arp_out();
 *           devicedriver_send();
 *         }
 *       } else if(BUF->type == HTONS(UIP_ETHTYPE_ARP)) {
 *         uip_arp_arpin();
 *         if(dev->d_len > 0) {
 *           devicedriver_send();
 *         }
 *       }
 */

extern void uip_input(struct uip_driver_s *dev);

/* Polling of connections
 *
 * These functions will traverse each active uIP connection structure and
 * perform appropriate operatios:  uip_timer() will perform TCP timer
 * operations (and UDP polling operations); uip_poll() will perform TCP
 * and UDP polling operations. The CAN driver MUST implement logic to
 * periodically call uip_timer(); uip_poll() may be called asychronously
 * from the network driver can accept another outgoing packet.
 *
 * In both cases, these functions will call the provided callback function
 * for every active connection. Polling will continue until all connections
 * have been polled or until the user-suplied function returns a non-zero
 * value (which it should do only if it cannot accept further write data).
 *
 * When the callback function is called, there may be an outbound packet
 * waiting for service in the uIP packet buffer, and if so the d_len field
 * is set to a value larger than zero. The device driver should then send
 * out the packet.
 *
 * Example:
 *   int driver_callback(struct uip_driver_dev *dev)
 *   {
 *     if (dev->d_len > 0)
 *       {
 *         devicedriver_send();
 *         return 1; <-- Terminates polling if necessary
 *       }
 *     return 0;
 *   }
 *
 *   ...
 *   uip_poll(dev, driver_callback);
 *
 * Note: If you are writing a uIP device driver that needs ARP (Address
 * Resolution Protocol), e.g., when running uIP over Ethernet, you will
 * need to call the uip_arp_out() function in the callback function
 * before sending the packet:
 *
 *   int driver_callback(struct uip_driver_dev *dev)
 *   {
 *     if (dev->d_len > 0)
 *       {
 *         uip_arp_out();
 *         devicedriver_send();
 *         return 1; <-- Terminates polling if necessary
 *       }
 *     return 0;
 *   }
 */

typedef int (*uip_poll_callback_t)(struct uip_driver_s *dev);
extern int uip_poll(struct uip_driver_s *dev, uip_poll_callback_t callback);
extern int uip_timer(struct uip_driver_s *dev, uip_poll_callback_t callback, int hsec);

/* By defining UIP_ARCH_CHKSUM, the architecture can replace up_incr32
 * with hardware assisted solutions.
 */

/* Carry out a 32-bit addition.
 *
 * op32 - A pointer to a 4-byte array representing a 32-bit
 *   integer in network byte order (big endian).  This value may not
 *   be word aligned. The value pointed to by op32 is modified in place
 *
 * op16 - A 16-bit integer in host byte order.
 */

extern void uip_incr32(uint8_t *op32, uint16_t op16);

/* Calculate the Internet checksum over a buffer.
 *
 * The Internet checksum is the one's complement of the one's
 * complement sum of all 16-bit words in the buffer.
 *
 * See RFC1071.
 *
 * Note: This function is not called in the current version of uIP,
 * but future versions might make use of it.
 *
 * buf A pointer to the buffer over which the checksum is to be
 * computed.
 *
 * len The length of the buffer over which the checksum is to
 * be computed.
 *
 * Return:  The Internet checksum of the buffer.
 */

extern uint16_t uip_chksum(uint16_t *buf, uint16_t len);

/* Calculate the IP header checksum of the packet header in d_buf.
 *
 * The IP header checksum is the Internet checksum of the 20 bytes of
 * the IP header.
 *
 * Return:  The IP header checksum of the IP header in the d_buf
 * buffer.
 */

extern uint16_t uip_ipchksum(struct uip_driver_s *dev);

/* Calculate the TCP checksum of the packet in d_buf and d_appdata.
 *
 * The TCP checksum is the Internet checksum of data contents of the
 * TCP segment, and a pseudo-header as defined in RFC793.
 *
 * Note: The d_appdata pointer that points to the packet data may
 * point anywhere in memory, so it is not possible to simply calculate
 * the Internet checksum of the contents of the d_buf buffer.
 *
 * Return:  The TCP checksum of the TCP segment in d_buf and pointed
 * to by d_appdata.
 */

extern uint16_t uip_tcpchksum(struct uip_driver_s *dev);

extern uint16_t uip_udpchksum(struct uip_driver_s *dev);
extern uint16_t uip_icmpchksum(struct uip_driver_s *dev, int len);

#endif /* __INCLUDE_NUTTX_NET_UIP_UIP_ARCH_H */

