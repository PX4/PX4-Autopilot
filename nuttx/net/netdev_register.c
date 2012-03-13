/****************************************************************************
 * net/netdev_register.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/socket.h>
#include <stdio.h>
#include <semaphore.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/uip/uip-arch.h>

#include "net_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_SLIP
#  define NETDEV_FORMAT "sl%d"
#else
#  define NETDEV_FORMAT "eth%d"
#endif

/****************************************************************************
 * Priviate Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_next_devnum = 0;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* List of registered ethernet device drivers */

struct uip_driver_s *g_netdevices = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_register
 *
 * Description:
 *   Register a network device driver and assign a name to it so tht it can
 *   be found in subsequent network ioctl operations on the device.
 *
 * Parameters:
 *   dev - The device driver structure to register
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 * Assumptions:
 *   Called during system initialization from normal user mode
 *
 ****************************************************************************/

int netdev_register(FAR struct uip_driver_s *dev)
{
  if (dev)
    {
      int devnum;
      netdev_semtake();

      /* Assign a device name to the interface */

      devnum = g_next_devnum++;
      snprintf(dev->d_ifname, IFNAMSIZ, NETDEV_FORMAT, devnum );

      /* Add the device to the list of known network devices */

      dev->flink  = g_netdevices;
      g_netdevices = dev;

      /* Configure the device for IGMP support */

#ifdef CONFIG_NET_IGMP
      uip_igmpdevinit(dev);
#endif
      netdev_semgive();

#ifdef CONFIG_NET_ETHERNET
      nlldbg("Registered MAC: %02x:%02x:%02x:%02x:%02x:%02x as dev: %s\n",
             dev->d_mac.ether_addr_octet[0], dev->d_mac.ether_addr_octet[1],
             dev->d_mac.ether_addr_octet[2], dev->d_mac.ether_addr_octet[3],
             dev->d_mac.ether_addr_octet[4], dev->d_mac.ether_addr_octet[5],
             dev->d_ifname);
#else
      nlldbg("Registered dev: %s\n", dev->d_ifname);
#endif
      return OK;
    }
  return -EINVAL;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
