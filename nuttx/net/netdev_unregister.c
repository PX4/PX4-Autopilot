/****************************************************************************
 * net/netdev_unregister.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_unregister
 *
 * Description:
 *   Unregister a network device driver.
 *
 * Parameters:
 *   dev - The device driver structure to un-register
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 * Assumptions:
 *   Currently only called for USB networking devices when the device is
 *   physically removed from the slot
 *
 ****************************************************************************/

int netdev_unregister(FAR struct uip_driver_s *dev)
{
  struct uip_driver_s *prev;
  struct uip_driver_s *curr;

  if (dev)
    {
      netdev_semtake();

      /* Find the device in the list of known network devices */

      for (prev = NULL, curr = g_netdevices;
           curr && curr != dev;
           prev = curr, curr = curr->flink);

      /* Remove the device to the list of known network devices */

      if (curr)
        {
          /* Where was the entry */

          if (prev)
            {
              /* The entry was in the middle or at the end of the list */

              prev->flink = curr->flink;
            }
          else
            {
               /* The entry was at the beginning of the list */

               g_netdevices = curr;
            }

          curr->flink = NULL;
        }

      netdev_semgive();

#ifdef CONFIG_NET_ETHERNET
      nlldbg("Unregistered MAC: %02x:%02x:%02x:%02x:%02x:%02x as dev: %s\n",
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
