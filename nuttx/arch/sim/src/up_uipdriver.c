/****************************************************************************
 * up_uipdriver.c
 *
 *   Copyright (C) 2007, 2009-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on code from uIP which also has a BSD-like license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_NET

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <nuttx/net/net.h>

#include <net/ethernet.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>
#include <nuttx/net/uip/uip-arp.h>

#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define BUF ((struct ether_header*)g_sim_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct timer
{
  uint32_t interval;
  uint32_t start;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timer g_periodic_timer;
static struct uip_driver_s g_sim_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void timer_set(struct timer *t, unsigned int interval)
{
  t->interval = interval;
  t->start    = up_getwalltime();
}

static bool timer_expired( struct timer *t )
{
  return (up_getwalltime() - t->start) >= t->interval;
}

void timer_reset(struct timer *t)
{
  t->start += t->interval;
}

#ifdef CONFIG_NET_PROMISCUOUS
# define up_comparemac(a,b) (0)
#else
static inline int up_comparemac(uint8_t *paddr1, struct ether_addr *paddr2)
{
  return memcmp(paddr1, paddr2->ether_addr_octet, ETHER_ADDR_LEN);
}
#endif

static int sim_uiptxpoll(struct uip_driver_s *dev)
{
  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (g_sim_dev.d_len > 0)
    {
      uip_arp_out(&g_sim_dev);
      netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void uipdriver_loop(void)
{
  /* netdev_read will return 0 on a timeout event and >0 on a data received event */

  g_sim_dev.d_len = netdev_read((unsigned char*)g_sim_dev.d_buf, CONFIG_NET_BUFSIZE);

  /* Disable preemption through to the following so that it behaves a little more
   * like an interrupt (otherwise, the following logic gets pre-empted an behaves
   * oddly.
   */

  sched_lock();
  if (g_sim_dev.d_len > 0)
    {
      /* Data received event.  Check for valid Ethernet header with destination == our
       * MAC address
       */

      if (g_sim_dev.d_len > UIP_LLH_LEN && up_comparemac(BUF->ether_dhost, &g_sim_dev.d_mac) == 0)
        {
          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
          if (BUF->ether_type == htons(UIP_ETHTYPE_IP6))
#else
          if (BUF->ether_type == htons(UIP_ETHTYPE_IP))
#endif
            {
              uip_arp_ipin(&g_sim_dev);
              uip_input(&g_sim_dev);

             /* If the above function invocation resulted in data that
              * should be sent out on the network, the global variable
              * d_len is set to a value > 0.
              */

              if (g_sim_dev.d_len > 0)
                {
                  uip_arp_out(&g_sim_dev);
                  netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
                }
            }
          else if (BUF->ether_type == htons(UIP_ETHTYPE_ARP))
            {
              uip_arp_arpin(&g_sim_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the global variable
               * d_len is set to a value > 0.
               */

              if (g_sim_dev.d_len > 0)
                {
                  netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
                }
            }
        }
    }

  /* Otherwise, it must be a timeout event */

  else if (timer_expired(&g_periodic_timer))
    {
      timer_reset(&g_periodic_timer);
      uip_timer(&g_sim_dev, sim_uiptxpoll, 1);
    }
  sched_unlock();
}

int uipdriver_init(void)
{
  /* Internal initalization */

  timer_set(&g_periodic_timer, 500);
  netdev_init();

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_sim_dev);
  return OK;
}

int uipdriver_setmacaddr(unsigned char *macaddr)
{
  (void)memcpy(g_sim_dev.d_mac.ether_addr_octet, macaddr, IFHWADDRLEN);
  return 0;
}

#endif /* CONFIG_NET */

