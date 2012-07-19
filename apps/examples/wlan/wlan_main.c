/****************************************************************************
 * examples/wlan/wlan_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Rafael Noronha <rafael@pdsolucoes.com.br>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usbhost.h>

#include <net/if.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>

#include <apps/netutils/uiplib.h>

/* DHCPC may be used in conjunction with any other feature (or not) */

#ifdef CONFIG_EXAMPLE_WLAN_DHCPC
#  include <arpa/inet.h>
#  include <apps/netutils/resolv.h>
#  include <apps/netutils/dhcpc.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Sanity checking */

#ifndef CONFIG_USBHOST
#  error "CONFIG_USBHOST is not defined"
#endif

#ifdef CONFIG_USBHOST_BULK_DISABLE
#  error "Bulk endpoints are disabled (CONFIG_USBHOST_BULK_DISABLE)"
#endif

#ifndef CONFIG_NFILE_DESCRIPTORS
#  error "CONFIG_NFILE_DESCRIPTORS > 0 needed"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_EXAMPLES_WLAN_DEFPRIO
#  define CONFIG_EXAMPLES_WLAN_DEFPRIO 50
#endif

#ifndef CONFIG_EXAMPLES_WLAN_STACKSIZE
#  define CONFIG_EXAMPLES_WLAN_STACKSIZE 1024
#endif

#ifndef CONFIG_EXAMPLES_WLAN_DEVNAME
#  define CONFIG_EXAMPLES_WLAN_DEVNAME "wlan0"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbhost_driver_s *g_drvr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wlan_bringup
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

static inline void wlan_bringup(void)
{
#if defined(CONFIG_EXAMPLE_WLAN_DHCPC) || defined(CONFIG_EXAMPLE_WLAN_NOMAC)
  uint8_t mac[IFHWADDRLEN];
#endif
  struct in_addr addr;
#ifdef CONFIG_EXAMPLE_WLAN_DHCPC
  void *handle;
#endif

  /* Many embedded network interfaces must have a software assigned
   * MAC
   */

#ifdef CONFIG_EXAMPLE_WLAN_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_WLAN_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_WLAN_NETMASK);
  uip_setnetmask("eth0", &addr);

  /* Set up our host address */

#ifdef CONFIG_EXAMPLE_WLAN_DHCPC
  addr.s_addr = 0;
#else
  addr.s_addr = HTONL(CONFIG_EXAMPLE_WLAN_IPADDR);
#endif
  uip_sethostaddr("eth0", &addr);

#ifdef CONFIG_EXAMPLE_WLAN_DHCPC
  /* Set up the resolver */

  resolv_init();

  /* Get the MAC address of the NIC */

  uip_getmacaddr("eth0", mac);

  /* Set up the DHCPC modules */

  handle = dhcpc_open(&mac, IFHWADDRLEN);

  /* Get an IP address.  Note:  there is no logic here for renewing
   * the address in this example.  The address should be renewed in
   * ds.lease_time/2 seconds.
   */

  printf("Getting IP address\n");
  if (handle)
    {
      struct dhcpc_state ds;
      (void)dhcpc_request(handle, &ds);
      uip_sethostaddr("eth1", &ds.ipaddr);
      if (ds.netmask.s_addr != 0)
        {
          uip_setnetmask("eth0", &ds.netmask);
        }
      if (ds.default_router.s_addr != 0)
        {
          uip_setdraddr("eth0", &ds.default_router);
        }
      if (ds.dnsaddr.s_addr != 0)
        {
          resolv_conf(&ds.dnsaddr);
        }
      dhcpc_close(handle);
      printf("IP: %s\n", inet_ntoa(ds.ipaddr));
    }
#endif
}

/****************************************************************************
 * Name: wlan_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

static int wlan_waiter(int argc, char *argv[])
{
  bool connected = false;
  int ret;

  printf("wlan_waiter: Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      ret = DRVR_WAIT(g_drvr, connected);
      DEBUGASSERT(ret == OK);

      connected = !connected;
      printf("wlan_waiter: %s\n", connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (connected)
        {
          /* Yes.. enumerate the newly connected device */

          ret = DRVR_ENUMERATE(g_drvr);

          /* If the enumeration was successful, then bring up the interface */

          wlan_bringup();
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}

/****************************************************************************
 * Name: user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  pid_t pid;
  int ret;

  /* First, register all of the USB host Wireless LAN drivers */

  printf("user_start: Register drivers\n");
  ret = usbhost_wlaninit();
  if (ret != OK)
    {
      printf("user_start: Failed to register the WLAN driver\n");
    }

  /* Then get an instance of the USB host interface */

  printf("user_start: Initialize USB host WLAN driver\n");
  g_drvr = usbhost_initialize(0);
  if (g_drvr)
    {
      /* Start a thread to handle device connection. */

      printf("user_start: Start wlan_waiter\n");

#ifndef CONFIG_CUSTOM_STACK
      pid = task_create("usbhost", CONFIG_EXAMPLES_WLAN_DEFPRIO,
                        CONFIG_EXAMPLES_WLAN_STACKSIZE,
                        (main_t)wlan_waiter, (const char **)NULL);
#else
      pid = task_create("usbhost", CONFIG_EXAMPLES_WLAN_DEFPRIO,
                        (main_t)wlan_waiter, (const char **)NULL);
#endif

      /* Now just sleep.  Eventually logic here will perform the device test. */

      for (;;)
        {
          sleep(5);
          printf("usert_start:  Still alive\n");
        }
    }
  return 0;
}
