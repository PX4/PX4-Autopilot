/****************************************************************************
 * examples/thttpd/thttpd_main.c
 *
 *   Copyright (C) 2009-2012 Gregory Nutt. All rights reserved.
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

#include <sys/ioctl.h>
#include <sys/mount.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <netinet/ether.h>

#include <nuttx/net/uip/uip-arp.h>
#include <apps/netutils/uiplib.h>
#include <apps/netutils/thttpd.h>

#include <nuttx/ramdisk.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/nxflat.h>
#ifdef CONFIG_NET_SLIP
#  include <nuttx/net/net.h>
#endif

#include "content/romfs.h"
#include "content/symtab.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Check configuration.  This is not all of the configuration settings that
 * are required -- only the more obvious.
 */

#if CONFIG_NFILE_DESCRIPTORS < 1
#  error "You must provide file descriptors via CONFIG_NFILE_DESCRIPTORS in your configuration file"
#endif

#ifndef CONFIG_NXFLAT
#  error "You must select CONFIG_NXFLAT in your configuration file"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "You must select CONFIG_FS_ROMFS in your configuration file"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "You must not disable mountpoints via CONFIG_DISABLE_MOUNTPOINT in your configuration file"
#endif

#ifdef CONFIG_BINFMT_DISABLE
#  error "You must not disable loadable modules via CONFIG_BINFMT_DISABLE in your configuration file"
#endif

/* SLIP-specific configuration */

#ifdef CONFIG_NET_SLIP

   /* No MAC address operations */

#  undef CONFIG_EXAMPLES_THTTPD_NOMAC

   /* TTY device to use */

#  ifndef CONFIG_NET_SLIPTTY
#    define CONFIG_NET_SLIPTTY "/dev/ttyS1"
#  endif

#  define SLIP_DEVNO 0
#  define NET_DEVNAME "sl0"
#else

   /* Otherwise, use the standard ethernet device name */

#  define NET_DEVNAME "eth0"
#endif

/* Describe the ROMFS file system */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b)+SECTORSIZE-1)/SECTORSIZE)
#define ROMFSDEV     "/dev/ram0"
#define MOUNTPT      CONFIG_THTTPD_PATH

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush()   fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message      lowsyslog
#    define msgflush()
#  else
#    define message      printf
#    define msgflush()   fflush(stdout)
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These values must be provided by the user before the THTTPD task daemon
 * is started:
 *
 * g_thttpdsymtab:  A symbol table describing all of the symbols exported
 *   from the base system.  These symbols are used to bind address references
 *   in CGI programs to NuttX.
 * g_nsymbols:  The number of symbols in g_thttpdsymtab[].
 */

FAR const struct symtab_s *g_thttpdsymtab;
int                         g_thttpdnsymbols;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * thttp_main
 ****************************************************************************/

int thttp_main(int argc, char *argv[])
{
  struct in_addr addr;
#ifdef CONFIG_EXAMPLES_THTTPD_NOMAC
  uint8_t mac[IFHWADDRLEN];
#endif
  char *thttpd_argv = "thttpd";
  int ret;

  /* Configure SLIP */

#ifdef CONFIG_NET_SLIP
  ret = slip_initialize(SLIP_DEVNO, CONFIG_NET_SLIPTTY);
  if (ret < 0)
    {
      message("ERROR: SLIP initialization failed: %d\n", ret);
      exit(1);
    }
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLES_THTTPD_NOMAC
  message("Assigning MAC\n");

  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr(NET_DEVNAME, mac);
#endif

  /* Set up our host address */

  message("Setup network addresses\n");
  addr.s_addr = HTONL(CONFIG_THTTPD_IPADDR);
  uip_sethostaddr(NET_DEVNAME, &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_THTTPD_DRIPADDR);
  uip_setdraddr(NET_DEVNAME, &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_THTTPD_NETMASK);
  uip_setnetmask(NET_DEVNAME, &addr);

  /* Initialize the NXFLAT binary loader */

  message("Initializing the NXFLAT binary loader\n");
  ret = nxflat_initialize();
  if (ret < 0)
    {
      message("ERROR: Initialization of the NXFLAT loader failed: %d\n", ret);
      exit(2);
    }

  /* Create a ROM disk for the ROMFS filesystem */

  message("Registering romdisk\n");
  ret = romdisk_register(0, (uint8_t*)romfs_img, NSECTORS(romfs_img_len), SECTORSIZE);
  if (ret < 0)
    {
      message("ERROR: romdisk_register failed: %d\n", ret);
      nxflat_uninitialize();
      exit(1);
    }

  /* Mount the file system */

  message("Mounting ROMFS filesystem at target=%s with source=%s\n",
         MOUNTPT, ROMFSDEV);

  ret = mount(ROMFSDEV, MOUNTPT, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      message("ERROR: mount(%s,%s,romfs) failed: %s\n",
              ROMFSDEV, MOUNTPT, errno);
      nxflat_uninitialize();
    }

  /* Start THTTPD.  At present, symbol table info is passed via global variables */

  g_thttpdsymtab   = exports;
  g_thttpdnsymbols = NEXPORTS;

  message("Starting THTTPD\n");
  msgflush();
  thttpd_main(1, &thttpd_argv);
  message("THTTPD terminated\n");
  msgflush();
  return 0;
}
