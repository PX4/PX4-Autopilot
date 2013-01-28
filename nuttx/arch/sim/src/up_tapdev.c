/****************************************************************************
 * up_tapdev.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __CYGWIN__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <linux/if.h>
#include <linux/if_tun.h>
#include <linux/net.h>

extern int syslog(const char *format, ...);
extern int uipdriver_setmacaddr(unsigned char *macaddr);

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define TAPDEV_DEBUG    1

#define DEVTAP          "/dev/net/tun"

#ifndef CONFIG_EXAMPLES_UIP_DHCPC
#  define UIP_IPADDR0   192
#  define UIP_IPADDR1   168
#  define UIP_IPADDR2   0
#  define UIP_IPADDR3   128
#else
#  define UIP_IPADDR0   0
#  define UIP_IPADDR1   0
#  define UIP_IPADDR2   0
#  define UIP_IPADDR3   0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Warning: This is very much Linux version specific! */

struct sel_arg_struct
{
  unsigned long   n;
  fd_set         *inp;
  fd_set         *outp;
  fd_set         *exp;
  struct timeval *tvp;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static int gdrop = 0;
#endif
static int gtapdevfd;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static inline void dump_ethhdr(const char *msg, unsigned char *buf, int buflen)
{
  syslog("TAPDEV: %s %d bytes\n", msg, buflen);
  syslog("        %02x:%02x:%02x:%02x:%02x:%02x %02x:%02x:%02x:%02x:%02x:%02x %02x%02x\n",
         buf[0], buf[1], buf[2], buf[3], buf[4],  buf[5],
         buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
#ifdef CONFIG_ENDIAN_BIG
         buf[13], buf[12]);
#else
         buf[12], buf[13]);
#endif
}
#else
#  define dump_ethhdr(m,b,l)
#endif

static int up_setmacaddr(void)
{
  int sockfd;
  int ret = -1;

  /* Get a socket (only so that we get access to the INET subsystem) */

  sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd >= 0)
    {
      struct ifreq req;
      memset(&req, 0, sizeof(struct ifreq));

      /* Put the driver name into the request */

      strncpy(req.ifr_name, "tap0", IFNAMSIZ);

      /* Perform the ioctl to get the MAC address */

      ret = ioctl(sockfd, SIOCGIFHWADDR, (unsigned long)&req);
      if (!ret)
        {
          /* Set the MAC address */

          ret = uipdriver_setmacaddr(&req.ifr_hwaddr.sa_data);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tapdev_init(void)
{
  struct ifreq ifr;
  char buf[1024];
  int ret;

  /* Open the tap device */

  gtapdevfd = open(DEVTAP, O_RDWR, 0644);
  if (gtapdevfd < 0)
    {
      syslog("TAPDEV: open failed: %d\n", -gtapdevfd );
      return;
    }

  /* Configure the tap device */

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP|IFF_NO_PI;
  ret = ioctl(gtapdevfd, TUNSETIFF, (unsigned long) &ifr);
  if (ret < 0)
    {
      syslog("TAPDEV: ioctl failed: %d\n", -ret );
      return;
   }

  /* Assign an IPv4 address to the tap device */

  snprintf(buf, sizeof(buf), "/sbin/ifconfig tap0 inet %d.%d.%d.%d\n",
           UIP_IPADDR0, UIP_IPADDR1, UIP_IPADDR2, UIP_IPADDR3);
  system(buf);

  /* Set the MAC address */

  up_setmacaddr();
}

unsigned int tapdev_read(unsigned char *buf, unsigned int buflen)
{
  fd_set                fdset;
  struct timeval        tv;
  int                   ret;

  /* We can't do anything if we failed to open the tap device */

  if (gtapdevfd < 0)
    {
      return 0;
    }

  /* Wait for data on the tap device (or a timeout) */

  tv.tv_sec  = 0;
  tv.tv_usec = 1000;

  FD_ZERO(&fdset);
  FD_SET(gtapdevfd, &fdset);

  ret = select(gtapdevfd + 1, &fdset, NULL, NULL, &tv);
  if(ret == 0)
    {
      return 0;
    }

  ret = read(gtapdevfd, buf, buflen);
  if (ret < 0)
    {
      syslog("TAPDEV: read failed: %d\n", -ret);
      return 0;
    }

  dump_ethhdr("read", buf, ret);
  return ret;
}

void tapdev_send(unsigned char *buf, unsigned int buflen)
{
  int ret;
#ifdef TAPDEV_DEBUG
  syslog("tapdev_send: sending %d bytes\n", buflen);

  gdrop++;
  if(gdrop % 8 == 7)
    {
      syslog("Dropped a packet!\n");
      return;
    }
#endif

  ret = write(gtapdevfd, buf, buflen);
  if (ret < 0)
    {
      syslog("TAPDEV: write failed: %d", -ret);
      exit(1);
    }
  dump_ethhdr("write", buf, buflen);
}

#endif /* !__CYGWIN__ */


