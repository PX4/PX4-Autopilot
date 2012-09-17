/****************************************************************************
 * examples/thttpd/netstat/netstat.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <netinet/ether.h>
#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip-arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
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

/* NOTEs:
 *
 * 1. One limitation in the use of NXFLAT is that functions that are
 *    referenced as a pointer-to-a-function must have global scope.  Otherwise
 *    ARM GCC will generate some bad logic.
 * 2. In general, when called back, there is no guarantee to that PIC registers
 *    will be valid and, unless you take special precautions, it could be
 *    dangerous to reference global variables in the callback function.
 */

/* static */ int netdev_callback(FAR struct uip_driver_s *dev, void *arg)
{
  struct in_addr addr;

  printf("  <dt>%s\r\n", dev->d_ifname);
#ifdef CONFIG_NET_ETHERNET
  printf("    <dd>HWaddr: %s<br>\r\n", ether_ntoa(&dev->d_mac));
#endif
  addr.s_addr = dev->d_ipaddr;
  printf("    IPaddr: %s<br>\r\n", inet_ntoa(addr));
  addr.s_addr = dev->d_draddr;
  printf("    DRaddr: %s<br>\r\n", inet_ntoa(addr));
  addr.s_addr = dev->d_netmask;
  printf("    Mask: %s\r\n", inet_ntoa(addr));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  puts(
	"Content-type: text/html\r\n"
	"Status: 200/html\r\n"
	"\r\n"
    "<html>\r\n"
      "<head>\r\n"
        "<title>Network Status</title>\r\n"
        "<link rel=\"stylesheet\" type=\"text/css\" href=\"/style.css\">\r\n"
      "</head>\r\n"
      "<body bgcolor=\"#fffeec\" text=\"black\">\r\n"
        "<div class=\"menu\">\r\n"
        "<div class=\"menubox\"><a href=\"/index.html\">Front page</a></div>\r\n"
        "<div class=\"menubox\"><a href=\"hello\">Say Hello</a></div>\r\n"
        "<div class=\"menubox\"><a href=\"tasks\">Tasks</a></div>\r\n"
        "<div class=\"menubox\"><a href=\"netstat\">Network status</a></div>\r\n"
        "<br>\r\n"
        "</div>\r\n"
        "<div class=\"contentblock\">\r\n"
        "<dl>\r\n");

  netdev_foreach(netdev_callback, NULL);

  puts(
        "</dl>\r\n"
      "</body>\r\n"
   "</html>\r\n");
  return 0;
}
