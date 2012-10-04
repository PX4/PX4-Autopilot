/****************************************************************************
 * up_wcap.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on code from uIP which also has a BSD-like license:
 *
 *   Copyright (c) 2007, Swedish Institute of Computer Science.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifdef __CYGWIN__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define WIN32_LEAN_AND_MEAN
#define _WIN32_WINNT 0x0501
#include <windows.h>
#include <winsock2.h>
#include <iphlpapi.h>

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>

extern int uipdriver_setmacaddr(unsigned char *macaddr);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

#ifndef CONFIG_EXAMPLES_UIP_DHCPC
#  define UIP_IPADDR (10 << 24 | 0 << 16 | 0 << 8 | 1)
#else
#  define UIP_IPADDR (0)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

__attribute__ ((dllimport)) extern char **__argv[];

struct pcap;

struct pcap_if
{
  struct pcap_if *next;
  char *name;
  char *description;
  struct pcap_addr
  {
    struct pcap_addr *next;
    struct sockaddr *addr;
    struct sockaddr *netmask;
    struct sockaddr *broadaddr;
    struct sockaddr *dstaddr;
  } *addresses;
  DWORD flags;
};

struct pcap_pkthdr
{
  struct timeval ts;
  DWORD caplen;
  DWORD len;
};

/* DLL function types (for casting) */

typedef int (*pcap_findalldevs_t)(struct pcap_if **, char *);
typedef struct pcap *(*pcap_open_live_t)(char *, int, int, int, char *);
typedef int (*pcap_next_ex_t)(struct pcap *, struct pcap_pkthdr **,
                              unsigned char **);
typedef int (*pcap_sendpacket_t)(struct pcap *, unsigned char *, int);

/****************************************************************************
 * Private Data
 ****************************************************************************/

HMODULE wpcap;
static struct pcap *pcap;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static pcap_findalldevs_t pcap_findalldevs;
static pcap_open_live_t   pcap_open_live;
static pcap_next_ex_t     pcap_next_ex;
static pcap_sendpacket_t  pcap_sendpacket;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void error_exit(char *message)
{
  printf("error_exit: %s", message);
  exit(EXIT_FAILURE);
}

static void init_pcap(struct in_addr addr)
{
  struct pcap_if *interfaces;
  char error[256];

  if (pcap_findalldevs(&interfaces, error) == -1)
    {
      error_exit(error);
    }

  while (interfaces != NULL)
    {
      printf("init_pcap: found interface: %s\n", interfaces->description);

      if (interfaces->addresses != NULL &&
          interfaces->addresses->addr != NULL &&
          interfaces->addresses->addr->sa_family == AF_INET)
        {

          struct in_addr interface_addr;
          interface_addr =
            ((struct sockaddr_in *)interfaces->addresses->addr)->sin_addr;
          printf("init_pcap: with address: %s\n", inet_ntoa(interface_addr));

          if (interface_addr.s_addr == addr.s_addr)
            {
              break;
            }
        }
      interfaces = interfaces->next;
    }

  if (interfaces == NULL)
    {
      error_exit("No interface found with IP address\n");
    }

  pcap = pcap_open_live(interfaces->name, NETDEV_BUFSIZE, 0, -1, error);
  if (pcap == NULL)
    {
      error_exit(error);
    }
}

static void set_ethaddr(struct in_addr addr)
{
  PIP_ADAPTER_ADDRESSES adapters;
  ULONG size = 0;

  if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST |
                           GAA_FLAG_SKIP_MULTICAST |
                           GAA_FLAG_SKIP_DNS_SERVER,
                           NULL, NULL, &size) != ERROR_BUFFER_OVERFLOW)
    {
      error_exit("error on access to adapter list size\n");
    }
  adapters = alloca(size);
  if (GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_ANYCAST |
                           GAA_FLAG_SKIP_MULTICAST |
                           GAA_FLAG_SKIP_DNS_SERVER,
                           NULL, adapters, &size) != ERROR_SUCCESS)
    {
      error_exit("error on access to adapter list\n");
    }

  while (adapters != NULL)
    {

      char buffer[256];
      WideCharToMultiByte(CP_ACP, 0, adapters->Description, -1,
                          buffer, sizeof(buffer), NULL, NULL);
      printf("set_ethaddr: found adapter: %s\n", buffer);

      if (adapters->FirstUnicastAddress != NULL &&
          adapters->FirstUnicastAddress->Address.lpSockaddr != NULL &&
          adapters->FirstUnicastAddress->Address.lpSockaddr->sa_family ==
          AF_INET)
        {

          struct in_addr adapter_addr;
          adapter_addr =
            ((struct sockaddr_in *)adapters->FirstUnicastAddress->Address.
             lpSockaddr)->sin_addr;
          printf("set_ethaddr: with address: %s\n", inet_ntoa(adapter_addr));

          if (adapter_addr.s_addr == addr.s_addr)
            {
              if (adapters->PhysicalAddressLength != 6)
                {
                  error_exit
                    ("ip addr specified does not belong to an ethernet card\n");
                }
              printf
                ("set_ethaddr:  ethernetaddr: %02X-%02X-%02X-%02X-%02X-%02X\n",
                 adapters->PhysicalAddress[0], adapters->PhysicalAddress[1],
                 adapters->PhysicalAddress[2], adapters->PhysicalAddress[3],
                 adapters->PhysicalAddress[4], adapters->PhysicalAddress[5]);

                 (void)uipdriver_setmacaddr(adapters->PhysicalAddress);
              break;
            }
        }
      adapters = adapters->Next;
    }

  if (adapters == NULL)
    {
      error_exit("No adaptor found with IP address\n");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void wpcap_init(void)
{
  struct in_addr addr;
  FARPROC dlladdr;

  addr.s_addr = htonl(UIP_IPADDR);
  printf("wpcap_init: IP address: %s\n", inet_ntoa(addr));

  wpcap = LoadLibrary("wpcap.dll");
  dlladdr = GetProcAddress(wpcap, "pcap_findalldevs");
  pcap_findalldevs = (pcap_findalldevs_t)dlladdr;

  dlladdr = GetProcAddress(wpcap, "pcap_open_live");
  pcap_open_live = (pcap_open_live_t)dlladdr;

  dlladdr = GetProcAddress(wpcap, "pcap_next_ex");
  pcap_next_ex = (pcap_next_ex_t)dlladdr;

  dlladdr = GetProcAddress(wpcap, "pcap_sendpacket");
  pcap_sendpacket = (pcap_sendpacket_t)dlladdr;

  if (pcap_findalldevs == NULL || pcap_open_live == NULL ||
      pcap_next_ex == NULL || pcap_sendpacket == NULL)
    {
      error_exit("error on access to winpcap library\n");
    }

  init_pcap(addr);
  set_ethaddr(addr);
}

unsigned int wpcap_read(unsigned char *buf, unsigned int buflen)
{
  struct pcap_pkthdr *packet_header;
  unsigned char *packet;

  switch (pcap_next_ex(pcap, &packet_header, &packet))
    {
    case -1:
      error_exit("error on read\n");
    case 0:
      return 0;
    }

  if (packet_header->caplen > buflen)
    {
      return 0;
    }

  memcpy(buf, packet, packet_header->caplen);
  return packet_header->caplen;
}

void wpcap_send(unsigned char *buf, unsigned int buflen)
{
  if (pcap_sendpacket(pcap, buf, buflen) == -1)
    {
      error_exit("error on send\n");
    }
}

#endif /* __CYGWIN__ */
