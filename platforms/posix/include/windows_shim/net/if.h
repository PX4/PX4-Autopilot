/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
/**
 * @file net/if.h
 *
 * Windows exposes interface metadata through netioapi/iphlpapi rather than
 * POSIX ioctls. This header keeps the public POSIX networking surface
 * available: IF_* sizing, interface flags, ifreq/ifconf containers,
 * if_nameindex, and the common SIOCGIF* request numbers used by callers as
 * compile-time API.
 */
#pragma once

#ifdef _WIN32

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <sys/socket.h>
#include <iphlpapi.h>
#include <netioapi.h>

#ifndef IF_NAMESIZE
#define IF_NAMESIZE 256
#endif

#ifndef IFNAMSIZ
#define IFNAMSIZ IF_NAMESIZE
#endif
#ifndef IFHWADDRLEN
#define IFHWADDRLEN 6
#endif
#ifndef MAX_IFINDEX
#define MAX_IFINDEX 256
#endif

/** @brief Hardware mapping payload carried by struct ifreq on POSIX. */
struct ifmap {
	unsigned long  mem_start;
	unsigned long  mem_end;
	unsigned short base_addr;
	unsigned char  irq;
	unsigned char  dma;
	unsigned char  port;
};

/**
 * @brief POSIX interface request container.
 *
 * POSIX <net/if.h> exposes `struct ifreq` for SIOCGIFNETMASK / SIOCGIFADDR
 * ioctls. Windows uses WSAIoctl with SIO_GET_INTERFACE_LIST instead. We
 * still need the type to exist so MAVLink signatures parse - the UDP
 * broadcast path (query_netmask_addr) is guarded with MAVLINK_UDP and
 * stays wired on Windows even though the runtime implementation differs.
 */
struct ifreq {
	char ifr_name[IF_NAMESIZE];
	union {
		struct sockaddr ifr_addr;
		struct sockaddr ifr_dstaddr;
		struct sockaddr ifr_broadaddr;
		struct sockaddr ifr_netmask;
		struct sockaddr ifr_hwaddr;
		short           ifr_flags;
		int             ifr_ifindex;
		int             ifr_metric;
		int             ifr_mtu;
		struct ifmap    ifr_map;
		char            ifr_slave[IF_NAMESIZE];
		char            ifr_newname[IF_NAMESIZE];
		char           *ifr_data;
	} ifr_ifru;
};

#define ifr_addr      ifr_ifru.ifr_addr
#define ifr_dstaddr   ifr_ifru.ifr_dstaddr
#define ifr_broadaddr ifr_ifru.ifr_broadaddr
#define ifr_netmask   ifr_ifru.ifr_netmask
#define ifr_hwaddr    ifr_ifru.ifr_hwaddr
#define ifr_flags     ifr_ifru.ifr_flags
#define ifr_ifindex   ifr_ifru.ifr_ifindex
#define ifr_metric    ifr_ifru.ifr_metric
#define ifr_mtu       ifr_ifru.ifr_mtu
#define ifr_map       ifr_ifru.ifr_map
#define ifr_newname   ifr_ifru.ifr_newname
#define ifr_data      ifr_ifru.ifr_data

#ifndef IFF_UP
#define IFF_UP        0x1
#endif
#ifndef IFF_DEBUG
#define IFF_DEBUG     0x4
#endif
#ifndef IFF_BROADCAST
#define IFF_BROADCAST 0x2
#endif
#ifndef IFF_LOOPBACK
#define IFF_LOOPBACK  0x8
#endif
#ifndef IFF_POINTOPOINT
#define IFF_POINTOPOINT 0x10
#endif
#ifndef IFF_NOTRAILERS
#define IFF_NOTRAILERS 0x20
#endif
#ifndef IFF_RUNNING
#define IFF_RUNNING   0x40
#endif
#ifndef IFF_NOARP
#define IFF_NOARP     0x80
#endif
#ifndef IFF_PROMISC
#define IFF_PROMISC   0x100
#endif
#ifndef IFF_ALLMULTI
#define IFF_ALLMULTI  0x200
#endif
#ifndef IFF_MASTER
#define IFF_MASTER    0x400
#endif
#ifndef IFF_SLAVE
#define IFF_SLAVE     0x800
#endif
#ifndef IFF_MULTICAST
#define IFF_MULTICAST 0x1000
#endif
#ifndef IFF_PORTSEL
#define IFF_PORTSEL   0x2000
#endif
#ifndef IFF_AUTOMEDIA
#define IFF_AUTOMEDIA 0x4000
#endif
#ifndef IFF_DYNAMIC
#define IFF_DYNAMIC   0x8000
#endif

#define IFF_IS_UP(f)          (((f) & IFF_UP) != 0)
#define IFF_IS_RUNNING(f)     (((f) & IFF_RUNNING) != 0)
#define IFF_IS_LOOPBACK(f)    (((f) & IFF_LOOPBACK) != 0)
#define IFF_IS_POINTOPOINT(f) (((f) & IFF_POINTOPOINT) != 0)
#define IFF_IS_NOARP(f)       (((f) & IFF_NOARP) != 0)
#define IFF_IS_MULTICAST(f)   (((f) & IFF_MULTICAST) != 0)
#define IFF_IS_BROADCAST(f)   (((f) & IFF_BROADCAST) != 0)

#ifndef SIOCGIFCONF
#define SIOCGIFCONF    0x8912
#define SIOCGIFFLAGS   0x8913
#define SIOCSIFFLAGS   0x8914
#define SIOCGIFADDR    0x8915
#define SIOCSIFADDR    0x8916
#define SIOCGIFDSTADDR 0x8917
#define SIOCSIFDSTADDR 0x8918
#define SIOCGIFBRDADDR 0x8919
#define SIOCSIFBRDADDR 0x891A
#define SIOCGIFNETMASK 0x891b
#define SIOCSIFNETMASK 0x891c
#define SIOCGIFMETRIC  0x891d
#define SIOCSIFMETRIC  0x891e
#define SIOCGIFMTU     0x8921
#define SIOCSIFMTU     0x8922
#define SIOCGIFHWADDR  0x8927
#define SIOCGIFINDEX   0x8933
#endif

/**
 * @brief POSIX variable-length interface request array for SIOCGIFCONF.
 *
 * Windows has no direct equivalent (interface enumeration goes through
 * GetAdaptersAddresses instead). MAVLink uses ifconf only as a
 * type container when walking broadcast addresses - the runtime
 * iteration path is stubbed by the shim's ioctl/SIOCGIFCONF handler.
 */
struct ifconf {
	int ifc_len;
	union {
		char          *ifc_buf;
		struct ifreq  *ifc_req;
	} ifc_ifcu;
};
#define ifc_buf ifc_ifcu.ifc_buf
#define ifc_req ifc_ifcu.ifc_req

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Interface index/name pair returned by if_nameindex(). */
struct if_nameindex {
	unsigned int if_index;
	char        *if_name;
};

/* `if_nametoindex` and `if_indextoname` are already declared (with
 * NET_IFINDEX / PCSTR / PCHAR) by MinGW's <netioapi.h>. Don't redeclare them here - the
 * signatures don't match byte-for-byte (NET_IFINDEX vs. unsigned int)
 * and GCC flags the conflict as an error. */

/**
 * @brief Return a NULL-terminated array of interface index/name pairs.
 *
 * The implementation queries GetAdaptersAddresses and converts names into the
 * POSIX ownership model. Free the returned array with if_freenameindex().
 */
struct if_nameindex *if_nameindex(void);

/** @brief Free an array returned by if_nameindex(). */
void                 if_freenameindex(struct if_nameindex *ptr);

#ifdef __cplusplus
}
#endif

#endif /* _WIN32 */
