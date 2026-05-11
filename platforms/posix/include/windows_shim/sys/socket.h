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
 * @file sys/socket.h
 *
 * MinGW ships winsock2.h; POSIX code expects <sys/socket.h>. Pull in
 * winsock2/ws2tcpip and expose the POSIX-style socket typedefs,
 * ancillary-message structs, MSG_* flags, sendmsg/recvmsg, and a
 * local socketpair() helper.
 * A matching <netinet/in.h> / <arpa/inet.h> sit next to this file and
 * simply re-include us.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef OPTIONAL
#define _PX4_WINDOWS_SHIM_DEFINED_OPTIONAL
#define OPTIONAL
#endif
#include <mswsock.h>
#ifdef _PX4_WINDOWS_SHIM_DEFINED_OPTIONAL
#undef OPTIONAL
#undef _PX4_WINDOWS_SHIM_DEFINED_OPTIONAL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* POSIX names mapped onto winsock types. */
typedef int socklen_t_compat_unused_;  /* placeholder, ws2tcpip provides socklen_t */
typedef SSIZE_T ssize_t_socket_unused_;

#ifndef _PX4_SA_FAMILY_T_DEFINED
#define _PX4_SA_FAMILY_T_DEFINED
typedef ADDRESS_FAMILY sa_family_t;
#endif

#ifndef AF_LOCAL
#define AF_LOCAL AF_UNIX
#endif

#ifndef PF_LOCAL
#define PF_LOCAL PF_UNIX
#endif

#ifndef _PX4_IOVEC_DEFINED
#define _PX4_IOVEC_DEFINED
/** @brief Scatter/gather buffer descriptor used by sendmsg()/recvmsg(). */
struct iovec {
	void  *iov_base;
	size_t iov_len;
};
#endif

#ifndef _PX4_MSGHDR_DEFINED
#define _PX4_MSGHDR_DEFINED
/**
 * @brief POSIX message header for vectored socket I/O.
 *
 * The Windows backend supports payload iovecs and the ancillary fields needed
 * for source compatibility. Unsupported control-message types are ignored by
 * the implementation.
 */
struct msghdr {
	void         *msg_name;
	socklen_t     msg_namelen;
	struct iovec *msg_iov;
	int           msg_iovlen;
	void         *msg_control;
	size_t        msg_controllen;
	int           msg_flags;
};

/** @brief Ancillary data header compatible with POSIX CMSG_* helpers.
 *
 * MinGW's <mswsock.h> already provides `struct cmsghdr` (via `_WSACMSGHDR`)
 * together with the CMSG_* helpers when targeting Vista+, which is always
 * our case (_WIN32_WINNT >= 0x0A00). Use the same sentinel as the macro
 * guards below to avoid a redefinition under MinGW while still providing
 * the struct on platforms that omit it. */
#ifndef CMSG_FIRSTHDR
struct cmsghdr {
	size_t cmsg_len;
	int    cmsg_level;
	int    cmsg_type;
};
#endif

#ifndef _PX4_UCRED_DEFINED
#define _PX4_UCRED_DEFINED
/** @brief Credential payload shape used by SCM_CREDENTIALS call sites. */
struct ucred {
	int          pid;
	unsigned int uid;
	unsigned int gid;
};
#endif
#endif

#ifndef CMSG_ALIGN
#define CMSG_ALIGN(len) (((len) + sizeof(size_t) - 1) & ~(sizeof(size_t) - 1))
#endif
#ifndef CMSG_SPACE
#define CMSG_SPACE(len) (CMSG_ALIGN(sizeof(struct cmsghdr)) + CMSG_ALIGN(len))
#endif
#ifndef CMSG_LEN
#define CMSG_LEN(len) (CMSG_ALIGN(sizeof(struct cmsghdr)) + (len))
#endif
#ifndef CMSG_DATA
#define CMSG_DATA(cmsg) ((unsigned char *)(cmsg) + CMSG_ALIGN(sizeof(struct cmsghdr)))
#endif
#ifndef CMSG_FIRSTHDR
#define CMSG_FIRSTHDR(msg) ((msg)->msg_controllen >= sizeof(struct cmsghdr) ? (struct cmsghdr *)(msg)->msg_control : (struct cmsghdr *)0)
#endif
#ifndef CMSG_NXTHDR
#define CMSG_NXTHDR(msg, cmsg) \
	(((uintptr_t)(CMSG_DATA(cmsg)) + CMSG_ALIGN((cmsg)->cmsg_len - CMSG_ALIGN(sizeof(struct cmsghdr))) + CMSG_ALIGN(sizeof(struct cmsghdr)) > \
	  (uintptr_t)(msg)->msg_control + (msg)->msg_controllen) ? \
	 (struct cmsghdr *)0 : \
	 (struct cmsghdr *)((unsigned char *)(cmsg) + CMSG_ALIGN((cmsg)->cmsg_len)))
#endif

#ifndef SCM_RIGHTS
#define SCM_RIGHTS 0x01
#endif
#ifndef SCM_CREDENTIALS
#define SCM_CREDENTIALS 0x02
#endif

/* POSIX send/recv flags. winsock covers MSG_OOB/PEEK/DONTROUTE; the rest
 * we map to 0 so callers still compile. */
#ifndef MSG_TRUNC
#  define MSG_TRUNC 0
#endif
#ifndef MSG_EOR
#  define MSG_EOR 0
#endif
#ifndef MSG_CTRUNC
#  define MSG_CTRUNC 0
#endif
#ifndef MSG_CONFIRM
#  define MSG_CONFIRM 0
#endif
#ifndef MSG_ERRQUEUE
#  define MSG_ERRQUEUE 0
#endif
#ifndef MSG_WAITALL
#  define MSG_WAITALL 0
#endif
#ifndef MSG_WAITFORONE
#  define MSG_WAITFORONE 0
#endif
#ifndef MSG_DONTWAIT
#  define MSG_DONTWAIT 0
#endif
#ifndef MSG_MORE
#  define MSG_MORE 0
#endif
#ifndef MSG_NOSIGNAL
#  define MSG_NOSIGNAL 0
#endif
#ifndef MSG_CMSG_CLOEXEC
#  define MSG_CMSG_CLOEXEC 0
#endif
#ifndef SOCK_CLOEXEC
#  define SOCK_CLOEXEC 0
#endif
#ifndef SOCK_NONBLOCK
#  define SOCK_NONBLOCK 0
#endif

/* Shutdown how values - winsock uses SD_RECEIVE/SD_SEND/SD_BOTH;
 * POSIX code expects SHUT_*. */
#ifndef SHUT_RD
#  define SHUT_RD   SD_RECEIVE
#endif
#ifndef SHUT_WR
#  define SHUT_WR   SD_SEND
#endif
#ifndef SHUT_RDWR
#  define SHUT_RDWR SD_BOTH
#endif

/**
 * @brief Send a POSIX msghdr over a Winsock socket.
 *
 * @return Number of bytes sent, or -1 with errno set from WSAGetLastError().
 */
ssize_t sendmsg(int socket, const struct msghdr *message, int flags);

/**
 * @brief Receive data into a POSIX msghdr from a Winsock socket.
 *
 * @return Number of bytes received, or -1 with errno set from WSAGetLastError().
 */
ssize_t recvmsg(int socket, struct msghdr *message, int flags);

/**
 * @brief Create a connected socket pair for PX4 daemon/shell IPC.
 *
 * Windows AF_UNIX support is not present on every target we care about, so the
 * implementation may fall back to a loopback TCP pair while preserving POSIX
 * socketpair() behavior for the caller.
 */
int socketpair(int domain, int type, int protocol, int socket_vector[2]);

#if defined(_MSC_VER) && !defined(PX4_WINDOWS_NO_SOCKET_MACROS)
/**
 * @name MSVC socket wrappers
 *
 * MSVC does not expose POSIX-like int socket descriptors. These wrappers keep
 * C source code using socket(), bind(), send(), etc. while centralizing
 * Winsock errno translation in the Windows backend. C++ has many methods named
 * send(), connect(), and shutdown(), so C++ call sites use the declarations
 * directly when they need an explicit wrapper.
 *
 * @{
 */
SOCKET WSAAPI px4_windows_socket(int af, int type, int protocol);
int    WSAAPI px4_windows_bind(SOCKET s, const struct sockaddr *name, int namelen);
int    WSAAPI px4_windows_listen(SOCKET s, int backlog);
SOCKET WSAAPI px4_windows_accept(SOCKET s, struct sockaddr *addr, int *addrlen);
int    WSAAPI px4_windows_connect(SOCKET s, const struct sockaddr *name, int namelen);
int    WSAAPI px4_windows_setsockopt(SOCKET s, int level, int optname, const char *optval, int optlen);
int    WSAAPI px4_windows_shutdown(SOCKET s, int how);
int    WSAAPI px4_windows_recv(SOCKET s, char *buf, int len, int flags);
int    WSAAPI px4_windows_send(SOCKET s, const char *buf, int len, int flags);
int    WSAAPI px4_windows_recvfrom(SOCKET s, char *buf, int len, int flags, struct sockaddr *from, int *fromlen);
int    WSAAPI px4_windows_sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr *to, int tolen);
char  *px4_windows_strerror(int e);
/** @} */

#ifndef __cplusplus
#define socket(...)    px4_windows_socket(__VA_ARGS__)
#define bind(...)      px4_windows_bind(__VA_ARGS__)
#define listen(...)    px4_windows_listen(__VA_ARGS__)
#define accept(...)    px4_windows_accept(__VA_ARGS__)
#define connect(...)   px4_windows_connect(__VA_ARGS__)
#define setsockopt(...) px4_windows_setsockopt(__VA_ARGS__)
#define shutdown(...)  px4_windows_shutdown(__VA_ARGS__)
#define recv(...)      px4_windows_recv(__VA_ARGS__)
#define send(...)      px4_windows_send(__VA_ARGS__)
#define recvfrom(...)  px4_windows_recvfrom(__VA_ARGS__)
#define sendto(...)    px4_windows_sendto(__VA_ARGS__)
#define strerror(...)  px4_windows_strerror(__VA_ARGS__)
#endif /* !__cplusplus */
#endif

#ifdef __cplusplus
}
#endif
