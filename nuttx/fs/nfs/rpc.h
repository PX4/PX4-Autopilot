/****************************************************************************
 * fs/nfs/rpc.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1982, 1986, 1988, 1993
 *      The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __FS_NFS_RPC_H
#define __FS_NFS_RPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RPC definitions for the portmapper. */

#define PMAPPORT            111
#define PMAPPROG            100000
#define PMAPVERS            2
#define PMAPPROC_NULL       0
#define PMAPPROC_SET        1
#define PMAPPROC_UNSET      2
#define PMAPPROC_GETPORT    3
#define PMAPPROC_DUMP       4
#define PMAPPROC_CALLIT     5

/* RPC definitions for bootparamd. */

#define BOOTPARAM_PROG      100026
#define BOOTPARAM_VERS      1
#define BOOTPARAM_WHOAMI    1
#define BOOTPARAM_GETFILE   2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int krpc_call(struct sockaddr_in *, unsigned int, unsigned int, unsigned int,
              int);
int krpc_portmap(struct sockaddr_in *, unsigned int, unsigned int, uint16_t *);

struct mbuf *xdr_string_encode(char *, int);
struct mbuf *xdr_string_decode(struct mbuf *, char *, int *);
struct mbuf *xdr_inaddr_encode(struct in_addr *);
struct mbuf *xdr_inaddr_decode(struct mbuf *, struct in_addr *);

#endif /* __FS_NFS_RPC_H */
