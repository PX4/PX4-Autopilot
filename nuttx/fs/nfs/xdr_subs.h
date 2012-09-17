/****************************************************************************
 * fs/nfs/xdr_subs.h
 * Definitions for Sun RPC Version 2, from
 * "RPC: Remote Procedure Call Protocol Specification" RFC1057
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1989, 1993
 *    The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Rick Macklem at The University of Guelph.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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

#ifndef __FS_NFS_XDR_SUBS_H
#define __FS_NFS_XDR_SUBS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arpa/inet.h>
 
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Macros used for conversion to/from xdr representation by nfs...
 * These use the MACHINE DEPENDENT routines ntohl, htonl
 * As defined by "XDR: External Data Representation Standard" RFC1014
 *
 * To simplify the implementation, we use ntohl/htonl even on big-endian
 * machines, and count on them being `#define'd away.  Some of these
 * might be slightly more efficient as int64_t copies on a big-endian,
 * but we cannot count on their alignment anyway.
 */

#define fxdr_unsigned(t, v)  ((t)ntohl(v))
#define txdr_unsigned(v)     (htonl(v))

#define fxdr_nfsv2time(f, t) \
{ \
  (t)->tv_sec = ntohl(((struct nfsv2_time *)(f))->nfsv2_sec); \
  if (((struct nfsv2_time *)(f))->nfsv2_usec != 0xffffffff) \
    (t)->tv_nsec = 1000 * ntohl(((struct nfsv2_time *)(f))->nfsv2_usec); \
  else \
    (t)->tv_nsec = 0; \
}

#define txdr_nfsv2time(f, t) \
{ \
  ((struct nfsv2_time *)(t))->nfsv2_sec = htonl((f)->tv_sec); \
  if ((f)->tv_nsec != -1) \
    ((struct nfsv2_time *)(t))->nfsv2_usec = htonl((f)->tv_nsec / 1000); \
  else \
    ((struct nfsv2_time *)(t))->nfsv2_usec = 0xffffffff; \
}

#define fxdr_nfsv3time(f, t) \
{ \
  (t)->tv_sec = ntohl(((struct nfsv3_time *)(f))->nfsv3_sec); \
  (t)->tv_nsec = ntohl(((struct nfsv3_time *)(f))->nfsv3_nsec); \
}

#define fxdr_nfsv3time2(f, t) { \
  (t)->nfsv3_sec = ntohl(((struct nfsv3_time *)(f))->nfsv3_sec); \
  (t)->nfsv3_nsec = ntohl(((struct nfsv3_time *)(f))->nfsv3_nsec); \
}

#define txdr_nfsv3time(f, t) \
{ \
  ((struct nfsv3_time *)(t))->nfsv3_sec = htonl((f)->tv_sec); \
  ((struct nfsv3_time *)(t))->nfsv3_nsec = htonl((f)->tv_nsec); \
}

#define txdr_nfsv3time2(f, t) \
{ \
  ((struct nfsv3_time *)(t))->nfsv3_sec = htonl((f)->nfsv3_sec); \
  ((struct nfsv3_time *)(t))->nfsv3_nsec = htonl((f)->nfsv3_nsec); \
}

#define fxdr_hyper(f) \
  ((((uint64_t)ntohl(((uint32_t *)(f))[0])) << 32) |  \
   (uint64_t)(ntohl(((uint32_t *)(f))[1])))

#define txdr_hyper(f, t) \
{ \
  ((uint32_t *)(t))[0] = htonl((uint32_t)((f) >> 32));    \
  ((uint32_t *)(t))[1] = htonl((uint32_t)((f) & 0xffffffff));  \
}

/* Macros for dealing with byte data saved in uint32_t aligned memory */

#define uint32_aligndown(b) ((b) & ~3)
#define uint32_alignup(b)   (((b) + 3) & ~3)
#define uint32_increment(b) (((b) + 3) >> 2)

#endif /* __FS_NFS_XDR_SUBS_H */
