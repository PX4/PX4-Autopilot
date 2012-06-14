/****************************************************************************
 * include/nuttx/fs/nfs.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Some of the content of this file was leveraged from OpenBSD:
 *
 *   Copyright (c) 1989, 1993, 1995
 *   The Regents of the University of California.  All rights reserved.
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

#ifndef __INCLUDE_NUTTX_FS_NFS_H
#define __INCLUDE_NUTTX_FS_NFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NFS mount option flags */

#define NFSMNT_SOFT              (1 << 0)      /* Soft mount (hard is default) */
#define NFSMNT_WSIZE             (1 << 1)      /* Set write size */
#define NFSMNT_RSIZE             (1 << 2)      /* Set read size */
#define NFSMNT_TIMEO             (1 << 3)      /* Set initial timeout */
#define NFSMNT_RETRANS           (1 << 4)      /* Set number of request retries */
#define NFSMNT_READDIRSIZE       (1 << 5)      /* Set readdir size */

/* Default PMAP port number to provide */

#define NFS_PMAPPORT             111
 
/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nfs_args
{
  uint8_t  addrlen;               /* Length of address */
  uint8_t  sotype;                /* Socket type */
  uint8_t  flags;                 /* Flags, determines if following are valid: */
  uint8_t  timeo;                 /* Time value in deciseconds (with NFSMNT_TIMEO) */
  uint8_t  retrans;               /* Times to retry send (with NFSMNT_RETRANS) */
  uint16_t wsize;                 /* Write size in bytes (with NFSMNT_WSIZE) */
  uint16_t rsize;                 /* Read size in bytes (with NFSMNT_RSIZE) */
  uint16_t readdirsize;           /* readdir size in bytes (with NFSMNT_READDIRSIZE) */
  char    *path;                  /* Server's path of the directory being mount */
  struct   sockaddr_storage addr; /* File server address (requires 32-bit alignment) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
 
#endif /* _NFS_NFS_H */
