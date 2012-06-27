/****************************************************************************
 * fs/nfs/nfs.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from OpenBSD:
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

#ifndef __FS_NFS_NFS_H
#define __FS_NFS_NFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nfs_mount.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NFS_TICKINTVL      MSEC_PER_TICK  /* Smallest that we can get */
#define NFS_TICKS          1              /* Number of system ticks */
#define NFS_HZ             CLOCKS_PER_SEC /* Ticks/sec */
#define NFS_TIMEO          (1 * NFS_HZ)   /* Default timeout = 1 second */
#define NFS_MINTIMEO       (1 * NFS_HZ)   /* Min timeout to use */
#define NFS_MAXTIMEO       (60 * NFS_HZ)  /* Max timeout to backoff to */
#define NFS_TIMEOUTMUL     2              /* Timeout/Delay multiplier */
#define NFS_MAXREXMIT      100            /* Stop counting after this many */
#define NFS_RETRANS        10             /* Num of retrans for soft mounts */
#define NFS_WSIZE          8192           /* Def. write data size <= 8192 */
#define NFS_RSIZE          8192           /* Def. read data size <= 8192 */
#define NFS_READDIRSIZE    8192           /* Def. readdir size */
#define NFS_NPROCS         23

/* Ideally, NFS_DIRBLKSIZ should be bigger, but I've seen servers with
 * broken NFS/ethernet drivers that won't work with anything bigger (Linux..)
 */

#define NFS_DIRBLKSIZ      1024           /* Must be a multiple of DIRBLKSIZ */

/* Increment NFS statistics */

#ifdef CONFIG_NFS_STATISTICS
#  define nfs_statistics(n) do { nfsstats.rpccnt[n]++; } while (0)
#else
#  define nfs_statistics(n)
#endif

/****************************************************************************
 *  Public Data
 ****************************************************************************/

extern uint32_t nfs_true;
extern uint32_t nfs_false;
extern uint32_t nfs_xdrneg1;
#ifdef CONFIG_NFS_STATISTICS
extern struct nfsstats nfsstats;
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NFS statistics structure */

struct nfsstats
{
  uint64_t rpccnt[NFS_NPROCS];
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

EXTERN void nfs_semtake(FAR struct nfsmount *nmp);
EXTERN void nfs_semgive(FAR struct nfsmount *nmp);
EXTERN int  nfs_checkmount(FAR struct nfsmount *nmp);
EXTERN int  nfs_fsinfo(FAR struct nfsmount *nmp);
EXTERN int nfs_request(struct nfsmount *nmp, int procnum,
                FAR void *request, size_t reqlen,
                FAR void *response, size_t resplen);
EXTERN int  nfs_lookup(FAR struct nfsmount *nmp, FAR const char *filename,
              FAR struct file_handle *fhandle,
              FAR struct nfs_fattr *obj_attributes,
              FAR struct nfs_fattr *dir_attributes);
EXTERN int  nfs_findnode(FAR struct nfsmount *nmp, FAR const char *relpath,
              FAR struct file_handle *fhandle,
              FAR struct nfs_fattr *obj_attributes,
              FAR struct nfs_fattr *dir_attributes);
EXTERN int nfs_finddir(FAR struct nfsmount *nmp, FAR const char *relpath,
              FAR struct file_handle *fhandle,
              FAR struct nfs_fattr *attributes, FAR char *filename);
EXTERN void nfs_attrupdate(FAR struct nfsnode *np,
              FAR struct nfs_fattr *attributes);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
 
#endif /* _NFS_NFS_H */
