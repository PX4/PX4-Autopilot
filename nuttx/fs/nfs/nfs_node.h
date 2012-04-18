/****************************************************************************
 * fs/nfs/nfs_node.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1989, 1993
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

#ifndef __FS_NFS_NFS_NODE_H
#define __FS_NFS_NFS_NODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for n_commitflags */

#define NFS_COMMIT_PUSH_VALID   0x0001/* push range valid */
#define NFS_COMMIT_PUSHED_VALID 0x0002/* pushed range valid */

#define n_atim          n_un1.nf_atim
#define n_mtim          n_un2.nf_mtim
#define n_cookieverf    n_un1.nd_cookieverf
#define n_direofoffset  n_un2.nd_direof

/* Flags for n_flag */

#define NFLUSHWANT      0x0001/* Want wakeup from a flush in prog. */
#define NFLUSHINPROG    0x0002/* Avoid multiple calls to vinvalbuf() */
#define NMODIFIED       0x0004/* Might have a modified buffer in bio */
#define NWRITEERR       0x0008/* Flag write errors so close will know */
#define NACC            0x0100/* Special file accessed */
#define NUPD            0x0200/* Special file updated */
#define NCHG            0x0400/* Special file times changed */

#define NFS_INVALIDATE_ATTRCACHE(np)    ((np)->n_attrstamp = 0)

/* Convert between nfsnode pointers and vnode pointers */

#define VTONFS(vp)      ((struct nfsnode *)(vp)->f_priv)
#define NFSTOV(np)      ((np)->n_vnode)

#define n_atim          n_un1.nf_atim
#define n_mtim          n_un2.nf_mtim
#define n_sillyrename   n_un3.nf_silly
#define n_cookieverf    n_un1.nd_cookieverf
#define n4_cookieverf   n_un1.nd4_cookieverf
#define n_direofoffset  n_un2.nd_direof
#define n_cookies       n_un3.nd_cook

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Silly rename structure that hangs off the nfsnode until the name
 * can be removed by nfs_inactive()

struct sillyrename
{
  struct  ucred *s_cred;
  struct  vnode *s_dvp;
  long    s_namlen;
  char    s_name[24];
};
*/

/* The nfsnode is the nfs equivalent to ufs's inode. Any similarity
 * is purely coincidental.
 * There is a unique nfsnode allocated for each active file,
 * each current directory, each mounted-on file, text file, and the root.
 * An nfsnode is 'named' by its file handle. (nget/nfs_node.c)
 * If this structure exceeds 256 bytes (it is currently 256 using 4.4BSD-Lite
 * type definitions), file handles of > 32 bytes should probably be split out
 * into a separate malloc()'d data structure. (Reduce the size of nfsfh_t by
 * changing the definition in sys/mount.h of NFS_SMALLFH.)
 * NB: Hopefully the current order of the fields is such that everything will
 *     be well aligned and, therefore, tightly packed.
 */

struct nfsnode
{
  struct nfsnode    *n_next;        /* Retained in a singly linked list filehandle/node tree. */
  bool               n_open;        /* true: The file is (still) open */
  uint64_t           n_size;        /* Current size of file */
  struct nfs_fattr   n_fattr;       /* nfs file attribute cache */
  nfstype            nfsv3_type;    /* File type */
  time_t             n_attrstamp;   /* Attr. cache timestamp */
  struct timespec    n_mtime;       /* Prev modify time. */
  time_t             n_ctime;       /* Prev create time. */
  nfsfh_t            n_fhp;         /* NFS File Handle */
  struct inode      *n_inode;       /* associated inode */
  int                n_error;       /* Save write error value */
  union
  {
    struct timespec  nf_atim;       /* Special file times */
    nfsuint64        nd_cookieverf; /* Cookie verifier (dir only) */
  } n_un1;
  union
  {
    struct timespec  nf_mtim;
    off_t            nd_direof;     /* Directory EOF offset cache */
  } n_un2;
  short              n_fhsize;      /* size in bytes, of fh */
  short              n_flag;        /* Flag for locking.. */
  nfsfh_t            n_fh;          /* Small File Handle */
  time_t             n_accstamp;    /* Access cache timestamp */
  uid_t              n_accuid;      /* Last access requester */
  int                n_accmode;     /* Last mode requested */
  int                n_accerror;    /* Last returned error */

  off_t              n_pushedlo;    /* 1st blk in commited range */
  off_t              n_pushedhi;    /* Last block in range */
  off_t              n_pushlo;      /* 1st block in commit range */
  off_t              n_pushhi;      /* Last block in range */
//struct rwlock      n_commitlock;  /* Serialize commits */
  int                n_commitflags;
};

#endif /* __FS_NFS_NFS_NODE_H */
