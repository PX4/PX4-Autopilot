/*
 * Copyright (c) 1989, 1993
 *      The Regents of the University of California.  All rights reserved.
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
 * 4. Neither the name of the University nor the names of its contributors
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
 */

#ifndef _NFSCLIENT_NFSMOUNT_H_
#define _NFSCLIENT_NFSMOUNT_H_

/*
 * Mount structure.
 * One allocated on every NFS mount.
 * Holds NFS specific information for mount.
 */

struct nfsmount
{
  int nm_flag;                /* Flags for soft/hard... */
  int nm_state;               /* Internal state flags */
  struct mount *nm_mountp;    /* Vfs structure for this filesystem */
  int nm_numgrps;             /* Max. size of groupslist */
  nfsfh_t nm_fh;              /* File handle of root dir */
  int nm_fhsize;              /* Size of root file handle */
  struct rpcclnt nm_rpcclnt;  /* rpc state */
  struct socket *nm_so;       /* Rpc socket */
  int nm_sotype;              /* Type of socket */
  int nm_soproto;             /* and protocol */
  int nm_soflags;             /* pr_flags for socket protocol */
  struct sockaddr *nm_nam;    /* Addr of server */
  int nm_timeo;               /* Init timer for NFSMNT_DUMBTIMR */
  int nm_retry;               /* Max retries */
  int nm_srtt[4];             /* Timers for rpcs */
  int nm_sdrtt[4];
  int nm_sent;                /* Request send count */
  int nm_cwnd;                /* Request send window */
  int nm_timeouts;            /* Request timeouts */
  int nm_deadthresh;          /* Threshold of timeouts-->dead server */
  int nm_rsize;               /* Max size of read rpc */
  int nm_wsize;               /* Max size of write rpc */
  int nm_readdirsize;         /* Size of a readdir rpc */
  int nm_readahead;           /* Num. of blocks to readahead */
  int nm_acdirmin;            /* Directory attr cache min lifetime */
  int nm_acdirmax;            /* Directory attr cache max lifetime */
  int nm_acregmin;            /* Reg file attr cache min lifetime */
  int nm_acregmax;            /* Reg file attr cache max lifetime */
  u_char nm_verf[NFSX_V3WRITEVERF];   /* V3 write verifier */
  TAILQ_HEAD(, buf) nm_bufq; /* async io buffer queue */
  short nm_bufqlen;           /* number of buffers in queue */
  short nm_bufqwant;          /* process wants to add to the queue */
  int nm_bufqiods;            /* number of iods processing queue */
  u_int64_t nm_maxfilesize;   /* maximum file size */

  struct nfsx_nfsops *nm_nfsops;      /* Version specific ops. */

  /* NFSv4 */

  uint64_t nm_clientid;
  fsid_t nm_fsid;
  u_int nm_lease_time;
  time_t nm_last_renewal;

  struct vnode *nm_dvp;
};

#define NFSOP(nmp, op) (*nmp->nm_nfsops->nn_##op)
#define NFSHASOP(nmp, op) (nmp->nm_nfsops->nn_##op != NULL)
#define NFSDAT(nmp, nam) (nmp->nm_nfsops->nn_##nam)

#if defined(_KERNEL)

/*
 * Convert mount ptr to nfsmount ptr.
 */

#  define VFSTONFS(mp)    ((struct nfsmount *)((mp)->mnt_data))

#endif

#endif
