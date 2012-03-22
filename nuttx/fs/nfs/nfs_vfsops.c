/****************************************************************************
 * fs/nfs/nfs_vfsops.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/proc.h>
#include <sys/namei.h>
#include <sys/vnode.h>
#include <sys/kernel.h>
#include <sys/mount.h>
#include <sys/buf.h>
#include <sys/mbuf.h>
#include <sys/dirent.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

#include <sys/statfs>
#include <sys/queue.h>

#include <net/if.h>
#include <net/route.h>
#include <netinet/in.h>

#include <nfs/rpcv2.h>
#include <nfs/nfsproto.h>
#include <nfs/nfsnode.h>
#include <nfs/nfs.h>
#include <nfs/nfsmount.h>
#include <nfs/xdr_subs.h>
#include <nfs/nfsm_subs.h>
#include <nfs/nfsdiskless.h>
#include <nfs/nfs_var.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* nfs vfs operations. */

const struct vfsops nfs_vfsops =
{
  nfs_mount,
  nfs_start,
  nfs_unmount,
  nfs_root,
  nfs_quotactl,
  nfs_statfs,
  nfs_sync,
  nfs_vget,
  nfs_fhtovp,
  nfs_vptofh,
  nfs_vfs_init,
  nfs_sysctl,
  nfs_checkexp
};

/****************************************************************************
 * External Public Data  (this belong in a header file)
 ****************************************************************************/

extern struct nfsstats nfsstats;
extern int nfs_ticks;
extern uint32_t nfs_procids[NFS_NPROCS];

/****************************************************************************
 * Public Function Prototypes (this belong in a header file)
 ****************************************************************************/

int nfs_sysctl(int *, u_int, void *, size_t *, void *, size_t, struct proc *);
int nfs_checkexp(struct mount *, struct mbuf *, int *, struct ucred **);
struct mount *nfs_mount_diskless(struct nfs_dlmount *, char *, int);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Publikc Functions
 ****************************************************************************/

/* nfs statfs call */

int nfs_statfs(struct file *filep, struct statfs *sbp)
{
  struct nfs_statfs *sfp = NULL;
  struct nfsmount *nmp = VFSTONFS(filep->f_inode);
  int error = 0;
  struct nfsnode *np = VTONFS(filep->f_priv);
  uint64_t tquad;

  error = nfs_nget(mp, (nfsfh_t *) nmp->nm_fh, nmp->nm_fhsize, &np);
  if (error)
    return (error);

  if ((nmp->nm_flag & NFSMNT_GOTFSINFO) == 0)
    (void)nfs_fsinfo(nmp);
  nfsstats.rpccnt[NFSPROC_FSSTAT]++;

  error = nfs_request(nmp, NFSPROC_FSSTAT);
  if (error)
    goto nfsmout;

  sbp->f_iosize = min(nmp->nm_rsize, nmp->nm_wsize);
  if (info.nmi_v3)
    {
      sbp->f_bsize = NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_tbytes);
      sbp->f_blocks = tquad / (u_quad_t) NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_fbytes);
      sbp->f_bfree = tquad / (u_quad_t) NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_abytes);
      sbp->f_bavail = (quad_t) tquad / (quad_t) NFS_FABLKSIZE;

      tquad = fxdr_hyper(&sfp->sf_tfiles);
      sbp->f_files = tquad;
      tquad = fxdr_hyper(&sfp->sf_ffiles);
      sbp->f_ffree = tquad;
      sbp->f_favail = tquad;
      sbp->f_namemax = MAXNAMLEN;
    }
  else
    {
      sbp->f_bsize = fxdr_unsigned(int32_t, sfp->sf_bsize);
      sbp->f_blocks = fxdr_unsigned(int32_t, sfp->sf_blocks);
      sbp->f_bfree = fxdr_unsigned(int32_t, sfp->sf_bfree);
      sbp->f_bavail = fxdr_unsigned(int32_t, sfp->sf_bavail);
      sbp->f_files = 0;
      sbp->f_ffree = 0;
    }
  copy_statfs_info(sbp, mp);
nfsmout:
  return (error);
}

/* nfs version 3 fsinfo rpc call */

int nfs_fsinfo(struct nfsmount *nmp)
{
  struct nfsv3_fsinfo *fsp;
  struct nfsm_info info;
  uint32_t pref, max;
  int error = 0;

  nfsstats.rpccnt[NFSPROC_FSINFO]++;

  error = nfs_request(nmp, NFSPROC_FSINFO);
  if (error)
    {
      goto nfsmout;
    }

  pref = fxdr_unsigned(uint32_t, fsp->fs_wtpref);
  if (pref < nmp->nm_wsize)
    nmp->nm_wsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
  max = fxdr_unsigned(uint32_t, fsp->fs_wtmax);
  if (max < nmp->nm_wsize)
    {
      nmp->nm_wsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize == 0)
        nmp->nm_wsize = max;
    }
  pref = fxdr_unsigned(uint32_t, fsp->fs_rtpref);
  if (pref < nmp->nm_rsize)
    nmp->nm_rsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
  max = fxdr_unsigned(uint32_t, fsp->fs_rtmax);
  if (max < nmp->nm_rsize)
    {
      nmp->nm_rsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize == 0)
        nmp->nm_rsize = max;
    }
  pref = fxdr_unsigned(uint32_t, fsp->fs_dtpref);
  if (pref < nmp->nm_readdirsize)
    nmp->nm_readdirsize = (pref + NFS_DIRBLKSIZ - 1) & ~(NFS_DIRBLKSIZ - 1);
  if (max < nmp->nm_readdirsize)
    {
      nmp->nm_readdirsize = max & ~(NFS_DIRBLKSIZ - 1);
      if (nmp->nm_readdirsize == 0)
        nmp->nm_readdirsize = max;
    }
  nmp->nm_flag |= NFSMNT_GOTFSINFO;

nfsmout:
  return (error);
}

/* Mount a remote root fs via. NFS.  It goes like this:
 * - Call nfs_boot_init() to fill in the nfs_diskless struct
 *   (using RARP, bootparam RPC, mountd RPC)
 * - hand craft the swap nfs vnode hanging off a fake mount point
 *      if swdevt[0].sw_dev == NODEV
 * - build the rootfs mount point and call mountnfs() to do the rest.
 */

int nfs_mountroot(void)
{
  struct nfs_diskless nd;
  struct vattr attr;
  struct mount *mp;
  struct vnode *vp;
  struct proc *procp;
  long n;
  int error;

  procp = curproc;              /* XXX */

  /* Call nfs_boot_init() to fill in the nfs_diskless struct.
   * Side effect:  Finds and configures a network interface.
   */

  bzero((caddr_t) & nd, sizeof(nd));
  nfs_boot_init(&nd, procp);

  /* Create the root mount point. */

  if (nfs_boot_getfh(&nd.nd_boot, "root", &nd.nd_root, -1))
    panic("nfs_mountroot: root");
  mp = nfs_mount_diskless(&nd.nd_root, "/", 0);
  nfs_root(mp, &rootvp);
  printf("root on %s\n", nd.nd_root.ndm_host);

  /* Link it into the mount list. */

  CIRCLEQ_INSERT_TAIL(&mountlist, mp, mnt_list);
  vfs_unbusy(mp);

  /* Get root attributes (for the time). */

  error = VOP_GETATTR(rootvp, &attr, procp->p_ucred, procp);
  if (error)
    panic("nfs_mountroot: getattr for root");
  n = attr.va_atime.tv_sec;
#ifdef  DEBUG
  printf("root time: 0x%lx\n", n);
#endif
  inittodr(n);

#ifdef notyet
  /* Set up swap credentials. */

  proc0.p_ucred->cr_uid = ntohl(nd.swap_ucred.cr_uid);
  proc0.p_ucred->cr_gid = ntohl(nd.swap_ucred.cr_gid);
  if ((proc0.p_ucred->cr_ngroups = ntohs(nd.swap_ucred.cr_ngroups)) > NGROUPS)
    proc0.p_ucred->cr_ngroups = NGROUPS;
  for (i = 0; i < proc0.p_ucred->cr_ngroups; i++)
    proc0.p_ucred->cr_groups[i] = ntohl(nd.swap_ucred.cr_groups[i]);
#endif

  /* "Mount" the swap device.
   *
   * On a "dataless" configuration (swap on disk) we will have:
   *      (swdevt[0].sw_dev != NODEV) identifying the swap device.
   */

  if (bdevvp(swapdev, &swapdev_vp))
    panic("nfs_mountroot: can't setup swap vp");
  if (swdevt[0].sw_dev != NODEV)
    {
      printf("swap on device 0x%x\n", swdevt[0].sw_dev);
      return (0);
    }

  /* If swapping to an nfs node:  (swdevt[0].sw_dev == NODEV)
   * Create a fake mount point just for the swap vnode so that the
   * swap file can be on a different server from the rootfs.
   *
   * Wait 5 retries, finally no swap is cool. -mickey
   */

  error = nfs_boot_getfh(&nd.nd_boot, "swap", &nd.nd_swap, 5);
  if (!error)
    {
      mp = nfs_mount_diskless(&nd.nd_swap, "/swap", 0);
      nfs_root(mp, &vp);
      vfs_unbusy(mp);

      /* Since the swap file is not the root dir of a file system,
       * hack it to a regular file.
       */

      vp->v_type = VREG;
      vp->v_flag = 0;

      /* Next line is a hack to make swapmount() work on NFS
       * swap files.
       * XXX-smurph
       */

      swdevt[0].sw_dev = NETDEV;
      /* end hack */
      swdevt[0].sw_vp = vp;

      /* Find out how large the swap file is. */

      error = VOP_GETATTR(vp, &attr, procp->p_ucred, procp);
      if (error)
        printf("nfs_mountroot: getattr for swap\n");
      n = (long)(attr.va_size >> DEV_BSHIFT);

      printf("swap on %s\n", nd.nd_swap.ndm_host);
#ifdef  DEBUG
      printf("swap size: 0x%lx (blocks)\n", n);
#endif
      return (0);
    }

  printf("WARNING: no swap\n");
  swdevt[0].sw_dev = NODEV;
  swdevt[0].sw_vp = NULL;

  return (0);
}

/* Internal version of mount system call for diskless setup. */

struct mount *nfs_mount_diskless(struct nfs_dlmount *ndmntp, char *mntname,
                                 int mntflag)
{
  struct nfs_args args;
  struct mount *mp;
  struct mbuf *m;
  int error;

  if (vfs_rootmountalloc("nfs", mntname, &mp))
    panic("nfs_mount_diskless: vfs_rootmountalloc failed");
  mp->mnt_flag |= mntflag;

  /* Initialize mount args. */

  bzero((caddr_t) & args, sizeof(args));
  args.addr = (struct sockaddr *)&ndmntp->ndm_saddr;
  args.addrlen = args.addr->sa_len;
  args.sotype = SOCK_DGRAM;
  args.fh = ndmntp->ndm_fh;
  args.fhsize = NFSX_V2FH;
  args.hostname = ndmntp->ndm_host;

#ifdef  NFS_BOOT_OPTIONS
  args.flags |= NFS_BOOT_OPTIONS;
#endif
#ifdef  NFS_BOOT_RWSIZE
  /* Reduce rsize,wsize for interfaces that consistently
   * drop fragments of long UDP messages.  (i.e. wd8003).
   * You can always change these later via remount.
   */

  args.flags |= NFSMNT_WSIZE | NFSMNT_RSIZE;
  args.wsize = NFS_BOOT_RWSIZE;
  args.rsize = NFS_BOOT_RWSIZE;
#endif

  /* Get mbuf for server sockaddr. */

  m = m_get(M_WAIT, MT_SONAME);
  bcopy((caddr_t) args.addr, mtod(m, caddr_t), (m->m_len = args.addr->sa_len));

  error = mountnfs(&args, mp, m, mntname, args.hostname);
  if (error)
    panic("nfs_mountroot: mount %s failed: %d", mntname, error);

  return (mp);
}

void
nfs_decode_args(struct nfsmount *nmp, struct nfs_args *argp,
                struct nfs_args *nargp)
{
  int s;
  int adjsock = 0;
  int maxio;

  s = splsoftnet();

#if 0
  /* Re-bind if rsrvd port requested and wasn't on one */

  adjsock = !(nmp->nm_flag & NFSMNT_RESVPORT)
    && (argp->flags & NFSMNT_RESVPORT);
#endif
  /* Also re-bind if we're switching to/from a connected UDP socket */

  adjsock |= ((nmp->nm_flag & NFSMNT_NOCONN) != (argp->flags & NFSMNT_NOCONN));

  /* Update flags atomically.  Don't change the lock bits. */

  nmp->nm_flag =
    (argp->flags & ~NFSMNT_INTERNAL) | (nmp->nm_flag & NFSMNT_INTERNAL);
  splx(s);

  if ((argp->flags & NFSMNT_TIMEO) && argp->timeo > 0)
    {
      nmp->nm_timeo = (argp->timeo * NFS_HZ + 5) / 10;
      if (nmp->nm_timeo < NFS_MINTIMEO)
        nmp->nm_timeo = NFS_MINTIMEO;
      else if (nmp->nm_timeo > NFS_MAXTIMEO)
        nmp->nm_timeo = NFS_MAXTIMEO;
    }

  if ((argp->flags & NFSMNT_RETRANS) && argp->retrans > 1)
    nmp->nm_retry = MIN(argp->retrans, NFS_MAXREXMIT);
  if (!(nmp->nm_flag & NFSMNT_SOFT))
    nmp->nm_retry = NFS_MAXREXMIT + 1;  /* past clip limit */

  if (argp->flags & NFSMNT_NFSV3)
    {
      if (argp->sotype == SOCK_DGRAM)
        maxio = NFS_MAXDGRAMDATA;
      else
        maxio = NFS_MAXDATA;
    }
  else
    maxio = NFS_V2MAXDATA;

  if ((argp->flags & NFSMNT_WSIZE) && argp->wsize > 0)
    {
      int osize = nmp->nm_wsize;
      nmp->nm_wsize = argp->wsize;

      /* Round down to multiple of blocksize */

      nmp->nm_wsize &= ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize <= 0)
        nmp->nm_wsize = NFS_FABLKSIZE;
      adjsock |= (nmp->nm_wsize != osize);
    }
  if (nmp->nm_wsize > maxio)
    nmp->nm_wsize = maxio;
  if (nmp->nm_wsize > MAXBSIZE)
    nmp->nm_wsize = MAXBSIZE;

  if ((argp->flags & NFSMNT_RSIZE) && argp->rsize > 0)
    {
      int osize = nmp->nm_rsize;
      nmp->nm_rsize = argp->rsize;

      /* Round down to multiple of blocksize */

      nmp->nm_rsize &= ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize <= 0)
        nmp->nm_rsize = NFS_FABLKSIZE;
      adjsock |= (nmp->nm_rsize != osize);
    }
  if (nmp->nm_rsize > maxio)
    nmp->nm_rsize = maxio;
  if (nmp->nm_rsize > MAXBSIZE)
    nmp->nm_rsize = MAXBSIZE;

  if ((argp->flags & NFSMNT_READDIRSIZE) && argp->readdirsize > 0)
    {
      nmp->nm_readdirsize = argp->readdirsize;

      /* Round down to multiple of blocksize */

      nmp->nm_readdirsize &= ~(NFS_DIRBLKSIZ - 1);
      if (nmp->nm_readdirsize < NFS_DIRBLKSIZ)
        nmp->nm_readdirsize = NFS_DIRBLKSIZ;
    }
  else if (argp->flags & NFSMNT_RSIZE)
    nmp->nm_readdirsize = nmp->nm_rsize;

  if (nmp->nm_readdirsize > maxio)
    nmp->nm_readdirsize = maxio;

  if ((argp->flags & NFSMNT_MAXGRPS) && argp->maxgrouplist >= 0 &&
      argp->maxgrouplist <= NFS_MAXGRPS)
    nmp->nm_numgrps = argp->maxgrouplist;
  if ((argp->flags & NFSMNT_READAHEAD) && argp->readahead >= 0 &&
      argp->readahead <= NFS_MAXRAHEAD)
    nmp->nm_readahead = argp->readahead;
  if (argp->flags & NFSMNT_ACREGMIN && argp->acregmin >= 0)
    {
      if (argp->acregmin > 0xffff)
        nmp->nm_acregmin = 0xffff;
      else
        nmp->nm_acregmin = argp->acregmin;
    }
  if (argp->flags & NFSMNT_ACREGMAX && argp->acregmax >= 0)
    {
      if (argp->acregmax > 0xffff)
        nmp->nm_acregmax = 0xffff;
      else
        nmp->nm_acregmax = argp->acregmax;
    }
  if (nmp->nm_acregmin > nmp->nm_acregmax)
    nmp->nm_acregmin = nmp->nm_acregmax;

  if (argp->flags & NFSMNT_ACDIRMIN && argp->acdirmin >= 0)
    {
      if (argp->acdirmin > 0xffff)
        nmp->nm_acdirmin = 0xffff;
      else
        nmp->nm_acdirmin = argp->acdirmin;
    }
  if (argp->flags & NFSMNT_ACDIRMAX && argp->acdirmax >= 0)
    {
      if (argp->acdirmax > 0xffff)
        nmp->nm_acdirmax = 0xffff;
      else
        nmp->nm_acdirmax = argp->acdirmax;
    }
  if (nmp->nm_acdirmin > nmp->nm_acdirmax)
    nmp->nm_acdirmin = nmp->nm_acdirmax;

  if (nmp->nm_so && adjsock)
    {
      nfs_disconnect(nmp);
      if (nmp->nm_sotype == SOCK_DGRAM)
        while (nfs_connect(nmp, NULL))
          {
            printf("nfs_args: retrying connect\n");
            (void)tsleep((caddr_t) & lbolt, PSOCK, "nfscon", 0);
          }
    }

  /* Update nargp based on nmp */

  nargp->wsize = nmp->nm_wsize;
  nargp->rsize = nmp->nm_rsize;
  nargp->readdirsize = nmp->nm_readdirsize;
  nargp->timeo = nmp->nm_timeo;
  nargp->retrans = nmp->nm_retry;
  nargp->maxgrouplist = nmp->nm_numgrps;
  nargp->readahead = nmp->nm_readahead;
  nargp->acregmin = nmp->nm_acregmin;
  nargp->acregmax = nmp->nm_acregmax;
  nargp->acdirmin = nmp->nm_acdirmin;
  nargp->acdirmax = nmp->nm_acdirmax;
}

/* VFS Operations.
 *
 * mount system call
 * It seems a bit dumb to copyinstr() the host and path here and then
 * bcopy() them in mountnfs(), but I wanted to detect errors before
 * doing the sockargs() call because sockargs() allocates an mbuf and
 * an error after that means that I have to release the mbuf.
 */

/* ARGSUSED */
int
nfs_mount(struct mount *mp, const char *path, void *data,
          struct nameidata *ndp, struct proc *p)
{
  int error;
  struct nfs_args args;
  struct mbuf *nam;
  char pth[MNAMELEN], hst[MNAMELEN];
  size_t len;
  u_char nfh[NFSX_V3FHMAX];

  error = copyin(data, &args, sizeof(args.version));
  if (error)
    return (error);
  if (args.version == 3)
    {
      error = copyin(data, (caddr_t) & args, sizeof(struct nfs_args3));
      args.flags &= ~(NFSMNT_INTERNAL | NFSMNT_NOAC);
    }
  else if (args.version == NFS_ARGSVERSION)
    {
      error = copyin(data, (caddr_t) & args, sizeof(struct nfs_args));
      args.flags &= ~NFSMNT_NOAC;       /* XXX - compatibility */
    }
  else
    return (EPROGMISMATCH);
  if (error)
    return (error);

  if ((args.flags & (NFSMNT_NFSV3 | NFSMNT_RDIRPLUS)) == NFSMNT_RDIRPLUS)
    return (EINVAL);

  if (nfs_niothreads < 0)
    {
      nfs_niothreads = 4;
      nfs_getset_niothreads(1);
    }

  if (mp->mnt_flag & MNT_UPDATE)
    {
      struct nfsmount *nmp = VFSTONFS(mp);

      if (nmp == NULL)
        return (EIO);

      /* When doing an update, we can't change from or to v3. */

      args.flags = (args.flags & ~(NFSMNT_NFSV3)) |
        (nmp->nm_flag & (NFSMNT_NFSV3));
      nfs_decode_args(nmp, &args, &mp->mnt_stat.mount_info.nfs_args);
      return (0);
    }
  if (args.fhsize < 0 || args.fhsize > NFSX_V3FHMAX)
    return (EINVAL);
  error = copyin((caddr_t) args.fh, (caddr_t) nfh, args.fhsize);
  if (error)
    return (error);
  error = copyinstr(path, pth, MNAMELEN - 1, &len);
  if (error)
    return (error);
  bzero(&pth[len], MNAMELEN - len);
  error = copyinstr(args.hostname, hst, MNAMELEN - 1, &len);
  if (error)
    return (error);
  bzero(&hst[len], MNAMELEN - len);

  /* sockargs() call must be after above copyin() calls */

  error = sockargs(&nam, args.addr, args.addrlen, MT_SONAME);
  if (error)
    return (error);
  args.fh = nfh;
  error = mountnfs(&args, mp, nam, pth, hst);
  return (error);
}

/* Common code for mount and mountroot */

int
mountnfs(struct nfs_args *argp, struct mount *mp, struct mbuf *nam, char *pth,
         char *hst)
{
  struct nfsmount *nmp;
  int error;

  if (mp->mnt_flag & MNT_UPDATE)
    {
      nmp = VFSTONFS(mp);

      /* update paths, file handles, etc, here XXX */

      m_freem(nam);
      return (0);
    }
  else
    {
      nmp = malloc(sizeof(struct nfsmount), M_NFSMNT, M_WAITOK | M_ZERO);
      mp->mnt_data = (qaddr_t) nmp;
    }

  vfs_getnewfsid(mp);
  nmp->nm_mountp = mp;
  nmp->nm_timeo = NFS_TIMEO;
  nmp->nm_retry = NFS_RETRANS;
  nmp->nm_wsize = NFS_WSIZE;
  nmp->nm_rsize = NFS_RSIZE;
  nmp->nm_readdirsize = NFS_READDIRSIZE;
  nmp->nm_numgrps = NFS_MAXGRPS;
  nmp->nm_readahead = NFS_DEFRAHEAD;
  nmp->nm_fhsize = argp->fhsize;
  nmp->nm_acregmin = NFS_MINATTRTIMO;
  nmp->nm_acregmax = NFS_MAXATTRTIMO;
  nmp->nm_acdirmin = NFS_MINATTRTIMO;
  nmp->nm_acdirmax = NFS_MAXATTRTIMO;
  bcopy((caddr_t) argp->fh, (caddr_t) nmp->nm_fh, argp->fhsize);
  strncpy(&mp->mnt_stat.f_fstypename[0], mp->mnt_vfc->vfc_name, MFSNAMELEN);
  bcopy(hst, mp->mnt_stat.f_mntfromname, MNAMELEN);
  bcopy(pth, mp->mnt_stat.f_mntonname, MNAMELEN);
  bcopy(argp, &mp->mnt_stat.mount_info.nfs_args, sizeof(*argp));
  nmp->nm_nam = nam;
  nfs_decode_args(nmp, argp, &mp->mnt_stat.mount_info.nfs_args);

  RB_INIT(&nmp->nm_ntree);
  TAILQ_INIT(&nmp->nm_reqsq);
  timeout_set(&nmp->nm_rtimeout, nfs_timer, nmp);

  /* Set up the sockets and per-host congestion */

  nmp->nm_sotype = argp->sotype;
  nmp->nm_soproto = argp->proto;

  /* For Connection based sockets (TCP,...) defer the connect until
   * the first request, in case the server is not responding.
   */

  if (nmp->nm_sotype == SOCK_DGRAM && (error = nfs_connect(nmp, NULL)))
    goto bad;

  /* This is silly, but it has to be set so that vinifod() works.
   * We do not want to do an nfs_statfs() here since we can get
   * stuck on a dead server and we are holding a lock on the mount
   * point.
   */

  mp->mnt_stat.f_iosize = NFS_MAXDGRAMDATA;

  return (0);
bad:
  nfs_disconnect(nmp);
  free((caddr_t) nmp, M_NFSMNT);
  m_freem(nam);
  return (error);
}

/* unmount system call */

int nfs_unmount(struct mount *mp, int mntflags, struct proc *p)
{
  struct nfsmount *nmp;
  int error, flags;

  nmp = VFSTONFS(mp);
  flags = 0;

  if (mntflags & MNT_FORCE)
    flags |= FORCECLOSE;

  error = vflush(mp, NULL, flags);
  if (error)
    return (error);

  nfs_disconnect(nmp);
  m_freem(nmp->nm_nam);
  timeout_del(&nmp->nm_rtimeout);
  free(nmp, M_NFSMNT);
  return (0);
}

/* Return root of a filesystem */

int nfs_root(struct mount *mp, struct vnode **vpp)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  int error;

  nmp = VFSTONFS(mp);
  error = nfs_nget(mp, (nfsfh_t *) nmp->nm_fh, nmp->nm_fhsize, &np);
  if (error)
    return (error);
  *vpp = NFSTOV(np);
  return (0);
}

/* Flush out the buffer cache */

int nfs_sync(struct mount *mp, int waitfor, struct ucred *cred, struct proc *p)
{
  struct vnode *vp;
  int error, allerror = 0;

  /* Don't traverse the vnode list if we want to skip all of them. */

  if (waitfor == MNT_LAZY)
    return (allerror);

  /* Force stale buffer cache information to be flushed. */

loop:
  LIST_FOREACH(vp, &mp->mnt_vnodelist, v_mntvnodes)
  {
    /* If the vnode that we are about to sync is no longer
     * associated with this mount point, start over.
     */

    if (vp->v_mount != mp)
      goto loop;
    if (VOP_ISLOCKED(vp) || LIST_FIRST(&vp->v_dirtyblkhd) == NULL)
      continue;
    if (vget(vp, LK_EXCLUSIVE, p))
      goto loop;
    error = VOP_FSYNC(vp, cred, waitfor, p);
    if (error)
      allerror = error;
    vput(vp);
  }

  return (allerror);
}

/* NFS flat namespace lookup.
 * Currently unsupported.
 */

/* ARGSUSED */
int nfs_vget(struct mount *mp, ino_t ino, struct vnode **vpp)
{
  return (EOPNOTSUPP);
}

/* Do that sysctl thang... */

int
nfs_sysctl(int *name, u_int namelen, void *oldp, size_t * oldlenp, void *newp,
           size_t newlen, struct proc *p)
{
  int rv;

  /* All names at this level are terminal. */

  if (namelen > 1)
    return ENOTDIR;             /* overloaded */

  switch (name[0])
    {
    case NFS_NFSSTATS:
      if (!oldp)
        {
          *oldlenp = sizeof nfsstats;
          return 0;
        }

      if (*oldlenp < sizeof nfsstats)
        {
          *oldlenp = sizeof nfsstats;
          return ENOMEM;
        }

      rv = copyout(&nfsstats, oldp, sizeof nfsstats);
      if (rv)
        return rv;

      if (newp && newlen != sizeof nfsstats)
        return EINVAL;

      if (newp)
        {
          return copyin(newp, &nfsstats, sizeof nfsstats);
        }
      return 0;

    case NFS_NIOTHREADS:
      nfs_getset_niothreads(0);

      rv = sysctl_int(oldp, oldlenp, newp, newlen, &nfs_niothreads);
      if (newp)
        nfs_getset_niothreads(1);

      return rv;

    default:
      return EOPNOTSUPP;
    }
}

/* At this point, this should never happen */

/* ARGSUSED */
int nfs_fhtovp(struct mount *mp, struct fid *fhp, struct vnode **vpp)
{
  return (EINVAL);
}

/* Vnode pointer to File handle, should never happen either */

/* ARGSUSED */
int nfs_vptofh(struct vnode *vp, struct fid *fhp)
{
  return (EINVAL);
}

/* Vfs start routine, a no-op. */

/* ARGSUSED */
int nfs_start(struct mount *mp, int flags, struct proc *p)
{
  return (0);
}

/* Do operations associated with quotas, not supported */

/* ARGSUSED */
int
nfs_quotactl(struct mount *mp, int cmd, uid_t uid, caddr_t arg, struct proc *p)
{
  return (EOPNOTSUPP);
}

/* check export permission, not supported */

/* ARGUSED */
int
nfs_checkexp(struct mount *mp, struct mbuf *nam, int *exflagsp,
             struct ucred **credanonp)
{
  return (EOPNOTSUPP);
}
