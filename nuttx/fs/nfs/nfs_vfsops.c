
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
#include <queue.h>

#include <net/if.h>
#include <netinet/in.h>

#include <nfs/rpcv2.h>
#include <nfs/nfsproto.h>
#include <nfs/nfsnode.h>
#include <nfs/nfs.h>
#include <nfs/nfsmount.h>
#include <nfs/xdr_subs.h>
#include <nfs/nfs_var.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NFS_DIRHDSIZ         (sizeof (struct nfs_dirent) - (MAXNAMLEN + 1))
#define NFS_DIRENT_OVERHEAD  offsetof(struct nfs_dirent, dirent)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct nfs_dirent
{
  uint32_t cookie[2];
  struct dirent dirent;
};

/****************************************************************************
 *  External Public Data  (this belong in a header file)
 ****************************************************************************/

extern uint32_t nfs_true, nfs_false;
extern uint32_t nfs_xdrneg1;
extern nfstype nfsv3_type[8];
extern struct nfsstats nfsstats;
extern int nfs_ticks;
extern uint32_t nfs_procids[NFS_NPROCS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

int nfs_numasync = 0;

/* nfs vfs operations. */

const struct mountpt_operations nfs_ops = {
  nfs_open,                     /* open */
  nfs_close,                    /* close */
  nfs_read,                     /* read */
  nfs_write,                    /* write */
  NULL,                         /* seek */
  nfs_ioctl,                    /* ioctl */
  nfs_sync,                     /* sync */

  NULL,                         /* opendir */
  NULL,                         /* closedir */
  nfs_readdir,                  /* readdir */
  NULL,                         /* rewinddir */

  nfs_mount,                    /* bind */
  nfs_unmount,                  /* unbind */
  nfs_statfs,                   /* statfs */

  nfs_remove,                   /* unlink */
  nfs_mkdir,                    /* mkdir */
  nfs_rmdir,                    /* rmdir */
  nfs_rename,                   /* rename */
  NULL                          /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* nfs open struct file
 * Check to see if the type is ok
 * and that deletion is not in progress.
 * For paged in text files, you will need to flush the page cache
 * if consistency is lost.
 */

int
nfs_open(FAR struct file *filep, FAR const char *relpath,
         int oflags, mode_t mode)
{
  struct inode *in = filep->f_inode;
  struct nfsmount *nmp = VFSTONFS(in);
  struct nfsnode *np = VTONFS(filep);
  int error;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv == NULL && filep->f_inode != NULL);

  if (np->nfsv3_type != NFREG && np->nfsv3_type != NFDIR)
    {
      dbg("open eacces typ=%d\n", np->nfsv3_type);
      return (EACCES);
    }

  NFS_INVALIDATE_ATTRCACHE(np);
  if (np->nfsv3_type == NFDIR)
    np->n_direofoffset = 0;
  np->n_mtime = vattr.va_mtime

  /* For open/close consistency. */

  NFS_INVALIDATE_ATTRCACHE(np);
  return (0);
}

#if 0
int
nfs_create(FAR struct file *filp, FAR const char *relpath,
           int oflags, mode_t mode)
{
  // struct vop_create_args *ap = v;
  struct inode *in = filp->f_inode;
  // struct vattr *vap = ap->a_vap;
  // struct componentname *cnp = ap->a_cnp;
  struct nfsv3_sattr *sp;
  // struct nfsm_info info;
  uint32_t *tl;
  int32_t t1;
  struct nfsnode *np = NULL;
  struct inode *newvp = NULL;
  caddr_t cp2;
  int error = 0, wccflag = NFSV3_WCCRATTR, gotvp = 0, fmode = 0;

  /* Oops, not for me.. */

  if (vap->va_type == VSOCK)
    return (nfs_mknodrpc(dvp, ap->a_vpp, cnp, vap));

  if (vap->va_vaflags & VA_EXCLUSIVE)
    fmode |= O_EXCL;

again:
  nfsstats.rpccnt[NFSPROC_CREATE]++;

  sp->sa_mode = vtonfsv2_mode(vap->va_type, vap->va_mode);
  sp->sa_uid = nfs_xdrneg1;
  sp->sa_gid = nfs_xdrneg1;
  sp->sa_size = 0;
  txdr_nfsv3time(&vap->va_atime, &sp->sa_atime);
  txdr_nfsv3time(&vap->va_mtime, &sp->sa_mtime);

  error = nfs_request(in, NFSPROC_CREATE);
  if (!error)
    {
      nfsm_mtofh(dvp, newvp, info.nmi_v3, gotvp);
      if (!gotvp)
        {
          if (newvp)
            {
              vrele(newvp);
              newvp = NULL;
            }
          error = nfs_lookitup(dvp, cnp->cn_nameptr,
                               cnp->cn_namelen, cnp->cn_cred, cnp->cn_proc,
                               &np);
          if (!error)
            newvp = NFSTOV(np);
        }
    }
  if (info_v3)
    nfsm_wcc_data(dvp, wccflag);

nfsmout:
  if (error)
    {
      if (info.nmi_v3 && (fmode & O_EXCL) && error == NFSERR_NOTSUPP)
        {
          fmode &= ~O_EXCL;
          goto again;
        }
      if (newvp)
        vrele(newvp);
    }
  else if (info.nmi_v3 && (fmode & O_EXCL))
    error = nfs_setattrrpc(newvp, vap, cnp->cn_cred, cnp->cn_proc);
  if (!error)
    {
      if (cnp->cn_flags & MAKEENTRY)
        nfs_cache_enter(dvp, newvp, cnp);
      *ap->a_vpp = newvp;
    }
  pool_put(&namei_pool, cnp->cn_pnbuf);
  VTONFS(dvp)->n_flag |= NMODIFIED;
  if (!wccflag)
    NFS_INVALIDATE_ATTRCACHE(VTONFS(dvp));
  VN_KNOTE(ap->a_dvp, NOTE_WRITE);
  vrele(dvp);
  return (error);
}
#endif

/* nfs close vnode op
 * What an NFS client should do upon close after writing is a debatable issue.
 * Most NFS clients push delayed writes to the server upon close, basically for
 * two reasons:
 * 1 - So that any write errors may be reported back to the client process
 *     doing the close system call. By far the two most likely errors are
 *     NFSERR_NOSPC and NFSERR_DQUOT to indicate space allocation failure.
 * 2 - To put a worst case upper bound on cache inconsistency between
 *     multiple clients for the file.
 * There is also a consistency problem for Version 2 of the protocol w.r.t.
 * not being able to tell if other clients are writing a file concurrently,
 * since there is no way of knowing if the changed modify time in the reply
 * is only due to the write for this client.
 * (NFS Version 3 provides weak cache consistency data in the reply that
 *  should be sufficient to detect and handle this case.)
 *
 * The current code does the following:
 * for NFS Version 2 - play it safe and flush/invalidate all dirty buffers
 * for NFS Version 3 - flush dirty buffers to the server but don't invalidate
 *           or commit them (this satisfies 1 and 2 except for the
 *           case where the server crashes after this close but
 *           before the commit RPC, which is felt to be "good
 *           enough". Changing the last argument to nfs_flush() to
 *           a 1 would force a commit operation, if it is felt a
 *           commit is necessary now.
 */
 
int nfs_close(FAR struct file *filep)
{
  struct inode *in = filep->f_inode;
  struct nfsmount *nmp = VFSTONFS(in);
  struct nfsnode *np = VTONFS(filep);
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  DEBUGASSERT(nmp != NULL);

  if (np->nfsv3_type == NFREG)
    {
      error = nfs_sync(filep);
      kfree(np);
      filep->f_priv = NULL;
    }
  return (error);
}

/* nfs read call.
 * Just call nfs_bioread() to do the work.
 */
 
int nfs_read(void *v)
{
  struct vop_read_args *ap = v;
  struct vnode *vp = ap->a_vp;

  if (vp->v_type != VREG)
    return (EPERM);
  return (nfs_bioread(vp, ap->a_uio, ap->a_ioflag, ap->a_cred));
}

/* nfs write call */

int
nfs_writerpc(struct vnode *vp, struct uio *uiop, int *iomode, int *must_commit)
{
  struct nfsm_info info;
  u_int32_t *tl;
  int32_t t1, backup;
  caddr_t cp2;
  struct nfsmount *nmp = VFSTONFS(vp->v_mount);
  int error = 0, len, tsiz, wccflag = NFSV3_WCCRATTR, rlen, commit;
  int committed = NFSV3WRITE_FILESYNC;

  info.nmi_v3 = NFS_ISV3(vp);

#ifdef DIAGNOSTIC
  if (uiop->uio_iovcnt != 1)
    panic("nfs: writerpc iovcnt > 1");
#endif
  *must_commit = 0;
  tsiz = uiop->uio_resid;
  if (uiop->uio_offset + tsiz > 0xffffffff && !info.nmi_v3)
    return (EFBIG);
  while (tsiz > 0)
    {
      nfsstats.rpccnt[NFSPROC_WRITE]++;
      len = (tsiz > nmp->nm_wsize) ? nmp->nm_wsize : tsiz;
      info.nmi_mb = info.nmi_mreq = nfsm_reqhead(NFSX_FH(info.nmi_v3)
                                                 + 5 * NFSX_UNSIGNED +
                                                 nfsm_rndup(len));
      nfsm_fhtom(&info, vp, info.nmi_v3);
      if (info.nmi_v3)
        {
          tl = nfsm_build(&info.nmi_mb, 5 * NFSX_UNSIGNED);
          txdr_hyper(uiop->uio_offset, tl);
          tl += 2;
          *tl++ = txdr_unsigned(len);
          *tl++ = txdr_unsigned(*iomode);
          *tl = txdr_unsigned(len);
        }
      else
        {
          u_int32_t x;

          tl = nfsm_build(&info.nmi_mb, 4 * NFSX_UNSIGNED);

          /* Set both "begin" and "current" to non-garbage. */

          x = txdr_unsigned((u_int32_t) uiop->uio_offset);
          *tl++ = x;            /* "begin offset" */
          *tl++ = x;            /* "current offset" */
          x = txdr_unsigned(len);
          *tl++ = x;            /* total to this offset */
          *tl = x;              /* size of this write */

        }
      nfsm_uiotombuf(&info.nmi_mb, uiop, len);

      info.nmi_procp = curproc;
      info.nmi_cred = VTONFS(vp)->n_wcred;
      error = nfs_request(vp, NFSPROC_WRITE, &info);
      if (info.nmi_v3)
        {
          wccflag = NFSV3_WCCCHK;
          nfsm_wcc_data(vp, wccflag);
        }

      if (error)
        {
          m_freem(info.nmi_mrep);
          goto nfsmout;
        }

      if (info.nmi_v3)
        {
          wccflag = NFSV3_WCCCHK;
          nfsm_dissect(tl, u_int32_t *, 2 * NFSX_UNSIGNED + NFSX_V3WRITEVERF);
          rlen = fxdr_unsigned(int, *tl++);
          if (rlen == 0)
            {
              error = NFSERR_IO;
              break;
            }
          else if (rlen < len)
            {
              backup = len - rlen;
              uiop->uio_iov->iov_base =
                (char *)uiop->uio_iov->iov_base - backup;
              uiop->uio_iov->iov_len += backup;
              uiop->uio_offset -= backup;
              uiop->uio_resid += backup;
              len = rlen;
            }
          commit = fxdr_unsigned(int, *tl++);

          /* Return the lowest committment level
           * obtained by any of the RPCs.
           */

          if (committed == NFSV3WRITE_FILESYNC)
            committed = commit;
          else if (committed == NFSV3WRITE_DATASYNC &&
                   commit == NFSV3WRITE_UNSTABLE)
            committed = commit;
          if ((nmp->nm_flag & NFSMNT_HASWRITEVERF) == 0)
            {
              bcopy((caddr_t) tl, (caddr_t) nmp->nm_verf, NFSX_V3WRITEVERF);
              nmp->nm_flag |= NFSMNT_HASWRITEVERF;
            }
          else if (bcmp((caddr_t) tl, (caddr_t) nmp->nm_verf, NFSX_V3WRITEVERF))
            {
              *must_commit = 1;
              bcopy((caddr_t) tl, (caddr_t) nmp->nm_verf, NFSX_V3WRITEVERF);
            }
        }
      else
        {
          nfsm_loadattr(vp, NULL);
        }
      if (wccflag)
        VTONFS(vp)->n_mtime = VTONFS(vp)->n_vattr.va_mtime;
      m_freem(info.nmi_mrep);
      tsiz -= len;
    }
nfsmout:
  *iomode = committed;
  if (error)
    uiop->uio_resid = tsiz;
  return (error);
}

/* nfs file remove call
 * To try and make nfs semantics closer to ufs semantics, a file that has
 * other processes using the vnode is renamed instead of removed and then
 * removed later on the last close.
 * - If v_usecount > 1
 *    If a rename is not already in the works
 *       call nfs_sillyrename() to set it up
 *     else
 *    do the remove rpc
 */

int nfs_remove(void *v)
{
  struct vop_remove_args *ap = v;
  struct vnode *vp = ap->a_vp;
  struct vnode *dvp = ap->a_dvp;
  struct componentname *cnp = ap->a_cnp;
  struct nfsnode *np = VTONFS(vp);
  int error = 0;
  struct vattr vattr;

#ifdef DIAGNOSTIC
  if ((cnp->cn_flags & HASBUF) == 0)
    panic("nfs_remove: no name");
  if (vp->v_usecount < 1)
    panic("nfs_remove: bad v_usecount");
#endif
  if (vp->v_type == VDIR)
    error = EPERM;
  else if (vp->v_usecount == 1 || (np->n_sillyrename &&
                                   VOP_GETATTR(vp, &vattr, cnp->cn_cred,
                                               cnp->cn_proc) == 0 &&
                                   vattr.va_nlink > 1))
    {
      /* Purge the name cache so that the chance of a lookup for
       * the name succeeding while the remove is in progress is
       * minimized. Without node locking it can still happen, such
       * that an I/O op returns ESTALE, but since you get this if
       * another host removes the file..
       */

      cache_purge(vp);

      /* throw away biocache buffers, mainly to avoid
       * unnecessary delayed writes later.
       */

      error = nfs_vinvalbuf(vp, 0, cnp->cn_cred, cnp->cn_proc);

      /* Do the rpc */

      if (error != EINTR)
        error = nfs_removerpc(dvp, cnp->cn_nameptr,
                              cnp->cn_namelen, cnp->cn_cred, cnp->cn_proc);

      /* Kludge City: If the first reply to the remove rpc is lost..
       *   the reply to the retransmitted request will be ENOENT
       *   since the file was in fact removed
       *   Therefore, we cheat and return success.
       */

      if (error == ENOENT)
        error = 0;
    }
  else if (!np->n_sillyrename)
    error = nfs_sillyrename(dvp, vp, cnp);
  pool_put(&namei_pool, cnp->cn_pnbuf);
  NFS_INVALIDATE_ATTRCACHE(np);
  vrele(dvp);
  vrele(vp);

  VN_KNOTE(vp, NOTE_DELETE);
  VN_KNOTE(dvp, NOTE_WRITE);

  return (error);
}

/* Nfs remove rpc, called from nfs_remove() and nfs_removeit(). */

int
nfs_removerpc(struct vnode *dvp, char *name, int namelen, struct ucred *cred,
              struct proc *proc)
{
  struct nfsm_info info;
  u_int32_t *tl;
  int32_t t1;
  caddr_t cp2;
  int error = 0, wccflag = NFSV3_WCCRATTR;

  info.nmi_v3 = NFS_ISV3(dvp);

  nfsstats.rpccnt[NFSPROC_REMOVE]++;
  info.nmi_mb = info.nmi_mreq = nfsm_reqhead(NFSX_FH(info.nmi_v3) +
                                             NFSX_UNSIGNED +
                                             nfsm_rndup(namelen));
  nfsm_fhtom(&info, dvp, info.nmi_v3);
  nfsm_strtom(name, namelen, NFS_MAXNAMLEN);

  info.nmi_procp = proc;
  info.nmi_cred = cred;
  error = nfs_request(dvp, NFSPROC_REMOVE, &info);
  if (info.nmi_v3)
    nfsm_wcc_data(dvp, wccflag);
  m_freem(info.nmi_mrep);

nfsmout:
  VTONFS(dvp)->n_flag |= NMODIFIED;
  if (!wccflag)
    NFS_INVALIDATE_ATTRCACHE(VTONFS(dvp));
  return (error);
}

/* nfs file rename call */

int nfs_rename(void *v)
{
  struct vop_rename_args *ap = v;
  struct vnode *fvp = ap->a_fvp;
  struct vnode *tvp = ap->a_tvp;
  struct vnode *fdvp = ap->a_fdvp;
  struct vnode *tdvp = ap->a_tdvp;
  struct componentname *tcnp = ap->a_tcnp;
  struct componentname *fcnp = ap->a_fcnp;
  int error;

#ifdef DIAGNOSTIC
  if ((tcnp->cn_flags & HASBUF) == 0 || (fcnp->cn_flags & HASBUF) == 0)
    panic("nfs_rename: no name");
#endif

  /* Check for cross-device rename */

  if ((fvp->v_mount != tdvp->v_mount) ||
      (tvp && (fvp->v_mount != tvp->v_mount)))
    {
      error = EXDEV;
      goto out;
    }

  /* If the tvp exists and is in use, sillyrename it before doing the
   * rename of the new file over it.
   */

  if (tvp && tvp->v_usecount > 1 && !VTONFS(tvp)->n_sillyrename &&
      tvp->v_type != VDIR && !nfs_sillyrename(tdvp, tvp, tcnp))
    {
      VN_KNOTE(tvp, NOTE_DELETE);
      vrele(tvp);
      tvp = NULL;
    }

  error = nfs_renamerpc(fdvp, fcnp->cn_nameptr, fcnp->cn_namelen,
                        tdvp, tcnp->cn_nameptr, tcnp->cn_namelen, tcnp->cn_cred,
                        tcnp->cn_proc);

  VN_KNOTE(fdvp, NOTE_WRITE);
  VN_KNOTE(tdvp, NOTE_WRITE);

  if (fvp->v_type == VDIR)
    {
      if (tvp != NULL && tvp->v_type == VDIR)
        cache_purge(tdvp);
      cache_purge(fdvp);
    }
out:
  if (tdvp == tvp)
    vrele(tdvp);
  else
    vput(tdvp);
  if (tvp)
    vput(tvp);
  vrele(fdvp);
  vrele(fvp);

  /* Kludge: Map ENOENT => 0 assuming that it is a reply to a retry. */

  if (error == ENOENT)
    error = 0;
  return (error);
}

/* nfs file rename rpc called from nfs_remove() above */

int
nfs_renameit(struct vnode *sdvp, struct componentname *scnp,
             struct sillyrename *sp)
{
  return (nfs_renamerpc(sdvp, scnp->cn_nameptr, scnp->cn_namelen,
                        sdvp, sp->s_name, sp->s_namlen, scnp->cn_cred,
                        curproc));
}

/* Do an nfs rename rpc. Called from nfs_rename() and nfs_renameit(). */

int
nfs_renamerpc(struct vnode *fdvp, char *fnameptr, int fnamelen,
              struct vnode *tdvp, char *tnameptr, int tnamelen,
              struct ucred *cred, struct proc *proc)
{
  struct nfsm_info info;
  u_int32_t *tl;
  int32_t t1;
  caddr_t cp2;
  int error = 0, fwccflag = NFSV3_WCCRATTR, twccflag = NFSV3_WCCRATTR;

  info.nmi_v3 = NFS_ISV3(fdvp);

  nfsstats.rpccnt[NFSPROC_RENAME]++;
  info.nmi_mb = info.nmi_mreq = nfsm_reqhead((NFSX_FH(info.nmi_v3) +
                                              NFSX_UNSIGNED) * 2 +
                                             nfsm_rndup(fnamelen) +
                                             nfsm_rndup(tnamelen));
  nfsm_fhtom(&info, fdvp, info.nmi_v3);
  nfsm_strtom(fnameptr, fnamelen, NFS_MAXNAMLEN);
  nfsm_fhtom(&info, tdvp, info.nmi_v3);
  nfsm_strtom(tnameptr, tnamelen, NFS_MAXNAMLEN);

  info.nmi_procp = proc;
  info.nmi_cred = cred;
  error = nfs_request(fdvp, NFSPROC_RENAME, &info);
  if (info.nmi_v3)
    {
      nfsm_wcc_data(fdvp, fwccflag);
      nfsm_wcc_data(tdvp, twccflag);
    }
  m_freem(info.nmi_mrep);

nfsmout:
  VTONFS(fdvp)->n_flag |= NMODIFIED;
  VTONFS(tdvp)->n_flag |= NMODIFIED;
  if (!fwccflag)
    NFS_INVALIDATE_ATTRCACHE(VTONFS(fdvp));
  if (!twccflag)
    NFS_INVALIDATE_ATTRCACHE(VTONFS(tdvp));
  return (error);
}

/* nfs make dir call */

int nfs_mkdir(void *v)
{
  struct vop_mkdir_args *ap = v;
  struct vnode *dvp = ap->a_dvp;
  struct vattr *vap = ap->a_vap;
  struct componentname *cnp = ap->a_cnp;
  struct nfsv2_sattr *sp;
  struct nfsm_info info;
  u_int32_t *tl;
  int32_t t1;
  int len;
  struct nfsnode *np = NULL;
  struct vnode *newvp = NULL;
  caddr_t cp2;
  int error = 0, wccflag = NFSV3_WCCRATTR;
  int gotvp = 0;

  info.nmi_v3 = NFS_ISV3(dvp);

  len = cnp->cn_namelen;
  nfsstats.rpccnt[NFSPROC_MKDIR]++;
  info.nmi_mb = info.nmi_mreq = nfsm_reqhead(NFSX_FH(info.nmi_v3) +
                                             NFSX_UNSIGNED + nfsm_rndup(len) +
                                             NFSX_SATTR(info.nmi_v3));
  nfsm_fhtom(&info, dvp, info.nmi_v3);
  nfsm_strtom(cnp->cn_nameptr, len, NFS_MAXNAMLEN);

  if (info.nmi_v3)
    {
      nfsm_v3attrbuild(&info.nmi_mb, vap, 0);
    }
  else
    {
      sp = nfsm_build(&info.nmi_mb, NFSX_V2SATTR);
      sp->sa_mode = vtonfsv2_mode(VDIR, vap->va_mode);
      sp->sa_uid = nfs_xdrneg1;
      sp->sa_gid = nfs_xdrneg1;
      sp->sa_size = nfs_xdrneg1;
      txdr_nfsv2time(&vap->va_atime, &sp->sa_atime);
      txdr_nfsv2time(&vap->va_mtime, &sp->sa_mtime);
    }

  info.nmi_procp = cnp->cn_proc;
  info.nmi_cred = cnp->cn_cred;
  error = nfs_request(dvp, NFSPROC_MKDIR, &info);
  if (!error)
    nfsm_mtofh(dvp, newvp, info.nmi_v3, gotvp);
  if (info.nmi_v3)
    nfsm_wcc_data(dvp, wccflag);
  m_freem(info.nmi_mrep);

nfsmout:
  VTONFS(dvp)->n_flag |= NMODIFIED;
  if (!wccflag)
    NFS_INVALIDATE_ATTRCACHE(VTONFS(dvp));

  if (error == 0 && newvp == NULL)
    {
      error = nfs_lookitup(dvp, cnp->cn_nameptr, len, cnp->cn_cred,
                           cnp->cn_proc, &np);
      if (!error)
        {
          newvp = NFSTOV(np);
          if (newvp->v_type != VDIR)
            error = EEXIST;
        }
    }
  if (error)
    {
      if (newvp)
        vrele(newvp);
    }
  else
    {
      VN_KNOTE(dvp, NOTE_WRITE | NOTE_LINK);
      if (cnp->cn_flags & MAKEENTRY)
        nfs_cache_enter(dvp, newvp, cnp);
      *ap->a_vpp = newvp;
    }
  pool_put(&namei_pool, cnp->cn_pnbuf);
  vrele(dvp);
  return (error);
}

/* nfs remove directory call */

int nfs_rmdir(void *v)
{
  struct vop_rmdir_args *ap = v;
  struct vnode *vp = ap->a_vp;
  struct vnode *dvp = ap->a_dvp;
  struct componentname *cnp = ap->a_cnp;
  struct nfsm_info info;
  u_int32_t *tl;
  int32_t t1;
  caddr_t cp2;
  int error = 0, wccflag = NFSV3_WCCRATTR;

  info.nmi_v3 = NFS_ISV3(dvp);

  if (dvp == vp)
    {
      vrele(dvp);
      vrele(dvp);
      pool_put(&namei_pool, cnp->cn_pnbuf);
      return (EINVAL);
    }

  nfsstats.rpccnt[NFSPROC_RMDIR]++;
  info.nmi_mb = info.nmi_mreq = nfsm_reqhead(NFSX_FH(info.nmi_v3) +
                                             NFSX_UNSIGNED +
                                             nfsm_rndup(cnp->cn_namelen));
  nfsm_fhtom(&info, dvp, info.nmi_v3);
  nfsm_strtom(cnp->cn_nameptr, cnp->cn_namelen, NFS_MAXNAMLEN);

  info.nmi_procp = cnp->cn_proc;
  info.nmi_cred = cnp->cn_cred;
  error = nfs_request(dvp, NFSPROC_RMDIR, &info);
  if (info.nmi_v3)
    nfsm_wcc_data(dvp, wccflag);
  m_freem(info.nmi_mrep);

nfsmout:
  pool_put(&namei_pool, cnp->cn_pnbuf);
  VTONFS(dvp)->n_flag |= NMODIFIED;
  if (!wccflag)
    NFS_INVALIDATE_ATTRCACHE(VTONFS(dvp));

  VN_KNOTE(dvp, NOTE_WRITE | NOTE_LINK);
  VN_KNOTE(vp, NOTE_DELETE);

  cache_purge(vp);
  vrele(vp);
  vrele(dvp);

  /* Kludge: Map ENOENT => 0 assuming that you have a reply to a retry. */

  if (error == ENOENT)
    error = 0;
  return (error);
}

/* The readdir logic below has a big design bug. It stores the NFS cookie in 
 * the returned uio->uio_offset but does not store the verifier (it cannot).
 * Instead, the code stores the verifier in the nfsnode and applies that
 * verifies to all cookies, no matter what verifier was originally with
 * the cookie.
 *
 * From a practical standpoint, this is not a problem since almost all
 * NFS servers do not change the validity of cookies across deletes
 * and inserts.
 */

/* nfs readdir call */

int nfs_readdir(void *v)
{
  struct vop_readdir_args *ap = v;
  struct vnode *vp = ap->a_vp;
  struct nfsnode *np = VTONFS(vp);
  struct uio *uio = ap->a_uio;
  int tresid, error = 0;
  struct vattr vattr;
  u_long *cookies = NULL;
  int ncookies = 0, cnt;
  u_int64_t newoff = uio->uio_offset;
  struct nfsmount *nmp = VFSTONFS(vp->v_mount);
  struct uio readdir_uio;
  struct iovec readdir_iovec;
  struct proc *p = uio->uio_procp;
  int done = 0, eof = 0;
  struct ucred *cred = ap->a_cred;
  void *data;

  if (vp->v_type != VDIR)
    return (EPERM);

  /* First, check for hit on the EOF offset cache */

  if (np->n_direofoffset != 0 && uio->uio_offset == np->n_direofoffset)
    {
      if (VOP_GETATTR(vp, &vattr, ap->a_cred, uio->uio_procp) == 0 &&
          timespeccmp(&np->n_mtime, &vattr.va_mtime, ==))
        {
          nfsstats.direofcache_hits++;
          *ap->a_eofflag = 1;
          return (0);
        }
    }

  if ((nmp->nm_flag & (NFSMNT_NFSV3 | NFSMNT_GOTFSINFO)) == NFSMNT_NFSV3)
    (void)nfs_fsinfo(nmp);

  cnt = 5;

  data = malloc(NFS_DIRBLKSIZ, M_TEMP, M_WAITOK);
  do
    {
      struct nfs_dirent *ndp = data;

      readdir_iovec.iov_len = NFS_DIRBLKSIZ;
      readdir_iovec.iov_base = data;
      readdir_uio.uio_offset = newoff;
      readdir_uio.uio_iov = &readdir_iovec;
      readdir_uio.uio_iovcnt = 1;
      readdir_uio.uio_segflg = UIO_SYSSPACE;
      readdir_uio.uio_rw = UIO_READ;
      readdir_uio.uio_resid = NFS_DIRBLKSIZ;
      readdir_uio.uio_procp = curproc;

      error = nfs_readdirrpc(vp, &readdir_uio, cred, &eof);

      if (error == NFSERR_BAD_COOKIE)
        error = EINVAL;

      while (error == 0 &&
             (ap->a_cookies == NULL || ncookies != 0) &&
             ndp < (struct nfs_dirent *)readdir_iovec.iov_base)
        {
          struct dirent *dp = &ndp->dirent;
          int reclen = dp->d_reclen;

          dp->d_reclen -= NFS_DIRENT_OVERHEAD;

          if (uio->uio_resid < dp->d_reclen)
            {
              eof = 0;
              done = 1;
              break;
            }

          error = uiomove((caddr_t) dp, dp->d_reclen, uio);
          if (error)
            break;

          newoff = fxdr_hyper(&ndp->cookie[0]);

          if (ap->a_cookies != NULL)
            {
              *cookies = newoff;
              cookies++;
              ncookies--;
            }

          ndp = (struct nfs_dirent *)((u_int8_t *) ndp + reclen);
        }
    }
  while (!error && !done && !eof && cnt--);

  free(data, M_TEMP);
  data = NULL;

  if (ap->a_cookies)
    {
      if (error)
        {
          free(*ap->a_cookies, M_TEMP);
          *ap->a_cookies = NULL;
          *ap->a_ncookies = 0;
        }
      else
        {
          *ap->a_ncookies -= ncookies;
        }
    }

  if (!error)
    uio->uio_offset = newoff;

  if (!error && (eof || uio->uio_resid == tresid))
    {
      nfsstats.direofcache_misses++;
      *ap->a_eofflag = 1;
      return (0);
    }

  *ap->a_eofflag = 0;
  return (error);
}

/* The function below stuff the cookies in after the name */

/* Readdir rpc call. */

int
nfs_readdirrpc(struct vnode *vp, struct uio *uiop, struct ucred *cred,
               int *end_of_directory)
{
  int len, left;
  struct nfs_dirent *ndp = NULL;
  struct dirent *dp = NULL;
  struct nfsm_info info;
  u_int32_t *tl;
  caddr_t cp;
  int32_t t1;
  caddr_t cp2;
  nfsuint64 cookie;
  struct nfsmount *nmp = VFSTONFS(vp->v_mount);
  struct nfsnode *dnp = VTONFS(vp);
  u_quad_t fileno;
  int error = 0, tlen, more_dirs = 1, blksiz = 0, bigenough = 1;
  int attrflag;

  info.nmi_v3 = NFS_ISV3(vp);

#ifdef DIAGNOSTIC
  if (uiop->uio_iovcnt != 1 || (uiop->uio_resid & (NFS_DIRBLKSIZ - 1)))
    panic("nfs readdirrpc bad uio");
#endif

  txdr_hyper(uiop->uio_offset, &cookie.nfsuquad[0]);

  /* Loop around doing readdir rpc's of size nm_readdirsize
   * truncated to a multiple of NFS_READDIRBLKSIZ.
   * The stopping criteria is EOF or buffer full.
   */

  while (more_dirs && bigenough)
    {
      nfsstats.rpccnt[NFSPROC_READDIR]++;
      info.nmi_mb = info.nmi_mreq = nfsm_reqhead(NFSX_FH(info.nmi_v3)
                                                 + NFSX_READDIR(info.nmi_v3));
      nfsm_fhtom(&info, vp, info.nmi_v3);
      if (info.nmi_v3)
        {
          tl = nfsm_build(&info.nmi_mb, 5 * NFSX_UNSIGNED);
          *tl++ = cookie.nfsuquad[0];
          *tl++ = cookie.nfsuquad[1];
          if (cookie.nfsuquad[0] == 0 && cookie.nfsuquad[1] == 0)
            {
              *tl++ = 0;
              *tl++ = 0;
            }
          else
            {
              *tl++ = dnp->n_cookieverf.nfsuquad[0];
              *tl++ = dnp->n_cookieverf.nfsuquad[1];
            }
        }
      else
        {
          tl = nfsm_build(&info.nmi_mb, 2 * NFSX_UNSIGNED);
          *tl++ = cookie.nfsuquad[1];
        }
      *tl = txdr_unsigned(nmp->nm_readdirsize);

      info.nmi_procp = uiop->uio_procp;
      info.nmi_cred = cred;
      error = nfs_request(vp, NFSPROC_READDIR, &info);
      if (info.nmi_v3)
        nfsm_postop_attr(vp, attrflag);

      if (error)
        {
          m_freem(info.nmi_mrep);
          goto nfsmout;
        }

      if (info.nmi_v3)
        {
          nfsm_dissect(tl, u_int32_t *, 2 * NFSX_UNSIGNED);
          dnp->n_cookieverf.nfsuquad[0] = *tl++;
          dnp->n_cookieverf.nfsuquad[1] = *tl;
        }

      nfsm_dissect(tl, u_int32_t *, NFSX_UNSIGNED);
      more_dirs = fxdr_unsigned(int, *tl);

      /* loop thru the dir entries, doctoring them to 4bsd form */

      while (more_dirs && bigenough)
        {
          if (info.nmi_v3)
            {
              nfsm_dissect(tl, u_int32_t *, 3 * NFSX_UNSIGNED);
              fileno = fxdr_hyper(tl);
              len = fxdr_unsigned(int, *(tl + 2));
            }
          else
            {
              nfsm_dissect(tl, u_int32_t *, 2 * NFSX_UNSIGNED);
              fileno = fxdr_unsigned(u_quad_t, *tl++);
              len = fxdr_unsigned(int, *tl);
            }
          if (len <= 0 || len > NFS_MAXNAMLEN)
            {
              error = EBADRPC;
              m_freem(info.nmi_mrep);
              goto nfsmout;
            }
          tlen = nfsm_rndup(len + 1);
          left = NFS_READDIRBLKSIZ - blksiz;
          if ((tlen + NFS_DIRHDSIZ) > left)
            {
              dp->d_reclen += left;
              uiop->uio_iov->iov_base += left;
              uiop->uio_iov->iov_len -= left;
              uiop->uio_resid -= left;
              blksiz = 0;
            }
          if ((tlen + NFS_DIRHDSIZ) > uiop->uio_resid)
            bigenough = 0;
          if (bigenough)
            {
              ndp = (struct nfs_dirent *)uiop->uio_iov->iov_base;
              dp = &ndp->dirent;
              dp->d_fileno = (int)fileno;
              dp->d_namlen = len;
              dp->d_reclen = tlen + NFS_DIRHDSIZ;
              dp->d_type = DT_UNKNOWN;
              blksiz += dp->d_reclen;
              if (blksiz == NFS_READDIRBLKSIZ)
                blksiz = 0;
              uiop->uio_resid -= NFS_DIRHDSIZ;
              uiop->uio_iov->iov_base =
                (char *)uiop->uio_iov->iov_base + NFS_DIRHDSIZ;
              uiop->uio_iov->iov_len -= NFS_DIRHDSIZ;
              nfsm_mtouio(uiop, len);
              cp = uiop->uio_iov->iov_base;
              tlen -= len;
              *cp = '\0';       /* null terminate */
              uiop->uio_iov->iov_base += tlen;
              uiop->uio_iov->iov_len -= tlen;
              uiop->uio_resid -= tlen;
            }
          else
            nfsm_adv(nfsm_rndup(len));
          if (info.nmi_v3)
            {
              nfsm_dissect(tl, u_int32_t *, 3 * NFSX_UNSIGNED);
            }
          else
            {
              nfsm_dissect(tl, u_int32_t *, 2 * NFSX_UNSIGNED);
            }
          if (bigenough)
            {
              if (info.nmi_v3)
                {
                  ndp->cookie[0] = cookie.nfsuquad[0] = *tl++;
                }
              else
                ndp->cookie[0] = 0;

              ndp->cookie[1] = cookie.nfsuquad[1] = *tl++;
            }
          else if (info.nmi_v3)
            tl += 2;
          else
            tl++;
          more_dirs = fxdr_unsigned(int, *tl);
        }

      /* If at end of rpc data, get the eof boolean */

      if (!more_dirs)
        {
          nfsm_dissect(tl, u_int32_t *, NFSX_UNSIGNED);
          more_dirs = (fxdr_unsigned(int, *tl) == 0);
        }
      m_freem(info.nmi_mrep);
    }

  /* Fill last record, iff any, out to a multiple of NFS_READDIRBLKSIZ
   * by increasing d_reclen for the last record.
   */

  if (blksiz > 0)
    {
      left = NFS_READDIRBLKSIZ - blksiz;
      dp->d_reclen += left;
      uiop->uio_iov->iov_base = (char *)uiop->uio_iov->iov_base + left;
      uiop->uio_iov->iov_len -= left;
      uiop->uio_resid -= left;
    }

  /* We are now either at the end of the directory or have filled the
   * block.
   */

  if (bigenough)
    {
      dnp->n_direofoffset = fxdr_hyper(&cookie.nfsuquad[0]);
      if (end_of_directory)
        *end_of_directory = 1;
    }
  else
    {
      if (uiop->uio_resid > 0)
        printf("EEK! readdirrpc resid > 0\n");
    }

nfsmout:
  return (error);
}

/* nfs statfs call */

int nfs_statfs(struct inode *mp, struct statfs *sbp)
{
  struct nfs_statfs *sfp = NULL;
  struct nfsmount *nmp = VFSTONFS(mp);
  int error = 0;
  uint64_t tquad;
  void *datareply;
  int info_v3 = (nmp->nm_flag & NFSMNT_NFSV3);

  /* Fill in the statfs info */

  memset(sbp, 0, sizeof(struct statfs));
  sbp->f_type = NFS_SUPER_MAGIC;

  if (info_v3 && (nmp->nm_flag & NFSMNT_GOTFSINFO) == 0)
    (void)nfs_fsinfo(nmp);
  nfsstats.rpccnt[NFSPROC_FSSTAT]++;

  error = nfs_request(nmp, NFSPROC_FSSTAT, datareply);
  if (error)
    goto nfsmout;
  sfp = (struct nfs_statfs *)datareply;
  if (info_v3)
    {
      sbp->f_bsize = NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_tbytes);
      sbp->f_blocks = tquad / (uint64_t) NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_fbytes);
      sbp->f_bfree = tquad / (uint64_t) NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_abytes);
      sbp->f_bavail = (quad_t) tquad / (quad_t) NFS_FABLKSIZE;

      tquad = fxdr_hyper(&sfp->sf_tfiles);
      sbp->f_files = tquad;
      tquad = fxdr_hyper(&sfp->sf_ffiles);
      sbp->f_ffree = tquad;
      sbp->f_namelen = MAXNAMLEN;
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
nfsmout:
  return (error);
}

/* Print out the contents of an nfsnode. */

int nfs_print(void *v)
{
  struct vop_print_args *ap = v;
  struct vnode *vp = ap->a_vp;
  struct nfsnode *np = VTONFS(vp);

  printf("tag VT_NFS, fileid %ld fsid 0x%lx",
         np->n_vattr.va_fileid, np->n_vattr.va_fsid);
  printf("\n");
  return (0);
}

/* nfs version 3 fsinfo rpc call */

int nfs_fsinfo(struct nfsmount *nmp)
{
  struct nfsv3_fsinfo *fsp;
  uint32_t pref, max;
  int error = 0;
  void *datareply;

  nfsstats.rpccnt[NFSPROC_FSINFO]++;

  error = nfs_request(nmp, NFSPROC_FSINFO, datareply);
  if (error)
    {
      goto nfsmout;
    }

  fsp = (struct nfsv3_fsinfo *)datareply;
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

void nfs_decode_args(struct nfsmount *nmp, struct nfs_args *argp)
{
  int adjsock = 0;
  int maxio;

#ifdef CONFIG_NFS_TCPIP
  /* Re-bind if rsrvd port requested and wasn't on one */

  adjsock = !(nmp->nm_flag & NFSMNT_RESVPORT)
    && (argp->flags & NFSMNT_RESVPORT);
#endif

  /* Also re-bind if we're switching to/from a connected UDP socket */

  adjsock |= ((nmp->nm_flag & NFSMNT_NOCONN) != (argp->flags & NFSMNT_NOCONN));

  /* Update flags atomically.  Don't change the lock bits. */

  nmp->nm_flag =
    (argp->flags & ~NFSMNT_INTERNAL) | (nmp->nm_flag & NFSMNT_INTERNAL);

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
        while (nfs_connect(nmp))
          {
            printf("nfs_args: retrying connect\n");
          }
    }

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
int nfs_mount(struct inode *mp, const char *path, void *data)
{
  int error;
  struct nfs_args args;
  struct sockaddr *nam;
  char pth[MNAMELEN];
  size_t len;
  nfsfh_t nfh[NFSX_V3FHMAX];

  bcopy(data, &args, sizeof(args.version));
  if (args.version == 3)
    {
      bcopy(data, &args, sizeof(struct nfs_args3));
      args.flags &= ~(NFSMNT_INTERNAL | NFSMNT_NOAC);
    }
  else if (args.version == NFS_ARGSVERSION)
    {
      error = copyin(data, &args, sizeof(struct nfs_args));
      args.flags &= ~NFSMNT_NOAC;       /* XXX - compatibility */
    }
  else
    return (EPROGMISMATCH);

  if ((args.flags & (NFSMNT_NFSV3 | NFSMNT_RDIRPLUS)) == NFSMNT_RDIRPLUS)
    return (EINVAL);

  if (mp->mnt_flag & MNT_UPDATE)
    {
      struct nfsmount *nmp = VFSTONFS(mp);

      if (nmp == NULL)
        return (EIO);

      /* When doing an update, we can't change from or to v3. */

      args.flags = (args.flags & ~(NFSMNT_NFSV3)) |
        (nmp->nm_flag & (NFSMNT_NFSV3));
      nfs_decode_args(nmp, &args);
      return (0);
    }
  if (args.fhsize < 0 || args.fhsize > NFSX_V3FHMAX)
    return (EINVAL);
  bcopy(args.fh, nfh, args.fhsize);
  memset(&pth[MNAMELEN], 0, sizeof(*pth[MNAMELEN]));
  bcopy(path, pth, MNAMELEN - 1);
  bcopy(args.addr, nam, sizeof(args.addr));
  args.fh = nfh;
  error = mountnfs(&args, mp, nam);
  return (error);
}

/* Common code for nfs_mount */

int mountnfs(struct nfs_args *argp, struct inode *mp, struct sockaddr *nam)
{
  struct nfsmount *nmp;
  int error;

  if (mp->mnt_flag & MNT_UPDATE)
    {
      nmp = VFSTONFS(mp);

      /* update paths, file handles, etc, here XXX */

      return (0);
    }
  else
    {
      nmp = (struct nfsmount *)zalloc(sizeof(struct nfmount));
      if (!nmp)
        {
          return -ENOMEM;
        }
      mp->i_private = &nmp;
    }

//vfs_getnewfsid(mp);
  nmp->nm_mountp = mp;
  nmp->nfs_mounted = true;
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
  memmove(nmp->nm_fh, argp->fh, argp->fhsize);
//strncpy(&mp->mnt_stat.f_fstypename[0], mp->mnt_vfc->vfc_name, MFSNAMELEN);
//memmove(hst, mp->mnt_stat.f_mntfromname, MNAMELEN);
//memmove(pth, mp->mnt_stat.f_mntonname, MNAMELEN);
//memmove(argp, &mp->mnt_stat.mount_info.nfs_args, sizeof(*argp));
  nmp->nm_nam = nam;
  nfs_decode_args(nmp, argp);

  /* Set up the sockets and per-host congestion */

  nmp->nm_sotype = argp->sotype;
  nmp->nm_soproto = argp->proto;

  /* For Connection based sockets (TCP,...) defer the connect until
   * the first request, in case the server is not responding.
   */

  if (nmp->nm_sotype == SOCK_DGRAM && (error = nfs_connect(nmp)))
    goto bad;

  nfs_init();

  return (0);
bad:
  nfs_disconnect(nmp);
  kfree(nmp);
  return (error);
}

/* unmount system call */

int nfs_unmount(struct inode *mp, int mntflags) // falta
{
  struct nfsmount *nmp;
  int error, flags;

  nmp = VFSTONFS(mp);
  flags = 0;

  if (mntflags & MNT_FORCE)
    flags |= FORCECLOSE;

  error = vflush(mp, NULL, flags);      // ?
  if (error)
    return (error);

  nfs_disconnect(nmp);
  kfree(nmp);
  return (0);
}

/* Flush out the buffer cache */

int nfs_sync(struct file *filep)
{
  struct inode *in = filep->f_inode;
  int error, allerror = 0;

  /* Force stale buffer cache information to be flushed. */

loop:
  LIST_FOREACH(vp, &mp->mnt_vnodelist, v_mntvnodes)
  {
    /* If the vnode that we are about to sync is no longer
     * associated with this mount point, start over.
     */

    if (in->nm_mountp != mp)
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
