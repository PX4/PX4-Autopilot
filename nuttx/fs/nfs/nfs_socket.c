/*
 * copyright (c) 2004
 * the regents of the university of michigan
 * all rights reserved
 * 
 * permission is granted to use, copy, create derivative works and redistribute
 * this software and such derivative works for any purpose, so long as the name
 * of the university of michigan is not used in any advertising or publicity
 * pertaining to the use or distribution of this software without specific,
 * written prior authorization.  if the above copyright notice or any other
 * identification of the university of michigan is included in any copy of any
 * portion of this software, then the disclaimer below must also be included.
 * 
 * this software is provided as is, without representation from the university
 * of michigan as to its fitness for any purpose, and without warranty by the
 * university of michigan of any kind, either express or implied, including
 * without limitation the implied warranties of merchantability and fitness for
 * a particular purpose. the regents of the university of michigan shall not be
 * liable for any damages, including special, indirect, incidental, or
 * consequential damages, with respect to any claim arising out of or in
 * connection with the use of the software, even if it has been or is hereafter
 * advised of the possibility of such damages.
 */

/* wrappers around rpcclnt */

/* XXX add tryagain code in nfs_request_xx */

/*
 * Socket operations for use by nfs
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mount.h>
#include <sys/socket.h>

#include "rpc_clnt.h"
#include "rpc_v2.h"
#include "nfs_proto.h
#include "nfs.h"
#include "xdr_subs.h"
#include "nfsmount.h"
//#include <nfsx/nfs_common.h>
#include "nfs_socket.h"

/* Flag translations */
#define nfsmnt_to_rpcclnt(nf, rf, name) do {    \
  if (nf & NFSMNT_##name)) {                    \
    rf |= RPCCLNT_##name                        \
  }                                             \
} while(0)

static struct rpc_program nfs2_program =
{
  NFS_PROG, NFS_VER2, "NFSv2"
};

static struct rpc_program nfs3_program =
{
  NFS_PROG, NFS_VER3, "NFSv3"
};

static struct rpc_program nfs4_program =
{
  NFS_PROG, NFS_VER4, "NFSv4"
};

/* XXXMARIUS: name collision */
int nfsx_connect(struct nfsmount *nmp)
{
  struct rpcclnt *rpc;
  int error = 0;

  if (nmp == NULL)
    return EFAULT;

  rpc = &nmp->nm_rpcclnt;

  rpc->rc_prog = &nfs3_program;

  printf("nfsxconnect!\n");

  /* translate nfsmnt flags -> rpcclnt flags */
  rpc->rc_flag = 0;
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, SOFT);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, INT);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, NOCONN);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, DUMBTIMR);

  rpc->rc_flag |= RPCCLNT_REDIRECT;     /* Make this a mount option. */

  rpc->rc_authtype = RPCAUTH_NULL;      /* for now */
  rpc->rc_servername = nmp->nm_mountp->mnt_stat.f_mntfromname;
  rpc->rc_name = (struct sockaddr *)nmp->nm_nam;

  rpc->rc_sotype = nmp->nm_sotype;
  rpc->rc_soproto = nmp->nm_soproto;
  rpc->rc_rsize = (nmp->nm_rsize > nmp->nm_readdirsize) ?
    nmp->nm_rsize : nmp->nm_readdirsize;
  rpc->rc_wsize = nmp->nm_wsize;
  rpc->rc_deadthresh = nmp->nm_deadthresh;
  rpc->rc_timeo = nmp->nm_timeo;
  rpc->rc_retry = nmp->nm_retry;

  /* XXX v2,3 need to use this */
  rpc->rc_proctlen = 0;
  rpc->rc_proct = NULL;

  if (error)
    return error;

  return rpcclnt_connect(rpc);
}

/* NFS disconnect. Clean up and unlink. */

/* XXXMARIUS: name collision */

void nfsx_disconnect(struct nfsmount *nmp)
{
  rpcclnt_disconnect(&nmp->nm_rpcclnt);
}

#ifdef CONFIG_NFS_TCPIP
void nfsx_safedisconnect(struct nfsmount *nmp)
{
  rpcclnt_safedisconnect(&nmp->nm_rpcclnt);
}
#endif

int
nfsx_request_xx(struct nfsmount *nm, struct vnode *vp, struct mbuf *mrest,
                int procnum, cthread_t * td, struct ucred *cred,
                struct mbuf **mrp, struct mbuf **mdp, caddr_t * dposp)
{
  int error;
  u_int32_t *tl;
  struct nfsmount *nmp;
  struct rpcclnt *clnt;
  struct mbuf *md, *mrep;
  caddr_t dpos;
  struct rpc_reply reply;
#if 0
  int t1;
#endif                                 /* 0 */

  if (vp != NULL)
    nmp = VFSTONFS(vp->v_mount);
  else
    nmp = nm;

  clnt = &nmp->nm_rpcclnt;

#if 0
tryagain:
#endif

  memset(&reply, 0, sizeof(reply));

  if ((error = rpcclnt_request(clnt, mrest, procnum, td, cred, &reply)) != 0)
    goto out;

  mrep = reply.mrep;
  md = reply.result_md;
  dpos = reply.result_dpos;

  tl = nfsm_dissect(u_int32_t *, NFSX_UNSIGNED);
  if (*tl != 0)
    {
      error = fxdr_unsigned(int, *tl);
#if 0
      if ((nmp->nm_flag & NFSMNT_NFSV3) && error == NFSERR_TRYLATER)
        {
          m_freem(mrep);
          error = 0;
          waituntil = time_second + trylater_delay;
          while (time_second < waituntil)
            (void)tsleep(&lbolt, PSOCK, "nqnfstry", 0);
          trylater_delay *= nfs_backoff[trylater_cnt];
          if (trylater_cnt < NFS_NBACKOFF - 1)
            trylater_cnt++;
          goto tryagain;
        }
#endif

      /* 
       ** If the File Handle was stale, invalidate the
       ** lookup cache, just in case.
       **/
      if (error == ESTALE)
        if (vp != NULL)
          cache_purge(vp);
        else
          printf("%s: ESTALE on mount from server  %s\n",
                 nmp->nm_rpcclnt.rc_prog->prog_name,
                 nmp->nm_rpcclnt.rc_servername);
      else
        printf("%s: unknown error %d from server %s\n",
               nmp->nm_rpcclnt.rc_prog->prog_name, error,
               nmp->nm_rpcclnt.rc_servername);
      goto out;
    }

  m_freem(mrest);

  *mrp = mrep;
  *mdp = md;
  *dposp = dpos;
  return (0);
nfsmout:
out:
  /* XXX: don't free mrest if an error occured, to allow caller to retry */
  m_freem(mrest);
  m_freem(reply.mrep);
  *mrp = NULL;
  *mdp = NULL;

  return (error);
}

/* terminate any outstanding RPCs. */

int nfsx_nmcancelreqs(struct nfsmount *nmp)
{
  return rpcclnt_cancelreqs(&nmp->nm_rpcclnt);
}
