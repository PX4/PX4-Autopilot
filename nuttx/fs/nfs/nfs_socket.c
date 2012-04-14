/****************************************************************************
 * fs/nfs/nfs_socket.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
 *
 *   copyright (c) 2004
 *   the regents of the university of michigan
 *   all rights reserved
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>
#include <queue.h>
#include <time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>

#include "nfs_args.h"
#include "rpc.h"
#include "rpc_v2.h"
#include "nfs_proto.h"
#include "nfs.h"
#include "xdr_subs.h"
#include "nfs_mount.h"
#include "nfs_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flag translations */

#define nfsmnt_to_rpcclnt(nf, rf, name) do \
  {                                        \
    if (nf & NFSMNT_##name)                \
      {                                    \
        rf |= RPCCLNT_##name;              \
      }                                    \
  } while(0)

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static struct rpc_program nfs3_program =
{
  NFS_PROG, NFS_VER3, "NFSv3"
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

int nfs_ticks;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nfs_init(void)
{
  rpcclnt_init();
}

int nfsx_connect(struct nfsmount *nmp)
{
  struct rpcclnt *rpc;
  int error = 0;

  if (nmp == NULL)
    {
      return EFAULT;
    }

  rpc = &nmp->nm_rpcclnt;

  rpc->rc_prog = &nfs3_program;

  nvdbg("nfsxconnect!\n");

  /* translate nfsmnt flags -> rpcclnt flags */

  rpc->rc_flag = 0;
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, SOFT);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, INT);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, NOCONN);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, DUMBTIMR);

  //rpc->rc_flag |= RPCCLNT_REDIRECT;     /* Make this a mount option. */

  rpc->rc_authtype = RPCAUTH_NULL;      /* for now */
  //rpc->rc_servername = nmp->nm_mountp->mnt_stat.f_mntfromname;
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
    {
      return error;
    }

  return rpcclnt_connect(rpc);
}

/* NFS disconnect. Clean up and unlink. */

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

int nfsx_request_xx(struct nfsmount *nm, int procnum,void *datain, void *dataout)
{
  int error;
  struct nfsmount *nmp;
  struct rpcclnt *clnt;
  struct rpc_reply *reply;
  int trylater_delay;

  nmp = nm;
  clnt = &nmp->nm_rpcclnt;

tryagain:

  memset(reply, 0, sizeof(struct rpc_reply));

  if ((error = rpcclnt_request(clnt, procnum, reply, datain)) != 0)
    {
      goto out;
    }

  dataout = reply->stat.where;

  if (reply->rpc_verfi.authtype != 0)
    {
      error = fxdr_unsigned(int, reply->rpc_verfi.authtype);

      if ((nmp->nm_flag & NFSMNT_NFSV3) && error == NFSERR_TRYLATER)
        {
          error = 0;
          trylater_delay *= NFS_TIMEOUTMUL;
          if (trylater_delay > NFS_MAXTIMEO)
            {
              trylater_delay = NFS_MAXTIMEO;
            }
          goto tryagain;
        }

      /* If the File Handle was stale, invalidate the
       * lookup cache, just in case.
       */

      if (error == ESTALE)
        {
          ndbg("%s: ESTALE on mount from server \n",
               nmp->nm_rpcclnt.rc_prog->prog_name);
        }
      else
        {
          ndbg("%s: unknown error %d from server \n",
               nmp->nm_rpcclnt.rc_prog->prog_name, error);
        }

      goto out;
    }
  return 0;
  
out:
  return error;
}

/* terminate any outstanding RPCs. */

int nfsx_nmcancelreqs(struct nfsmount *nmp)
{
  return rpcclnt_cancelreqs(&nmp->nm_rpcclnt);
}
