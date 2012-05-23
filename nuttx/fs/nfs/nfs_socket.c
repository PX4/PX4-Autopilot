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
#include <nuttx/kmalloc.h>

#include "nfs.h"
#include "rpc.h"
#include "rpc_v2.h"
#include "nfs_proto.h"
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

uint32_t nfs_true;
uint32_t nfs_false;
uint32_t nfs_xdrneg1;
int nfs_ticks;
struct nfsstats nfsstats;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nfs_init(void)
{
   nfs_true = txdr_unsigned(TRUE);
   nfs_false = txdr_unsigned(FALSE);
   nfs_xdrneg1 = txdr_unsigned(-1);

   nfs_ticks = (CLOCKS_PER_SEC * NFS_TICKINTVL + 500) / 1000;
   if (nfs_ticks < 1)
    {
      nfs_ticks = 1;
    }

   rpcclnt_init();
}

int nfs_connect(struct nfsmount *nmp)
{
  struct rpcclnt *rpc;

  if (nmp == NULL)
    {
      return EFAULT;
    }

  /* Create an instance of the rpc state structure */

  rpc = (struct rpcclnt *)kzalloc(sizeof(struct rpcclnt));
  if (!rpc)
    {
      ndbg("Failed to allocate rpc structure\n");
      return -ENOMEM;
    }

  rpc->rc_prog = &nfs3_program;

  nvdbg("nfs connect!\n");

  /* translate nfsmnt flags -> rpcclnt flags */

  rpc->rc_flag = 0;
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, SOFT);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, INT);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, NOCONN);
  nfsmnt_to_rpcclnt(nmp->nm_flag, rpc->rc_flag, DUMBTIMR);

//rpc->rc_flag |= RPCCLNT_REDIRECT;     /* Make this a mount option. */

//rpc->rc_authtype = RPCAUTH_NULL;      /* for now */
  rpc->rc_path = nmp->nm_path;
  rpc->rc_name = &nmp->nm_nam;
//rpc->rc_fh = nmp->nm_fh;

  rpc->rc_sotype = nmp->nm_sotype;
  rpc->rc_soproto = nmp->nm_soproto;
  rpc->rc_rsize = (nmp->nm_rsize > nmp->nm_readdirsize) ?
    nmp->nm_rsize : nmp->nm_readdirsize;
  rpc->rc_wsize = nmp->nm_wsize;
//rpc->rc_deadthresh = nmp->nm_deadthresh;
  rpc->rc_timeo = nmp->nm_timeo;
  rpc->rc_retry = nmp->nm_retry;

  /* v3 need to use this */

  rpc->rc_proctlen = 0;
  rpc->rc_proct = NULL;

  nmp->nm_rpcclnt = rpc;

  return rpcclnt_connect(rpc);
}

/* NFS disconnect. Clean up and unlink. */

void nfs_disconnect(struct nfsmount *nmp)
{
  rpcclnt_disconnect(nmp->nm_rpcclnt);
}

#ifdef CONFIG_NFS_TCPIP
void nfs_safedisconnect(struct nfsmount *nmp)
{
  rpcclnt_safedisconnect(nmp->nm_rpcclnt);
}
#endif

int nfs_request(struct nfsmount *nmp, int procnum, FAR const void *datain,
                FAR void *dataout)
{
  int error;
  struct rpcclnt *clnt= nmp->nm_rpcclnt;
  struct rpc_reply_header replyh;
  int trylater_delay;

tryagain:

  memset(&replyh, 0, sizeof(struct rpc_reply_header));

  error = rpcclnt_request(clnt, procnum, nmp->nm_rpcclnt->rc_prog->prog_id,
                          nmp->nm_rpcclnt->rc_prog->prog_version, dataout,
                          datain);
  if (error != 0)
    {
      goto out;
    }

  bcopy(dataout, &replyh, sizeof(replyh));

  if (replyh.rpc_verfi.authtype != 0)
    {
      error = fxdr_unsigned(int, replyh.rpc_verfi.authtype);

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
               nmp->nm_rpcclnt->rc_prog->prog_name);
        }
      else
        {
          ndbg("%s: unknown error %d from server \n",
               nmp->nm_rpcclnt->rc_prog->prog_name, error);
        }

      goto out;
    }

  return 0;

out:
  return error;
}

#undef COMP
#ifdef COMP

/* terminate any outstanding RPCs. */

int nfs_nmcancelreqs(struct nfsmount *nmp)
{
  return 0; //rpcclnt_cancelreqs(nmp->nm_rpcclnt);
}
#endif