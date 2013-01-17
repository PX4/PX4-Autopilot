/****************************************************************************
 * fs/nfs/rpc_clnt.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 2004 The Regents of the University of Michigan.
 *   All rights reserved.
 *
 *   Copyright (c) 2004 Weston Andros Adamson <muzzle@umich.edu>.
 *   Copyright (c) 2004 Marius Aamodt Eriksen <marius@umich.edu>.
 *   All rights reserved.
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
 * 3. Neither the name of the University nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (c) 1989, 1991, 1993, 1995 The Regents of the University of
 *   California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by Rick Macklem at
 * The University of Guelph.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. 2.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution. 3. All advertising
 * materials mentioning features or use of this software must display the
 * following acknowledgement: This product includes software developed by the
 * University of California, Berkeley and its contributors. 4. Neither the
 * name of the University nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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

#include "xdr_subs.h"
#include "nfs_proto.h"
#include "rpc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Increment RPC statistics */

#ifdef CONFIG_NFS_STATISTICS
#  define rpc_statistics(n) do { rpcstats.(n)++; } while (0)
#else
#  define rpc_statistics(n)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Static data, mostly RPC constants in XDR form */

static uint32_t rpc_reply;
static uint32_t rpc_call;
static uint32_t rpc_vers;
static uint32_t rpc_msgdenied;
static uint32_t rpc_mismatch;
static uint32_t rpc_auth_unix;
static uint32_t rpc_msgaccepted;
static uint32_t rpc_autherr;
static uint32_t rpc_auth_null;

/* Global statics for all client instances.  Cleared by NuttX on boot-up. */

#ifdef CONFIG_NFS_STATISTICS
static struct rpcstats rpcstats;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpcclnt_send(FAR struct rpcclnt *rpc, int procid, int prog,
                        FAR void *call, int reqlen);
static int rpcclnt_receive(FAR struct rpcclnt *rpc, struct sockaddr *aname,
                           int proc, int program, void *reply, size_t resplen);
static int rpcclnt_reply(FAR struct rpcclnt *rpc, int procid, int prog,
                         void *reply, size_t resplen);
static uint32_t rpcclnt_newxid(void);
static void rpcclnt_fmtheader(FAR struct rpc_call_header *ch,
                              uint32_t xid, int procid, int prog, int vers);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpcclnt_send
 *
 * Description:
 *   This is the nfs send routine.
 *
 * Returned Value:
 *   Returns zero on success or a (positive) errno value on failure.
 *
 ****************************************************************************/

static int rpcclnt_send(FAR struct rpcclnt *rpc, int procid, int prog,
                        FAR void *call, int reqlen)
{
  ssize_t nbytes;
  int error = OK;

  /* Send the call message
   *
   * On success, psock_sendto returns the number of bytes sent;
   * On failure, it returns -1 with the specific error in errno.
   */

  nbytes = psock_sendto(rpc->rc_so, call, reqlen, 0,
                        rpc->rc_name, sizeof(struct sockaddr));
  if (nbytes < 0)
    {
      /* psock_sendto failed */

      error = errno;
      fdbg("ERROR: psock_sendto failed: %d\n", error);
    }

  return error;
}

/****************************************************************************
 * Name: rpcclnt_receive
 *
 * Description:
 *   Receive a Sun RPC Request/Reply. For SOCK_DGRAM, the work is all done
 *   by psock_recvfrom().
 *
 ****************************************************************************/

static int rpcclnt_receive(FAR struct rpcclnt *rpc, FAR struct sockaddr *aname,
                           int proc, int program, FAR void *reply,
                           size_t resplen)
{
  ssize_t nbytes;
  int error = 0;

  socklen_t fromlen = sizeof(struct sockaddr);
  nbytes = psock_recvfrom(rpc->rc_so, reply, resplen, 0, aname, &fromlen);
  if (nbytes < 0)
    {
      error = errno;
      fdbg("ERROR: psock_recvfrom failed: %d\n", error);
    }

  return error;
}

/****************************************************************************
 * Name: rpcclnt_reply
 *
 * Description:
 *   Received the RPC reply on the socket.
 *
 ****************************************************************************/

static int rpcclnt_reply(FAR struct rpcclnt *rpc, int procid, int prog,
                         FAR void *reply, size_t resplen)
{
  int error;

  /* Get the next RPC reply from the socket */

  error = rpcclnt_receive(rpc, rpc->rc_name, procid, prog, reply, resplen);
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_receive returned: %d\n", error);

      /* If we failed because of a timeout, then try sending the CALL 
       * message again.
       */

      if (error == EAGAIN || error == ETIMEDOUT)
        {
          rpc->rc_timeout = true;
       }
    }

  /* Get the xid and check that it is an RPC replysvr */

  else
    {
      FAR struct rpc_reply_header *replyheader =
        (FAR struct rpc_reply_header *)reply;

      if (replyheader->rp_direction != rpc_reply)
        {
          fdbg("ERROR: Different RPC REPLY returned\n");
          rpc_statistics(rpcinvalid);
          error = EPROTO;
        }
    }

  return error;
}

/****************************************************************************
 * Name: rpcclnt_newxid
 *
 * Description:
 *   Get a new (non-zero) xid
 *
 ****************************************************************************/

static uint32_t rpcclnt_newxid(void)
{
  static uint32_t rpcclnt_xid = 0;
  static uint32_t rpcclnt_xid_touched = 0;

  srand(time(NULL));
  if ((rpcclnt_xid == 0) && (rpcclnt_xid_touched == 0))
    {
      rpcclnt_xid = rand();
      rpcclnt_xid_touched = 1;
    }
  else
    {
      int xidp = 0;
      do
        {
          xidp = rand();
        }
      while ((xidp % 256) == 0);

      rpcclnt_xid += xidp;
    }

  return rpcclnt_xid;
}

/****************************************************************************
 * Name: rpcclnt_fmtheader
 *
 * Description:
 *   Format the common part of the call header 
 *
 ****************************************************************************/

static void rpcclnt_fmtheader(FAR struct rpc_call_header *ch,
                              uint32_t xid, int prog, int vers, int procid)
{
  /* Format the call header */

  ch->rp_xid            = txdr_unsigned(xid);
  ch->rp_direction      = rpc_call;
  ch->rp_rpcvers        = rpc_vers;
  ch->rp_prog           = txdr_unsigned(prog);
  ch->rp_vers           = txdr_unsigned(vers);
  ch->rp_proc           = txdr_unsigned(procid);

  /* rpc_auth part (auth_null) */

  ch->rpc_auth.authtype = rpc_auth_null;
  ch->rpc_auth.authlen  = 0;

  /* rpc_verf part (auth_null) */

  ch->rpc_verf.authtype  = rpc_auth_null;
  ch->rpc_verf.authlen   = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpcclnt_init
 *
 * Description:
 *   Initialize the RPC client
 *
 ****************************************************************************/

void rpcclnt_init(void)
{
  /* RPC constants how about actually using more than one of these! */

  rpc_reply = txdr_unsigned(RPC_REPLY);
  rpc_vers = txdr_unsigned(RPC_VER2);
  rpc_call = txdr_unsigned(RPC_CALL);
  rpc_msgdenied = txdr_unsigned(RPC_MSGDENIED);
  rpc_msgaccepted = txdr_unsigned(RPC_MSGACCEPTED);
  rpc_mismatch = txdr_unsigned(RPC_MISMATCH);
  rpc_autherr = txdr_unsigned(RPC_AUTHERR);
  rpc_auth_unix = txdr_unsigned(RPCAUTH_UNIX);
  rpc_auth_null = txdr_unsigned(RPCAUTH_NULL);

  fvdbg("RPC initialized\n");
}

/****************************************************************************
 * Name: rpcclnt_connect
 *
 * Description:
 *   Initialize sockets for a new RPC connection.  We do not free the
 *   sockaddr if an error occurs.
 *
 ****************************************************************************/

int rpcclnt_connect(struct rpcclnt *rpc)
{
  struct socket *so;
  int error;
  struct sockaddr *saddr;
  struct sockaddr_in sin;
  struct sockaddr_in *sa;

  union
  {
    struct rpc_call_pmap  sdata;
    struct rpc_call_mount mountd;
  } request;

  union
  {
    struct rpc_reply_pmap  rdata;
    struct rpc_reply_mount mdata;
  } response;

  struct timeval tv;
  uint16_t tport;
  int errval;

  fvdbg("Connecting\n");

  /* Create the socket */

  saddr = rpc->rc_name;

  /* Create an instance of the socket state structure */

  so = (struct socket *)kzalloc(sizeof(struct socket));
  if (!so)
    {
      fdbg("ERROR: Failed to allocate socket structure\n");
      return ENOMEM;
    }

  error = psock_socket(saddr->sa_family, rpc->rc_sotype, IPPROTO_UDP, so);
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_socket failed: %d", errval);
      return error;
    }

  so->s_crefs     = 1;
  rpc->rc_so      = so;

   /* Always set receive timeout to detect server crash and reconnect.
    * Otherwise, we can get stuck in psock_receive forever.
    */

  tv.tv_sec  = 1;
  tv.tv_usec = 0;

  error = psock_setsockopt(rpc->rc_so, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&tv, sizeof(tv));
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_setsockopt failed: %d\n", errval);
      goto bad;
    }

  /* Some servers require that the client port be a reserved port
   * number. We always allocate a reserved port, as this prevents
   * filehandle disclosure through UDP port capture.
   */

  sin.sin_family      = AF_INET;
  sin.sin_addr.s_addr = INADDR_ANY;
  tport               = 1024;

  errval = 0;
  do
    {
      tport--;
      sin.sin_port = htons(tport);
      error = psock_bind(rpc->rc_so, (struct sockaddr *)&sin, sizeof(sin));
      if (error < 0)
        {
          errval = errno;
          fdbg("ERROR: psock_bind failed: %d\n", errval);
        }
    }
  while (errval == EADDRINUSE && tport > 1024 / 2);

  if (error)
    {
      fdbg("ERROR: psock_bind failed: %d\n", errval);
      goto bad;
    }

  /* Protocols that do not require connections may be optionally left
   * unconnected for servers that reply from a port other than
   * NFS_PORT.
   */

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_connect to PMAP port failed: %d", errval);
      goto bad;
    }

  /* Do the RPC to get a dynamic bounding with the server using ppmap.
   * Get port number for MOUNTD.
   */

  request.sdata.pmap.prog = txdr_unsigned(RPCPROG_MNT);
  request.sdata.pmap.vers = txdr_unsigned(RPCMNT_VER1);
  request.sdata.pmap.proc = txdr_unsigned(IPPROTO_UDP);
  request.sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&request.sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&response.rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  sa = (FAR struct sockaddr_in *)saddr;
  sa->sin_port = htons(fxdr_unsigned(uint32_t, response.rdata.pmap.port));

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_connect MOUNTD port failed: %d\n", errval);
      goto bad;
    }

  /* Do RPC to mountd. */

  strncpy(request.mountd.mount.rpath, rpc->rc_path, 90);
  request.mountd.mount.len =  txdr_unsigned(sizeof(request.mountd.mount.rpath));

  error = rpcclnt_request(rpc, RPCMNT_MOUNT, RPCPROG_MNT, RPCMNT_VER1,
                          (FAR void *)&request.mountd, sizeof(struct call_args_mount),
                          (FAR void *)&response.mdata, sizeof(struct rpc_reply_mount));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  error = fxdr_unsigned(uint32_t, response.mdata.mount.status);
  if (error != 0)
    {
      fdbg("ERROR: Bad mount status: %d\n", error);
      goto bad;
    }

  memcpy(&rpc->rc_fh, &response.mdata.mount.fhandle, sizeof(nfsfh_t));

  /* Do the RPC to get a dynamic bounding with the server using PMAP.
   * NFS port in the socket.
   */

  sa->sin_port = htons(PMAPPORT);

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_connect PMAP port failed: %d\n", errval);
      goto bad;
    }

  request.sdata.pmap.prog = txdr_unsigned(NFS_PROG);
  request.sdata.pmap.vers = txdr_unsigned(NFS_VER3);
  request.sdata.pmap.proc = txdr_unsigned(IPPROTO_UDP);
  request.sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&request.sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&response.rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  sa->sin_port = htons(fxdr_unsigned(uint32_t, response.rdata.pmap.port));

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error)
    {
      fdbg("psock_connect NFS port returns %d\n", error);
      goto bad;
    }

  return OK;

bad:
  rpcclnt_disconnect(rpc);
  return error;
}

/****************************************************************************
 * Name: rpcclnt_disconnect
 *
 * Description:
 *   Disconnect from the NFS server.
 *
 ****************************************************************************/

void rpcclnt_disconnect(struct rpcclnt *rpc)
{
  if (rpc->rc_so != NULL)
    {
      (void)psock_close(rpc->rc_so);
    }
}

/****************************************************************************
 * Name: rpcclnt_umount
 *
 * Description:
 *   Un-mount the NFS file system.
 *
 ****************************************************************************/

int rpcclnt_umount(struct rpcclnt *rpc)
{
  struct sockaddr *saddr;
  struct sockaddr_in *sa;

  union
  {
    struct rpc_call_pmap   sdata;
    struct rpc_call_umount mountd;
  } request;

  union
  {
    struct rpc_reply_pmap   rdata;
    struct rpc_reply_umount mdata;
  } response;

  int error;
  int ret;

  saddr = rpc->rc_name;
  sa = (FAR struct sockaddr_in *)saddr;

  /* Do the RPC to get a dynamic bounding with the server using ppmap.
   * Get port number for MOUNTD.
   */

  sa->sin_port = htons(PMAPPORT);

  ret = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (ret < 0)
    {
      error = errno;
      fdbg("ERROR: psock_connect failed [port=%d]: %d\n",
            ntohs(sa->sin_port), error);
      goto bad;
    }

  request.sdata.pmap.prog = txdr_unsigned(RPCPROG_MNT);
  request.sdata.pmap.vers = txdr_unsigned(RPCMNT_VER1);
  request.sdata.pmap.proc = txdr_unsigned(IPPROTO_UDP);
  request.sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&request.sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&response.rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  sa->sin_port = htons(fxdr_unsigned(uint32_t, response.rdata.pmap.port));

  ret = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (ret < 0)
    {
      error = errno;
      fdbg("ERROR: psock_connect failed [port=%d]: %d\n",
            ntohs(sa->sin_port), error);
      goto bad;
    }

  /* Do RPC to umountd. */

  strncpy(request.mountd.umount.rpath, rpc->rc_path, 92);
  request.mountd.umount.len =  txdr_unsigned(sizeof(request.mountd.umount.rpath));

  error = rpcclnt_request(rpc, RPCMNT_UMOUNT, RPCPROG_MNT, RPCMNT_VER1,
                          (FAR void *)&request.mountd, sizeof(struct call_args_umount),
                          (FAR void *)&response.mdata, sizeof(struct rpc_reply_umount));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  return OK;

bad:
  rpcclnt_disconnect(rpc);
  return error;
}

/****************************************************************************
 * Name: rpcclnt_request
 *
 * Description:
 *   Perform the RPC request.  Logic formats the RPC CALL message and calls
 *   rpcclnt_send to send the RPC CALL message.  It then calls rpcclnt_reply()
 *   to get the response.  It may attempt to re-send the CALL message on
 *   certain errors.
 *
 *   On successful receipt, it verifies the RPC level of the returned values.
 *   (There may still be be NFS layer errors that will be deted by calling
 *   logic).
 *
 ****************************************************************************/

int rpcclnt_request(FAR struct rpcclnt *rpc, int procnum, int prog,
                    int version, FAR void *request, size_t reqlen,
                    FAR void *response, size_t resplen)
{
  struct rpc_reply_header *replymsg;
  uint32_t tmp;
  uint32_t xid;
  int retries;
  int error = 0;

  /* Get a new (non-zero) xid */

  xid = rpcclnt_newxid();

  /* Initialize the RPC header fields */

  rpcclnt_fmtheader((FAR struct rpc_call_header *)request,
                    xid, prog, version, procnum);

  /* Get the full size of the message (the size of variable data plus the size of
   * the messages header).
   */

  reqlen += sizeof(struct rpc_call_header);

  /* Send the RPC call messsages and receive the RPC response.  A limited
   * number of re-tries will be attempted, but only for the case of response
   * timeouts.
   */

  retries = 0;
  do
    {
      /* Do the client side RPC. */

      rpc_statistics(rpcrequests);
      rpc->rc_timeout = false;
  
      /* Send the RPC CALL message */

      error = rpcclnt_send(rpc, procnum, prog, request, reqlen);
      if (error != OK)
        {
          fvdbg("ERROR rpcclnt_send failed: %d\n", error);
        }

      /* Wait for the reply from our send */

      else
        {
          error = rpcclnt_reply(rpc, procnum, prog, response, resplen);
          if (error != OK)
            {
              fvdbg("ERROR rpcclnt_reply failed: %d\n", error);
            }
        }

      retries++;
    }
  while (rpc->rc_timeout && retries <= rpc->rc_retry);

  if (error != OK)
    {
      fdbg("ERROR: RPC failed: %d\n", error);
      return error;
    }

  /* Break down the RPC header and check if it is OK */

  replymsg = (FAR struct rpc_reply_header *)response;

  tmp = fxdr_unsigned(uint32_t, replymsg->type);
  if (tmp == RPC_MSGDENIED)
    {
      tmp = fxdr_unsigned(uint32_t, replymsg->status);
      switch (tmp)
        {
        case RPC_MISMATCH:
          fdbg("RPC_MSGDENIED: RPC_MISMATCH error\n");
          return EOPNOTSUPP;

        case RPC_AUTHERR:
          fdbg("RPC_MSGDENIED: RPC_AUTHERR error\n");
          return EACCES;

        default:
          return EOPNOTSUPP;
        }
    }
  else if (tmp != RPC_MSGACCEPTED)
    {
      return EOPNOTSUPP;
    }

  tmp = fxdr_unsigned(uint32_t, replymsg->status);
  if (tmp == RPC_SUCCESS)
    {
      fvdbg("RPC_SUCCESS\n");
    }
  else if (tmp == RPC_PROGMISMATCH)
    {
      fdbg("RPC_MSGACCEPTED: RPC_PROGMISMATCH error\n");
      return EOPNOTSUPP;
    }
  else if (tmp > 5)
    {
      fdbg("ERROR:  Other RPC type: %d\n", tmp);
      return EOPNOTSUPP;
    }

  return OK;
}
