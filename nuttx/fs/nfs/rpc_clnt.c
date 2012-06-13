/****************************************************************************
 * fs/nfs/rpc_clnt.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/* There is a congestion window for outstanding RPCs maintained per mount
 * point. The cwnd size is adjusted in roughly the way that: Van Jacobson,
 * Congestion avoidance and Control, In "Proceedings of SIGCOMM '88". ACM,
 * August 1988. describes for TCP. The cwnd size is chopped in half on a
 * retransmit timeout and incremented by 1/cwnd when each RPC reply is
 * received and a full cwnd of RPCs is in progress. (The sent count and cwnd
 * are scaled for integer arith.) Variants of "slow start" were tried and
 * were found to be too much of a performance hit (ave. rtt 3 times larger),
 * I suspect due to the large rtt that nfs RPCs have.
 */

#define RPC_CWNDSCALE   256
#define RPC_MAXCWND     (RPC_CWNDSCALE * 32)

#define RPC_ERRSTR_ACCEPTED_SIZE 6
#define RPC_ERRSTR_AUTH_SIZE 6

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
#ifdef CONFIG_NFS_TCPIP
static int rpcclnt_reconnect(FAR struct rpcclnt *rpc);
#endif
static uint32_t rpcclnt_newxid(void);
static void rpcclnt_fmtheader(FAR struct rpc_call_header *ch,
                              uint32_t xid, int procid, int prog, int vers);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the nfs send routine.  Returns EINTR if the RPC is terminated, 0
 * otherwise - set RPCCALL_MUSTRESEND if the send fails for any reason - do any
 * cleanup required by recoverable socket errors.
 */

static int rpcclnt_send(FAR struct rpcclnt *rpc, int procid, int prog,
                        FAR void *call, int reqlen)
{
  ssize_t nbytes;
  int error = OK;

  DEBUGASSERT(rpc && call);

  /* Assume that we will not have to resend */

  rpc->rc_callflags &= ~RPCCALL_MUSTRESEND;

  /* Send the call message
   *
   * On success, psock_sendto returns the number of bytes sent;
   * On failure, it returns -1 with the specific error in errno.
   */

  nbytes = psock_sendto(rpc->rc_so, call, reqlen, 0,
                        rpc->rc_name, sizeof(struct sockaddr));
  if (nbytes < 0)
    {
      /* psock_sendto failed,  Sample the error value (subsequent
       * calls can change the errno value!)
       */

      error = errno;
      fdbg("ERROR: psock_sendto failed: %d\n", error);

      rpc->rc_callflags |= RPCCALL_MUSTRESEND;
    }

  return error;
}

/* Receive a Sun RPC Request/Reply. For SOCK_DGRAM, the work is all
 * done by psock_recvfrom(). For SOCK_STREAM, first get the
 * Record Mark to find out how much more there is to get. We must
 * lock the socket against other receivers until we have an entire
 * rpc request/reply.
 */

static int rpcclnt_receive(FAR struct rpcclnt *rpc, struct sockaddr *aname,
                           int proc, int program, void *reply, size_t resplen)
{
  ssize_t nbytes;
#ifdef CONFIG_NFS_TCPIP
  uint32_t resplen;
  int sotype;
#endif
  int error = 0;
  int errval;

#ifdef CONFIG_NFS_TCPIP
  /* Set up arguments for psock_recvfrom() */

  sotype = rpc->rc_sotype;

  if (sotype != SOCK_DGRAM)
    {
    tryagain:

      /* Check for fatal errors and resending request. */

      if (rpc->rc_so == NULL)
        {
          error = rpcclnt_reconnect(rpc);
          if (error)
            {
              return error;
            }

          goto tryagain;
        }

      while (rpc->rc_callflags & RPCCALL_MUSTRESEND)
        {
          rpc_statistics(rpcretries);
          error = rpcclnt_send(rpc, proc, program, call, reqlen);
          if (error)
            {
              if (error == EINTR || error == ERESTART)
                {
                  return error;
                }

              error = rpcclnt_reconnect(rpc);
              if (error != OK)
                {
                  return error;
                }

              goto tryagain;
            }
        }

      if (sotype == SOCK_STREAM)
        {
          errval = 0;
          do
            {
              socklen_t fromlen = sizeof(*rpc->rc_name)
              nbytes = psock_recvfrom(rpc->rc_so, reply, resplen,
                                      MSG_WAITALL, rpc->rc_name,
                                      &fromlen);
              if (nbytes < 0)
                {
                  errval = errno;
                  fdbg("ERROR: psock_recvfrom returned %d\n", errval);
                }
            }
          while (errval == EWOULDBLOCK);

          if (nbytes < 0)
            {
              error = errval;
            }
          else if (nbytes < resplen)
            {
              fdbg("ERROR: Short receive from RPC server\n");
              fvdbg("       Expected %d bytes, received %d bytes\n",
                    resplen, nbytes);
              error = EPIPE;
            }
          else
            {
              error = 0;
            }

#warning "What is resplen?  This logic is not right!"
          resplen = ntohl(resplen) & ~0x80000000;

          /* This is SERIOUS! We are out of sync with the
           * sender and forcing a disconnect/reconnect is all I
           * can do.
           */

          else if (resplen > RPC_MAXPACKET)
            {
              fdbg("ERROR: Impossible length rom RPC server: %d\n", resplen);
              error = EFBIG;
              goto errout;
            }

          errval = 0
          do
            {
              socklen_t fromlen = sizeof(*rpc->rc_name);
              nbytes = psock_recvfrom(so, reply, sizeof(*reply),
                                      MSG_WAITALL, rpc->rc_name,
                                      &fromlen);
              if (nbytes < 0)
                {
                  errval = errno;
                  fdbg("ERROR: psock_recvfrom failed: %d\n", errval);
                }
            }
          while (errval == EWOULDBLOCK || errval == EINTR || errval == ERESTART);

          if (nbytes < 0)
            {
              error = errval;
              goto errout;
            }
          else if (nbytes < resplen)
            {
              fdbg("ERROR: Short receive from RPC server\n");
              fvdbg("       Expected %d bytes, received %d bytes\n",
                    resplen, nbytes);
              error = EPIPE;
            }
          else
            {
              error = 0;
            }
        }
      else
        {
          /* NB: Since uio_resid is big, MSG_WAITALL is ignored
           * and psock_recvfrom() will return when it has either a
           * control msg or a data msg. We have no use for
           * control msg., but must grab them and then throw
           * them away so we know what is going on.
           */

          errval = 0;
          do
            {
              socklen_t fromlen = sizeof(*rpc->rc_name);
              nbytes = psock_recvfrom(so, reply, sizeof(*reply), 0,
                                      rpc->rc_name, &fromlen);
              if (nbytes < 0)
                {
                  errval = errno;
                  fdbg("ERROR: psock_recvfrom failed: %d\n", errval);
                }
            }
          while (errval == EWOULDBLOCK || nbytes == 0);

          if (nbytes < 0)
            {
              error = errval;
              goto errout;
            }
          else if (nbytes < resplen)
            {
              fdbg("ERROR: Short receive from RPC server\n");
              fvdbg("       Expected %d bytes, received %d bytes\n",
                    resplen, nbytes);
              error = EPIPE;
            }
          else
            {
              error = 0;
            }
        }

    errout:
      if (error != 0 && error != EINTR && error != ERESTART)
        {
          if (error != EPIPE)
            {
              fdbg("ERROR: Receive error %d from RPC server\n", error);
            }

          error = rpcclnt_reconnect(rpc);
          if (error == 0)
            {
              goto tryagain;
            }
        }
    }
  else
#endif
    {
      if (rpc->rc_so == NULL)
        {
          return EACCES;
        }

      socklen_t fromlen = sizeof(struct sockaddr);
      nbytes = psock_recvfrom(rpc->rc_so, reply, resplen, 0, aname, &fromlen);
      if (nbytes < 0)
        {
          errval = errno;
          fdbg("ERROR: psock_recvfrom failed: %d\n", errval);
          error = errval;
        }
    }

  return error;
}

/* Implement receipt of reply on a socket. We must search through the list of
 * received datagrams matching them with outstanding requests using the xid,
 * until ours is found.
 */

static int rpcclnt_reply(FAR struct rpcclnt *rpc, int procid, int prog,
                         void *reply, size_t resplen)
{
  FAR struct rpc_reply_header *replyheader;
  uint32_t rxid;
  int error;
  int count;

  /* Loop around until we get our own reply */

  for (count = 0; count < 9; count++)
    {
      /* Get the next RPC reply off the socket */

      error = rpcclnt_receive(rpc, rpc->rc_name, procid, prog, reply, resplen);
      if (error != 0)
        {
          fdbg("ERROR: rpcclnt_receive returned: %d\n");

          /* Ignore non-fatal errors and try again */

          if (error != EINTR && error != ERESTART && error != EWOULDBLOCK)
            {
              fdbg("       Ignoring routing error\n");
              continue;
            }

          return error;
        }

      /* Get the xid and check that it is an RPC replysvr */

      replyheader = (FAR struct rpc_reply_header *)reply;
      rxid       = replyheader->rp_xid;
 
      if (replyheader->rp_direction != rpc_reply)
        {
          rpc_statistics(rpcinvalid);
          continue;
        }

      return OK;
    }

  /* Here if we tried to receive the response 9 times.  If we failed
   * because of a timeout, then try sending the CALL message again.
   */

  if (error == EAGAIN || error == ETIMEDOUT)
   {
      rpc->rc_callflags |= RPCCALL_MUSTRESEND;
   }
  return error;
}

/* Get a new (non-zero) xid */

static uint32_t rpcclnt_newxid(void)
{
  static uint32_t rpcclnt_xid = 0;
  static uint32_t rpcclnt_xid_touched = 0;
  int xidp = 0;

  srand(time(NULL));
  if ((rpcclnt_xid == 0) && (rpcclnt_xid_touched == 0))
    {
      rpcclnt_xid = rand();
      rpcclnt_xid_touched = 1;
    }
  else
    {
      do
        {
          xidp = rand();
        }
      while ((xidp % 256) == 0);

      rpcclnt_xid += xidp;
    }

  return rpcclnt_xid;
}

/* Format the common part of the call header */

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

#ifdef CONFIG_NFS_UNIX_AUTH
  ch->rpc_unix.stamp    = txdr_unsigned(1);
  ch->rpc_unix.hostname = 0;
  ch->rpc_unix.uid      = setuid;
  ch->rpc_unix.gid      = setgid;
  ch->rpc_unix.gidlist  = 0;
#endif

  /* rpc_verf part (auth_null) */

  ch->rpc_verf.authtype  = rpc_auth_null;
  ch->rpc_verf.authlen   = 0;
}

/****************************************************************************
 * Public Functions
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

/* Initialize sockets and congestion for a new RPC connection. We do not free
 * the sockaddr if error.
 */

int rpcclnt_connect(struct rpcclnt *rpc)
{
  struct socket *so;
  int error;
  struct sockaddr *saddr;
  struct sockaddr_in sin;
  struct sockaddr_in *sa;
  struct rpc_call_pmap sdata;
  struct rpc_call_mount mountd;
  struct rpc_reply_pmap rdata;
  struct rpc_reply_mount mdata;
  struct timeval tv;
  uint16_t tport;
  int errval;

  fvdbg("Connecting\n");

  /* Create the socket */

  saddr = rpc->rc_name;
  memset(&sin, 0, sizeof(sin));

  /* Create an instance of the socket state structure */

  so = (struct socket *)kzalloc(sizeof(struct socket));
  if (!so)
    {
      fdbg("ERROR: Failed to allocate socket structure\n");
      return ENOMEM;
    }

  error = psock_socket(saddr->sa_family, rpc->rc_sotype, rpc->rc_soproto, so);
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

  memset(&sdata, 0, sizeof(struct rpc_call_pmap));
  memset(&rdata, 0, sizeof(struct rpc_reply_pmap));
  sdata.pmap.prog = txdr_unsigned(RPCPROG_MNT);
  sdata.pmap.vers = txdr_unsigned(RPCMNT_VER1);
  sdata.pmap.proc = txdr_unsigned(IPPROTO_UDP);
  sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  sa = (FAR struct sockaddr_in *)saddr;
  sa->sin_port = htons(fxdr_unsigned(uint32_t, rdata.pmap.port));

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_connect MOUNTD port failed: %d\n", errval);
      goto bad;
    }

  /* Do RPC to mountd. */

  memset(&mountd, 0, sizeof(struct rpc_call_mount));
  memset(&mdata, 0, sizeof(struct rpc_reply_mount));
  strncpy(mountd.mount.rpath, rpc->rc_path, 90);
  mountd.mount.len =  txdr_unsigned(sizeof(mountd.mount.rpath));

  error = rpcclnt_request(rpc, RPCMNT_MOUNT, RPCPROG_MNT, RPCMNT_VER1,
                          (FAR void *)&mountd, sizeof(struct call_args_mount),
                          (FAR void *)&mdata, sizeof(struct rpc_reply_mount));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  error = fxdr_unsigned(uint32_t, mdata.mount.status);
  if (error != 0)
    {
      fdbg("ERROR: Bad mount status: %d\n", error);
      goto bad;
    }

  memcpy(&rpc->rc_fh, &mdata.mount.fhandle, sizeof(nfsfh_t));

  /* Do the RPC to get a dynamic bounding with the server using PMAP.
   * NFS port in the socket.
   */

  memset(&sdata, 0, sizeof(struct rpc_call_pmap));
  memset(&rdata, 0, sizeof(struct rpc_reply_pmap));
  sa->sin_port = htons(PMAPPORT);

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error < 0)
    {
      errval = errno;
      fdbg("ERROR: psock_connect PMAP port failed: %d\n", errval);
      goto bad;
    }

  sdata.pmap.prog = txdr_unsigned(NFS_PROG);
  sdata.pmap.vers = txdr_unsigned(NFS_VER3);
  sdata.pmap.proc = txdr_unsigned(IPPROTO_UDP);
  sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  sa->sin_port = htons(fxdr_unsigned(uint32_t, rdata.pmap.port));

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

/* Reconnect routine: Called when a connection is broken on a reliable
 * protocol. - clean up the old socket - nfs_connect() again - set
 * RPCCALL_MUSTRESEND for all outstanding requests on mount point If this
 * fails the mount point is DEAD!
 */

#ifdef CONFIG_NFS_TCPIP
int rpcclnt_reconnect(FAR struct rpcclnt *rpc)
{
  int error;

  rpcclnt_disconnect(rpc);
  do
    {
      error = rpcclnt_connect(rpc);
      if (error != OK)
        {
          fdbg("ERROR: rpcclnt_connect failed: %d\n", error);
          if (error == EINTR || error == ERESTART)
            {
              return EINTR;
            }
        }
    }
  while (error != OK)
  return OK;
}
#endif

void rpcclnt_disconnect(struct rpcclnt *rpc)
{
  struct socket *so;

  if (rpc->rc_so != NULL)
    {
      so = rpc->rc_so;
      (void)psock_close(so);
    }
}

int rpcclnt_umount(struct rpcclnt *rpc)
{
  struct sockaddr *saddr;
  struct sockaddr_in *sa;
  struct rpc_call_pmap sdata;
  struct rpc_reply_pmap rdata;
  struct rpc_call_mount mountd;
  struct rpc_reply_mount mdata;
  int error;

  saddr = rpc->rc_name;
  sa = (FAR struct sockaddr_in *)saddr;

  /* Do the RPC to get a dynamic bounding with the server using ppmap.
   * Get port number for MOUNTD.
   */

  memset(&sdata, 0, sizeof(struct rpc_call_pmap));
  memset(&rdata, 0, sizeof(struct rpc_reply_pmap));
  sa->sin_port = htons(PMAPPORT);

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error)
    {
      fdbg("psock_connect MOUNTD port returns %d\n", error);
      goto bad;
    }

  sdata.pmap.prog = txdr_unsigned(RPCPROG_MNT);
  sdata.pmap.vers = txdr_unsigned(RPCMNT_VER1);
  sdata.pmap.proc = txdr_unsigned(IPPROTO_UDP);
  sdata.pmap.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                          (FAR void *)&sdata, sizeof(struct call_args_pmap),
                          (FAR void *)&rdata, sizeof(struct rpc_reply_pmap));
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

  sa->sin_port = htons(fxdr_unsigned(uint32_t, rdata.pmap.port));

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error)
    {
      fdbg("psock_connect MOUNTD port returns %d\n", error);
      goto bad;
    }

  /* Do RPC to umountd. */

  memset(&mountd, 0, sizeof(struct rpc_call_mount));
  memset(&mdata, 0, sizeof(struct rpc_reply_mount));

  strncpy(mountd.mount.rpath, rpc->rc_path, 92);
  mountd.mount.len =  txdr_unsigned(sizeof(mountd.mount.rpath));

  error = rpcclnt_request(rpc, RPCMNT_UMOUNT, RPCPROG_MNT, RPCMNT_VER1,
                          (FAR void *)&mountd, sizeof(struct call_args_mount),
                          (FAR void *)&mdata, sizeof(struct rpc_reply_mount));
  if (error != 0)
    {
     fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      goto bad;
    }

 if ((fxdr_unsigned(uint32_t, mdata.mount.status)) != 0)
    {
      fdbg("error unmounting with the server %d\n", error);
      goto bad;
    }

  return OK;

bad:
  rpcclnt_disconnect(rpc);
  return error;
}

/* Code from nfs_request - goes something like this - fill in task struct -
 * links task into list - calls nfs_send() for first transmit - calls
 * nfs_receive() to get reply - fills in reply (which should be initialized
 * prior to calling), which is valid when 0.
 *
 * Note that reply->result_* are invalid unless reply->type ==
 * RPC_MSGACCEPTED and reply->status == RPC_SUCCESS and that reply->verf_*
 * are invalid unless reply->type == RPC_MSGACCEPTED
 */

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
  while ((rpc->rc_callflags & RPCCALL_MUSTRESEND) != 0 && retries <= RPC_MAXREXMIT);

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
