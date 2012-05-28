/****************************************************************************
 * fs/nfs/rpc_clnt.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
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

#ifndef __FS_NFS_NFS_SOCKET_H
#define __FS_NFS_NFS_SOCKET_H

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

#define RPC_RETURN(X) do { nvdbg("returning %d\n", X); return X; } while(0)

/* Estimate rto for an nfs rpc sent via. an unreliable datagram. Use the mean
 * and mean deviation of rtt for the appropriate type of rpc for the frequent
 * rpcs and a default for the others. The justification for doing "other"
 * this way is that these rpcs happen so infrequently that timer est. would
 * probably be stale. Also, since many of these rpcs are non-idempotent, a
 * conservative timeout is desired. getattr, lookup - A+2D read, write     -
 * A+4D other           - nm_timeo
 */

#define RPC_RTO(n, t) \
        ((t) == 0 ? (n)->rc_timeo : \
         ((t) < 3 ? \
          (((((n)->rc_srtt[t-1] + 3) >> 2) + (n)->rc_sdrtt[t-1] + 1) >> 1) : \
          ((((n)->rc_srtt[t-1] + 7) >> 3) + (n)->rc_sdrtt[t-1] + 1)))

#define RPC_SRTT(s,r)   (r)->r_rpcclnt->rc_srtt[rpcclnt_proct((s),\
                                (r)->r_procnum) - 1]

#define RPC_SDRTT(s,r)  (r)->r_rpcclnt->rc_sdrtt[rpcclnt_proct((s),\
                                (r)->r_procnum) - 1]

/* There is a congestion window for outstanding rpcs maintained per mount
 * point. The cwnd size is adjusted in roughly the way that: Van Jacobson,
 * Congestion avoidance and Control, In "Proceedings of SIGCOMM '88". ACM,
 * August 1988. describes for TCP. The cwnd size is chopped in half on a
 * retransmit timeout and incremented by 1/cwnd when each rpc reply is
 * received and a full cwnd of rpcs is in progress. (The sent count and cwnd
 * are scaled for integer arith.) Variants of "slow start" were tried and
 * were found to be too much of a performance hit (ave. rtt 3 times larger),
 * I suspect due to the large rtt that nfs rpcs have.
 */

#define RPC_CWNDSCALE   256
#define RPC_MAXCWND     (RPC_CWNDSCALE * 32)

#define RPC_ERRSTR_ACCEPTED_SIZE 6
#define RPC_ERRSTR_AUTH_SIZE 6

/****************************************************************************
 * Public Data
 ****************************************************************************/

char *rpc_errstr_accepted[RPC_ERRSTR_ACCEPTED_SIZE] =
{
  "",                           /* no good message... */
  "remote server hasn't exported program.",
  "remote server can't support version number.",
  "program can't support procedure.",
  "procedure can't decode params.",
  "remote error.  remote side memory allocation failure?"
};

char *rpc_errstr_denied[2] =
{
  "remote server doesnt support rpc version 2!",
  "remote server authentication error."
};

char *rpc_errstr_auth[RPC_ERRSTR_AUTH_SIZE] =
{
  "",
  "auth error: bad credential (seal broken).",
  "auth error: client must begin new session.",
  "auth error: bad verifier (seal broken).",
  "auth error: verifier expired or replayed.",
  "auth error: rejected for security reasons.",
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

//static int rpcclnt_backoff[8] = { 2, 4, 8, 16, 32, 64, 128, 256, };

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

static uint32_t rpcclnt_xid = 0;
static uint32_t rpcclnt_xid_touched = 0;
int rpcclnt_ticks;
struct rpcstats rpcstats;
//struct rpc_call *callmgs;
//struct rpc_reply *replymsg;

/* Queue head for rpctask's */

static dq_queue_t rpctask_q;
//struct callout_handle rpcclnt_timer_handle;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpcclnt_send(struct socket *, struct sockaddr *, int, int, void *,
                        struct rpctask *);
static int rpcclnt_receive(struct rpctask *, struct sockaddr *, int, int,
                           void *);//, struct rpc_call *);
static int rpcclnt_reply(struct rpctask *, int, int, void *);
#ifdef CONFIG_NFS_TCPIP
static int rpcclnt_sndlock(int *, struct rpctask *);
static void rpcclnt_sndunlock(int *);
static int rpcclnt_rcvlock(struct rpctask *);
static void rpcclnt_rcvunlock(int *);
static int  rpcclnt_sigintr(struct rpcclnt *, struct rpctask *, cthread_t *);
#endif
#ifdef COMP
static void rpcclnt_softterm(struct rpctask *task);
static void rpcclnt_timer(void *, struct rpc_call *);
#endif
static uint32_t rpcclnt_proct(struct rpcclnt *, uint32_t);
static int rpcclnt_buildheader(struct rpcclnt *, int, int, int, struct xidr *, FAR const void *,
                               void *);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* This is the nfs send routine. For connection based socket types, it must
 * be called with an nfs_sndlock() on the socket. "rep == NULL" indicates
 * that it has been called from a server. For the client side: - return EINTR
 * if the RPC is terminated, 0 otherwise - set TASK_MUSTRESEND if the send fails
 * for any reason - do any cleanup required by recoverable socket errors
 * (???) For the server side: - return EINTR or ERESTART if interrupted by a
 * signal - return EPIPE if a connection is lost for connection based sockets
 * (TCP...) - do any cleanup required by recoverable socket errors (???)
 */

static int
rpcclnt_send(struct socket *so, struct sockaddr *nam, int procid, int prog,
             void *call, struct rpctask *rep)
{
  struct sockaddr *sendnam;
  int error = ESRCH;
#ifdef CONFIG_NFS_TCPIP
  int soflags;
#endif
  int flags;

  if (rep != NULL)
    {
      if (rep->r_flags & TASK_SOFTTERM)
        {
          RPC_RETURN(EINTR);
        }

      if ((so = rep->r_rpcclnt->rc_so) == NULL)
        {
          rep->r_flags |= TASK_MUSTRESEND;
          RPC_RETURN(0);
        }

      rep->r_flags &= ~TASK_MUSTRESEND;
#ifdef CONFIG_NFS_TCPIP
      soflags = rep->r_rpcclnt->rc_soflags;
#endif
    }
#ifdef CONFIG_NFS_TCPIP
  else
    {
      soflags = so->s_flags;
    }

  if ((soflags & PR_CONNREQUIRED))
    {
      sendnam = NULL;
    {
  else
#endif
    {
      sendnam = nam;
    }

  if (so->s_type == SOCK_SEQPACKET)
    {
      flags = MSG_EOR;
    }
  else
    {
      flags = 0;
    }

  if (prog == PMAPPROG)
    {
      if (procid == PMAPPROC_GETPORT)
        {
          struct rpc_call_pmap *callmsg = (struct rpc_call_pmap *)call;
          error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                               sendnam, sizeof(*sendnam));
        }
      else if (procid == PMAPPROC_UNSET)
        {
          struct rpc_call_pmap *callmsg = (struct rpc_call_pmap *)call;
          error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                               sendnam, sizeof(*sendnam));
        }
    }
  else if (prog == RPCPROG_MNT)
    {
      if (procid == RPCMNT_UMOUNT)
        {
          struct rpc_call_mount *callmsg = (struct rpc_call_mount *)call;
          error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                               sendnam, sizeof(*sendnam));
        }
      else if (procid == RPCMNT_MOUNT)
        {
          struct rpc_call_mount *callmsg = (struct rpc_call_mount *)call;
          error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                               sendnam, sizeof(*sendnam));
        }
    }
  else if (prog == NFS_PROG)
    {
      switch (procid)
        {
        case NFSPROC_CREATE:
          {
            struct rpc_call_create *callmsg = (struct rpc_call_create *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_READ:
          {
            struct rpc_call_read *callmsg = (struct rpc_call_read *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_WRITE:
          {
            struct rpc_call_write *callmsg = (struct rpc_call_write *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_READDIR:
          {
            struct rpc_call_readdir *callmsg = (struct rpc_call_readdir *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_FSSTAT:
          {
            struct rpc_call_fs *callmsg = (struct rpc_call_fs *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_REMOVE:
          {
            struct rpc_call_remove *callmsg  = (struct rpc_call_remove *)call;
                  error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                           sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_MKDIR:
          {
            struct rpc_call_mkdir *callmsg = (struct rpc_call_mkdir *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_RMDIR:
          {
            struct rpc_call_rmdir *callmsg  = (struct rpc_call_rmdir *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_RENAME:
          {
            struct rpc_call_rename *callmsg  = (struct rpc_call_rename *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        case NFSPROC_FSINFO:
          {
            struct rpc_call_fs *callmsg = (struct rpc_call_fs *)call;
            error = psock_sendto(so, callmsg, sizeof(*callmsg), flags,
                                 sendnam, sizeof(*sendnam));
          }
          break;

        default:
          break;
        }
    }

  if (error < 0)
    {
      if (rep != NULL)
        {
          ndbg("rpc send error %d for service %s\n", error,
               rep->r_rpcclnt->rc_prog->prog_name);

          /* Deal with errors for the client side. */

          if (rep->r_flags & TASK_SOFTTERM)
            {
              error = EINTR;
            }
          else
            {
              rep->r_flags |= TASK_MUSTRESEND;
            }
        }
      else
        {
          ndbg("rpc service send error %d\n", error);
        }

      RPC_RETURN(error);

    }
  else
    {
      RPC_RETURN(0);
    }
}

/* Receive a Sun RPC Request/Reply. For SOCK_DGRAM, the work is all
 * done by soreceive().For SOCK_STREAM, first get the
 * Record Mark to find out how much more there is to get. We must
 * lock the socket against other receivers until we have an entire
 * rpc request/reply.
 */

static int rpcclnt_receive(struct rpctask *rep, struct sockaddr *aname,
                           int proc, int program, void *reply)//, struct rpc_call *call)
{
  struct socket *so;
#ifdef CONFIG_NFS_TCPIP
  uint32_t len;
  int sotype;
#endif
  int error = 0;
  int rcvflg;

#ifdef CONFIG_NFS_TCPIP
  /* Set up arguments for soreceive() */

  sotype = rep->r_rpcclnt->rc_sotype;

  /* For reliable protocols, lock against other senders/receivers in
   * case a reconnect is necessary. For SOCK_STREAM, first get the
   * Record Mark to find out how much more there is to get. We must
   * lock the socket against other receivers until we have an entire
   * rpc request/reply.
   */

  if (sotype != SOCK_DGRAM)
    {
      error = rpcclnt_sndlock(&rep->r_rpcclnt->rc_flag, rep);
      if (error != 0)
        {
          return error;
        }

    tryagain:

      /* Check for fatal errors and resending request.
       *
       * Ugh: If a reconnect attempt just happened, rc_so would
       * have changed. NULL indicates a failed attempt that has
       * essentially shut down this mount point.
       */

      if (rep->r_flags & TASK_SOFTTERM)
        {
          rpcclnt_sndunlock(&rep->r_rpcclnt->rc_flag);
          return EINTR;
        }

      so = rep->r_rpcclnt->rc_so;
      if (so == NULL)
        {
          error = rpcclnt_reconnect(rep);
          if (error)
            {
              rpcclnt_sndunlock(&rep->r_rpcclnt->rc_flag);
              return error;
            }

          goto tryagain;
        }
      while (rep->r_flags & TASK_MUSTRESEND)
        {
          rpcstats.rpcretries++;
          error = rpcclnt_send(so, rep->r_rpcclnt->rc_name, call, rep);
          if (error)
            {
              if (error == EINTR || error == ERESTART ||
                  (error = rpcclnt_reconnect(rep)) != 0)
                {
                  rpcclnt_sndunlock(&rep->r_rpcclnt->rc_flag);
                  return error;
                }

              goto tryagain;
            }
        }

      rpcclnt_sndunlock(&rep->r_rpcclnt->rc_flag);
      if (sotype == SOCK_STREAM)
        {
          do
            {
              socklen_t fromlen = sizeof(*rep->r_rpcclnt->rc_name)
              rcvflg = MSG_WAITALL;
              error = psock_recvfrom(so, reply, sizeof(*reply),
                                     &rcvflg, rep->r_rpcclnt->rc_name,
                                     &fromlen);
              if (error == EWOULDBLOCK && rep && (rep->r_flags & TASK_SOFTTERM))
                {
                  RPC_RETURN(EINTR);
                }
            }
          while (error == EWOULDBLOCK);

          if (error == 0)
            {
              ndbg("short receive from rpc server %s\n",
                   rep->r_rpcclnt->rc_prog->prog_name);
              error = EPIPE;
            }

          len = ntohl(len) & ~0x80000000;

          /* This is SERIOUS! We are out of sync with the
           * sender and forcing a disconnect/reconnect is all I
           * can do.
           */

          if (len > RPC_MAXPACKET)
            {
              ndbg("%s (%d) from rpc server %s\n",
                   "impossible packet length",
                   len, rep->r_rpcclnt->rc_prog->prog_name);
              error = EFBIG;
              goto errout;
            }
          do
            {
              socklen_t fromlen = sizeof(*rep->r_rpcclnt->rc_name);
              rcvflg = MSG_WAITALL;
              error = psock_recvfrom(so, reply, sizeof(*reply),
                                     &rcvflg, rep->r_rpcclnt->rc_name,
                                     &fromlen);
            }
          while (error == EWOULDBLOCK || error == EINTR || error == ERESTART);

          if (error == 0)
            {
              ndbg("short receive from rpc server %s\n",
                   rep->r_rpcclnt->rc_prog->prog_name);
              error = EPIPE;
            }

          if (error != 0)
            {
              goto errout;
            }
        }
      else
        {
          /* NB: Since uio_resid is big, MSG_WAITALL is ignored
           * and soreceive() will return when it has either a
           * control msg or a data msg. We have no use for
           * control msg., but must grab them and then throw
           * them away so we know what is going on.
           */

          do
            {
              socklen_t fromlen = sizeof(*rep->r_rpcclnt->rc_name);
              rcvflg = 0;
              error = psock_recvfrom(so, reply, sizeof(*reply), &rcvflg,
                                     rep->r_rpcclnt->rc_name, &fromlen);
              if (error == EWOULDBLOCK && rep)
                {
                  if (rep->r_flags & TASK_SOFTTERM)
                    {
                      return EINTR;
                    }
                }
            }
          while (error == EWOULDBLOCK || (!error));

          if ((rcvflg & MSG_EOR) == 0)
            {
              ndbg("Egad!!\n");
            }

          if (error == 0)
            {
              error = EPIPE;
            }
        }

    errout:
      if (error != 0 && error != EINTR && error != ERESTART)
        {
          if (error != EPIPE)
            {
              ndbg("receive error %d from rpc server %s\n",
                   error, rep->r_rpcclnt->rc_prog->prog_name);
            }

          error = rpcclnt_sndlock(&rep->r_rpcclnt->rc_flag, rep);
          if (error == 0)
            {
              error = rpcclnt_reconnect(rep);
            }

          if (error == 0)
            {
              goto tryagain;
            }
        }
    }
  else
#endif
    {
      so = rep->r_rpcclnt->rc_so;
      if (so == NULL)
        {
          RPC_RETURN(EACCES);
        }

      socklen_t fromlen = sizeof(*aname);
      rcvflg = 0;

      if (program == PMAPPROG)
        {
          if (proc == PMAPPROC_GETPORT)
            {
              struct rpc_reply_pmap *replymsg = (struct rpc_reply_pmap *)reply;
              error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
            }

          else if (proc == PMAPPROC_UNSET)
            {
             struct rpc_reply_pmap *replymsg = (struct rpc_reply_pmap *)reply;
             error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
            }
        }

      else if (program == RPCPROG_MNT)
        {
          if (proc== RPCMNT_UMOUNT)
            {
              struct rpc_reply_mount *replymsg = (struct rpc_reply_mount *)reply;
              error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
            }
          else if (proc == RPCMNT_MOUNT)
            {
              struct rpc_reply_mount *replymsg = (struct rpc_reply_mount *)reply;
              error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
            }
        }

      else if (program == NFS_PROG)
        {
          switch (proc)
            {
            case NFSPROC_CREATE:
              {
                struct rpc_reply_create *replymsg = (struct rpc_reply_create *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_READ:
              {
                struct rpc_reply_read *replymsg = (struct rpc_reply_read *)reply;
               error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_WRITE:
              {
                struct rpc_reply_write *replymsg = (struct rpc_reply_write *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_READDIR:
              {
                struct rpc_reply_readdir *replymsg = (struct rpc_reply_readdir *)reply;
               error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_FSSTAT:
              {
                struct rpc_reply_fsstat *replymsg = (struct rpc_reply_fsstat *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_REMOVE:
              {
                struct rpc_reply_remove *replymsg  = (struct rpc_reply_remove *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_MKDIR:
              {
                struct rpc_reply_mkdir *replymsg = (struct rpc_reply_mkdir *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_RMDIR:
              {
                struct rpc_reply_rmdir *replymsg  = (struct rpc_reply_rmdir *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_RENAME:
              {
                struct rpc_reply_rename *replymsg  = (struct rpc_reply_rename *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            case NFSPROC_FSINFO:
              {
                struct rpc_reply_fsinfo *replymsg = (struct rpc_reply_fsinfo *)reply;
                error = psock_recvfrom(so, replymsg, sizeof(*replymsg), rcvflg,
                                     aname, &fromlen);
              }
              break;

            default:
              break;
            }
        }

      nvdbg("psock_recvfrom returns %d\n", error);
      if (error > 0)
        {
          RPC_RETURN(0);
        }
    }

  RPC_RETURN(ENONET);
}

/* Implement receipt of reply on a socket. We must search through the list of
 * received datagrams matching them with outstanding requests using the xid,
 * until ours is found.
 */

static int rpcclnt_reply(struct rpctask *myrep, int procid, int prog, void *reply)
{
  struct rpctask *rep;
  struct rpc_reply_header replyheader;
  struct rpcclnt *rpc = myrep->r_rpcclnt;
  int32_t t1;
  uint32_t rxid;
  int error;
  int count;

  /* Loop around until we get our own reply */

  for (count = 0; count < 9; count++)
    {
      /* Lock against other receivers so that I don't get stuck in
       * sbwait() after someone else has received my reply for me.
       * Also necessary for connection based protocols to avoid
       * race conditions during a reconnect.
       */

#ifdef CONFIG_NFS_TCPIP
      error = rpcclnt_rcvlock(myrep);
      if (error)
        {
          return error;
        }
#endif
      /* Get the next Rpc reply off the socket */

      error = rpcclnt_receive(myrep, rpc->rc_name, procid, prog, reply);
#ifdef CONFIG_NFS_TCPIP
      rpcclnt_rcvunlock(&rpc->rc_flag);
#endif

      if (error != 0)
        {
          /* Ignore routing errors on connectionless
           * protocols??
           */

          if (RPCIGNORE_SOERROR(rpc->rc_soflags, error))
            {
              if (myrep->r_flags & TASK_GETONEREP)
                {
                  RPC_RETURN(0);
                }

              ndbg("ignoring routing error on connectionless protocol.");
              continue;
            }
          RPC_RETURN(error);
        }

      bcopy(reply, &replyheader, sizeof(struct rpc_reply_header));

      /* Get the xid and check that it is an rpc replysvr */

      rxid = replyheader.rp_xid;
      if (replyheader.rp_direction != rpc_reply)
        {
          rpcstats.rpcinvalid++;
          if (myrep->r_flags & TASK_GETONEREP)
            {
              RPC_RETURN(0);
            }

          continue;
        }

      /* Loop through the request list to match up the reply Iff no
       * match, just drop the datagram
       */

      for (rep = (struct rpctask *)rpctask_q.head; rep != NULL;
           rep = (struct rpctask *)rep->r_chain.flink)
        {
          if (rxid == rep->r_xid)
            {
              /* Update congestion window. Do the additive
               * increase of one rpc/rtt.
               */

              if (rpc->rc_cwnd <= rpc->rc_sent)
                {
                  rpc->rc_cwnd +=
                    (RPC_CWNDSCALE * RPC_CWNDSCALE +
                     (rpc->rc_cwnd >> 1)) / rpc->rc_cwnd;

                  if (rpc->rc_cwnd > RPC_MAXCWND)
                    {
                      rpc->rc_cwnd = RPC_MAXCWND;
                    }
                }

              rep->r_flags &= ~TASK_SENT;
              rpc->rc_sent -= RPC_CWNDSCALE;

              /* Update rtt using a gain of 0.125 on the
               * mean and a gain of 0.25 on the deviation.
               */

              if (rep->r_flags & TASK_TIMING)
                {
                  /* Since the timer resolution of
                   * NFS_HZ is so course, it can often
                   * result in r_rtt == 0. Since r_rtt
                   * == N means that the actual rtt is
                   * between N+dt and N+2-dt ticks, add
                   * 1.
                   */

                  t1 = rep->r_rtt + 1;
                  t1 -= (RPC_SRTT(rpc, rep) >> 3);
                  RPC_SRTT(rpc, rep) += t1;
                  if (t1 < 0)
                    {
                      t1 = -t1;
                    }

                  t1 -= (RPC_SDRTT(rpc, rep) >> 2);
                  RPC_SDRTT(rpc, rep) += t1;
                }

              rpc->rc_timeouts = 0;
              break;
            }
        }

      /* If not matched to a request, drop it. If it's mine, get
       * out.
       */

      if (rep == 0)
        {
          ndbg("rpc reply not matched\n");
          rpcstats.rpcunexpected++;
          RPC_RETURN(ENOMSG);
        }
      else if (rep == myrep)
        {
          RPC_RETURN(0);
        }

      if (myrep->r_flags & TASK_GETONEREP)
        {
          RPC_RETURN(0);
        }
    }

  RPC_RETURN(ENONET);
}

#ifdef CONFIG_NFS_TCPIP
static int
rpcclnt_sigintr( struct rpcclnt *rpc, struct rpctask *task, cthread_t *td)
{
  struct proc *p;
  sigset_t     tmpset;

  if (rpc == NULL)
    {
      return EFAULT;
    }

  if (ISSET(rpc->rc_flag, RPCCLNT_REDIRECT))
    {
      return 0;
    }

  if (task && ISSET(task->r_flags, TASK_SOFTTERM))
    {
      RPC_RETURN(EINTR);
    }

  if (!ISSET(rpc->rc_flag, RPCCLNT_INT))
    {
      RPC_RETURN(0);
    }

  if (td == NULL)
    {
      return 0;
    }

  p = cthread_proc(td);

  PROC_LOCK(p);
  tmpset = p->p_siglist;
  SIGSETNAND(tmpset, td->td_sigmask);
  mtx_lock(&p->p_sigacts->ps_mtx);
  SIGSETNAND(tmpset, p->p_sigacts->ps_sigignore);
  mtx_unlock(&p->p_sigacts->ps_mtx);

  if (SIGNOTEMPTY(p->p_siglist) && RPCCLNTINT_SIGMASK(tmpset))
    {
      PROC_UNLOCK(p);
      RPC_RETURN(EINTR);
    }

  PROC_UNLOCK(p);
  RPC_RETURN(0);
}

/* Lock a socket against others. Necessary for STREAM sockets to ensure you
 * get an entire rpc request/reply and also to avoid race conditions between
 * the processes with nfs requests in progress when a reconnect is necessary.
 */

static int rpcclnt_sndlock(int *flagp, struct rpctask *task)
{
  int slpflag = 0, slptimeo = 0;

  if (task)
    {
      if (task->r_rpcclnt->rc_flag & RPCCLNT_INT)
        slpflag = PCATCH;
    }

  while (*flagp & RPCCLNT_SNDLOCK)
    {
      if (rpcclnt_sigintr(task->r_rpcclnt, task, p))
        {
          return EINTR;
        }

      *flagp |= RPCCLNT_WANTSND;
      if (slpflag == PCATCH)
        {
          slpflag = 0;
          slptimeo = 2 * CLOCKS_PER_SEC;
        }
    }

  *flagp |= RPCCLNT_SNDLOCK;
  return 0;
}

/* Unlock the stream socket for others. */

static void rpcclnt_sndunlock(int *flagp)
{
  if ((*flagp & RPCCLNT_SNDLOCK) == 0)
    {
      panic("rpc sndunlock");
    }

  *flagp &= ~RPCCLNT_SNDLOCK;
  if (*flagp & RPCCLNT_WANTSND)
    {
      *flagp &= ~RPCCLNT_WANTSND;
    }
}

static int rpcclnt_rcvlock(struct rpctask *task)
{
  int *flagp = &task->r_rpcclnt->rc_flag;
  int slpflag, slptimeo = 0;

  if (*flagp & RPCCLNT_INT)
    {
      slpflag = PCATCH;
    }
  else
    {
      slpflag = 0;
    }

  while (*flagp & RPCCLNT_RCVLOCK)
    {
      if (rpcclnt_sigintr(task->r_rpcclnt, task, task->r_td))
        {
          return EINTR;
        }

      *flagp |= RPCCLNT_WANTRCV;
      tsleep((caddr_t) flagp, slpflag | (PZERO - 1), "rpcrcvlk", slptimeo);
      if (slpflag == PCATCH)
        {
          slpflag = 0;
          slptimeo = 2 * CLOCKS_PER_SEC;
        }
    }

  *flagp |= RPCCLNT_RCVLOCK;
  return 0;
}

/* Unlock the stream socket for others. */

static void rpcclnt_rcvunlock(int *flagp)
{
  if ((*flagp & RPCCLNT_RCVLOCK) == 0)
    {
      panic("nfs rcvunlock");
    }

  *flagp &= ~RPCCLNT_RCVLOCK;
  if (*flagp & RPCCLNT_WANTRCV)
    {
      *flagp &= ~RPCCLNT_WANTRCV;
      wakeup((caddr_t) flagp);
    }
}
#endif

static uint32_t rpcclnt_proct(struct rpcclnt *rpc, uint32_t procid)
{
  if (rpc->rc_proctlen != 0 && rpc->rc_proct != NULL &&
      procid < rpc->rc_proctlen)
    {
      return rpc->rc_proct[procid];
    }

  return 0;
}

#ifdef COMP
static void rpcclnt_softterm(struct rpctask *task)
{
  task->r_flags |= TASK_SOFTTERM;
  if (task->r_flags & TASK_SENT)
    {
      task->r_rpcclnt->rc_sent -= RPC_CWNDSCALE;
      task->r_flags &= ~TASK_SENT;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rpcclnt_init(void)
{
  rpcclnt_ticks = (CLOCKS_PER_SEC * RPC_TICKINTVL + 500) / 1000;
  if (rpcclnt_ticks < 1)
    {
      rpcclnt_ticks = 1;
    }

  rpcstats.rpcretries = 0;
  rpcstats.rpcrequests = 0;
  rpcstats.rpctimeouts = 0;
  rpcstats.rpcunexpected = 0;
  rpcstats.rpcinvalid = 0;

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

  /* Initialize rpctask queue */

  dq_init(&rpctask_q);

  //rpcclnt_timer(NULL, callmgs);

  nvdbg("rpc initialized\n");
  return;
}

/*
void
rpcclnt_uninit(void)
{
  nvdbg("uninit");
  untimeout(rpcclnt_timer, (void *)NULL, rpcclnt_timer_handle);
}
*/

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
  struct call_args_pmap sdata;
  struct call_args_mount mountd;
  struct rpc_reply_pmap rdata;
  struct rpc_reply_mount mdata;
  struct timeval tv;
  uint16_t tport;

  /* Create the socket */

  saddr = rpc->rc_name;
  memset(&sin, 0, sizeof(sin));

  /* Create an instance of the socket state structure */

  so = (struct socket *)kzalloc(sizeof(struct socket));
  if (!so)
    {
      ndbg("Failed to allocate socket structure\n");
      return -ENOMEM;
    }

  error = psock_socket(saddr->sa_family, rpc->rc_sotype, rpc->rc_soproto, so);
  if (error != 0)
    {
      ndbg("error %d in psock_socket()", error);
      RPC_RETURN(error);
    }

  so->s_crefs = 1;
  rpc->rc_so = so;
  rpc->rc_soflags = so->s_flags;

   /* Always set receive timeout to detect server crash and reconnect.
    * Otherwise, we can get stuck in psock_receive forever.
    */

  tv.tv_sec = 1;
  tv.tv_usec = 0;

  error = psock_setsockopt(rpc->rc_so, SOL_SOCKET, SO_RCVTIMEO,
                          (const void *)&tv, sizeof(tv));
  if (error != 0)
    {
      goto bad;
    }

  /* Some servers require that the client port be a reserved port
   * number. We always allocate a reserved port, as this prevents
   * filehandle disclosure through UDP port capture.
   */

  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = INADDR_ANY;
  tport = 1024;

  do
    {
      tport--;
      sin.sin_port = htons(tport);
      error = psock_bind(rpc->rc_so, (struct sockaddr *)&sin, sizeof(sin));
    }
  while (error == EADDRINUSE && tport > 1024 / 2);

  if (error)
    {
      ndbg("bind failed\n");
      goto bad;
    }

  /* Protocols that do not require connections may be optionally left
   * unconnected for servers that reply from a port other than
   * NFS_PORT.
   */

#ifdef CONFIG_NFS_TCPIP
  if (rpc->rc_soflags == PR_CONNREQUIRED)
    {
      error = ENOTCONN;
      goto bad;
    }
  else
#endif
    {
      error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));

      if (error)
        {
          ndbg("psock_connect to ppmap port returns %d", error);
          goto bad;
        }

      /* Do the RPC to get a dynamic bounding with the server using ppmap.
       * Get port number for MOUNTD.
       */

      memset(&sdata, 0, sizeof(sdata));
      memset(&rdata, 0, sizeof(rdata));
      sdata.prog = txdr_unsigned(RPCPROG_MNT);
      sdata.vers = txdr_unsigned(RPCMNT_VER1);
      sdata.proc = txdr_unsigned(IPPROTO_UDP);
      sdata.port = 0;

      error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                              (void *)&rdata, (FAR const void *)&sdata);
      if (error != 0)
        {
          goto bad;
        }

      sa = (FAR struct sockaddr_in *)saddr;
      sa->sin_port = htons(fxdr_unsigned(uint32_t, rdata.pmap.port));

      error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
      if (error)
        {
          ndbg("psock_connect MOUNTD port returns %d\n", error);
          goto bad;
        }

      /* Do RPC to mountd. */

      memset(&mountd, 0, sizeof(mountd));
      memset(&mdata, 0, sizeof(mdata));
      strncpy(mountd.rpath, rpc->rc_path, 90);
      mountd.len =  txdr_unsigned(sizeof(mountd.rpath));

      error = rpcclnt_request(rpc, RPCMNT_MOUNT, RPCPROG_MNT, RPCMNT_VER1,
                              (void *)&mdata, (FAR const void *)&mountd);
      if (error != 0)
        {
          goto bad;
        }

      if ((fxdr_unsigned(uint32_t, mdata.mount.status)) != 0)
        {
          ndbg("error mounting with the server %d\n", error);
          goto bad;
        }

      rpc->rc_fh = mdata.mount.fhandle;

      /* NFS port in the socket*/

      sa->sin_port = htons(NFS_PORT);
      error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
      if (error)
        {
          ndbg("psock_connect NFS port returns %d\n", error);
          goto bad;
        }
    }

  /* Initialize other non-zero congestion variables */

  rpc->rc_srtt[0] = rpc->rc_srtt[1] = rpc->rc_srtt[2] = rpc->rc_srtt[3] = (RPC_TIMEO << 3);
  rpc->rc_sdrtt[0] = rpc->rc_sdrtt[1] = rpc->rc_sdrtt[2] = rpc->rc_sdrtt[3] = 0;
  rpc->rc_cwnd = RPC_MAXCWND / 2;       /* Initial send window */
  rpc->rc_sent = 0;
  rpc->rc_timeouts = 0;

  RPC_RETURN(0);

bad:
  rpcclnt_disconnect(rpc);
  RPC_RETURN(error);
}

/* Reconnect routine: Called when a connection is broken on a reliable
 * protocol. - clean up the old socket - nfs_connect() again - set
 * TASK_MUSTRESEND for all outstanding requests on mount point If this
 * fails the mount point is DEAD! nb: Must be called with the
 * nfs_sndlock() set on the mount point.
 */

#ifdef CONFIG_NFS_TCPIP
int rpcclnt_reconnect(struct rpctask *rep)
{
  struct rpctask *rp;
  struct rpcclnt *rpc = rep->r_rpcclnt;
  int error;

  rpcclnt_disconnect(rpc);
  while ((error = rpcclnt_connect(rpc)) != 0)
    {
      if (error == EINTR || error == ERESTART)
        {
          return EINTR;
        }
    }

  /* Loop through outstanding request list and fix up all
   * requests on old socket.
   */

  for (rp = (struct rpctask *)rpctask_q->head; rp != NULL;
       rp = (struct rpctask *)rp->r_chain.blink)
    {
      if (rp->r_rpcclnt == rpc)
        {
          rp->r_flags |= TASK_MUSTRESEND;
        }
    }

  return 0;
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
  struct call_args_pmap sdata;
  struct rpc_reply_pmap rdata;
  struct call_args_mount mountd;
  struct rpc_reply_mount mdata;
  int error;

  saddr = rpc->rc_name;
  sa = (FAR struct sockaddr_in *)saddr;

  /* Do the RPC to get a dynamic bounding with the server using ppmap.
   * Get port number for MOUNTD.
   */

  memset(&sdata, 0, sizeof(sdata));
  memset(&rdata, 0, sizeof(rdata));
  sa->sin_port = htons(PMAPPORT);

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error)
    {
      ndbg("psock_connect MOUNTD port returns %d\n", error);
      goto bad;
    }

  sdata.prog = txdr_unsigned(RPCPROG_MNT);
  sdata.vers = txdr_unsigned(RPCMNT_VER1);
  sdata.proc = txdr_unsigned(IPPROTO_UDP);
  sdata.port = 0;

  error = rpcclnt_request(rpc, PMAPPROC_GETPORT, PMAPPROG, PMAPVERS,
                           (void *)&rdata, (FAR const void *)&sdata);
  if (error != 0)
    {
      goto bad;
    }

  sa->sin_port = htons(fxdr_unsigned(uint32_t, rdata.pmap.port));

  error = psock_connect(rpc->rc_so, saddr, sizeof(*saddr));
  if (error)
    {
      ndbg("psock_connect MOUNTD port returns %d\n", error);
      goto bad;
    }

  /* Do RPC to umountd. */

  memset(&mountd, 0, sizeof(mountd));
  memset(&mdata, 0, sizeof(mdata));

  strncpy(mountd.rpath, rpc->rc_path, 92);
  mountd.len =  txdr_unsigned(sizeof(mountd.rpath));

  error = rpcclnt_request(rpc, RPCMNT_UMOUNT, RPCPROG_MNT, RPCMNT_VER1,
                          (void *)&mdata, (FAR const void *)&mountd);
  if (error != 0)
    {
      goto bad;
    }

 if ((fxdr_unsigned(uint32_t, mdata.mount.status)) != 0)
    {
      ndbg("error unmounting with the server %d\n", error);
      goto bad;
    }

  RPC_RETURN(0);

bad:
  rpcclnt_disconnect(rpc);
  RPC_RETURN(error);
}

#ifdef CONFIG_NFS_TCPIP
void rpcclnt_safedisconnect(struct rpcclnt *rpc)
{
  struct rpctask dummytask;

  memset((void *)dummytask, 0, sizeof(*call));
  dummytask.r_rpcclnt = rpc;
  rpcclnt_rcvlock(&dummytask);
  rpcclnt_disconnect(rpc);
  rpcclnt_rcvunlock(&rpc->rc_flag);
}
#endif

/* Code from nfs_request - goes something like this - fill in task struct -
 * links task into list - calls nfs_send() for first transmit - calls
 * nfs_receive() to get reply - fills in reply (which should be initialized
 * prior to calling), which is valid when 0.
 *
 * note that reply->result_* are invalid unless reply->type ==
 * RPC_MSGACCEPTED and reply->status == RPC_SUCCESS and that reply->verf_*
 * are invalid unless reply->type == RPC_MSGACCEPTED
 */

int rpcclnt_request(struct rpcclnt *rpc, int procnum, int prog, int version,
                    void *dataout, FAR const void *datain)
{
  struct rpc_reply_header replymgs;
  struct rpc_reply_header replyheader;
  struct rpctask *task = NULL;
  struct xidr value;
  struct rpc_call_pmap pmap;
  struct rpc_call_mount mountd;
  struct rpc_call_create create;
  struct rpc_call_write write;
  struct rpc_call_read read;
  struct rpc_call_remove removef;
  struct rpc_call_rename renamef;
  struct rpc_call_mkdir mkdir;
  struct rpc_call_rmdir rmdir;
  struct rpc_call_readdir readdir;
  struct rpc_call_fs fs;
  void *msgcall = NULL;
  int error = 0;

//memset(&replyheader, 0, sizeof(replyheader));

  if (prog == PMAPPROG)
    {
      if (procnum == PMAPPROC_GETPORT)
        {
          memset(&pmap, 0, sizeof(struct rpc_call_pmap));
          msgcall = &pmap;
        }
      else if (procnum == PMAPPROC_UNSET)
        {
          memset(&pmap, 0, sizeof(struct rpc_call_pmap));
          msgcall = &pmap;
        }
    }
  else if (prog == RPCPROG_MNT)
    {
      if (procnum == RPCMNT_UMOUNT)
        {
          memset(&mountd, 0, sizeof(struct rpc_call_mount));
          msgcall = &mountd;
        }
      else if (procnum == RPCMNT_MOUNT)
        {
          memset(&mountd, 0, sizeof(struct rpc_call_mount));
          msgcall = &mountd;
        }
    }
  else if (prog == NFS_PROG)
    {
      switch (procnum)
        {
        case NFSPROC_CREATE:
          {
            memset(&create, 0, sizeof(struct rpc_call_create));
            msgcall = &create;
          }
          break;

        case NFSPROC_READ:
          {
            memset(&read, 0, sizeof(struct rpc_call_read));
            msgcall = &read;
          }
          break;

        case NFSPROC_WRITE:
          {
            memset(&write, 0, sizeof(struct rpc_call_write));
            msgcall = &write;
          }
          break;

        case NFSPROC_RENAME:
          {
            memset(&renamef, 0, sizeof(struct rpc_call_rename));
            msgcall = &renamef;
          }
          break;

        case NFSPROC_REMOVE:
          {
            memset(&removef, 0, sizeof(struct rpc_call_remove));
            msgcall = &removef;
          }
          break;

        case NFSPROC_MKDIR:
          {
            memset(&mkdir, 0, sizeof(struct rpc_call_mkdir));
            msgcall = &mkdir;
          }
          break;

        case NFSPROC_RMDIR:
          {
            memset(&rmdir, 0, sizeof(struct rpc_call_rmdir));
            msgcall = &rmdir;
          }
          break;

        case NFSPROC_READDIR:
          {
            memset(&readdir, 0, sizeof(struct rpc_call_readdir));
            msgcall = &readdir;
          }
          break;

        case NFSPROC_FSSTAT:
          {
            memset(&fs, 0, sizeof(struct rpc_call_fs));
            msgcall = &fs;
          }
          break;

        case NFSPROC_FSINFO:
          {
            memset(&fs, 0, sizeof(struct rpc_call_fs));
            msgcall = &fs;
          }
          break;

        default:
          {
            error = ESRCH;
            goto rpcmout;
          }
        }
    }

  /* Create an instance of the task state structure */

  task = (struct rpctask *)kzalloc(sizeof(struct rpctask));
  if (!task)
    {
      ndbg("Failed to allocate reply msg structure\n");
      return -ENOMEM;
    }

  error = rpcclnt_buildheader(rpc, procnum, prog, version, &value, datain, msgcall);
  if (error)
    {
      ndbg("building call header error");
      goto rpcmout;
    }

  task->r_rpcclnt = rpc;
  task->r_xid = value.xid;
  task->r_procnum = procnum;

  if (rpc->rc_flag & RPCCLNT_SOFT)
    {
      task->r_retry = rpc->rc_retry;
    }
  else
    {
      task->r_retry = RPC_MAXREXMIT + 1;  /* past clip limit */
    }

  task->r_rtt = task->r_rexmit = 0;

  if (rpcclnt_proct(rpc, procnum) > 0)
    {
      task->r_flags = TASK_TIMING;
    }
  else
    {
      task->r_flags = 0;
    }

  /* Do the client side RPC. */

  rpcstats.rpcrequests++;

  /* Chain request into list of outstanding requests. Be sure to put it
   * LAST so timer finds oldest requests first.
   */

  dq_addlast(&task->r_chain, &rpctask_q);

  /* If backing off another request or avoiding congestion, don't send
   * this one now but let timer do it. If not timing a request, do it
   * now.
   */

  if (rpc->rc_so && (rpc->rc_sotype == SOCK_DGRAM ||
                     (rpc->rc_flag & RPCCLNT_DUMBTIMR) ||
                     rpc->rc_sent < rpc->rc_cwnd))
    {
#ifdef CONFIG_NFS_TCPIP
      if (rpc->rc_soflags & PR_CONNREQUIRED)
        {
          error = rpcclnt_sndlock(&rpc->rc_flag, task);
        }
#endif
      if (error == 0)
        {
          error = rpcclnt_send(rpc->rc_so, rpc->rc_name, procnum, prog,
                               msgcall, task);

#ifdef CONFIG_NFS_TCPIP
          if (rpc->rc_soflags & PR_CONNREQUIRED)
            {
              rpcclnt_sndunlock(&rpc->rc_flag);
            }
#endif
        }

      if (error == 0 && (task->r_flags & TASK_MUSTRESEND) == 0)
        {
          rpc->rc_sent += RPC_CWNDSCALE;
          task->r_flags |= TASK_SENT;
        }
    }
  else
    {
      task->r_rtt = -1;
    }

  /* Wait for the reply from our send. */

  if (error == 0 || error == EPIPE)
    {
      error = rpcclnt_reply(task, procnum, prog, dataout);
    }
  nvdbg("out for reply %d\n", error);

  /* RPC done, unlink the request. */

  dq_rem(&task->r_chain, &rpctask_q);

  /* Decrement the outstanding request count. */

  if (task->r_flags & TASK_SENT)
    {
      task->r_flags &= ~TASK_SENT;      /* paranoia */
      rpc->rc_sent -= RPC_CWNDSCALE;
    }

  if (error != 0)
    {
      goto rpcmout;
    }

  /* Break down the rpc header and check if ok */

  memset(&replymgs, 0, sizeof(replymgs));
  bcopy(dataout, &replyheader, sizeof(struct rpc_reply_header));
  replymgs.type = fxdr_unsigned(uint32_t, replyheader.type);
  if (replymgs.type == RPC_MSGDENIED)
    {
      replymgs.status = fxdr_unsigned(uint32_t, replyheader.status);
      switch (replymgs.status)
        {
        case RPC_MISMATCH:
        /*replymgs.stat.mismatch_info.low =
            fxdr_unsigned(uint32_t, replyheader.stat.mismatch_info.low);
          replymgs.stat.mismatch_info.high =
            fxdr_unsigned(uint32_t, replyheader.stat.mismatch_info.high);
          ndbg("RPC_MSGDENIED: RPC_MISMATCH error");*/
          error = EOPNOTSUPP;
          break;

        case RPC_AUTHERR:
        //replymgs.stat.autherr = fxdr_unsigned(uint32_t, replyheader.stat.autherr);
          ndbg("RPC_MSGDENIED: RPC_AUTHERR error\n");
          error = EACCES;
          break;

        default:
          error = EOPNOTSUPP;
          break;
        }
      goto rpcmout;
    }
  else if (replymgs.type != RPC_MSGACCEPTED)
    {
      error = EOPNOTSUPP;
      goto rpcmout;
    }

  /* Verifier */

/*replymgs.rpc_verfi.authtype =
    fxdr_unsigned(enum auth_flavor, replyheader.rpc_verfi.authtype);
  replymgs.rpc_verfi.authlen =
    fxdr_unsigned(uint32_t, replyheader.rpc_verfi.authlen);*/

  if (replymgs.status == RPC_SUCCESS)
    {
      nvdbg("RPC_SUCCESS\n");
    }
  else if (replymgs.status == RPC_PROGMISMATCH)
    {
    /*replymgs.stat.mismatch_info.low =
        fxdr_unsigned(uint32_t, replyheader.stat.mismatch_info.low);
      replymgs.stat.mismatch_info.high =
        fxdr_unsigned(uint32_t, replyheader.stat.mismatch_info.high);*/

      ndbg("RPC_MSGACCEPTED: RPC_PROGMISMATCH error\n");
      error = EOPNOTSUPP;
    }
  else if (replymgs.status > 5)
    {
      error = EOPNOTSUPP;
      goto rpcmout;
    }

rpcmout:
  kfree(task);
  RPC_RETURN(error);
}

#undef COMP
#ifdef COMP
/* Nfs timer routine Scan the nfsreq list and retranmit any requests that
 * have timed out To avoid retransmission attempts on STREAM sockets (in the
 * future) make sure to set the r_retry field to 0 (implies nm_retry == 0).
 */

void rpcclnt_timer(void *arg, struct rpc_call *call)
{
  struct rpctask *rep;
  struct socket *so;
  struct rpcclnt *rpc;
  int timeo, error;

  for (rep = (struct rpctask *)rpctask_q.head; rep != NULL;
       rep = (struct rpctask *)rep->r_chain.flink)
    {
      rpc = rep->r_rpcclnt;
      if (rep->r_flags & TASK_SOFTTERM)
        {
          continue;
        }

      if (rep->r_rtt >= 0)
        {
          rep->r_rtt++;
          if (rpc->rc_flag & RPCCLNT_DUMBTIMR)
            {
              timeo = rpc->rc_timeo;
            }
          else
            {
              timeo = RPC_RTO(rpc, rpcclnt_proct(rep->r_rpcclnt, rep->r_procnum));
            }

          if (rpc->rc_timeouts > 0)
            {
              timeo *= rpcclnt_backoff[rpc->rc_timeouts - 1];
            }

          if (rep->r_rtt <= timeo)
            {
              continue;
            }

          if (rpc->rc_timeouts < 8)
            {
              rpc->rc_timeouts++;
            }
        }

      /* Check for server not responding */

      if ((rep->r_flags & TASK_TPRINTFMSG) == 0 &&
          rep->r_rexmit > rpc->rc_deadthresh)
        {
          ndbg("Server is not responding\n");
          rep->r_flags |= TASK_TPRINTFMSG;
        }

      if (rep->r_rexmit >= rep->r_retry)
        {                       /* too many */
          rpcstats.rpctimeouts++;
          rep->r_flags |= TASK_SOFTTERM;
          continue;
        }

      if (rpc->rc_sotype != SOCK_DGRAM)
        {
          if (++rep->r_rexmit > RPC_MAXREXMIT)
            {
              rep->r_rexmit = RPC_MAXREXMIT;
            }
          continue;
        }

      if ((so = rpc->rc_so) == NULL)
        {
          continue;
        }

      /* If there is enough space and the window allows.. Resend it
       * Set r_rtt to -1 in case we fail to send it now.
       */

      rep->r_rtt = -1;
      if ((rpc->rc_flag & RPCCLNT_DUMBTIMR) || (rep->r_flags & TASK_SENT) ||
          rpc->rc_sent < rpc->rc_cwnd)
        {

          if ((rpc->rc_flag & RPCCLNT_NOCONN) == 0)
            {
              error = psock_sendto(so, call, sizeof(*call), 0, NULL, 0);
            }
          else
            {
              error = psock_sendto(so, call, sizeof(*call), 0, rpc->rc_name,
                                   sizeof(*rpc->rc_name));
            }

          if (!error)
            {
              /* Iff first send, start timing else turn
               * timing off, backoff timer and divide
               * congestion window by 2.
               */

              if (rep->r_flags & TASK_SENT)
                {
                  rep->r_flags &= ~TASK_TIMING;
                  if (++rep->r_rexmit > RPC_MAXREXMIT)
                    {
                      rep->r_rexmit = RPC_MAXREXMIT;
                    }

                  rpc->rc_cwnd >>= 1;
                  if (rpc->rc_cwnd < RPC_CWNDSCALE)
                    {
                      rpc->rc_cwnd = RPC_CWNDSCALE;
                    }

                  rpcstats.rpcretries++;
                }
              else
                {
                  rep->r_flags |= TASK_SENT;
                  rpc->rc_sent += RPC_CWNDSCALE;
                }

              rep->r_rtt = 0;
            }
        }
    }

  // rpcclnt_timer_handle = timeout(rpcclnt_timer, NULL, rpcclnt_ticks);
}
#endif

/* Build the RPC header and fill in the authorization info. */

int rpcclnt_buildheader(struct rpcclnt *rpc, int procid, int prog, int vers,
                        struct xidr *value, FAR const void *datain,
                        void *dataout)
{
  srand(time(NULL));
  int xidp = 0;

  /* The RPC header.*/

  /* Get a new (non-zero) xid */

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

  /* Perform the binding depending on the protocol type */

  if (prog == PMAPPROG)
    {
      if (procid == PMAPPROC_GETPORT)
        {
          struct rpc_call_pmap *callmsg = (struct rpc_call_pmap *)dataout;
          bcopy(datain, &callmsg->pmap, sizeof(struct call_args_pmap));

          callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
          value->xid = callmsg->ch.rp_xid;
          callmsg->ch.rp_direction = rpc_call;
          callmsg->ch.rp_rpcvers = rpc_vers;
          callmsg->ch.rp_prog = txdr_unsigned(prog);
          callmsg->ch.rp_vers = txdr_unsigned(vers);
          callmsg->ch.rp_proc = txdr_unsigned(procid);

          /* rpc_auth part (auth_unix as root) */

          callmsg->ch.rpc_auth.authtype = rpc_auth_null;
        //call->rpc_auth.authlen = 0;
#ifdef CONFIG_NFS_UNIX_AUTH
          callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
          callmsg->ch.rpc_unix.hostname = 0;
          callmsg->ch.rpc_unix.uid = setuid;
          callmsg->ch.rpc_unix.gid = setgid;
          callmsg->ch.rpc_unix.gidlist = 0;
#endif
          /* rpc_verf part (auth_null) */

          callmsg->ch.rpc_verf.authtype = rpc_auth_null;
        //call->rpc_verf.authlen = 0;
          return 0;
        }
      else if (procid == PMAPPROC_UNSET)
        {
          struct rpc_call_pmap *callmsg = (struct rpc_call_pmap *)dataout;;
          bcopy(datain, &callmsg->pmap, sizeof(struct call_args_pmap));
          callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
          value->xid = callmsg->ch.rp_xid;
          callmsg->ch.rp_direction = rpc_call;
          callmsg->ch.rp_rpcvers = rpc_vers;
          callmsg->ch.rp_prog = txdr_unsigned(prog);
          callmsg->ch.rp_vers = txdr_unsigned(vers);
          callmsg->ch.rp_proc = txdr_unsigned(procid);

          /* rpc_auth part (auth_unix as root) */

          callmsg->ch.rpc_auth.authtype = rpc_auth_null;
        //call->rpc_auth.authlen = 0;
#ifdef CONFIG_NFS_UNIX_AUTH
          callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
          callmsg->ch.rpc_unix.hostname = 0;
          callmsg->ch.rpc_unix.uid = setuid;
          callmsg->ch.rpc_unix.gid = setgid;
          callmsg->ch.rpc_unix.gidlist = 0;
#endif
          /* rpc_verf part (auth_null) */

          callmsg->ch.rpc_verf.authtype = rpc_auth_null;
        //call->rpc_verf.authlen = 0;
          return 0;
        }
    }
  else if (prog == RPCPROG_MNT)
    {
      if (procid == RPCMNT_UMOUNT)
        {
          struct rpc_call_mount *callmsg = (struct rpc_call_mount *)dataout;
          bcopy(datain, &callmsg->mount, sizeof(struct call_args_mount));
          callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
          value->xid = callmsg->ch.rp_xid;
          callmsg->ch.rp_direction = rpc_call;
          callmsg->ch.rp_rpcvers = rpc_vers;
          callmsg->ch.rp_prog = txdr_unsigned(prog);
          callmsg->ch.rp_vers = txdr_unsigned(vers);
          callmsg->ch.rp_proc = txdr_unsigned(procid);

          /* rpc_auth part (auth_unix as root) */

          callmsg->ch.rpc_auth.authtype = rpc_auth_null;
        //call->rpc_auth.authlen = 0;
#ifdef CONFIG_NFS_UNIX_AUTH
          callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
          callmsg->ch.rpc_unix.hostname = 0;
          callmsg->ch.rpc_unix.uid = setuid;
          callmsg->ch.rpc_unix.gid = setgid;
          callmsg->ch.rpc_unix.gidlist = 0;
#endif
          /* rpc_verf part (auth_null) */

          callmsg->ch.rpc_verf.authtype = rpc_auth_null;
        //call->rpc_verf.authlen = 0;
          return 0;
        }
      else if (procid == RPCMNT_MOUNT)
        {
          struct rpc_call_mount *callmsg = (struct rpc_call_mount *)dataout;
          bcopy(datain, &callmsg->mount, sizeof(struct call_args_mount));
          callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
          value->xid = callmsg->ch.rp_xid;
          callmsg->ch.rp_direction = rpc_call;
          callmsg->ch.rp_rpcvers = rpc_vers;
          callmsg->ch.rp_prog = txdr_unsigned(prog);
          callmsg->ch.rp_vers = txdr_unsigned(vers);
          callmsg->ch.rp_proc = txdr_unsigned(procid);

          /* rpc_auth part (auth_unix as root) */

          callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;
#ifdef CONFIG_NFS_UNIX_AUTH
          callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
          callmsg->ch.rpc_unix.hostname = 0;
          callmsg->ch.rpc_unix.uid = setuid;
          callmsg->ch.rpc_unix.gid = setgid;
          callmsg->ch.rpc_unix.gidlist = 0;
#endif
          /* rpc_verf part (auth_null) */

          callmsg->ch.rpc_verf.authtype = rpc_auth_null;
        //call->rpc_verf.authlen = 0;
          return 0;
        }
    }
  else if (prog == NFS_PROG)
    {
      switch (procid)
        {
        case NFSPROC_CREATE:
          {
            struct rpc_call_create *callmsg = (struct rpc_call_create *)dataout;
            bcopy(datain, &callmsg->create, sizeof(struct CREATE3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;
#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_READ:
          {
            struct rpc_call_read *callmsg = (struct rpc_call_read *)dataout;
            bcopy(datain, &callmsg->read, sizeof(struct READ3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_WRITE:
          {
            struct rpc_call_write *callmsg = (struct rpc_call_write *)dataout;
            bcopy(datain, &callmsg->write, sizeof(struct WRITE3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_READDIR:
          {
            struct rpc_call_readdir *callmsg = (struct rpc_call_readdir *)dataout;
            bcopy(datain, &callmsg->readdir, sizeof(struct READDIR3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_FSSTAT:
          {
            struct rpc_call_fs *callmsg = (struct rpc_call_fs *)dataout;
            bcopy(datain, &callmsg->fs, sizeof(struct FS3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_REMOVE:
          {
            struct rpc_call_remove *callmsg  = (struct rpc_call_remove *)dataout;
            bcopy(datain, &callmsg->remove, sizeof(struct REMOVE3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_MKDIR:
          {
            struct rpc_call_mkdir *callmsg = (struct rpc_call_mkdir *)dataout;
            bcopy(datain, &callmsg->mkdir, sizeof(struct MKDIR3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_RMDIR:
          {
            struct rpc_call_rmdir *callmsg  = (struct rpc_call_rmdir *)dataout;
            bcopy(datain, &callmsg->rmdir, sizeof(struct RMDIR3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_RENAME:
          {
            struct rpc_call_rename *callmsg  = (struct rpc_call_rename *)dataout;
            bcopy(datain, &callmsg->rename, sizeof(struct RENAME3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
            //call->rpc_verf.authlen = 0;
            return 0;
          }

        case NFSPROC_FSINFO:
          {
            struct rpc_call_fs *callmsg = (struct rpc_call_fs *)dataout;
            bcopy(datain, &callmsg->fs, sizeof(struct FS3args));
            callmsg->ch.rp_xid = txdr_unsigned(rpcclnt_xid);
            value->xid = callmsg->ch.rp_xid;
            callmsg->ch.rp_direction = rpc_call;
            callmsg->ch.rp_rpcvers = rpc_vers;
            callmsg->ch.rp_prog = txdr_unsigned(prog);
            callmsg->ch.rp_vers = txdr_unsigned(vers);
            callmsg->ch.rp_proc = txdr_unsigned(procid);

            /* rpc_auth part (auth_unix as root) */

            callmsg->ch.rpc_auth.authtype = rpc_auth_null;
          //call->rpc_auth.authlen = 0;

#ifdef CONFIG_NFS_UNIX_AUTH
            callmsg->ch.rpc_unix.stamp = txdr_unsigned(1);
            callmsg->ch.rpc_unix.hostname = 0;
            callmsg->ch.rpc_unix.uid = setuid;
            callmsg->ch.rpc_unix.gid = setgid;
            callmsg->ch.rpc_unix.gidlist = 0;
#endif
            /* rpc_verf part (auth_null) */

            callmsg->ch.rpc_verf.authtype = rpc_auth_null;
          //call->rpc_verf.authlen = 0;
            return 0;
          }

        default:
          break;
        }
    }

  return ESRCH;
}

#ifdef COMP
int rpcclnt_cancelreqs(struct rpcclnt *rpc)
{
  struct rpctask *task;
  int i;

  for (task = (struct rpctask *)rpctask_q.head; task;
       task = (struct rpctask *)task->r_chain.flink)
    {
      if (rpc != task->r_rpcclnt || (task->r_flags & TASK_SOFTTERM))
        {
          continue;
        }

      rpcclnt_softterm(task);
    }

  for (i = 0; i < 30; i++)
    {
      for (task = (struct rpctask *)&rpctask_q.head; task;
           task = (struct rpctask *)task->r_chain.flink)
        {
          if (rpc == task->r_rpcclnt)
            {
              break;
            }
        }

      if (task == NULL)
        {
          return 0;
        }
    }

  return EBUSY;
}
#endif
#endif
