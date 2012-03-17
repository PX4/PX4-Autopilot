/*
 * copyright (c) 2003
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Berkeley and its contributors.
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

#ifndef _RPCCLNT_H_
#define _RPCCLNT_H_

#include <sys/types.h>

/* for rpcclnt's rc_flags */

#define RPCCLNT_SOFT            0x001 /* soft mount (hard is details) */
#define RPCCLNT_INT             0x002 /* allow interrupts on hard mounts */
#define RPCCLNT_NOCONN          0x004 /* dont connect the socket (udp) */
#define RPCCLNT_DUMBTIMR        0x010 

/* XXX should be replaced with real locks */

#define RPCCLNT_SNDLOCK         0x100
#define RPCCLNT_WANTSND         0x200
#define RPCCLNT_RCVLOCK         0x400
#define RPCCLNT_WANTRCV         0x800

struct rpc_program
{
  uint32_t prog_id;
  uint32_t prog_version;
  char * prog_name;
};

struct rpctask
{
  dq_entry_t      r_chain;
        
  struct rpcclnt  *r_rpcclnt;

  uint32_t        r_xid;
  int             r_flags;        /* flags on request, see below */
  int             r_retry;        /* max retransmission count */
  int             r_rexmit;       /* current retrans count */
  int             r_timer;        /* tick counter on reply */
  int             r_procnum;      /* NFS procedure number */
  int             r_rtt;          /* RTT for rpc */
};

/* Generic RPC headers */

struct rpc_auth_info
{
  uint32_t authtype;          /* auth type */
  uint32_t authlen;           /* auth length */
};

struct auth_unix
{
  int32_t ua_time;
  int32_t ua_hostname;        /* null */
  int32_t ua_uid;
  int32_t ua_gid;
  int32_t ua_gidlist;         /* null */
};

struct rpc_call
{
  uint32_t rp_xid;            /* request transaction id */
  int32_t rp_direction;       /* call direction (0) */
  uint32_t rp_rpcvers;        /* rpc version (2) */
  uint32_t rp_prog;           /* program */
  uint32_t rp_vers;           /* version */
  uint32_t rp_proc;           /* procedure */
  struct rpc_auth_info rpc_auth;
  struct auth_unix rpc_unix;  
  struct rpc_auth_info rpc_verf;
};

struct rpc_reply
{
  uint32_t rp_xid;            /* request transaction id */
  int32_t rp_direction;       /* call direction (1) */
  struct
  {
    uint32_t type;
    uint32_t status;

    /* used only when reply == RPC_MSGDENIED and status == RPC_AUTHERR */

    uint32_t autherr;

    /* rpc mismatch info if reply == RPC_MSGDENIED and status == RPC_MISMATCH */

    struct
    {
      uint32_t low;
      uint32_t high;
    } mismatch_info;
  } stat;
        
  struct rpc_auth_info rpc_verfi;
};

/*
 * RPC Client connection context.
 * One allocated on every NFS mount.
 * Holds RPC specific information for mount.
 */

/* XXX: please note that all pointer type variables are just set (not copied),
 *      so it is up to the user to free these values */

struct  rpcclnt
{
  int     rc_flag;                /* For RPCCLNT_* flags  */

  int     rc_wsize;               /* Max size of the request data */
  int     rc_rsize;               /* Max size of the response data */

  struct  sockaddr *rc_name;              
  struct  socket *rc_so;          /* Rpc socket */

  int     rc_sotype;              /* Type of socket */
  int     rc_soproto;             /* and protocol */
  int     rc_soflags;             /* pr_flags for socket protocol */

  int     rc_timeo;               /* Init timer for NFSMNT_DUMBTIMR */
  int     rc_retry;               /* Max retries */
  int     rc_srtt[4];             /* Timers for rpcs */
  int     rc_sdrtt[4];
  int     rc_sent;                /* Request send count */
  int     rc_cwnd;                /* Request send window */
  int     rc_timeouts;            /* Request timeouts */

  int     rc_deadthresh;          /* Threshold of timeouts-->dead server*/

  /* authentication: */
  /* currently can be RPCAUTH_NULL, RPCAUTH_KERBV4, RPCAUTH_UNIX */
  /* should be kept in XDR form */

  int     rc_authtype;            /* Authenticator type */
#ifdef CONFIG_NFS_UNIX_AUTH
  /* RPCAUTH_UNIX*/

  struct rpc_auth_info rc_oldauth;        /* authentication */
#endif
  void                *rc_auth;

  struct rpc_program * rc_prog;

  char *rc_servername;

  int rc_proctlen;                /* if == 0 then rc_proct == NULL */
  int * rc_proct;
};

/*
void rpcclnt_create(struct rpcclnt ** rpc);
void rpcclnt_destroy(struct rpcclnt * rpc);

#define rpcclnt_get(X) rpcclnt_create(&(X))
#define rpcclnt_put(X) rpcclnt_destroy(X)

*/

void rpcclnt_init(void);
//void rpcclnt_uninit(void);

int  rpcclnt_setup(struct rpcclnt *, struct rpc_program *, struct sockaddr *, int, int, struct rpc_auth_info *, int, int, int);
int  rpcclnt_connect(struct rpcclnt *);
int  rpcclnt_reconnect(struct rpctask *);
void rpcclnt_disconnect(struct rpcclnt *);
void rpcclnt_safedisconnect(struct rpcclnt *);
int  rpcclnt_request(struct rpcclnt *, int, struct rpc_reply *);
int  rpcclnt_cancelreqs(struct rpcclnt *);

#endif /* _RPCCLNT_H_ */
