/****************************************************************************
 * fs/nfs/rpc_subr.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1995 Gordon Ross, Adam Glass
 *   Copyright (c) 1992 Regents of the University of California.
 *   All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the University of
 *    California, Lawrence Berkeley Laboratory and its contributors.
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
 *
 * partially based on:
 *      libnetboot/rpc.c
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nuttx/kmalloc.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "rpc_mbuf.h"
#include "rpc_types.h"
#include "rpc_v2.h"
#include "rpc.h"
#include "xdr_subs.h"
#include "rpc_idgen.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIN_REPLY_HDR 16        /* xid, dir, astat, errno */

/* What is the longest we will wait before re-sending a request?
 * Note this is also the frequency of "RPC timeout" messages.
 * The re-send loop count sup linearly to this maximum, so the
 * first complaint will happen after (1+2+3+4+5)=15 seconds.
 */

#define MAX_RESEND_DELAY 5   /* seconds */
#define IP_PORTRANGE     19  /* int; range to choose for unspec port */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Generic RPC headers */

struct auth_info
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
  struct auth_info rpc_auth;
  struct auth_unix rpc_unix;
  struct auth_info rpc_verf;
};

struct rpc_reply
{
  uint32_t rp_xid;            /* request transaction id */
  int32_t rp_direction;       /* call direction (1) */
  int32_t rp_astatus;         /* accept status (0: accepted) */
  union
    {
      uint32_t rpu_errno;
      struct
        {
          struct auth_info rok_auth;
          uint32_t rok_status;
        } rpu_rok;
    } rp_u;
};

#define rp_errno  rp_u.rpu_errno
#define rp_auth   rp_u.rpu_rok.rok_auth
#define rp_status rp_u.rpu_rok.rok_status

/* String representation for RPC. */

struct xdr_string
{
  uint32_t len;               /* length without null or padding */
  char data[4];               /* data (longer, of course) */
  /* data is padded to a long-word boundary */
};

/* Inet address in RPC messages (Note, really four ints, NOT chars.  Blech.) */

struct xdr_inaddr
{
  uint32_t atype;
  uint32_t addr[4];
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Return an unpredictable XID. */

uint32_t krpc_get_xid(void)
{
  static struct idgen32_ctx krpc_xid_ctx;
  static int called = 0;

  if (!called)
    {
      called = 1;
      idgen32_init(&krpc_xid_ctx);
    }
  return idgen32(&krpc_xid_ctx);
}

/* Call portmap to lookup a port number for a particular rpc program
 * Returns non-zero error on failure.
 */

int krpc_portmap(struct sockaddr_in *sin, unsigned int prog, unsigned int vers,
                 uint16_t * portp)
{
  struct sdata
    {
      uint32_t prog;            /* call program */
      uint32_t vers;            /* call version */
      uint32_t proto;           /* call protocol */
      uint32_t port;            /* call port (unused) */
    } *sdata;
  struct rdata
    {
      uint16_t pad;
      uint16_t port;
    } *rdata;
  int error;

  /* The portmapper port is fixed. */

  if (prog == PMAPPROG)
    {
      *portp = htons(PMAPPORT);
      return 0;
    }

  /* Do the RPC to get it. */

  sdata->prog = txdr_unsigned(prog);
  sdata->vers = txdr_unsigned(vers);
  sdata->proto = txdr_unsigned(IPPROTO_UDP);
  sdata->port = 0;

  sin->sin_port = htons(PMAPPORT);
  error = krpc_call(sin, PMAPPROG, PMAPVERS, PMAPPROC_GETPORT, -1);
  if (error)
    {
      return error;
    }

  *portp = rdata->port;

  return 0;
}

/* Do a remote procedure call (RPC) and wait for its reply.
 * If from_p is non-null, then we are doing broadcast, and
 * the address from whence the response came is saved there.
 * data:    input/output
 * from_p:    output
 */
 
int krpc_call(struct sockaddr_in *sa, unsigned int prog, unsigned int vers,
              unsigned int func, int retries)
{
  struct socket *so;
  struct sockaddr_in *sin;
  struct mbuf *m, *nam, *mhead, *from;
  struct rpc_call *call;
  struct rpc_reply *reply;
  struct uio auio;
  int error, rcvflg, timo, secs, len;
  static uint32_t xid = 0;
  int *ip;
  struct timeval *tv;

  /* Validate address family.
   * Sorry, this is INET specific...
   */

  if (sa->sin_family != AF_INET)
    {
      return (EAFNOSUPPORT);
    }

  /* Create socket and set its receive timeout. */

  if ((error = psock_socket(AF_INET, SOCK_DGRAM, 0, so)))
    {
      goto out;
    }
  
  tv->tv_sec  = 1;
  tv->tv_usec = 0;
  
  if ((error = psock_setsockopt(so, SOL_SOCKET, SO_RCVTIMEO,(const void *) tv, sizeof (*tv))))
    {
      goto out;
    }

  /*  Enable broadcast if necessary. */

  if (from_p)
    {
      int32_t *on;
      *on = 1;

      if ((error = psock_setsockopt(so, SOL_SOCKET, SO_BROADCAST, (const void *) on, sizeof (*on))))
        {
          goto out;
        }
    }

  /* Bind the local endpoint to a reserved port,
   * because some NFS servers refuse requests from
   * non-reserved (non-privileged) ports.
   */

  *ip = 2;
  error = psock_setsockopt(so, IPPROTO_IP, IP_PORTRANGE, (const void *) ip, sizeof (*ip));
  if (error)
    {
      goto out;
    }

  sin->sin_len = sizeof(struct sockaddr_in);
  sin->sin_family = AF_INET;
  sin->sin_addr.s_addr = INADDR_ANY;
  sin->sin_port = htons(0);
  error = psock_bind(so, m, &proc0);
  
  if (error)
    {
      printf("bind failed\n");
      goto out;
    }

  *ip = 0;
  error = psock_setsockopt(so, IPPROTO_IP, IP_PORTRANGE, (const void *) ip, sizeof (*ip));
  if (error)
    {
      goto out;
    }

  /* Setup socket address for the server. */

  bcopy((caddr_t *) sa, (caddr_t *) sin, sa->sin_len);

  /* Prepend RPC message header. */

  bzero((caddr_t *) call, sizeof(*call));

  /* rpc_call part */

  xid = krpc_get_xid();
  call->rp_xid = txdr_unsigned(xid);

  /* call->rp_direction = 0; */

  call->rp_rpcvers = txdr_unsigned(2);
  call->rp_prog = txdr_unsigned(prog);
  call->rp_vers = txdr_unsigned(vers);
  call->rp_proc = txdr_unsigned(func);

  /* rpc_auth part (auth_unix as root) */

  call->rpc_auth.authtype = txdr_unsigned(RPCAUTH_UNIX);
  call->rpc_auth.authlen = txdr_unsigned(sizeof(struct auth_unix));

  /* rpc_verf part (auth_null) */

  call->rpc_verf.authtype = 0;
  call->rpc_verf.authlen = 0;


  /* Send it, repeatedly, until a reply is received,
   * but delay each re-send by an increasing amount.
   * If the delay hits the maximum, start complaining.
   */

  for (timo = 0; retries; retries--)
    {
      /* Send RPC request (or re-send). */

      error = sosend(so, nam, NULL, m, NULL, 0);
      if (error)
        {
          printf("krpc_call: sosend: %d\n", error);
          goto out;
        }

      m = NULL;

      /* Determine new timeout. */

      if (timo < MAX_RESEND_DELAY)
        {
          timo++;
        }
      else
        {
          printf("RPC timeout for server %s (0x%x) prog %u\n",
                 inet_ntoa(sin->sin_addr), ntohl(sin->sin_addr.s_addr), prog);
        }

      /* Wait for up to timo seconds for a reply.
       * The socket receive timeout was set to 1 second.
       */

      secs = timo;
      while (secs > 0)
        {
          if (from)
            {
              m_freem(from);
              from = NULL;
            }

          if (m)
            {
              m_freem(m);
              m = NULL;
            }

          auio.uio_resid = len = 1 << 16;
          auio.uio_procp = NULL;
          rcvflg = 0;

          error = soreceive(so, &from, &auio, &m, NULL, &rcvflg, 0);
          if (error == EWOULDBLOCK)
            {
              secs--;
              continue;
            }

          if (error)
            {
              goto out;
            }

          len -= auio.uio_resid;

          /* Does the reply contain at least a header? */

          if (len < MIN_REPLY_HDR)
            {
              continue;
            }

          if (m->m_len < MIN_REPLY_HDR)
            {
              continue;
            }

          reply = mtod(m, struct rpc_reply *);

          /* Is it the right reply? */

          if (reply->rp_direction != txdr_unsigned(RPC_REPLY))
            {
              continue;
            }

          if (reply->rp_xid != txdr_unsigned(xid))
            {
              continue;
            }

          /* Was RPC accepted? (authorization OK) */

          if (reply->rp_astatus != 0)
            {
              error = fxdr_unsigned(uint32_t, reply->rp_errno);
              printf("rpc denied, error=%d\n", error);
              continue;
            }

          /* Did the call succeed? */

          if (reply->rp_status != 0)
            {
              error = fxdr_unsigned(uint32_t, reply->rp_status);
              printf("rpc denied, status=%d\n", error);
              continue;
            }

          goto gotreply;        /* break two levels */
        }
    }

  error = ETIMEDOUT;
  goto out;

gotreply:

  /* Get RPC reply header into first mbuf,
   * get its length, then strip it off.
   */

  len = sizeof(*reply);
  if (m->m_len < len)
    {
      m = m_pullup(m, len);
      if (m == NULL)
        {
          error = ENOBUFS;
          goto out;
        }
    }

  if (reply->rp_auth.authtype != 0)
    {
      len += fxdr_unsigned(uint32_t, reply->rp_auth.authlen);
      len = (len + 3) & ~3;     /* XXX? */
    }

  /* result */

  *data = m;
  if (from_p && error == 0)
    {
      *from_p = from;
      from = NULL;
    }

out:
  (void)psock_close(so);
  return error;
}

/* eXternal Data Representation routines. (but with non-standard args...) */

struct mbuf *xdr_string_encode(char *str, int len)
{
  struct mbuf *m;
  struct xdr_string *xs;
  int dlen;                     /* padded string length */
  int mlen;                     /* message length */

  dlen = (len + 3) & ~3;
  mlen = dlen + 4;

  if (mlen > MCLBYTES)          /* If too big, we just can't do it. */
    {
      return (NULL);
    }

  m = m_get(M_WAIT, MT_DATA);
  if (mlen > MLEN)
    {
      MCLGET(m, M_WAIT);
      if ((m->m_flags & M_EXT) == 0)
        {
          (void)m_free(m);      /* There can be only one. */
          return (NULL);
        }
    }

  xs = mtod(m, struct xdr_string *);
  m->m_len = mlen;
  xs->len = txdr_unsigned(len);
  strncpy(xs->data, str, len);
  return (m);
}

struct mbuf *xdr_string_decode(struct mbuf *m, char *str, int *len_p)
{
  struct xdr_string *xs;
  int mlen;                     /* message length */
  int slen;                     /* string length */

  if (m->m_len < 4)
    {
      m = m_pullup(m, 4);
      if (m == NULL)
        return (NULL);
    }

  xs = mtod(m, struct xdr_string *);
  slen = fxdr_unsigned(uint32_t, xs->len);
  mlen = 4 + ((slen + 3) & ~3);

  if (slen > *len_p)
    {
      slen = *len_p;
    }

  if (slen > m->m_pkthdr.len)
    {
      m_freem(m);
      return (NULL);
    }

  m_copydata(m, 4, slen, str);
  m_adj(m, mlen);

  str[slen] = '\0';
  *len_p = slen;

  return (m);
}

struct mbuf *xdr_inaddr_encode(struct in_addr *ia)
{
  struct mbuf *m;
  struct xdr_inaddr *xi;
  uint8_t *cp;
  uint32_t *ip;

  m = m_get(M_WAIT, MT_DATA);
  xi = mtod(m, struct xdr_inaddr *);
  m->m_len = sizeof(*xi);
  xi->atype = txdr_unsigned(1);
  ip = xi->addr;
  cp = (uint8_t *) & ia->s_addr;
  *ip++ = txdr_unsigned(*cp++);
  *ip++ = txdr_unsigned(*cp++);
  *ip++ = txdr_unsigned(*cp++);
  *ip++ = txdr_unsigned(*cp++);

  return (m);
}

struct mbuf *xdr_inaddr_decode(struct mbuf *m, struct in_addr *ia)
{
  struct xdr_inaddr *xi;
  uint8_t *cp;
  uint32_t *ip;

  if (m->m_len < sizeof(*xi))
    {
      m = m_pullup(m, sizeof(*xi));
      if (m == NULL)
        return (NULL);
    }

  xi = mtod(m, struct xdr_inaddr *);
  if (xi->atype != txdr_unsigned(1))
    {
      ia->s_addr = INADDR_ANY;
      goto out;
    }

  ip = xi->addr;
  cp = (uint8_t *) & ia->s_addr;
  *cp++ = fxdr_unsigned(uint8_t, *ip++);
  *cp++ = fxdr_unsigned(uint8_t, *ip++);
  *cp++ = fxdr_unsigned(uint8_t, *ip++);
  *cp++ = fxdr_unsigned(uint8_t, *ip++);

out:
  m_adj(m, sizeof(*xi));
  return (m);
}
