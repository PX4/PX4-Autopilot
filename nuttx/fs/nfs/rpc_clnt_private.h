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

#ifndef _RPCCLNT_PRIVATE_H_
#define _RPCCLNT_PRIVATE_H_

#define RPCCLNT_DEBUG 1

#define RPC_TICKINTVL     5

/* from nfs/nfsproto.h */

#define RPC_MAXDATA     32768
#define RPC_MAXPKTHDR   404
#define RPC_MAXPACKET   (RPC_MAXPKTHDR + RPC_MAXDATA)

#define RPCX_UNSIGNED   4

#define RPC_SUCCESS 0

/* Flag values for r_flags */

#define TASK_TIMING        0x01            /* timing request (in mntp) */
#define TASK_SENT          0x02            /* request has been sent */
#define TASK_SOFTTERM      0x04            /* soft mnt, too many retries */


#define TASK_INTR          0x08            /* intr mnt, signal pending */
#define TASK_SOCKERR       0x10            /* Fatal error on socket */


#define TASK_TPRINTFMSG    0x20            /* Did a tprintf msg. */

#define TASK_MUSTRESEND    0x40            /* Must resend request */
#define TASK_GETONEREP     0x80            /* Probe for one reply only */


#define RPC_HZ          (CLOCKS_PER_SEC / rpcclnt_ticks) /* Ticks/sec */
#define RPC_TIMEO       (1 * RPC_HZ)    /* Default timeout = 1 second */

#define RPC_MAXREXMIT   100             /* Stop counting after this many */


#define RPCIGNORE_SOERROR(s, e) \
                        ((e) != EINTR && (e) != ERESTART && (e) != EWOULDBLOCK)

#define RPCINT_SIGMASK  (sigmask(SIGINT)|sigmask(SIGTERM)|sigmask(SIGKILL)| \
                         sigmask(SIGHUP)|sigmask(SIGQUIT))

#define RPCMADV(m, s)   (m)->m_data += (s)

#define RPCAUTH_ROOTCREDS NULL

#define RPCCLNTINT_SIGMASK(set)             \
  (SIGISMEMBER(set, SIGINT) || SIGISMEMBER(set, SIGTERM) || \
         SIGISMEMBER(set, SIGHUP) || SIGISMEMBER(set, SIGKILL) || \
         SIGISMEMBER(set, SIGQUIT))

/* global rpcstats 
 * XXX should be per rpcclnt */

struct rpcstats
{
  int rpcretries;
  int rpcrequests;
  int rpctimeouts;
  int rpcunexpected;
  int rpcinvalid;
};

#endif /* _RPCCLNT_PRIVATE_H_ */
