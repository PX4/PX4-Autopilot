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

#ifndef __NFSX_H_
#define __NFSX_H_

/* nfs_socket interface */

/* XXXMARIUS: name collision */

int nfsx_connect(struct nfsmount *);
void nfsx_disconnect(struct nfsmount *);
int nfsx_sigintr(struct nfsmount *, struct nfsreq *, cthread_t *);
void nfsx_safedisconnect(struct nfsmount *);
int nfsx_request_xx(struct nfsmount *, struct vnode *, struct mbuf *, int,
                    cthread_t *, struct ucred *, struct mbuf **, struct mbuf **,
                    caddr_t *);
int nfsx_nmcancelreqs(struct nfsmount *);

#define nfs_connect nfs_connect_nfsx
#define nfs_disconnect nfs_disconnect_nfsx
#define nfs_sigintr nfs_sigintr_nfsx

/* XXX dros: defined in nfs.h */

#if 0
void nfs_safedisconnect(struct nfsmount *);
#endif

#define nfsx_request(vp, m, p, td, cr, m2, m3, c) \
    nfsx_request_xx(NULL, vp, m, p, td, cr, m2, m3, c)

#define nfsx_request_mnt(nmp, m, p, td, cr, m2, m3, c) \
    nfsx_request_xx(nmp, NULL, m, p, td, cr, m2, m3, c)

/* don't use this.. use nfsx_request() of nfsx_request_mnt() */

int nfs_request_xx(struct nfsmount *, struct vnode *, struct mbuf *, int,
                   cthread_t *, struct ucred *, struct mbuf **, struct mbuf **,
                   caddr_t *);

/* XXX dros: defined in nfs.h */

#if 0
int nfs_nmcancelreqs(struct nfsmount *);
#endif

#endif /* __NFSX_H_ */
