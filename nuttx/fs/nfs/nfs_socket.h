/****************************************************************************
 * fs/nfs/nfs_socket.h
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

#ifndef __FS_NFS_NFS_SOCKET_H
#define __FS_NFS_NFS_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define nfs_connect(nmp)  nfs_connect_nfsx (nmp)
#define nfs_disconnect(nmp) nfs_disconnect_nfsx(nmp)
#define nfs_nmcancelreqs (nmp) nfsx_nmcancelreqs(nmp)
#define nfsx_request(nmp, m, i, o) \
    nfsx_request_xx(nmp, m, i, o)

#ifdef CONFIG_NFS_TCPIP
#  define nfs_sigintr nfs_sigintr_nfsx
#define nfs_safedisconnect nfsx_safedisconnect
#endif
 
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN void nfs_init(void);
EXTERN int nfsx_connect(struct nfsmount *);
EXTERN void nfsx_disconnect(struct nfsmount *);
#ifdef CONFIG_NFS_TCPIP
EXTERN int nfsx_sigintr(struct nfsmount *, struct nfsreq *, cthread_t *);
EXTERN void nfsx_safedisconnect(struct nfsmount *);
#endif
EXTERN int nfsx_request_xx(struct nfsmount *, int, void*, void*);
EXTERN int nfsx_nmcancelreqs(struct nfsmount *);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_NFS_NFS_SOCKET_H */
