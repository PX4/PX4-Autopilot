/****************************************************************************
 * include/nuttx/fs/nfs.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Some of the content of this file was leveraged from OpenBSD:
 *
 *   Copyright (c) 1989, 1993, 1995
 *   The Regents of the University of California.  All rights reserved.
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
 * 3. Neither the name of the University nor the names of its contributors
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_FS_NFS_H
#define __INCLUDE_NUTTX_FS_NFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NFSMNT_NFSV3     0x00000200    /* Use NFS Version 3 protocol */
#define NFS_ARGSVERSION  3             /* change when nfs_args changes */
#define NFS_PMAPPORT     111
 
/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nfs_args
{
  uint8_t version;          /* args structure version number */
  struct  sockaddr addr;    /* file server address */
  uint8_t addrlen;          /* length of address */
  uint8_t sotype;           /* Socket type */
  uint8_t proto;            /* and Protocol */
  int     flags;            /* flags */
  int     wsize;            /* write size in bytes */
  int     rsize;            /* read size in bytes */
  int     readdirsize;      /* readdir size in bytes */
  int     timeo;            /* initial timeout in .1 secs */
  int     retrans;          /* times to retry send */
//int     maxgrouplist;     /* Max. size of group list */
//int     readahead;        /* # of blocks to readahead */
//int     leaseterm;        /* Term (sec) of lease */
//int     deadthresh;       /* Retrans threshold */
  char   *path;             /* server's path of the directory being mount */
  int     acregmin;         /* cache attrs for reg files min time */
  int     acregmax;         /* cache attrs for reg files max time */
  int     acdirmin;         /* cache attrs for dirs min time */
  int     acdirmax;         /* cache attrs for dirs max time */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif
 
#endif /* _NFS_NFS_H */
