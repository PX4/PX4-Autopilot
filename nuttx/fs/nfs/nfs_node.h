/****************************************************************************
 * fs/nfs/nfs_node.h
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1989, 1993
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

#ifndef __FS_NFS_NFS_NODE_H
#define __FS_NFS_NFS_NODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nfs_proto.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flags for struct nfsnode n_flag */

#define NFSNODE_OPEN           (1 << 0) /* File is still open */
#define NFSNODE_MODIFIED       (1 << 1) /* Might have a modified buffer */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* There is a unique nfsnode allocated for each active file.  An nfsnode is
 * 'named' by its file handle.
 */

struct nfsnode
{
  struct nfsnode    *n_next;        /* Retained in a singly linked list. */
  uint8_t            n_crefs;       /* Reference count (for nfs_dup) */
  uint8_t            n_type;        /* File type */
  uint8_t            n_fhsize;      /* Size in bytes of the file handle */
  uint8_t            n_flags;       /* Node flags */
  struct timespec    n_mtime;       /* File modification time (see NOTE) */
  time_t             n_ctime;       /* File creation time (see NOTE) */
  nfsfh_t            n_fhandle;     /* NFS File Handle */
  uint64_t           n_size;        /* Current size of file (see NOTE) */
};

#endif /* __FS_NFS_NFS_NODE_H */
