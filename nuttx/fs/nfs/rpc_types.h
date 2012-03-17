/****************************************************************************
 * fs/nfs/rpc_types.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
 *
 *   Copyright (c) 1982, 1986, 1988, 1993
 *      The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#ifndef __FS_NFS_RPC_TYPES_H
#define __FS_NFS_RPC_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lock
{
  int dummy;
};

typedef struct { int32_t val[2]; } fsid_t;      /* file system id type */

/*
 * File identifier.
 * These are unique per filesystem on a single machine.
 */

#define MAXFIDSZ        16

struct fid
{
  unsigned short  fid_len;                /* length of data in bytes */
  unsigned short  fid_reserved;           /* force longword alignment */
  char            fid_data[MAXFIDSZ];     /* data (variable length) */
};

/*
 * Generic file handle
 */

struct fhandle
{
  fsid_t  fh_fsid;        /* File system id of mount point */
  struct  fid fh_fid;     /* File sys specific id */
};

typedef struct fhandle  fhandle_t;

struct componentname
{
  /* Arguments to lookup. */

  unsigned long cn_nameiop;   /* namei operation */
  uint64_t cn_flags;          /* flags to namei */
  struct thread *cn_thread;   /* thread requesting lookup */
  struct ucred *cn_cred;      /* credentials */
  int cn_lkflags;             /* Lock flags LK_EXCLUSIVE or LK_SHARED */

  /* Shared between lookup and commit routines. */

  char *cn_pnbuf;             /* pathname buffer */
  char *cn_nameptr;           /* pointer to looked up name */
  long cn_namelen;            /* length of looked up component */
  long cn_consume;            /* chars to consume in lookup() */
};

#endif /* __FS_NFS_RPC_TYPES_H */
