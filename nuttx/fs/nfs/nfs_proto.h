/****************************************************************************
 * fs/nfs/nfs_proto.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
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

#ifndef __FS_NFS_NFS_PROTO_H
#define __FS_NFS_NFS_PROTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Constants as defined in the Sun NFS Version 2 and 3 specs.
 * "NFS: Network File System Protocol Specification" RFC1094
 * and in the "NFS: Network File System Version 3 Protocol
 * Specification"
 */

#define NFS_PORT                2049
#define NFS_PROG                100003
#define NFS_VER2                2
#define NFS_VER3                3
#define NFS_VER4                4
#define NFS_V2MAXDATA           8192
#define NFS_MAXDGRAMDATA        32768
#define MAXBSIZE                64000
#define NFS_MAXDATA             MAXBSIZE
#define NFS_MAXPATHLEN          1024
#define NFS_MAXNAMLEN           255
#define NFS_MAXPKTHDR           404
#define NFS_MAXPACKET           (NFS_MAXPKTHDR + NFS_MAXDATA)
#define NFS_MINPACKET           20
#define NFS_FABLKSIZE           512   /* Size in bytes of a block wrt fa_blocks */

/* Stat numbers for rpc returns (version 2 and 3) */

#define NFS_OK                  0
#define NFSERR_PERM             1
#define NFSERR_NOENT            2
#define NFSERR_IO               5
#define NFSERR_NXIO             6
#define NFSERR_ACCES            13
#define NFSERR_EXIST            17
#define NFSERR_XDEV             18    /* Version 3 only */
#define NFSERR_NODEV            19
#define NFSERR_NOTDIR           20
#define NFSERR_ISDIR            21
#define NFSERR_INVAL            22    /* Version 3 only */
#define NFSERR_FBIG             27
#define NFSERR_NOSPC            28
#define NFSERR_ROFS             30
#define NFSERR_MLINK            31    /* Version 3 only */
#define NFSERR_NAMETOL          63
#define NFSERR_NOTEMPTY         66
#define NFSERR_DQUOT            69
#define NFSERR_STALE            70
#define NFSERR_REMOTE           71    /* Version 3 only */
#define NFSERR_WFLUSH           99    /* Version 2 only */
#define NFSERR_BADHANDLE        10001 /* The rest Version 3 only */
#define NFSERR_NOT_SYNC         10002
#define NFSERR_BAD_COOKIE       10003
#define NFSERR_NOTSUPP          10004
#define NFSERR_TOOSMALL         10005
#define NFSERR_SERVERFAULT      10006
#define NFSERR_BADTYPE          10007
#define NFSERR_JUKEBOX          10008
#define NFSERR_TRYLATER         NFSERR_JUKEBOX
#define NFSERR_STALEWRITEVERF   30001 /* Fake return for nfs_commit() */

#define NFSERR_RETVOID          0x20000000    /* Return void, not error */
#define NFSERR_AUTHERR          0x40000000    /* Mark an authentication error */
#define NFSERR_RETERR           0x80000000    /* Mark an error return for V3 */

/* Sizes in bytes of various nfs rpc components */

#define NFSX_UNSIGNED           4

/* specific to NFS Version 2 */

#define NFSX_V2FH               32
#define NFSX_V2FATTR            68
#define NFSX_V2SATTR            32
#define NFSX_V2COOKIE           4
#define NFSX_V2STATFS           20

/* specific to NFS Version 3 */

#define NFSX_V3FH               (sizeof (fhandle_t))  /* size this server uses */
#define NFSX_V3FHMAX            64    /* max. allowed by protocol */
#define NFSX_V3FATTR            84
#define NFSX_V3SATTR            60    /* max. all fields filled in */
#define NFSX_V3SRVSATTR         (sizeof (struct nfsv3_sattr))
#define NFSX_V3POSTOPATTR       (NFSX_V3FATTR + NFSX_UNSIGNED)
#define NFSX_V3WCCDATA          (NFSX_V3POSTOPATTR + 8 * NFSX_UNSIGNED)
#define NFSX_V3COOKIEVERF       8
#define NFSX_V3WRITEVERF        8
#define NFSX_V3CREATEVERF       8
#define NFSX_V3STATFS           52
#define NFSX_V3FSINFO           48
#define NFSX_V3PATHCONF         24

/* Variants for both versions */

#define NFSX_FH(v3)             ((v3) ? (NFSX_V3FHMAX + NFSX_UNSIGNED) : \
                                        NFSX_V2FH)
#define NFSX_SRVFH(v3)          ((v3) ? NFSX_V3FH : NFSX_V2FH)
#define NFSX_FATTR(v3)          ((v3) ? NFSX_V3FATTR : NFSX_V2FATTR)
#define NFSX_PREOPATTR(v3)      ((v3) ? (7 * NFSX_UNSIGNED) : 0)
#define NFSX_POSTOPATTR(v3)     ((v3) ? (NFSX_V3FATTR + NFSX_UNSIGNED) : 0)
#define NFSX_POSTOPORFATTR(v3)  ((v3) ? (NFSX_V3FATTR + NFSX_UNSIGNED) : \
                                        NFSX_V2FATTR)
#define NFSX_WCCDATA(v3)        ((v3) ? NFSX_V3WCCDATA : 0)
#define NFSX_WCCORFATTR(v3)     ((v3) ? NFSX_V3WCCDATA : NFSX_V2FATTR)
#define NFSX_SATTR(v3)          ((v3) ? NFSX_V3SATTR : NFSX_V2SATTR)
#define NFSX_COOKIEVERF(v3)     ((v3) ? NFSX_V3COOKIEVERF : 0)
#define NFSX_WRITEVERF(v3)      ((v3) ? NFSX_V3WRITEVERF : 0)
#define NFSX_READDIR(v3)        ((v3) ? (5 * NFSX_UNSIGNED) : \
                                        (2 * NFSX_UNSIGNED))
#define NFSX_STATFS(v3)         ((v3) ? NFSX_V3STATFS : NFSX_V2STATFS)

/* NFS RPC procedure numbers (before version mapping) */

#define NFSPROC_NULL            0
#define NFSPROC_GETATTR         1
#define NFSPROC_SETATTR         2
#define NFSPROC_LOOKUP          3
#define NFSPROC_ACCESS          4
#define NFSPROC_READLINK        5
#define NFSPROC_READ            6
#define NFSPROC_WRITE           7
#define NFSPROC_CREATE          8
#define NFSPROC_MKDIR           9
#define NFSPROC_SYMLINK         10
#define NFSPROC_MKNOD           11
#define NFSPROC_REMOVE          12
#define NFSPROC_RMDIR           13
#define NFSPROC_RENAME          14
#define NFSPROC_LINK            15
#define NFSPROC_READDIR         16
#define NFSPROC_READDIRPLUS     17
#define NFSPROC_FSSTAT          18
#define NFSPROC_FSINFO          19
#define NFSPROC_PATHCONF        20
#define NFSPROC_COMMIT          21
#define NFSPROC_NOOP            22
#define NFS_NPROCS              23

/* Actual Version 2 procedure numbers */

#define NFSV2PROC_NULL          0
#define NFSV2PROC_GETATTR       1
#define NFSV2PROC_SETATTR       2
#define NFSV2PROC_NOOP          3
#define NFSV2PROC_ROOT          NFSV2PROC_NOOP/* Obsolete */
#define NFSV2PROC_LOOKUP        4
#define NFSV2PROC_READLINK      5
#define NFSV2PROC_READ          6
#define NFSV2PROC_WRITECACHE    NFSV2PROC_NOOP/* Obsolete */
#define NFSV2PROC_WRITE         8
#define NFSV2PROC_CREATE        9
#define NFSV2PROC_REMOVE        10
#define NFSV2PROC_RENAME        11
#define NFSV2PROC_LINK          12
#define NFSV2PROC_SYMLINK       13
#define NFSV2PROC_MKDIR         14
#define NFSV2PROC_RMDIR         15
#define NFSV2PROC_READDIR       16
#define NFSV2PROC_STATFS        17

/* Constants used by the Version 3 protocol for various RPCs */

#define NFSV3SATTRTIME_DONTCHANGE 0
#define NFSV3SATTRTIME_TOSERVER   1
#define NFSV3SATTRTIME_TOCLIENT   2

#define NFSV3ACCESS_READ          0x01
#define NFSV3ACCESS_LOOKUP        0x02
#define NFSV3ACCESS_MODIFY        0x04
#define NFSV3ACCESS_EXTEND        0x08
#define NFSV3ACCESS_DELETE        0x10
#define NFSV3ACCESS_EXECUTE       0x20

#define NFSV3WRITE_UNSTABLE       0
#define NFSV3WRITE_DATASYNC       1
#define NFSV3WRITE_FILESYNC       2

#define NFSV3CREATE_UNCHECKED     0
#define NFSV3CREATE_GUARDED       1
#define NFSV3CREATE_EXCLUSIVE     2

#define NFSV3FSINFO_LINK          0x01
#define NFSV3FSINFO_SYMLINK       0x02
#define NFSV3FSINFO_HOMOGENEOUS   0x08
#define NFSV3FSINFO_CANSETTIME    0x10

/* NFS mount option flags */

#define NFSMNT_SOFT             0x00000001    /* soft mount (hard is default) */
#define NFSMNT_WSIZE            0x00000002    /* set write size */
#define NFSMNT_RSIZE            0x00000004    /* set read size */
#define NFSMNT_TIMEO            0x00000008    /* set initial timeout */
#define NFSMNT_RETRANS          0x00000010    /* set number of request retries */
#define NFSMNT_MAXGRPS          0x00000020    /* set maximum grouplist size */
#define NFSMNT_INT              0x00000040    /* allow interrupts on hard mount */
#define NFSMNT_NOCONN           0x00000080    /* Don't Connect the socket */

/* 0x100 free, was NFSMNT_NQNFS */

#define NFSMNT_NFSV3            0x00000200    /* Use NFS Version 3 protocol */

/* 0x400 free, was NFSMNT_KERB */

#define NFSMNT_DUMBTIMR         0x00000800    /* Don't estimate rtt dynamically */

/* 0x1000 free, was NFSMNT_LEASETERM */

#define NFSMNT_READAHEAD        0x00002000    /* set read ahead */
#define NFSMNT_DEADTHRESH       0x00004000    /* set dead server retry thresh */
#define NFSMNT_RESVPORT         0x00008000    /* Allocate a reserved port */
#define NFSMNT_RDIRPLUS         0x00010000    /* Use Readdirplus for V3 */
#define NFSMNT_READDIRSIZE      0x00020000    /* Set readdir size */
#define NFSMNT_ACREGMIN         0x00040000
#define NFSMNT_ACREGMAX         0x00080000
#define NFSMNT_ACDIRMIN         0x00100000
#define NFSMNT_ACDIRMAX         0x00200000
#define NFSMNT_NOLOCKD          0x00400000    /* Locks are local */
#define NFSMNT_NFSV4            0x00800000    /* Use NFS Version 4 protocol */
#define NFSMNT_HASWRITEVERF     0x01000000    /* NFSv4 Write verifier */
#define NFSMNT_GOTFSINFO        0x00000004    /* Got the V3 fsinfo */
#define NFSMNT_INTERNAL         0xfffc0000    /* Bits set internally */
#define NFSMNT_NOAC             0x00080000    /* Turn off attribute cache */

/* Conversion macros */

#define vtonfsv2_mode(t,m) \
    txdr_unsigned(((t) == VFIFO) ? MAKEIMODE(VCHR, (m)) : \
    MAKEIMODE((t), (m)))
#define vtonfsv3_mode(m)        txdr_unsigned((m) & 07777)
#define nfstov_mode(a)          (fxdr_unsigned(u_int16_t, (a))&07777)
#define vtonfsv2_type(a)        txdr_unsigned(nfsv2_type[((int32_t)(a))])
#define vtonfsv3_type(a)        txdr_unsigned(nfsv3_type[((int32_t)(a))])
#define nfsv2tov_type(a)        nv2tov_type[fxdr_unsigned(uint32_t,(a))&0x7]
#define nfsv3tov_type(a)        nv3tov_type[fxdr_unsigned(uint32_t,(a))&0x7]

#define NFS_MAXFHSIZE           64

/* File identifier */

#define MAXFIDSZ                16

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* File types */

typedef enum
{
  NFNON = 0,
  NFREG = 1,
  NFDIR = 2,
  NFBLK = 3,
  NFCHR = 4,
  NFLNK = 5,
  NFSOCK = 6,
  NFFIFO = 7
} nfstype;

typedef struct
{
  int32_t val[2];
} fsid_t;                      /* file system id type */

/* File identifier.
 * These are unique per filesystem on a single machine.
 */

struct fid
{
  unsigned short fid_len;      /* length of data in bytes */
  unsigned short fid_reserved; /* force longword alignment */
  char fid_data[MAXFIDSZ];     /* data (variable length) */
};

/* Generic file handle */

struct fhandle
{
  fsid_t fh_fsid;              /* File system id of mount point */
  struct fid fh_fid;           /* File sys specific id */
};

typedef struct fhandle fhandle_t;

/* File Handle (32 bytes for version 2), variable up to 64 for version 3. */

union nfsfh
{
//fhandle_t fh_generic;
  uint8_t       fh_bytes[NFSX_V2FH];
};
typedef union nfsfh nfsfh_t;

struct nfsv2_time
{
  uint32_t nfsv2_sec;
  uint32_t nfsv2_usec;
};
typedef struct nfsv2_time nfstime2;

struct nfsv3_time
{
  uint32_t nfsv3_sec;
  uint32_t nfsv3_nsec;
};
typedef struct nfsv3_time nfstime3;

/* Quads are defined as arrays of 2 longs to ensure dense packing for the
 * protocol and to facilitate xdr conversion.
 */

struct nfs_uquad
{
  uint32_t nfsuquad[2];
};
typedef struct nfs_uquad nfsuint64;

/* NFS Version 3 special file number. */

struct nfsv3_spec
{
  uint32_t specdata1;
  uint32_t specdata2;
};
typedef struct nfsv3_spec nfsv3spec;

/* File attributes and setable attributes. These structures cover both
 * NFS version 2 and the version 3 protocol. Note that the union is only
 * used so that one pointer can refer to both variants. These structures
 * go out on the wire and must be densely packed, so no quad data types
 * are used. (all fields are longs or u_longs or structures of same)
 * NB: You can't do sizeof(struct nfs_fattr), you must use the
 *     NFSX_FATTR(v3) macro.
 */

struct nfs_fattr
{
  uint32_t fa_type;
  uint32_t fa_mode;
  uint32_t fa_nlink;
  uint32_t fa_uid;
  uint32_t fa_gid;
  union
  {
    /*struct
    {
      uint32_t nfsv2fa_size;
      uint32_t nfsv2fa_blocksize;
      uint32_t nfsv2fa_rdev;
      uint32_t nfsv2fa_blocks;
      uint32_t nfsv2fa_fsid;
      uint32_t nfsv2fa_fileid;
      nfstime2 nfsv2fa_atime;
      nfstime2 nfsv2fa_mtime;
      nfstime2 nfsv2fa_ctime;
    } fa_nfsv2;*/
    struct
    {
      nfsuint64 nfsv3fa_size;
      nfsuint64 nfsv3fa_used;
      nfsv3spec nfsv3fa_rdev;
      nfsuint64 nfsv3fa_fsid;
      nfsuint64 nfsv3fa_fileid;
      nfstime3 nfsv3fa_atime;
      nfstime3 nfsv3fa_mtime;
      nfstime3 nfsv3fa_ctime;
    } fa_nfsv3;
  } fa_un;
};

/* And some ugly defines for accessing union components */

/*#define fa2_size                fa_un.fa_nfsv2.nfsv2fa_size
#define fa2_blocksize           fa_un.fa_nfsv2.nfsv2fa_blocksize
#define fa2_rdev                fa_un.fa_nfsv2.nfsv2fa_rdev
#define fa2_blocks              fa_un.fa_nfsv2.nfsv2fa_blocks
#define fa2_fsid                fa_un.fa_nfsv2.nfsv2fa_fsid
#define fa2_fileid              fa_un.fa_nfsv2.nfsv2fa_fileid
#define fa2_atime               fa_un.fa_nfsv2.nfsv2fa_atime
#define fa2_mtime               fa_un.fa_nfsv2.nfsv2fa_mtime
#define fa2_ctime               fa_un.fa_nfsv2.nfsv2fa_ctime*/
#define fa3_size                fa_un.fa_nfsv3.nfsv3fa_size
#define fa3_used                fa_un.fa_nfsv3.nfsv3fa_used
#define fa3_rdev                fa_un.fa_nfsv3.nfsv3fa_rdev
#define fa3_fsid                fa_un.fa_nfsv3.nfsv3fa_fsid
#define fa3_fileid              fa_un.fa_nfsv3.nfsv3fa_fileid
#define fa3_atime               fa_un.fa_nfsv3.nfsv3fa_atime
#define fa3_mtime               fa_un.fa_nfsv3.nfsv3fa_mtime
#define fa3_ctime               fa_un.fa_nfsv3.nfsv3fa_ctime

struct nfsv2_sattr
{
  uint32_t sa_mode;
  uint32_t sa_uid;
  uint32_t sa_gid;
  uint32_t sa_size;
  nfstime2 sa_atime;
  nfstime2 sa_mtime;
};

/* NFS Version 3 sattr structure for the new node creation case. */

struct nfsv3_sattr
{
  uint32_t sa_modetrue;
  uint32_t sa_mode;
  uint32_t sa_uidfalse;
//uint32_t sa_uid;
  uint32_t sa_gidfalse;
//uint32_t sa_gid;
  uint32_t sa_sizefalse;
  uint32_t sa_atimetype;
//nfstime3 sa_atime;
  uint32_t sa_mtimetype;
//nfstime3 sa_mtime;
};

struct nfs_statfs
{
  struct nfs_fattr obj_attributes;
  union
  {
    /*struct
    {
      uint32_t nfsv2sf_tsize;
      uint32_t nfsv2sf_bsize;
      uint32_t nfsv2sf_blocks;
      uint32_t nfsv2sf_bfree;
      uint32_t nfsv2sf_bavail;
    } sf_nfsv2;*/
    struct
    {
      nfsuint64 nfsv3sf_tbytes;
      nfsuint64 nfsv3sf_fbytes;
      nfsuint64 nfsv3sf_abytes;
      nfsuint64 nfsv3sf_tfiles;
      nfsuint64 nfsv3sf_ffiles;
      nfsuint64 nfsv3sf_afiles;
      uint32_t nfsv3sf_invarsec;
    } sf_nfsv3;
  } sf_un;
};

/*#define sf_tsize        sf_un.sf_nfsv2.nfsv2sf_tsize
#define sf_bsize        sf_un.sf_nfsv2.nfsv2sf_bsize
#define sf_blocks       sf_un.sf_nfsv2.nfsv2sf_blocks
#define sf_bfree        sf_un.sf_nfsv2.nfsv2sf_bfree
#define sf_bavail       sf_un.sf_nfsv2.nfsv2sf_bavail*/
#define sf_tbytes       sf_un.sf_nfsv3.nfsv3sf_tbytes
#define sf_fbytes       sf_un.sf_nfsv3.nfsv3sf_fbytes
#define sf_abytes       sf_un.sf_nfsv3.nfsv3sf_abytes
#define sf_tfiles       sf_un.sf_nfsv3.nfsv3sf_tfiles
#define sf_ffiles       sf_un.sf_nfsv3.nfsv3sf_ffiles
#define sf_afiles       sf_un.sf_nfsv3.nfsv3sf_afiles
#define sf_invarsec     sf_un.sf_nfsv3.nfsv3sf_invarsec

struct post_attr
{
  uint32_t           obj_attributesfalse;
  struct nfs_fattr   attributes;
};

struct nfsv3_fsinfo
{
//struct post_attr   obj_attributes;
  uint32_t           obj_attributesfalse;
  uint32_t           fs_rtmax;
  uint32_t           fs_rtpref;
  uint32_t           fs_rtmult;
  uint32_t           fs_wtmax;
  uint32_t           fs_wtpref;
  uint32_t           fs_wtmult;
  uint32_t           fs_dtpref;
  nfsuint64          fs_maxfilesize;
  nfstime3           fs_timedelta;
  uint32_t           fs_properties;
};

/* NFS procedures args */

struct wcc_attr
{
  nfsuint64          size;
  nfstime3           mtime;
  nfstime3           ctime;
};

struct wcc_data
{
  struct wcc_attr    before;
  struct nfs_fattr   after;
};

struct file_handle
{
  uint32_t           length;
  nfsfh_t            handle;
};

struct diropargs3
{
  struct file_handle dir;
  uint32_t           length;
  char               name[64];
};

struct CREATE3args
{
  struct diropargs3  where;
  uint32_t           create_mode;
  struct nfsv3_sattr how;
};

struct CREATE3resok
{
  struct file_handle fshandle;
  struct nfs_fattr   attributes;
  struct wcc_data    dir_wcc;
};

struct READ3args
{
  nfstype            file;
  uint64_t           offset;
  uint32_t           count;
};

struct READ3resok
{
  struct nfs_fattr   file_attributes;
  uint32_t           count;
  bool               eof;
  const char        *data;
};


struct WRITE3args
{
  nfstype            file;
  uint64_t           offset;
  uint32_t           count;
  uint32_t           stable;
  const char        *data;
};

struct WRITE3resok
{
  struct wcc_data    file_wcc;
  uint32_t           count;
  uint32_t           committed;
  uint8_t            verf[NFSX_V3WRITEVERF];
};

struct REMOVE3args
{
  struct diropargs3  object;
};

struct REMOVE3resok
{
  struct wcc_data    dir_wcc;
};

struct RENAME3args
{
  struct diropargs3  from;
  struct diropargs3  to;
};

struct RENAME3resok
{
  struct wcc_data    fromdir_wcc;
  struct wcc_data    todir_wcc;
};

struct MKDIR3args
{
  struct diropargs3  where;
  struct nfsv3_sattr attributes;
};

struct MKDIR3resok
{
  struct file_handle fshandle;
  uint32_t           obj_attributesfalse;
  struct nfs_fattr   obj_attributes;
  struct wcc_data    dir_wcc;
};

struct RMDIR3args 
{
  struct diropargs3  object;
};

struct RMDIR3resok 
{
  struct wcc_data    dir_wcc;
};

struct READDIR3args 
{
  struct file_handle dir;
  nfsuint64          cookie;
  uint8_t            cookieverf[NFSX_V3COOKIEVERF];
  uint32_t           count;
};

/* The READDIR reply is variable length and consists of multiple entries, each
 * of form:
 *
 *  EOF - OR -
 *
 *  File ID (8 bytes)
 *  Name length (4 bytes)
 *  Name string (varaiable size but in multiples of 4 bytes)
 *  Cookie (8 bytes)
 *  next entry (4 bytes)
 */
 
struct READDIR3resok 
{
  struct nfs_fattr   dir_attributes;
  uint8_t            cookieverf[NFSX_V3COOKIEVERF];
  uint32_t           reply[1]; /* Variable length reply begins here */
};

struct FS3args
{
  struct file_handle fsroot;
};

#endif /* __FS_NFS_NFS_PROTO_H */
