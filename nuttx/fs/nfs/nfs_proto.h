/****************************************************************************
 * fs/nfs/nfs_proto.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/* Specific to NFS Version 3 */

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


/* Constants used by the Version 3 protocol for various RPCs */

#define NFSV3SATTRTIME_DONTCHANGE 0
#define NFSV3SATTRTIME_TOSERVER   1
#define NFSV3SATTRTIME_TOCLIENT   2

#define NFSV3ACCESS_READ         0x01
#define NFSV3ACCESS_LOOKUP       0x02
#define NFSV3ACCESS_MODIFY       0x04
#define NFSV3ACCESS_EXTEND       0x08
#define NFSV3ACCESS_DELETE       0x10
#define NFSV3ACCESS_EXECUTE      0x20

#define NFSV3WRITE_UNSTABLE      0
#define NFSV3WRITE_DATASYNC      1
#define NFSV3WRITE_FILESYNC      2

#define NFSV3CREATE_UNCHECKED    0
#define NFSV3CREATE_GUARDED      1
#define NFSV3CREATE_EXCLUSIVE    2

#define NFSV3FSINFO_LINK         0x01
#define NFSV3FSINFO_SYMLINK      0x02
#define NFSV3FSINFO_HOMOGENEOUS  0x08
#define NFSV3FSINFO_CANSETTIME   0x10

/* Conversion macros */

#define vtonfsv3_mode(m)         txdr_unsigned((m) & 07777)
#define nfstov_mode(a)           (fxdr_unsigned(u_int16_t, (a))&07777)
#define vtonfsv3_type(a)         txdr_unsigned(nfsv3_type[((int32_t)(a))])
#define nfsv3tov_type(a)         nv3tov_type[fxdr_unsigned(uint32_t,(a))&0x7]

/* Mode bit values */

#define NFSMODE_IXOTH            (1 << 0)      /* Execute permission for others on a file */
#define NFSMODE_IWOTH            (1 << 1)      /* Write permission for others */
#define NFSMODE_IROTH            (1 << 2)      /* Read permission for others */
#define NFSMODE_IXGRP            (1 << 3)      /* Execute permission for group on a file */
#define NFSMODE_IWGRP            (1 << 4)      /* Write permission for group */
#define NFSMODE_IRGRP            (1 << 5)      /* Read permission for group */
#define NFSMODE_IXUSR            (1 << 6)      /* Execute permission for owner on a file */
#define NFSMODE_IWUSR            (1 << 7)      /* Write permission for owner */
#define NFSMODE_IRUSR            (1 << 8)      /* Read permission for owner */
#define NFSMODE_SAVETEXT         (1 << 9)      /* Save swapped text */
#define NFSMODE_ISGID            (1 << 10)     /* Set group ID on execution */
#define NFSMODE_ISUID            (1 << 11)     /* Set user ID on execution */

/* File identifier */

#define MAXFIDSZ                 16

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* File types */

typedef enum
{
  NFNON  = 0,                  /* Unknown type */
  NFREG  = 1,                  /* Regular file */
  NFDIR  = 2,                  /* Directory */
  NFBLK  = 3,                  /* Block special device file */
  NFCHR  = 4,                  /* Character special device file */
  NFLNK  = 5,                  /* Symbolic link */
  NFSOCK = 6,                  /* Socket */
  NFFIFO = 7                   /* Named FIFO */
} nfstype;

/* File Handle variable is up to 64 bytes for version 3. This structures a 
 * ariable sized and are provided only for setting aside maximum memory
 * allocations for a file handle.
 */

struct nfsfh
{
  uint8_t fh_bytes[NFSX_V3FHMAX];
};
typedef struct nfsfh nfsfh_t;
#define SIZEOF_nfsfh_t(n) (n)

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
 */

struct nfs_fattr
{
  uint32_t           fa_type;
  uint32_t           fa_mode;
  uint32_t           fa_nlink;
  uint32_t           fa_uid;
  uint32_t           fa_gid;
  nfsuint64          fa_size;
  nfsuint64          fa_used;
  nfsv3spec          fa_rdev;
  nfsuint64          fa_fsid;
  nfsuint64          fa_fileid;
  nfstime3           fa_atime;
  nfstime3           fa_mtime;
  nfstime3           fa_ctime;
};

/* NFS Version 3 sattr structure for the new node creation case.  This is the
 * maximum size of the attributes; the actual size may vary if values are not
 * include.
 */

struct nfsv3_sattr
{
  uint32_t           sa_modefollows;   /* TRUE: Mode value follows */
  uint32_t           sa_mode;          /* Mode value */
  uint32_t           sa_uidfollows;    /* TRUE: Uid value follows */
  uint32_t           sa_uid;           /* Uid value */
  uint32_t           sa_gidfollows;    /* TRUE: Mode value follows */
  uint32_t           sa_gid;           /* Mode value */
  uint32_t           sa_sizefollows;   /* TRUE: Size value follows */
  uint32_t           sa_size;          /* Size value */
  uint32_t           sa_atimetype;     /* Don't change, use server timer, or use client time  */
  nfstime3           sa_atime;         /* Client time */
  uint32_t           sa_mtimetype;     /* Don't change, use server timer, or use client time  */
  nfstime3           sa_mtime;         /* Client time */
};

struct nfs_statfs
{
  struct nfs_fattr   obj_attributes;
  nfsuint64          sf_tbytes;
  nfsuint64          sf_fbytes;
  nfsuint64          sf_abytes;
  nfsuint64          sf_tfiles;
  nfsuint64          sf_ffiles;
  nfsuint64          sf_afiles;
  uint32_t           sf_invarsec;
};
 
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
  uint32_t           wcc_attr_follows;       /* True if data follows */
  struct wcc_attr    before;
  uint32_t           nfs_attr_follow;        /* True if attributes present */
  struct nfs_fattr   after;
};

struct file_handle
{
  uint32_t           length;
  nfsfh_t            handle;
};
#define SIZEOF_file_handle(n) (sizeof(uint32_t) + SIZEOF_nfsfh_t(n))

struct diropargs3
{
  struct file_handle fhandle;                  /* Variable length */
  uint32_t           length;                   /* Size of name[] */
  uint32_t           name[(NAME_MAX+3) >> 2];  /* Variable length */
};

struct CREATE3args
{
  struct diropargs3  where;
  uint32_t           create_mode;
  struct nfsv3_sattr how;
};

struct CREATE3resok
{
  uint32_t           handle_follows;           /* True, handle follows */
  struct file_handle fhandle;                  /* Variable length */
  uint32_t           attributes_follows;       /* True, attributes follows */
  struct nfs_fattr   attributes;               /* File attributes */
  struct wcc_data    dir_wcc;
};

/* The actual size of the lookup argument is variable.  These structures are, therefore,
 * only useful in setting aside maximum memory usage for the LOOKUP arguments.
 */

struct LOOKUP3filename
{
  uint32_t           namelen;                  /* Size of name[] */
  uint32_t           name[(NAME_MAX+3) >> 2];  /* Variable length */
};

struct LOOKUP3args
{
  struct file_handle     dirhandle;            /* Variable length */
  struct LOOKUP3filename name;                 /* Variable length  */
};

struct SETATTR3args
{
  struct file_handle     fhandle;              /* Variable length */
  struct nfsv3_sattr     new_attributes;       /* Variable length */
  uint32_t               guard;                /* Guard value */
};

struct SETATTR3resok
{
  struct wcc_data         wcc_data;
};

/* Actual size of LOOKUP3args */

#define SIZEOF_LOOKUP3filename(b) (sizeof(uint32_t) + (((b)+3) & ~3))
#define SIZEOF_LOOKUP3args(a,b)   (SIZEOF_file_handle(a) + SIZEOF_LOOKUP3filename(b))

struct LOOKUP3resok
{
  struct file_handle fhandle;
  uint32_t           obj_attributes_follow;
  struct nfs_fattr   obj_attributes;
  uint32_t           dir_attributes_follow;
  struct nfs_fattr   dir_attributes;
};

struct READ3args
{
  struct file_handle fhandle;      /* Variable length */
  uint64_t           offset;
  uint32_t           count;
};

struct nfs_rdhdr_s
{
  uint32_t           attributes_follow;
  struct nfs_fattr   attributes;   /* Will not be present if attributes_follow == 0 */
  uint32_t           count;        /* Number of bytes read */
  uint32_t           eof;          /* Non-zero if at the end of file */
  uint32_t           length;       /* Length of data (same as count?) */
};

struct READ3resok
{
  struct nfs_rdhdr_s hdr;
  uint8_t            data[1];      /* Actual data size depends on count */
};
#define SIZEOF_READ3resok(n) (sizeof(struct nfs_rdhdr_s) + (n))

struct nfs_wrhdr_s
{
  struct file_handle fhandle;     /* Variable length */
  uint64_t           offset;
  uint32_t           count;
  uint32_t           stable;
};

struct WRITE3args
{
  struct nfs_wrhdr_s hdr;
  uint8_t            data[1]; /* Actual data size depends on count */
};
#define SIZEOF_WRITE3args(n) (sizeof(struct nfs_wrhdr_s) + (n))

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
  struct nfsv3_sattr how;
};

struct MKDIR3resok
{
  uint32_t           handle_follows;           /* True, handle follows */
  struct file_handle fhandle;                  /* Variable length */
  uint32_t           attributes_follows;       /* True, attributes follows */
  struct nfs_fattr   attributes;               /* Directory attributes */
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

/* The actual size of the lookup argument is variable.  This structures is, therefore,
 * only useful in setting aside maximum memory usage for the LOOKUP arguments.
 */

struct READDIR3args 
{
  struct file_handle dir;                      /* Variable length */
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
  uint32_t           attributes_follow;
  struct nfs_fattr   dir_attributes;
  uint8_t            cookieverf[NFSX_V3COOKIEVERF];
  uint32_t           value_follows;
  uint32_t           reply[1]; /* Variable length reply begins here */
};

struct FS3args
{
  struct file_handle fsroot;
};

#endif /* __FS_NFS_NFS_PROTO_H */

