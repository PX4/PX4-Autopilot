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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NFS_TICKINTVL      5             /* Desired time for a tick (msec) */
#define NFS_HZ             (CLOCKS_PER_SEC / nfs_ticks)  /* Ticks/sec */
#define NFS_TIMEO          (1 * NFS_HZ)  /* Default timeout = 1 second */
#define NFS_MINTIMEO       (1 * NFS_HZ)  /* Min timeout to use */
#define NFS_MAXTIMEO       (60 * NFS_HZ) /* Max timeout to backoff to */
#define NFS_MINIDEMTIMEO   (5 * NFS_HZ)  /* Min timeout for non-idempotent ops */
#define NFS_TIMEOUTMUL     2             /* Timeout/Delay multiplier */
#define NFS_MAXREXMIT      100           /* Stop counting after this many */
#define NFS_RETRANS        10            /* Num of retrans for soft mounts */
#define NFS_MAXGRPS        16            /* Max. size of groups list */
#define NFS_MINATTRTIMO    5             /* Attribute cache timeout in sec */
#define NFS_MAXATTRTIMO    60
#define NFS_WSIZE          8192          /* Def. write data size <= 8192 */
#define NFS_RSIZE          8192          /* Def. read data size <= 8192 */
#define NFS_READDIRSIZE    8192          /* Def. readdir size */
#define NFS_DEFRAHEAD      1             /* Def. read ahead # blocks */
#define NFS_MAXRAHEAD      4             /* Max. read ahead # blocks */
#define NFS_MAXASYNCDAEMON 20            /* Max. number async_daemons runable */

/* Ideally, NFS_DIRBLKSIZ should be bigger, but I've seen servers with
 * broken NFS/ethernet drivers that won't work with anything bigger (Linux..)
 */

#define NFS_DIRBLKSIZ      1024          /* Must be a multiple of DIRBLKSIZ */
#define NFS_READDIRBLKSIZ  512           /* Size of read dir blocks. XXX */

/* Oddballs */

#define NFS_CMPFH(n, f, s) \
    ((n)->n_fhsize == (s) && !bcmp((void *)(n)->n_fhp, (void *)(f), (s)))
#define NFS_ISV3(i)     (VFSTONFS((i)->f_inode)->nm_flag & NFSMNT_NFSV3)
#define NFS_SRVMAXDATA(n) \
    (((n)->nd_flag & ND_NFSV3) ? (((n)->nd_nam2) ? \
    NFS_MAXDGRAMDATA : NFS_MAXDATA) : NFS_V2MAXDATA)

/* sys/malloc.h needs M_NFSDIROFF, M_NFSRVDESC and M_NFSBIGFH added. */

#ifndef M_NFSRVDESC
#  define M_NFSRVDESC     M_TEMP
#endif
#ifndef M_NFSDIROFF
#  define M_NFSDIROFF     M_TEMP
#endif
#ifndef M_NFSBIGFH
#  define M_NFSBIGFH      M_TEMP
#endif

/* The B_INVAFTERWRITE flag should be set to whatever is required by the
 * buffer cache code to say "Invalidate the block after it is written back".
 */

#define B_INVAFTERWRITE   B_INVAL

/* Flags for nfssvc() system call. */

#define NFSSVC_BIOD       0x002
#define NFSSVC_NFSD       0x004
#define NFSSVC_ADDSOCK    0x008
#define NFSSVC_AUTHIN     0x010
#define NFSSVC_GOTAUTH    0x040
#define NFSSVC_AUTHINFAIL 0x080
#define NFSSVC_MNTD       0x100

/* fs.nfs sysctl(3) identifiers */

#define NFS_NFSSTATS    1     /* struct: struct nfsstats */
#define NFS_NIOTHREADS  2     /* number of i/o threads */
#define NFS_MAXID       3

#define FS_NFS_NAMES { \
    { 0, 0 }, \
    { "nfsstats", CTLTYPE_STRUCT }, \
    { "iothreads", CTLTYPE_INT } \
}

#define NFSINT_SIGMASK  (sigmask(SIGINT)|sigmask(SIGTERM)|sigmask(SIGKILL)| \
                         sigmask(SIGHUP)|sigmask(SIGQUIT))

/* Socket errors ignored for connectionless sockets??
 * For now, ignore them all
 */

#define NFSIGNORE_SOERROR(s, e) \
    ((e) != EINTR && (e) != ERESTART && (e) != EWOULDBLOCK && \
    ((s) & PR_CONNREQUIRED) == 0)

/* Flag values for r_flags */

#define R_TIMING        0x01/* timing request (in mntp) */
#define R_SENT          0x02/* request has been sent */
#define R_SOFTTERM      0x04/* soft mnt, too many retries */
#define R_INTR          0x08/* intr mnt, signal pending */
#define R_SOCKERR       0x10/* Fatal error on socket */
#define R_TPRINTFMSG    0x20/* Did a tprintf msg. */
#define R_MUSTRESEND    0x40/* Must resend request */

/* On fast networks, the estimator will try to reduce the
 * timeout lower than the latency of the server's disks,
 * which results in too many timeouts, so cap the lower
 * bound.
 */

#define NFS_MINRTO      (NFS_HZ >> 2)

/* Keep the RTO from increasing to unreasonably large values
 * when a server is not responding.
 */

#define NFS_MAXRTO      (20 * NFS_HZ)

#define NFS_MAX_TIMER   (NFS_WRITE_TIMER)
#define NFS_INITRTT     (NFS_HZ << 3)

/* Bits for "ns_flag" */

#define SLP_VALID       0x01/* connection is usable */
#define SLP_DOREC       0x02/* receive operation required */
#define SLP_NEEDQ       0x04/* connection has data to queue from socket */
#define SLP_DISCONN     0x08/* connection is closed */
#define SLP_GETSTREAM   0x10/* extracting RPC from TCP connection */
#define SLP_LASTFRAG    0x20/* last fragment received on TCP connection */
#define SLP_ALLFLAGS    0xff/* convenience */

#define SLP_INIT        0x01/* NFS data undergoing initialization */
#define SLP_WANTINIT    0x02/* thread waiting on NFS initialization */

/* Bits for "nfsd_flag" */

#define NFSD_WAITING    0x01
#define NFSD_REQINPROG  0x02
#define NFSD_NEEDAUTH   0x04
#define NFSD_AUTHFAIL   0x08

/* Bits for "nd_flag" */

#define ND_NFSV3        0x08
#define NFSD_CHECKSLP   0x01

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

#define NFS_ARGSVERSION         3             /* change when nfs_args changes */
#define NFS_NPROCS              23
#define NFS_MAXFHSIZE   64

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* File Handle (32 bytes for version 2), variable up to 64 for version 3. */

union nfsfh
{
  unsigned char fh_bytes[NFS_MAXFHSIZE];
};
typedef union nfsfh nfsfh_t;
 
#undef COMP
#ifdef COMP
/* Structures for the nfssvc(2) syscall.
 * Not that anyone besides nfsd(8) should ever use it.
 */

struct nfsd_args
{
  int sock;                   /* Socket to serve */
  void *name;                 /* Client addr for connection based sockets */
  int namelen;                /* Length of name */
};

struct nfsd_srvargs
{
  struct nfsd *nsd_nfsd;        /* Pointer to in kernel nfsd struct */
  uid_t nsd_uid;                /* Effective uid mapped to cred */
  uint32_t nsd_haddr;           /* IP address of client */
  int nsd_authlen;              /* Length of auth string (ret) */
  unsigned char *nsd_authstr;   /* Auth string (ret) */
  int nsd_verflen;              /* and the verifier */
  unsigned char *nsd_verfstr;
  struct timeval nsd_timestamp; /* timestamp from verifier */
  uint32_t nsd_ttl;             /* credential ttl (sec) */
};
#endif

#ifdef COMP
/* The set of signals the interrupt an I/O in progress for NFSMNT_INT mounts.
 * What should be in this set is open to debate, but I believe that since
 * I/O system calls on ufs are never interrupted by signals the set should
 * be minimal. My reasoning is that many current programs that use signals
 * such as SIGALRM will not expect file I/O system calls to be interrupted
 * by them and break.
 */

/* Nfs outstanding request list element */

struct nfsmount;
struct nfsreq
{
  dq_entry_t r_chain;
  struct nfsmount *r_nmp;
  uint32_t r_xid;
  int r_flags;                /* flags on request, see below */
  int r_rexmit;               /* current retrans count */
  int r_timer;                /* tick counter on reply */
  int r_procnum;              /* NFS procedure number */
  int r_rtt;                  /* RTT for rpc */
};

enum nfs_rto_timers
{
  NFS_DEFAULT_TIMER,
  NFS_GETATTR_TIMER,
  NFS_LOOKUP_TIMER,
  NFS_READ_TIMER,
  NFS_WRITE_TIMER,
};

/* Network address hash list element */

union nethostaddr
{
  uint32_t had_inetaddr;
};

struct nfssvc_sock
{
 // TAILQ_ENTRY(nfssvc_sock) ns_chain;  /* List of all nfssvc_sock's */
  struct file *ns_fp;         /* fp from the... */
  struct socket *ns_so;       /* ...socket this struct wraps */
  int ns_flag;                /* socket status flags */
  int ns_solock;              /* lock for connected socket */
  int ns_cc;                  /* actual chars queued */
  int ns_reclen;              /* length of first queued record */
  uint32_t ns_sref;           /* # of refs to this struct */
};

/* One of these structures is allocated for each nfsd. */

struct nfsd
{
  //TAILQ_ENTRY(nfsd) nfsd_chain;  /* List of all nfsd's */
  int nfsd_flag;                   /* NFSD_ flags */
  struct nfssvc_sock *nfsd_slp;    /* Current socket */
  struct nfsrv_descript *nfsd_nd;  /* Associated nfsrv_descript */
};

/* This structure is used by the server for describing each request. */

struct nfsrv_descript
{
  unsigned int nd_procnum;    /* RPC # */
  int nd_flag;                /* nd_flag */
  int nd_repstat;             /* Reply status */
  uint32_t nd_retxid;         /* Reply xid */
};
#endif

/* Arguments to mount NFS */

struct nfs_args
{
  uint8_t version;          /* args structure version number */
  struct  sockaddr *addr;   /* file server address */
  uint8_t addrlen;          /* length of address */
  uint8_t sotype;           /* Socket type */
  uint8_t proto;            /* and Protocol */
  nfsfh_t fh;               /* File handle to be mounted */
  int     fhsize;           /* Size, in bytes, of fh */
  int     flags;            /* flags */
  int     wsize;            /* write size in bytes */
  int     rsize;            /* read size in bytes */
  int     readdirsize;      /* readdir size in bytes */
  int     timeo;            /* initial timeout in .1 secs */
  int     retrans;          /* times to retry send */
  int     maxgrouplist;     /* Max. size of group list */
  int     readahead;        /* # of blocks to readahead */
  int     leaseterm;        /* Term (sec) of lease */
  int     deadthresh;       /* Retrans threshold */
//char   *hostname;         /* server's name */
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
