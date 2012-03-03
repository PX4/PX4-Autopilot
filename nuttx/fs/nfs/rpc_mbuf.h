/****************************************************************************
 * fs/nfs/rpc_mbuf.h
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

#ifndef __FS_NFS_RPC_MBUF_H
#define __FS_NFS_RPC_MBUF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Mbufs are of a single size, MSIZE (sys/param.h), which
 * includes overhead.  An mbuf may add a single "mbuf cluster" of size
 * MCLBYTES (also in sys/param.h), which has no additional overhead
 * and is used instead of the internal data area; this is done when
 * at least MINCLSIZE of data must be stored.
 */

#define  MSIZE        128
#define  MCLSHIFT     11                             /* Convert bytes to m_buf clusters */
                                                     /* 2K cluster can hold Ether frame */
#define  MCLBYTES     (1 << MCLSHIFT)                /* Size of a m_buf cluster */
#define  MLEN         (MSIZE - sizeof(struct m_hdr)) /* Normal data len */
#define  MHLEN        (MLEN - sizeof(struct pkthdr)) /* Data len w/pkthdr */

/* smallest amount to put in cluster */

#define MINCLSIZE     (MHLEN + MLEN + 1)
#define M_MAXCOMPRESS (MHLEN / 2)                    /* Max amount to copy for compression */

/* Macros for type conversion
 * mtod(m,t) -    convert mbuf pointer to data pointer of correct type
 */

#define  mtod(m,t)    ((t)((m)->m_data))

/* mbuf flags */

#define M_EXT         0x0001  /* has associated external storage */
#define M_PKTHDR      0x0002  /* start of record */
#define M_EOR         0x0004  /* end of record */
#define M_CLUSTER     0x0008  /* external storage is a cluster */
#define M_PROTO1      0x0010  /* protocol-specific */

/* mbuf pkthdr flags, also in m_flags */

#define M_VLANTAG     0x0020  /* ether_vtag is valid */
#define M_LOOP        0x0040  /* for Mbuf statistics */
#define M_FILDROP     0x0080  /* dropped by bpf filter */
#define M_BCAST       0x0100  /* send/received as link-level broadcast */
#define M_MCAST       0x0200  /* send/received as link-level multicast */
#define M_CONF        0x0400  /* payload was encrypted (ESP-transport) */
#define M_AUTH        0x0800  /* payload was authenticated (AH or ESP auth) */
#define M_TUNNEL      0x1000  /* IP-in-IP added by tunnel mode IPsec */
#define M_AUTH_AH     0x2000  /* header was authenticated (AH) */
#define M_COMP        0x4000  /* header was decompressed */
#define M_LINK0       0x8000  /* link layer specific flag */

/* Flags copied when copying m_pkthdr */

#define M_COPYFLAGS   (M_PKTHDR|M_EOR|M_PROTO1|M_BCAST|M_MCAST|M_CONF|M_COMP|\
                       M_AUTH|M_LOOP|M_TUNNEL|M_LINK0|M_VLANTAG|M_FILDROP)

/* Checksumming flags */

#define M_IPV4_CSUM_OUT    0x0001  /* IPv4 checksum needed */
#define M_TCP_CSUM_OUT     0x0002  /* TCP checksum needed */
#define M_UDP_CSUM_OUT     0x0004  /* UDP checksum needed */
#define M_IPV4_CSUM_IN_OK  0x0008  /* IPv4 checksum verified */
#define M_IPV4_CSUM_IN_BAD 0x0010  /* IPv4 checksum bad */
#define M_TCP_CSUM_IN_OK   0x0020  /* TCP/IPv4 checksum verified */
#define M_TCP_CSUM_IN_BAD  0x0040  /* TCP/IPv4 checksum bad */
#define M_UDP_CSUM_IN_OK   0x0080  /* UDP/IPv4 checksum verified */
#define M_UDP_CSUM_IN_BAD  0x0100  /* UDP/IPv4 checksum bad */
#define M_ICMP_CSUM_OUT    0x0200  /* ICMP checksum needed */
#define M_ICMP_CSUM_IN_OK  0x0400  /* ICMP checksum verified */
#define M_ICMP_CSUM_IN_BAD 0x0800  /* ICMP checksum bad */

/* mbuf types */

#define MT_FREE       0       /* should be on free list */
#define MT_DATA       1       /* dynamic (data) allocation */
#define MT_HEADER     2       /* packet header */
#define MT_SONAME     3       /* socket name */
#define MT_SOOPTS     4       /* socket options */
#define MT_FTABLE     5       /* fragment reassembly header */
#define MT_CONTROL    6       /* extra-data protocol message */
#define MT_OOBDATA    7       /* expedited data */

/* Flags to m_get/MGET */

#define M_DONTWAIT    0x0000
#define M_WAIT        0x0001

/* mbuf allocation/deallocation macros:
 *
 *    MGET(struct mbuf *m, int how, int type)
 * allocates an mbuf and initializes it to contain internal data.
 *
 *    MGETHDR(struct mbuf *m, int how, int type)
 * allocates an mbuf and initializes it to contain a packet header
 * and internal data.
 */

#define MGET(m, how, type) m = m_get((how), (type))
#define MGETHDR(m, how, type) m = m_gethdr((how), (type))

/* Macros for tracking external storage associated with an mbuf.
 *
 * Note: add and delete reference must be called at splnet().
 */

#ifdef CONFIG_DEBUG
#  define MCLREFDEBUGN(m, file, line) do { \
        (m)->m_ext.ext_nfile = (file); \
        (m)->m_ext.ext_nline = (line); \
    } while (/* CONSTCOND */ 0)
#  define MCLREFDEBUGO(m, file, line) do {  \
        (m)->m_ext.ext_ofile = (file);  \
        (m)->m_ext.ext_oline = (line); \
    } while (/* CONSTCOND */ 0)
#else
#  define MCLREFDEBUGN(m, file, line)
#  define MCLREFDEBUGO(m, file, line)
#endif

#define MCLISREFERENCED(m)    ((m)->m_ext.ext_nextref != (m))

#define MCLADDREFERENCE(o, n)    do { \
        int ms = splnet();  \
        (n)->m_flags |= ((o)->m_flags & (M_EXT|M_CLUSTER)); \
        (n)->m_ext.ext_nextref = (o)->m_ext.ext_nextref; \
        (n)->m_ext.ext_prevref = (o); \
        (o)->m_ext.ext_nextref = (n); \
        (n)->m_ext.ext_nextref->m_ext.ext_prevref = (n); \
        splx(ms); \
        MCLREFDEBUGN((n), __FILE__, __LINE__);  \
    } while (/* CONSTCOND */ 0)

#define MCLINITREFERENCE(m)    do { \
        (m)->m_ext.ext_prevref = (m); \
        (m)->m_ext.ext_nextref = (m); \
        MCLREFDEBUGO((m), __FILE__, __LINE__); \
        MCLREFDEBUGN((m), NULL, 0); \
    } while (/* CONSTCOND */ 0)

/* Macros for mbuf external storage.
 *
 * MEXTADD adds pre-allocated external storage to
 * a normal mbuf; the flag M_EXT is set.
 *
 * MCLGET allocates and adds an mbuf cluster to a normal mbuf;
 * the flag M_EXT is set upon success.
 */

#define MEXTADD(m, buf, size, type, free, arg) do { \
    (m)->m_data = (m)->m_ext.ext_buf = (caddr_t)(buf); \
    (m)->m_flags |= M_EXT; \
    (m)->m_flags &= ~M_CLUSTER; \
    (m)->m_ext.ext_size = (size); \
    (m)->m_ext.ext_free = (free); \
    (m)->m_ext.ext_arg = (arg); \
    (m)->m_ext.ext_type = (type); \
    MCLINITREFERENCE(m);  \
} while (/* CONSTCOND */ 0)

#define MCLGET(m, how) (void) m_clget((m), (how), NULL, MCLBYTES)
#define MCLGETI(m, how, ifp, l) m_clget((m), (how), (ifp), (l))

/* MFREE(struct mbuf *m, struct mbuf *n)
 * Free a single mbuf and associated external storage.
 * Place the successor, if any, in n.
 */
 
#define MFREE(m, n) n = m_free((m))

/* Move just m_pkthdr from from to to,
 * remove M_PKTHDR and clean flags/tags for from.
 */
 
#define M_MOVE_HDR(to, from) do { \
    (to)->m_pkthdr = (from)->m_pkthdr; \
    (from)->m_flags &= ~M_PKTHDR; \
    SLIST_INIT(&(from)->m_pkthdr.tags); \
} while (/* CONSTCOND */ 0)

/* MOVE mbuf pkthdr from from to to.
 * from must have M_PKTHDR set, and to must be empty.
 */
 
#define M_MOVE_PKTHDR(to, from) do { \
    (to)->m_flags = ((to)->m_flags & (M_EXT | M_CLUSTER)); \
    (to)->m_flags |= (from)->m_flags & M_COPYFLAGS; \
    M_MOVE_HDR((to), (from)); \
    if (((to)->m_flags & M_EXT) == 0) \
        (to)->m_data = (to)->m_pktdat; \
} while (/* CONSTCOND */ 0)

/* Set the m_data pointer of a newly-allocated mbuf (m_get/MGET) to place
 * an object of the specified size at the end of the mbuf, longword aligned.
 */
 
#define M_ALIGN(m, len) \
    (m)->m_data += (MLEN - (len)) &~ (sizeof(long) - 1)

/* As above, for mbufs allocated with m_gethdr/MGETHDR
 * or initialized by M_MOVE_PKTHDR.
 */

#define MH_ALIGN(m, len) \
    (m)->m_data += (MHLEN - (len)) &~ (sizeof(long) - 1)

/* Determine if an mbuf's data area is read-only. This is true for
 * non-cluster external storage and for clusters that are being
 * referenced by more than one mbuf.
 */

#define M_READONLY(m) \
    (((m)->m_flags & M_EXT) != 0 &&  \
      (((m)->m_flags & M_CLUSTER) == 0 || MCLISREFERENCED(m)))

/* Compute the amount of space available
 * before the current start of data in an mbuf.
 */

#define M_LEADINGSPACE(m) m_leadingspace(m)

/* Compute the amount of space available
 * after the end of data in an mbuf.
 */

#define M_TRAILINGSPACE(m) m_trailingspace(m)

/* Arrange to prepend space of size plen to mbuf m.
 * If a new mbuf must be allocated, how specifies whether to wait.
 * If how is M_DONTWAIT and allocation fails, the original mbuf chain
 * is freed and m is set to NULL.
 */

#define M_PREPEND(m, plen, how) \
        (m) = m_prepend((m), (plen), (how))

/* Length to m_copy to copy all */

#define M_COPYALL    1000000000

/* Compatibility with 4.3 */

#define  m_copy(m, o, l)    m_copym((m), (o), (l), M_DONTWAIT)



/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Header at beginning of each mbuf: */

struct m_hdr
{
  struct mbuf *mh_next;          /* Next buffer in chain */
  struct mbuf *mh_nextpkt;       /* Next chain in queue/record */
  char *mh_data;                 /* Location of data */
  unsigned int mh_len;           /* Amount of data in this mbuf */
  short mh_type;                 /* Type of data in this mbuf */
  unsigned short mh_flags;       /* Flags; see below */
};

/* Record/packet header in first mbuf of chain; valid if M_PKTHDR set */

struct pkthdr
{
  struct ifnet *rcvif;                 /* rcv interface */
  int len;                             /* Total packet length */
};

/* Description of external storage mapped into mbuf, valid if M_EXT set */

struct mbuf_ext
{
  caddr_t ext_buf;              /* start of buffer */
  /* free routine if not the usual */
  // void (*ext_free)();
  // void *ext_arg; /* argument for ext_free */
  // unsigned int ext_size; /* size of buffer, for ext_free */
   unsigned int   ext_size;           /* size of buffer, for ext_free */
 };

struct mbuf
  {
    struct m_hdr m_hdr;
    union
      {
        struct
          {
            struct pkthdr MH_pkthdr;    /* M_PKTHDR set */
            union
              {
                struct mbuf_ext MH_ext; /* M_EXT set */
                char MH_databuf[MHLEN];
              } MH_dat;
          } MH;
        char M_databuf[MLEN];   /* !M_PKTHDR, !M_EXT */
      } M_dat;
  };

#define m_next       m_hdr.mh_next
#define m_len        m_hdr.mh_len
#define m_data       m_hdr.mh_data
#define m_type       m_hdr.mh_type
#define m_flags      m_hdr.mh_flags
#define m_nextpkt    m_hdr.mh_nextpkt
#define m_act        m_nextpkt
#define m_pkthdr     M_dat.MH.MH_pkthdr
#define m_ext        M_dat.MH.MH_dat.MH_ext
#define m_pktdat     M_dat.MH.MH_dat.MH_databuf
#define m_dat        M_dat.M_databuf

/* Mbuf statistics.
 * For statistics related to mbuf and cluster allocations, see also the
 * pool headers (mbpool and mclpool).
 */

struct mbstat
{
  unsigned long _m_spare;     /* formerly m_mbufs */
  unsigned long _m_spare1;    /* formerly m_clusters */
  unsigned long _m_spare2;    /* spare field */
  unsigned long _m_spare3;    /* formely m_clfree - free clusters */
  unsigned long m_drops;      /* times failed to find space */
  unsigned long m_wait;       /* times waited for space */
  unsigned long m_drain;      /* times drained protocols for space */
  unsigned short m_mtypes[256];       /* type specific mbuf allocations */
};

struct mclsizes
{
  unsigned int size;
  unsigned int hwm;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct mbstat mbstat;
extern int nmbclust;            /* limit on the # of clusters */
extern int mblowat;             /* mbuf low water mark */
extern int mcllowat;            /* mbuf cluster low water mark */
extern int max_linkhdr;         /* largest link-level header */
extern int max_protohdr;        /* largest protocol header */
extern int max_hdr;             /* largest link+protocol header */
extern int max_datalen;         /* MHLEN - max_hdr */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mbinit(void);
struct mbuf *m_copym2(struct mbuf *, int, int, int);
struct mbuf *m_copym(struct mbuf *, int, int, int);
struct mbuf *m_free(struct mbuf *);
struct mbuf *m_free_unlocked(struct mbuf *);
struct mbuf *m_get(int, int);
struct mbuf *m_getclr(int, int);
struct mbuf *m_gethdr(int, int);
struct mbuf *m_inithdr(struct mbuf *);
int m_defrag(struct mbuf *, int);
struct mbuf *m_prepend(struct mbuf *, int, int);
struct mbuf *m_pulldown(struct mbuf *, int, int, int *);
struct mbuf *m_pullup(struct mbuf *, int);
struct mbuf *m_split(struct mbuf *, int, int);
struct mbuf *m_inject(struct mbuf *, int, int, int);
struct mbuf *m_getptr(struct mbuf *, int, int *);
int m_leadingspace(struct mbuf *);
int m_trailingspace(struct mbuf *);
struct mbuf *m_clget(struct mbuf *, int, struct ifnet *, unsigned int);
void m_clsetwms(struct ifnet *, unsigned int, unsigned int, unsigned int);
int m_cldrop(struct ifnet *, int);
void m_clcount(struct ifnet *, int);
void m_cluncount(struct mbuf *, int);
void m_clinitifp(struct ifnet *);
void m_adj(struct mbuf *, int);
int m_copyback(struct mbuf *, int, int, const void *, int);
void m_freem(struct mbuf *);
void m_reclaim(void *, int);
void m_copydata(struct mbuf *, int, int, caddr_t);
void m_cat(struct mbuf *, struct mbuf *);
struct mbuf *m_devget(char *, int, int, struct ifnet *,
                      void (*)(const void *, void *, size_t));
void m_zero(struct mbuf *);
int m_apply(struct mbuf *, int, int,
            int (*)(caddr_t, caddr_t, unsigned int), caddr_t);
int m_dup_pkthdr(struct mbuf *, struct mbuf *, int);

/* Packet tag routines */

struct m_tag *m_tag_get(int, int, int);
void m_tag_prepend(struct mbuf *, struct m_tag *);
void m_tag_delete(struct mbuf *, struct m_tag *);
void m_tag_delete_chain(struct mbuf *);
struct m_tag *m_tag_find(struct mbuf *, int, struct m_tag *);
struct m_tag *m_tag_copy(struct m_tag *, int);
int m_tag_copy_chain(struct mbuf *, struct mbuf *, int);
void m_tag_init(struct mbuf *);
struct m_tag *m_tag_first(struct mbuf *);
struct m_tag *m_tag_next(struct mbuf *, struct m_tag *);

#endif /* __FS_NFS_RPC_MBUF_H */

