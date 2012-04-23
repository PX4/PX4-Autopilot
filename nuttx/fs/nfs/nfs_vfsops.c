/****************************************************************************
 * fs/nfs/nfs_vfsops.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *
 * Leveraged from OpenBSD:
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
 
#include <sys/socket.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <queue.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nfs.h>

#include <net/if.h>
#include <netinet/in.h>

#include "nfs.h"
#include "rpc_v2.h"
#include "rpc.h"
#include "nfs_proto.h"
#include "nfs_node.h"
#include "nfs_mount.h"
#include "xdr_subs.h"
#include "nfs_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NFS_DIRHDSIZ         (sizeof (struct nfs_dirent) - (MAXNAMLEN + 1))
#define NFS_DIRENT_OVERHEAD  offsetof(struct nfs_dirent, dirent)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/
 
struct nfs_dirent
{
  uint32_t cookie[2];
  struct dirent dirent;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     nfs_open(FAR struct file *filep, const char *relpath,
                        int oflags, mode_t mode);
static ssize_t nfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static ssize_t nfs_write(FAR struct file *filep, const char *buffer,
                         size_t buflen);
static int     nfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir);
static int     nfs_bind(FAR struct inode *blkdriver, const void *data,
                         void **handle);
static int     nfs_unbind(void *handle, FAR struct inode **blkdriver);
static int     nfs_statfs(struct inode *mountpt, struct statfs *buf);
static int     nfs_remove(struct inode *mountpt, const char *relpath);
static int     nfs_mkdir(struct inode *mountpt, const char *relpath,
                         mode_t mode);
static int     nfs_rmdir(struct inode *mountpt, const char *relpath);
static int     nfs_rename(struct inode *mountpt, const char *oldrelpath,
                          const char *newrelpath);
static int     nfs_fsinfo(struct inode *mountpt, const char *relpath,
                          struct stat *buf);

/****************************************************************************
 *  External Public Data  (this belong in a header file)
 ****************************************************************************/

extern uint32_t nfs_true;
extern uint32_t nfs_false;
extern uint32_t nfs_xdrneg1;
extern struct nfsstats nfsstats;
extern int nfs_ticks;

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* nfs vfs operations. */

const struct mountpt_operations nfs_operations =
{
  nfs_open,                     /* open */
  NULL,                         /* close */
  nfs_read,                     /* read */
  nfs_write,                    /* write */
  NULL,                         /* seek */
  NULL,                         /* ioctl */
  NULL,                         /* sync */

  NULL,                         /* opendir */
  NULL,                         /* closedir */
  nfs_readdir,                  /* readdir */
  NULL,                         /* rewinddir */

  nfs_bind,                     /* bind */
  nfs_unbind,                   /* unbind */
  nfs_statfs,                   /* statfs */

  nfs_remove,                   /* unlink */
  nfs_mkdir,                    /* mkdir */
  nfs_rmdir,                    /* rmdir */
  nfs_rename,                   /* rename */
  nfs_fsinfo                    /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: nfs_open
 *
 * Description: if oflags == O_CREAT it creates a file, if not it
 * check to see if the type is ok and that deletion is not in progress.
 ****************************************************************************/

static int
nfs_open(FAR struct file *filep, FAR const char *relpath, 
         int oflags, mode_t mode)
{
  struct inode *in;
  struct nfs_fattr vap;
  struct nfsv3_sattr sp;
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct CREATE3args *create = NULL;
  struct CREATE3resok *resok = NULL;
  void *datareply = NULL;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  in = filep->f_inode;
  nmp = (struct nfsmount*)in->i_private;
  np = (struct nfsnode*)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  if (oflags == O_CREAT)
    {
      /* Sanity checks */

      DEBUGASSERT(filep->f_priv == NULL);
again:
      nfsstats.rpccnt[NFSPROC_CREATE]++;
      vap = nmp->nm_head->n_fattr;
      sp.sa_modetrue = true;
      sp.sa_mode = txdr_unsigned(vap.fa_mode);
      sp.sa_uidfalse = nfs_xdrneg1;
      sp.sa_gidfalse = nfs_xdrneg1;
      sp.sa_sizefalse = nfs_xdrneg1;
      sp.sa_atimetype = txdr_unsigned(NFSV3SATTRTIME_TOCLIENT);
      sp.sa_mtimetype = txdr_unsigned(NFSV3SATTRTIME_TOCLIENT);
      sp.sa_atime = vap.fa3_atime;
      sp.sa_mtime = vap.fa3_mtime;

      create->how = sp;
      create->where.dir = nmp->nm_fh;
      create->where.name = relpath;

      error = nfs_request(nmp, NFSPROC_CREATE, create, datareply);
      if (!error)
        {
          /* Create an instance of the file private data to describe the opened
           * file.
           */

          np = (struct nfsnode *)kzalloc(sizeof(struct nfsnode));
          if (!np)
            {
              fdbg("Failed to allocate private data\n", error);
              error = -ENOMEM;
              goto errout_with_semaphore;
            }

          /* Initialize the file private data (only need to initialize
           * non-zero elements)
           */

          resok = (struct CREATE3resok *) datareply;
          np->n_open        = true;
          np->nfsv3_type    = NFREG;
          np->n_fhp         = resok->handle;
          np->n_size        = fxdr_hyper(&resok->attributes.fa3_size);
          np->n_fattr       = resok->attributes;
          fxdr_nfsv3time(&resok->attributes.fa3_mtime, &np->n_mtime)
          np->n_ctime       = fxdr_hyper(&resok->attributes.fa3_ctime);

          /* Attach the private date to the struct file instance */

          filep->f_priv = np;

          /* Then insert the new instance into the mountpoint structure.
           * It needs to be there (1) to handle error conditions that effect
           * all files, and (2) to inform the umount logic that we are busy
           * (but a simple reference count could have done that).
           */

          np->n_next = nmp->nm_head;
          nmp->nm_head = np->n_next;
          error = 0;
        }
      else
        {
          if (error == NFSERR_NOTSUPP)
            {
              goto again;
            }
        }

      np->n_flag |= NMODIFIED;
    }
  else
    {
      if (np->nfsv3_type != NFREG && np->nfsv3_type != NFDIR)
        {
          fdbg("open eacces typ=%d\n", np->nfsv3_type);
          return EACCES;
        }

      if (np->n_flag & NMODIFIED)
        {
          if (np->nfsv3_type == NFDIR)
            {
              np->n_direofoffset = 0;
            }
        }
    }

  /* For open/close consistency. */

  NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

#undef COMP
#ifdef COMP
/****************************************************************************
 * Name: nfs_close
 ****************************************************************************/

static int nfs_close(FAR struct file *filep) done
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  int error = 0;

  fvdbg("Closing\n");

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  np = filep->f_priv;
  nmp = filep->f_inode->i_private;

  DEBUGASSERT(nmp != NULL);

  if (np->nfsv3_type == NFREG)
    {
      error = nfs_sync(filep);
      kfree(np);
      filep->f_priv = NULL;
    }

  return error;
}
#endif

/****************************************************************************
 * Name: nfs_read
 ****************************************************************************/

static ssize_t nfs_read(FAR struct file *filep, char *buffer, size_t buflen) 
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  unsigned int readsize;
  int bytesleft;
  uint64_t offset;
  void *datareply = NULL;
  struct READ3args *read = NULL;
  struct READ3resok *resok = NULL;
  uint8_t *userbuffer = (uint8_t*)buffer;
  int error = 0;
  int len;
  bool eof;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  np = (struct nfsnode*) filep->f_priv;
  nmp = (struct nfsmount*) filep->f_inode->i_private;
  eof = false;
  offset = 0;

  DEBUGASSERT(nmp != NULL);

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      fdbg("nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  if (np->nfsv3_type != NFREG)
    {
      fdbg("read eacces typ=%d\n", np->nfsv3_type);
      return EACCES;
    }

  if ((nmp->nm_flag & (NFSMNT_NFSV3 | NFSMNT_GOTFSINFO)) == NFSMNT_NFSV3)
    {
      (void)nfs_fsinfo(filep->f_inode, NULL, NULL);
    }

  /* Get the number of bytes left in the file */

  bytesleft = np->n_size - filep->f_pos;
  readsize = 0;

  /* Truncate read count so that it does not exceed the number
   * of bytes left in the file.
   */

  if (buflen > bytesleft)
    {
      buflen = bytesleft;
    }

    len = nmp->nm_rsize;
    if (len < buflen)
      {
        error = EFBIG;
        goto errout_with_semaphore;
      }

      nfsstats.rpccnt[NFSPROC_READ]++;
again:
      read->file = np->nfsv3_type;
      read->count = buflen;
      read->offset = offset;

      error = nfs_request(nmp, NFSPROC_READ, read, datareply);
      if (error)
        {
          goto errout_with_semaphore;
        }

      resok = (struct READ3resok *) datareply;
      eof = resok->eof;
      if (eof == true)
        {
          readsize = resok->count;
          np->n_fattr = resok->file_attributes;
          memcpy(userbuffer, resok->data, readsize);
        }
      else
        {
          goto again;
        }

nfs_semgive(nmp);
  return readsize;

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_write
 ****************************************************************************/

static ssize_t
nfs_write(FAR struct file *filep, const char *buffer, size_t buflen) 
{
  struct inode *inode;
  struct nfsmount *nmp;
  struct nfsnode *np;
  unsigned int  writesize;
  void *datareply = NULL;
  struct WRITE3args *write = NULL;
  struct WRITE3resok *resok =NULL;
  uint8_t  *userbuffer = (uint8_t*)buffer;
  int error = 0;
  uint64_t  offset;
  int len;
  enum stable_how commit;
  int committed = NFSV3WRITE_FILESYNC;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  np     = (struct nfsnode *)filep->f_priv;
  inode  = filep->f_inode;
  nmp    = (struct nfsmount *)inode->i_private;
  offset = 0;

  DEBUGASSERT(nmp != NULL);

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  /* Check if the file size would exceed the range of off_t */

  if (np->n_size + buflen < np->n_size)
    {
      error = -EFBIG;
      goto errout_with_semaphore;
    }

  len = nmp->nm_wsize;
  if (len < buflen)
    {
      error = -EFBIG;
      goto errout_with_semaphore;
    }
  writesize = 0;

  nfsstats.rpccnt[NFSPROC_WRITE]++;
  write->file = np->nfsv3_type;
  write->offset = offset;
  write->count = buflen;
  write->stable = committed;
  memcpy((void *)write->data, userbuffer, buflen);

  error = nfs_request(nmp, NFSPROC_WRITE, write, datareply);
  if (error)
    {
      goto errout_with_semaphore;
    }

  resok = (struct WRITE3resok *)datareply;
  writesize = resok->count;
  if (writesize == 0)
    {
       error = NFSERR_IO;
       goto errout_with_semaphore;
    }

  commit = resok->committed;
  np->n_fattr = resok->file_wcc.after;

  /* Return the lowest committment level obtained by any of the RPCs. */

  if (committed == NFSV3WRITE_FILESYNC)
    {
      committed = commit;
    }
  else if (committed == NFSV3WRITE_DATASYNC &&
           commit == NFSV3WRITE_UNSTABLE)
    {
       committed = commit;
    }

  if ((nmp->nm_flag & NFSMNT_HASWRITEVERF) == 0)
    {
      bcopy((void*) resok->verf, (void*) nmp->nm_verf, NFSX_V3WRITEVERF);
      nmp->nm_flag |= NFSMNT_HASWRITEVERF;
    }
  else if (strncmp((char*) resok->verf, (char*) nmp->nm_verf, NFSX_V3WRITEVERF))
    {
      bcopy((void*) resok->verf, (void*) nmp->nm_verf, NFSX_V3WRITEVERF);
    }

  fxdr_nfsv3time(&np->n_fattr.fa3_mtime, &np->n_mtime)

  nfs_semgive(nmp);
  return writesize;

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_readdirrpc
 *
 * Description: The function below stuff the cookies in after the name.
 ****************************************************************************/

int nfs_readdirrpc(struct nfsmount *nmp, struct nfsnode *np, bool end_of_directory, struct fs_dirent_s *dir) 
{
  int error = 0;
  void *datareply = NULL;
  struct READDIR3args *readir = NULL;
  struct READDIR3resok *resok = NULL;

  /* Loop around doing readdir rpc's of size nm_readdirsize
   * truncated to a multiple of NFS_READDIRBLKSIZ.
   * The stopping criteria is EOF.
   */

  while (end_of_directory == false)
    {
      nfsstats.rpccnt[NFSPROC_READDIR]++;
      readir->dir = np->n_fhp;
      readir->count = nmp->nm_readdirsize;
      if (nfsstats.rpccnt[NFSPROC_READDIR] == 1)
        {
          readir->cookie.nfsuquad[0] = 0;
          readir->cookie.nfsuquad[1] = 0;
          readir->cookieverf.nfsuquad[0] = 0;
          readir->cookieverf.nfsuquad[1] = 0;
        }
      else
        {
          readir->cookie.nfsuquad[0] = dir->u.nfs.cookie[0];
          readir->cookie.nfsuquad[1] = dir->u.nfs.cookie[1];
          readir->cookieverf.nfsuquad[0] = np->n_cookieverf.nfsuquad[0];
          readir->cookieverf.nfsuquad[1] = np->n_cookieverf.nfsuquad[1];
        }

      error = nfs_request(nmp, NFSPROC_READDIR, readir, datareply);

      if (error)
        {
          goto nfsmout;
        }

     resok = (struct READDIR3resok*) datareply;
     np->n_fattr = resok->dir_attributes;
     np->n_cookieverf.nfsuquad[0] = resok->cookieverf.nfsuquad[0];
     np->n_cookieverf.nfsuquad[1] = resok->cookieverf.nfsuquad[1];
     dir->fd_dir.d_type = resok->reply.entries->fileid;
     memcpy(&dir->fd_dir.d_name[NAME_MAX], &resok->reply.entries->name, NAME_MAX);
     //dir->fd_dir.d_name = resok->reply.entries->name;//
     dir->u.nfs.cookie[0] = resok->reply.entries->cookie.nfsuquad[0];
     dir->u.nfs.cookie[1] = resok->reply.entries->cookie.nfsuquad[1];
     
     if(resok->reply.eof == true)
      {
        end_of_directory = true;        
      }

     //more_dirs = fxdr_unsigned(int, *dp);

      /* loop thru the dir entries*/
/*
      while (more_dirs && bigenough)
        {
          if (bigenough)
            {
              if (info_v3)
                {
                  dir->u.nfs.cookie[0] = cookie.nfsuquad[0];
                }
              else
                {
                  dir->u.nfs.cookie[0] = ndp->cookie[0] = 0;
                }

              dir->u.nfs.cookie[1] = ndp->cookie[1] = cookie.nfsuquad[1];
            }

          more_dirs = fxdr_unsigned(int, *ndp);
        }
        */
    }

  /* We are now either at the end of the directory */

  if (resok->reply.entries == NULL)
    {
      np->n_direofoffset = fxdr_hyper(&dir->u.nfs.cookie[0]);

      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      fdbg("End of directory\n");
      error = -ENOENT;
    }

nfsmout:
  return error;
}


/****************************************************************************
 * Name: nfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int nfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir) 
{
  int error = 0;
  struct nfsmount *nmp;
  struct nfsnode *np;
  bool eof = false;
  //struct nfs_dirent *ndp;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  nmp = mountpt->i_private;
  np  = nmp->nm_head;
  dir->fd_root = mountpt;

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      fdbg("romfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  if (np->nfsv3_type != NFDIR)
    {
      error = EPERM;
      goto errout_with_semaphore;
    }

  dir->u.nfs.nd_direoffset = np->n_direofoffset;

  /* First, check for hit on the EOF offset */

  if (dir->u.nfs.nd_direoffset != 0)
    {
      nfsstats.direofcache_hits++;
      //np->n_open = true;
      return 0;
    }

  if ((nmp->nm_flag & (NFSMNT_NFSV3 | NFSMNT_GOTFSINFO)) == NFSMNT_NFSV3)
    {
      (void)nfs_fsinfo(mountpt, NULL, NULL);
    }

  error = nfs_readdirrpc(nmp, np, eof, dir);

  if (error == NFSERR_BAD_COOKIE)
    {
      error = EINVAL;
    }
  
  if (!error && eof)
    {
      nfsstats.direofcache_misses++;
      nfs_semgive(nmp);
      return 0;
    }

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_decode_args
 ****************************************************************************/

void nfs_decode_args(struct nfsmount *nmp, struct nfs_args *argp) 
{
  int adjsock = 0;
  int maxio;

#ifdef CONFIG_NFS_TCPIP
  /* Re-bind if rsrvd port requested and wasn't on one */

  adjsock = !(nmp->nm_flag & NFSMNT_RESVPORT)
    && (argp->flags & NFSMNT_RESVPORT);
#endif

  /* Also re-bind if we're switching to/from a connected UDP socket */

  adjsock |= ((nmp->nm_flag & NFSMNT_NOCONN) != (argp->flags & NFSMNT_NOCONN));

  /* Update flags atomically.  Don't change the lock bits. */

  nmp->nm_flag =
    (argp->flags & ~NFSMNT_INTERNAL) | (nmp->nm_flag & NFSMNT_INTERNAL);

  if ((argp->flags & NFSMNT_TIMEO) && argp->timeo > 0)
    {
      nmp->nm_timeo = (argp->timeo * NFS_HZ + 5) / 10;
      if (nmp->nm_timeo < NFS_MINTIMEO)
        {
          nmp->nm_timeo = NFS_MINTIMEO;
        }
      else if (nmp->nm_timeo > NFS_MAXTIMEO)
        {
          nmp->nm_timeo = NFS_MAXTIMEO;
        }
    }

  if ((argp->flags & NFSMNT_RETRANS) && argp->retrans > 1)
    {
      nmp->nm_retry = (argp->retrans < NFS_MAXREXMIT)? argp->retrans : NFS_MAXREXMIT;
    }

  if (!(nmp->nm_flag & NFSMNT_SOFT))
    {
      nmp->nm_retry = NFS_MAXREXMIT + 1;  /* past clip limit */
    }

  if (argp->flags & NFSMNT_NFSV3)
    {
      if (argp->sotype == SOCK_DGRAM)
        {
          maxio = NFS_MAXDGRAMDATA;
        }
      else
        {
          maxio = NFS_MAXDATA;
        }
    }
  else
    {
      maxio = NFS_V2MAXDATA;
    }

  if ((argp->flags & NFSMNT_WSIZE) && argp->wsize > 0)
    {
      int osize = nmp->nm_wsize;
      nmp->nm_wsize = argp->wsize;

      /* Round down to multiple of blocksize */

      nmp->nm_wsize &= ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize <= 0)
        {
          nmp->nm_wsize = NFS_FABLKSIZE;
        }

      adjsock |= (nmp->nm_wsize != osize);
    }

  if (nmp->nm_wsize > maxio)
    {
      nmp->nm_wsize = maxio;
    }

  if (nmp->nm_wsize > MAXBSIZE)
    {
      nmp->nm_wsize = MAXBSIZE;
    }

  if ((argp->flags & NFSMNT_RSIZE) && argp->rsize > 0)
    {
      int osize = nmp->nm_rsize;
      nmp->nm_rsize = argp->rsize;

      /* Round down to multiple of blocksize */

      nmp->nm_rsize &= ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize <= 0)
        {
          nmp->nm_rsize = NFS_FABLKSIZE;
        }

      adjsock |= (nmp->nm_rsize != osize);
    }

  if (nmp->nm_rsize > maxio)
    {
      nmp->nm_rsize = maxio;
    }

  if (nmp->nm_rsize > MAXBSIZE)
    {
      nmp->nm_rsize = MAXBSIZE;
    }

  if ((argp->flags & NFSMNT_READDIRSIZE) && argp->readdirsize > 0)
    {
      nmp->nm_readdirsize = argp->readdirsize;

      /* Round down to multiple of blocksize */

      nmp->nm_readdirsize &= ~(NFS_DIRBLKSIZ - 1);
      if (nmp->nm_readdirsize < NFS_DIRBLKSIZ)
        {
          nmp->nm_readdirsize = NFS_DIRBLKSIZ;
        }
    }
  else if (argp->flags & NFSMNT_RSIZE)
    {
      nmp->nm_readdirsize = nmp->nm_rsize;
    }

  if (nmp->nm_readdirsize > maxio)
    {
      nmp->nm_readdirsize = maxio;
    }

  if ((argp->flags & NFSMNT_MAXGRPS) && argp->maxgrouplist >= 0 &&
      argp->maxgrouplist <= NFS_MAXGRPS)
    {
      nmp->nm_numgrps = argp->maxgrouplist;
    }

  if ((argp->flags & NFSMNT_READAHEAD) && argp->readahead >= 0 &&
      argp->readahead <= NFS_MAXRAHEAD)
    {
      nmp->nm_readahead = argp->readahead;
    }

  if (argp->flags & NFSMNT_ACREGMIN && argp->acregmin >= 0)
    {
      if (argp->acregmin > 0xffff)
        {
          nmp->nm_acregmin = 0xffff;
        }
      else
        {
          nmp->nm_acregmin = argp->acregmin;
        }
    }

  if (argp->flags & NFSMNT_ACREGMAX && argp->acregmax >= 0)
    {
      if (argp->acregmax > 0xffff)
        {
          nmp->nm_acregmax = 0xffff;
        }
      else
        {
          nmp->nm_acregmax = argp->acregmax;
        }
    }

  if (nmp->nm_acregmin > nmp->nm_acregmax)
    {
      nmp->nm_acregmin = nmp->nm_acregmax;
    }

  if (argp->flags & NFSMNT_ACDIRMIN && argp->acdirmin >= 0)
    {
      if (argp->acdirmin > 0xffff)
        {
          nmp->nm_acdirmin = 0xffff;
        }
      else
        {
          nmp->nm_acdirmin = argp->acdirmin;
        }
    }

  if (argp->flags & NFSMNT_ACDIRMAX && argp->acdirmax >= 0)
    {
      if (argp->acdirmax > 0xffff)
        {
          nmp->nm_acdirmax = 0xffff;
        }
      else
        {
          nmp->nm_acdirmax = argp->acdirmax;
        }
    }

  if (nmp->nm_acdirmin > nmp->nm_acdirmax)
    {
      nmp->nm_acdirmin = nmp->nm_acdirmax;
    }

  if (nmp->nm_so && adjsock)
    {
      nfs_disconnect(nmp);
      if (nmp->nm_sotype == SOCK_DGRAM)
        while (nfs_connect(nmp))
          {
            nvdbg("nfs_args: retrying connect\n");
          }
    }
}

/****************************************************************************
 * Name: mountnfs
 *
 * Description: Common code for nfs_mount.
 *
 ****************************************************************************/

int mountnfs(struct nfs_args *argp, struct sockaddr *nam, void **handle) 
{
  struct nfsmount *nmp;
  int error;

  /* Open the block driver */
/*
  if (!blkdriver || !blkdriver->u.i_bops)
    {
      fdbg("No block driver/ops\n");
      return -ENODEV;
    }

  if (blkdriver->u.i_bops->open &&
      blkdriver->u.i_bops->open(blkdriver) != OK)
    {
      fdbg("No open method\n");
      return -ENODEV;
    }
*/
  /* Create an instance of the mountpt state structure */
/*
  nmp = (struct nfsmount *)kzalloc(sizeof(struct nfsmount));
  if (!nmp)
    {
      fdbg("Failed to allocate mountpoint structure\n");
      return -ENOMEM;
    }
*/
  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  sem_init(&nmp->nm_sem, 0, 0);     /* Initialize the semaphore that controls access */

//nmp->nm_blkdriver = blkdriver;          /* Save the block driver reference */
  nfs_init();
  nmp->nm_timeo = NFS_TIMEO;
  nmp->nm_retry = NFS_RETRANS;
  nmp->nm_wsize = NFS_WSIZE;
  nmp->nm_rsize = NFS_RSIZE;
  nmp->nm_readdirsize = NFS_READDIRSIZE;
  nmp->nm_numgrps = NFS_MAXGRPS;
  nmp->nm_readahead = NFS_DEFRAHEAD;
  nmp->nm_fhsize = NFSX_V3FHMAX;
  nmp->nm_acregmin = NFS_MINATTRTIMO;
  nmp->nm_acregmax = NFS_MAXATTRTIMO;
  nmp->nm_acdirmin = NFS_MINATTRTIMO;
  nmp->nm_acdirmax = NFS_MAXATTRTIMO;
  nmp->nm_fh = argp->fh;
//strncpy(&mp->mnt_stat.f_fstypename[0], mp->mnt_vfc->vfc_name, MFSNAMELEN);
//memmove(hst, mp->mnt_stat.f_mntfromname, MNAMELEN);
//bcopy(pth, nmp->nm_mntonname, 90);
//memmove(argp, &mp->mnt_stat.mount_info.nfs_args, sizeof(*argp));
  nmp->nm_nam = nam;
  nfs_decode_args(nmp, argp);

  /* Set up the sockets and per-host congestion */

  nmp->nm_sotype = argp->sotype;
  nmp->nm_soproto = argp->proto;

  /* For Connection based sockets (TCP,...) defer the connect until
   * the first request, in case the server is not responding.
   */

  if (nmp->nm_sotype == SOCK_DGRAM && (error = nfs_connect(nmp)))
    {
      goto bad;
    }

  /* Mounted! */

  nmp->nm_mounted = true;
  *handle = &nmp;
  nfs_semgive(nmp);

  return 0;

bad:
  nfs_disconnect(nmp);
  sem_destroy(&nmp->nm_sem);
  kfree(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int nfs_bind(struct inode *blkdriver, const void *data, void **handle) 
{
  int error;
  struct nfs_args args;
  struct sockaddr *nam;

  bcopy(data, &args, sizeof(struct nfs_args));
  if (args.version == NFS_ARGSVERSION)
    {
      args.flags &= ~(NFSMNT_INTERNAL | NFSMNT_NOAC);
    }
   else
    {
      return -EINVAL;
    }

  if ((args.flags & (NFSMNT_NFSV3 | NFSMNT_RDIRPLUS)) == NFSMNT_RDIRPLUS)
    {
      return -EINVAL;
    }

  if (args.fhsize < 0 || args.fhsize > NFSX_V3FHMAX)
    {
      return -EINVAL;
    }

  nam = args.addr;
  error = mountnfs(&args, nam, handle);
  return error;
}

/****************************************************************************
 * Name: nfs_unmount
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

int nfs_unbind(void *handle, struct inode **blkdriver) 
{
  struct nfsmount *nmp = (struct nfsmount *) handle ;
  int error;

  fvdbg("Entry\n");

  if (!nmp)
    {
      return -EINVAL;
    }

  nfs_semtake(nmp);
  if (nmp->nm_head)
    {
      /* We cannot unmount now.. there are open files */

      error = -EBUSY;
    }
  else
    {
      /* Unmount ... close the block driver */
      /*
      if (nmp->nm_blkdriver)
        {
          struct inode *inode = nmp->nm_blkdriver;
          if (inode)
            {
              if (inode->u.i_bops && inode->u.i_bops->close)
                {
                  (void)inode->u.i_bops->close(inode);
                }
       */
              /* We hold a reference to the block driver but should
               * not but mucking with inodes in this context.  So, we will just return
               * our contained reference to the block driver inode and let the umount
               * logic dispose of it.
               */
              /*
              if (blkdriver)
                {
                  *blkdriver = inode;
                }
                
            }
        }
      */
      /* Release the mountpoint private data */
      
      nfs_disconnect(nmp);
      sem_destroy(&nmp->nm_sem);
      kfree(nmp);

      return 0;
    }

  nfs_semgive(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int nfs_statfs(struct inode *mountpt, struct statfs *sbp) 
{
  struct nfs_statfs *sfp = NULL;
  struct nfsmount *nmp;
  int error = 0;
  uint64_t tquad;
  void *datareply = NULL;
  struct FSSTAT3args *fsstat = NULL;
  int info_v3;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;
  info_v3 = (nmp->nm_flag & NFSMNT_NFSV3);

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error < 0)
    {
      fdbg("romfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Fill in the statfs info */

  memset(sbp, 0, sizeof(struct statfs));
  sbp->f_type = NFS_SUPER_MAGIC;

  if (info_v3 && (nmp->nm_flag & NFSMNT_GOTFSINFO) == 0)
    {
      (void)nfs_fsinfo(mountpt, NULL, NULL);
    }

  nfsstats.rpccnt[NFSPROC_FSSTAT]++;
  fsstat->fsroot = nmp->nm_fh;
  error = nfs_request(nmp, NFSPROC_FSSTAT, fsstat, datareply);
  if (error)
    {
      goto errout_with_semaphore;
    }

  sfp = (struct nfs_statfs *)datareply;
  nmp->nm_head->n_fattr = sfp->obj_attributes;
  if (info_v3)
    {
      sbp->f_bsize = NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_tbytes);
      sbp->f_blocks = tquad / (uint64_t) NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_fbytes);
      sbp->f_bfree = tquad / (uint64_t) NFS_FABLKSIZE;
      tquad = fxdr_hyper(&sfp->sf_abytes);
      sbp->f_bavail = tquad / (uint64_t) NFS_FABLKSIZE;

      tquad = fxdr_hyper(&sfp->sf_tfiles);
      sbp->f_files = tquad;
      tquad = fxdr_hyper(&sfp->sf_ffiles);
      sbp->f_ffree = tquad;
      sbp->f_namelen = NAME_MAX;
    }
  else
    {
      sbp->f_bsize = fxdr_unsigned(int32_t, sfp->sf_bsize);
      sbp->f_blocks = fxdr_unsigned(int32_t, sfp->sf_blocks);
      sbp->f_bfree = fxdr_unsigned(int32_t, sfp->sf_bfree);
      sbp->f_bavail = fxdr_unsigned(int32_t, sfp->sf_bavail);
      sbp->f_files = 0;
      sbp->f_ffree = 0;
    }

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_remove
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int nfs_remove(struct inode *mountpt, const char *relpath)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  void *datareply = NULL;
  struct REMOVE3args *remove = NULL;
  struct REMOVE3resok *resok = NULL;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;
  np = nmp->nm_head;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error == 0)
    {
      /* If the file is open, the correct behavior is to remove the file
       * name, but to keep the file cluster chain in place until the last
       * open reference to the file is closed.
       */

      /* Remove the file */

      if (np->nfsv3_type != NFREG)
        {
          error = EPERM;
          goto errout_with_semaphore;
        }

      /* Do the rpc */

      nfsstats.rpccnt[NFSPROC_REMOVE]++;
      remove->object.dir = np->n_fhp;
      remove->object.name = relpath;

      error = nfs_request(nmp, NFSPROC_REMOVE, remove, datareply);

      /* Kludge City: If the first reply to the remove rpc is lost..
       *   the reply to the retransmitted request will be ENOENT
       *   since the file was in fact removed
       *   Therefore, we cheat and return success.
       */

      if (error == ENOENT)
        {
          error = 0;
        }

      if (error)
        {
          goto errout_with_semaphore;
        }

      resok = (struct REMOVE3resok *)datareply;
      np->n_fattr = resok->dir_wcc.after;
      np->n_flag |= NMODIFIED;
    }
   NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
   nfs_semgive(nmp);
   return error;
}

/****************************************************************************
 * Name: nfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int nfs_mkdir(struct inode *mountpt, const char *relpath, mode_t mode)
{
  struct nfs_fattr vap;
  struct nfsv3_sattr sp;
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct MKDIR3args *mkir = NULL;
  struct MKDIR3resok *resok = NULL;
  void *datareply = NULL;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*) mountpt->i_private;
  np = nmp->nm_head;
  vap = np->n_fattr;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  nfsstats.rpccnt[NFSPROC_MKDIR]++;
  mkir->where.dir = nmp->nm_fh;
  mkir->where.name = relpath;

  sp.sa_modetrue = nfs_true;
  sp.sa_mode = txdr_unsigned(vap.fa_mode);
  sp.sa_uidfalse = nfs_xdrneg1;
  sp.sa_gidfalse = nfs_xdrneg1;
  sp.sa_sizefalse = nfs_xdrneg1;
  sp.sa_atimetype = txdr_unsigned(NFSV3SATTRTIME_TOCLIENT);
  sp.sa_mtimetype = txdr_unsigned(NFSV3SATTRTIME_TOCLIENT);

  fxdr_nfsv3time2(&vap.fa3_atime, &sp.sa_atime);
  fxdr_nfsv3time2(&vap.fa3_mtime, &sp.sa_mtime);

  mkir->attributes = sp;

  error = nfs_request(nmp, NFSPROC_MKDIR, mkdir, datareply);
  if (error)
    {
      goto errout_with_semaphore;
    }

  resok = (struct MKDIR3resok *) datareply;
  np->nfsv3_type = NFDIR;
  np->n_fhp = resok->handle;
  np->n_fattr = resok->obj_attributes; 
  np->n_flag |= NMODIFIED;
  NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

/****************************************************************************
 * Name: nfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

static int nfs_rmdir(struct inode *mountpt, const char *relpath)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct RMDIR3args *rmdir = NULL;
  struct RMDIR3resok *resok = NULL;
  void *datareply = NULL;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount *)mountpt->i_private;
  np = nmp->nm_head;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error == 0)
    {
      /* Remove the directory */

      if (np->nfsv3_type != NFDIR)
        {
          error = EPERM;
          goto errout_with_semaphore;
        }

      /* Do the rpc */
      
      nfsstats.rpccnt[NFSPROC_RMDIR]++;
      rmdir->object.dir = np->n_fhp;
      rmdir->object.name = relpath;
      error = nfs_request(nmp, NFSPROC_RMDIR, rmdir, datareply);
  
      if (error == ENOENT)
        {
          error = 0;
        }

      if (error)
        {
          goto errout_with_semaphore;
        }

      resok = (struct RMDIR3resok *)datareply;
      np->n_fattr = resok->dir_wcc.after;
      np->n_flag |= NMODIFIED;
    }
   NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
   nfs_semgive(nmp);
   return error;
}

/****************************************************************************
 * Name: nfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

static int nfs_rename(struct inode *mountpt, const char *oldrelpath, 
                          const char *newrelpath)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  void *datareply = NULL;
  struct RENAME3args *rename = NULL;
  struct RENAME3resok *resok = NULL;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount *)mountpt->i_private;
  np = nmp->nm_head;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  if (np->nfsv3_type != NFREG && np->nfsv3_type != NFDIR)
    {
      fdbg("open eacces typ=%d\n", np->nfsv3_type);
      error= -EACCES;
      goto errout_with_semaphore;
    }

  nfsstats.rpccnt[NFSPROC_RENAME]++;
  rename->from.dir = np->n_fhp;
  rename->from.name = oldrelpath;
  rename->to.dir = np->n_fhp;
  rename->to.name = newrelpath;

  error = nfs_request(nmp, NFSPROC_RENAME, rename, datareply);

  /* Kludge: Map ENOENT => 0 assuming that it is a reply to a retry. */

  if (error == ENOENT)
    {
      error = 0;
    }

  if (error)
    {
      goto errout_with_semaphore;
    }

  resok = (struct RENAME3resok *) datareply;
  np->n_fattr = resok->todir_wcc.after;
  np->n_flag |= NMODIFIED;
  NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
   nfs_semgive(nmp);
   return error;
}

/****************************************************************************
 * Name: nfs_fsinfo
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int nfs_fsinfo(struct inode *mountpt, const char *relpath, struct stat *buf) 
{
  struct nfsv3_fsinfo *fsp;
  struct FSINFOargs *fsinfo = NULL;
  struct nfsmount *nmp;
  uint32_t pref, max;
  int error = 0;
  void *datareply = NULL;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  memset(buf, 0, sizeof(struct stat));
  nfsstats.rpccnt[NFSPROC_FSINFO]++;
  fsinfo->fsroot = nmp->nm_fh;
  error = nfs_request(nmp, NFSPROC_FSINFO, fsinfo, datareply);
  if (error)
    {
      goto errout_with_semaphore;
    }

  fsp = (struct nfsv3_fsinfo *)datareply;
  nmp->nm_head->n_fattr = fsp->obj_attributes;
  pref = fxdr_unsigned(uint32_t, fsp->fs_wtpref);
  if (pref < nmp->nm_wsize)
    {
      nmp->nm_wsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  max = fxdr_unsigned(uint32_t, fsp->fs_wtmax);
  if (max < nmp->nm_wsize)
    {
      nmp->nm_wsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize == 0)
        nmp->nm_wsize = max;
    }

  pref = fxdr_unsigned(uint32_t, fsp->fs_rtpref);
  if (pref < nmp->nm_rsize)
    {
      nmp->nm_rsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  max = fxdr_unsigned(uint32_t, fsp->fs_rtmax);
  if (max < nmp->nm_rsize)
    {
      nmp->nm_rsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize == 0)
        {
          nmp->nm_rsize = max;
        }
    }

  pref = fxdr_unsigned(uint32_t, fsp->fs_dtpref);
  if (pref < nmp->nm_readdirsize)
    {
      nmp->nm_readdirsize = (pref + NFS_DIRBLKSIZ - 1) & ~(NFS_DIRBLKSIZ - 1);
    }

  if (max < nmp->nm_readdirsize)
    {
      nmp->nm_readdirsize = max & ~(NFS_DIRBLKSIZ - 1);
      if (nmp->nm_readdirsize == 0)
        {
          nmp->nm_readdirsize = max;
        }
    }

  buf->st_mode = fxdr_hyper(&fsp->obj_attributes.fa_mode);
  buf->st_size = fxdr_hyper(&fsp->obj_attributes.fa3_size);
  buf->st_blksize = 0;
  buf->st_blocks = 0;
  buf->st_mtime = fxdr_hyper(&fsp->obj_attributes.fa3_mtime);
  buf->st_atime = fxdr_hyper(&fsp->obj_attributes.fa3_atime);
  buf->st_ctime = fxdr_hyper(&fsp->obj_attributes.fa3_ctime);
  nmp->nm_flag |= NFSMNT_GOTFSINFO;

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}

#ifdef COMP
/****************************************************************************
 * Name: nfs_sync
 *
 * Description: Flush out the buffer cache
 *
 ****************************************************************************/

int nfs_sync(struct file *filep) 
{
  struct inode *inode;
  struct nfsmount *nmp;
  struct nfsnode *np;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  np    = filep->f_priv;
  inode = filep->f_inode;
  nmp   = inode->i_private;

  DEBUGASSERT(nmp != NULL);

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  /* Force stale buffer cache information to be flushed. */

  /* Check if the has been modified in any way */

  if ((np->n_flag & NMODIFIED) != 0)
    {
      //error = VOP_FSYNC(vp, cred, waitfor, p); 
    }

  return error;

errout_with_semaphore:
  nfs_semgive(nmp);
  return error;
}
#endif
