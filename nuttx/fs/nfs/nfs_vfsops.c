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
#include <semaphore.h>

#include <net/if.h>
#include <netinet/in.h>

#include "nfs.h"
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
static int     nfs_opendir(struct inode *mountpt, const char *relpath,
                   struct fs_dirent_s *dir);
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
static int     nfs_getfsinfo(struct nfsmount *nmp, const char *relpath,
                   struct stat *buf);
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

  nfs_opendir,                  /* opendir */
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
 * Description:
 *   If oflags == O_CREAT it creates a file, if not it check to see if the
 *   type is ok and that deletion is not in progress.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_open(FAR struct file *filep, FAR const char *relpath,
                    int oflags, mode_t mode)
{
  struct inode *in;
//struct nfs_fattr vap;
  struct nfsv3_sattr sp;
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct CREATE3args create;
  struct rpc_reply_create resok;
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
      memset(&sp, 0, sizeof(struct nfsv3_sattr));
    //memset(&vap, 0, sizeof(struct nfs_fattr));
    //vap = nmp->nm_head->n_fattr;
      sp.sa_modetrue  = true;
      sp.sa_mode      = txdr_unsigned(mode);
      sp.sa_uidfalse  = nfs_xdrneg1;
      sp.sa_gidfalse  = nfs_xdrneg1;
      sp.sa_sizefalse = 0;
      sp.sa_atimetype = txdr_unsigned(NFSV3SATTRTIME_DONTCHANGE);
      sp.sa_mtimetype = txdr_unsigned(NFSV3SATTRTIME_DONTCHANGE);
    //txdr_nfsv3time2(&vap.fa3_atime, &sp.sa_atime);
    //txdr_nfsv3time2(&vap.fa3_mtime, &sp.sa_mtime);

      memset(&create, 0, sizeof(struct CREATE3args));
      memset(&resok, 0, sizeof(struct rpc_reply_create));
      create.how = sp;
      create.where.dir.length = txdr_unsigned(np->n_fhsize);
      create.where.dir.handle = np->n_fhp;
      create.where.length = txdr_unsigned(64);
      strncpy(create.where.name, relpath, 64);

      error = nfs_request(nmp, NFSPROC_CREATE, (FAR const void *)&create,
                          (void *)&resok, sizeof(struct rpc_reply_create));
      if (!error)
        {
          /* Create an instance of the file private data to describe the opened
           * file.
           */

          np = (struct nfsnode *)kzalloc(sizeof(struct nfsnode));
          if (!np)
            {
              fdbg("ERROR: Failed to allocate private data\n");
              error = -ENOMEM;
              goto errout_with_semaphore;
            }

          /* Initialize the file private data (only need to initialize
           * non-zero elements)
           */

       // np->nfsv3_type = fxdr_unsigned(uint32_t, resok.attributes.fa_type);

          /* The full path exists -- but is the final component a file
           * or a directory?
           */

          if (np->nfsv3_type == NFDIR)
            {
              /* It is a directory */

              error = EISDIR;
              fdbg("ERROR: '%s' is a directory\n", relpath);
              goto errout_with_semaphore;
            }

          np->n_open        = true;
          bcopy(&resok.create.fshandle.handle, &np->n_fhp, sizeof(nfsfh_t));
          np->n_size        = fxdr_hyper(&resok.create.attributes.fa3_size);
          bcopy(&resok.create.attributes, &np->n_fattr, sizeof(struct nfs_fattr));
          fxdr_nfsv3time(&resok.create.attributes.fa3_mtime, &np->n_mtime)
          np->n_ctime       = fxdr_hyper(&resok.create.attributes.fa3_ctime);

          /* Attach the private date to the struct file instance */

          filep->f_priv = np;

          /* Then insert the new instance into the mountpoint structure.
           * It needs to be there (1) to handle error conditions that effect
           * all files, and (2) to inform the umount logic that we are busy
           * (but a simple reference count could have done that).
           */

          np->n_next   = nmp->nm_head;
          nmp->nm_head = np->n_next;
          error        = 0;
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
      if (np->nfsv3_type != NFREG)
        {
          fdbg("ERROR: open eacces typ=%d\n", np->nfsv3_type);
          error = EACCES;
          goto errout_with_semaphore;
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
  kfree(np);
  nfs_semgive(nmp);
  return -error;
}

#undef COMP
#ifdef COMP
/****************************************************************************
 * Name: nfs_close
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
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

  return -error;
}
#endif

/****************************************************************************
 * Name: nfs_read
 *
 * Returned Value:
 *   The (non-negative) number of bytes read on success; a negated errno
 *   value on failure.
 *
 ****************************************************************************/

static ssize_t nfs_read(FAR struct file *filep, char *buffer, size_t buflen)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  uint32_t readsize;
  int bytesleft;
  uint64_t offset;
  struct READ3args read;
  struct rpc_reply_read resok;
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
      error = EACCES;
      goto errout_with_semaphore;
    }

  if ((nmp->nm_flag & (NFSMNT_NFSV3 | NFSMNT_GOTFSINFO)) == NFSMNT_NFSV3)
    {
      (void)nfs_getfsinfo(nmp, NULL, NULL);
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
  memset(&read, 0, sizeof(struct READ3args));
  read.file = txdr_unsigned(np->nfsv3_type);
  read.count = txdr_unsigned(buflen);
  read.offset = txdr_unsigned(offset);

  error = nfs_request(nmp, NFSPROC_READ, (FAR const void *)&read,
                      (void *)&resok, sizeof(struct rpc_reply_read));
  if (error)
    {
      goto errout_with_semaphore;
    }

//eof = resok.eof;
  if (eof == true)
    {
      readsize = fxdr_unsigned(uint32_t, resok.read.count);
      np->n_fattr = resok.read.file_attributes;//
      memcpy(userbuffer, resok.read.data, readsize);
    }
  else
    {
      goto again;
    }

  nfs_semgive(nmp);
  return readsize;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_write
 *
 * Returned Value:
 *   The (non-negative) number of bytes written on success; a negated errno
 *   value on failure.
 *
 ****************************************************************************/

static ssize_t
nfs_write(FAR struct file *filep, const char *buffer, size_t buflen)
{
  struct inode *inode;
  struct nfsmount *nmp;
  struct nfsnode *np;
  unsigned int  writesize;
  struct WRITE3args write;
  struct rpc_reply_write resok;
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
      error = EFBIG;
      goto errout_with_semaphore;
    }

  len = nmp->nm_wsize;
  if (len < buflen)
    {
      error = EFBIG;
      goto errout_with_semaphore;
    }

  writesize = 0;

  nfsstats.rpccnt[NFSPROC_WRITE]++;
  memset(&write, 0, sizeof(struct WRITE3args));
  write.file = txdr_unsigned(np->nfsv3_type);
  write.offset = txdr_unsigned(offset);
  write.count = txdr_unsigned(buflen);
  write.stable = txdr_unsigned(committed);
  memcpy((void *)write.data, userbuffer, buflen);

  error = nfs_request(nmp, NFSPROC_WRITE, (FAR const void *)&write,
                      (FAR void *)&resok, sizeof(struct rpc_reply_write));
  if (error)
    {
      goto errout_with_semaphore;
    }

//writesize = resok.count;
  if (writesize == 0)
    {
       error = NFSERR_IO;
       goto errout_with_semaphore;
    }

//commit = resok.committed;
  np->n_fattr = resok.write.file_wcc.after;

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
      bcopy((void*) resok.write.verf, (void*) nmp->nm_verf, NFSX_V3WRITEVERF);
      nmp->nm_flag |= NFSMNT_HASWRITEVERF;
    }
  else if (strncmp((char*) resok.write.verf, (char*) nmp->nm_verf, NFSX_V3WRITEVERF))
    {
      bcopy((void*) resok.write.verf, (void*) nmp->nm_verf, NFSX_V3WRITEVERF);
    }

  fxdr_nfsv3time(&np->n_fattr.fa3_mtime, &np->n_mtime)

  nfs_semgive(nmp);
  return writesize;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_opendir(struct inode *mountpt, const char *relpath,
                         struct fs_dirent_s *dir)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
//struct nfs_dirinfo_s dirinfo;
  int ret;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  nmp = mountpt->i_private;
  np = nmp->nm_head;

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  ret = nfs_checkmount(nmp);
  if (ret != OK)
    {
      fdbg("nfs_checkmount failed: %d\n", ret);
      goto errout_with_semaphore;
    }

  /* The entry is a directory */

  if (np->nfsv3_type != NFDIR)
    {
      fdbg("open eacces type=%d\n", np->nfsv3_type);
      ret = EACCES;
      goto errout_with_semaphore;
    }

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      ret = ENOENT;
      goto errout_with_semaphore;
    }

  if (np->n_flag & NMODIFIED)
    {
      if (np->nfsv3_type == NFDIR)
        {
           np->n_direofoffset = 0;
           dir->u.nfs.nd_direoffset = 0;
           dir->u.nfs.cookie[0] = 0;
           dir->u.nfs.cookie[1] = 0;
        }
    }

  ret = OK;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -ret;
}

/****************************************************************************
 * Name: nfs_readdirrpc
 *
 * Description:
 *   The function below stuff the cookies in after the name.
 *
 * Returned Value:
 *   0 on success; a positive errno value on failure.
 *
 ****************************************************************************/

int nfs_readdirrpc(struct nfsmount *nmp, struct nfsnode *np,
                   bool end_of_directory, struct fs_dirent_s *dir)
{
  struct READDIR3args readir;
  struct rpc_reply_readdir resok;
  int error = 0;

  /* Loop around doing readdir rpc's of size nm_readdirsize
   * truncated to a multiple of NFS_READDIRBLKSIZ.
   * The stopping criteria is EOF.
   */

  while (end_of_directory == false)
    {
      nfsstats.rpccnt[NFSPROC_READDIR]++;
      memset(&readir, 0, sizeof(struct READDIR3args));
      readir.dir.length = txdr_unsigned(np->n_fhsize);
      readir.dir.handle = np->n_fhp;
      readir.count = nmp->nm_readdirsize;

      if (nfsstats.rpccnt[NFSPROC_READDIR] == 1)
        {
          readir.cookie.nfsuquad[0] = 0;
          readir.cookie.nfsuquad[1] = 0;
          readir.cookieverf.nfsuquad[0] = 0;
          readir.cookieverf.nfsuquad[1] = 0;
        }
      else
        {
          readir.cookie.nfsuquad[0] = dir->u.nfs.cookie[0];
          readir.cookie.nfsuquad[1] = dir->u.nfs.cookie[1];
          readir.cookieverf.nfsuquad[0] = np->n_cookieverf.nfsuquad[0];
          readir.cookieverf.nfsuquad[1] = np->n_cookieverf.nfsuquad[1];
        }

      error = nfs_request(nmp, NFSPROC_READDIR, (FAR const void *)&readir,
                          (FAR void *)&resok, sizeof(struct rpc_reply_readdir));
      if (error)
        {
          goto nfsmout;
        }
    //dir->fd_dir.d_name = resok->reply.entries->name;//
      /*np->n_fattr = resok.readir.dir_attributes;
      np->n_cookieverf.nfsuquad[0] = resok.readir.cookieverf.nfsuquad[0];
      np->n_cookieverf.nfsuquad[1] = resok.readir.cookieverf.nfsuquad[1];
      dir->fd_dir.d_type = resok.readir.reply.entries->fileid;
      memcpy(&dir->fd_dir.d_name[NAME_MAX], &resok.readir.reply.entries->name, NAME_MAX);
      dir->u.nfs.cookie[0] = resok.readir.reply.entries->cookie.nfsuquad[0];
      dir->u.nfs.cookie[1] = resok.readir.reply.entries->cookie.nfsuquad[1];

      if (resok.readir.reply.eof == true)
        {
          end_of_directory = true;
        }
*/
    //more_dirs = fxdr_unsigned(int, *dp);

      /* loop thru the dir entries */
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
        
    }
*/
  /* We are now either at the end of the directory */
/*
  if (resok.readir.reply.entries == NULL)
    {
      np->n_direofoffset = fxdr_hyper(&dir->u.nfs.cookie[0]);*/

      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      fdbg("End of directory\n");
      error = ENOENT;
    }

nfsmout:
  return error;
}

/****************************************************************************
 * Name: nfs_readdir
 *
 * Description: Read from directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
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

  nmp          = mountpt->i_private;
  np           = nmp->nm_head;
  dir->fd_root = mountpt;

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  dir->fd_dir.d_name[0] = '\0';

  if (np->nfsv3_type != NFDIR)
    {
      error = EPERM;
      goto errout_with_semaphore;
    }

  dir->u.nfs.nd_direoffset = np->n_direofoffset;
  dir->fd_dir.d_type = np->nfsv3_type;

  /* First, check for hit on the EOF offset */

  if (dir->u.nfs.nd_direoffset != 0)
    {
      nfsstats.direofcache_hits++;
    //np->n_open = true;
      goto success_with_semaphore;
    }

  if ((nmp->nm_flag & (NFSMNT_NFSV3 | NFSMNT_GOTFSINFO)) == NFSMNT_NFSV3)
    {
      (void)nfs_getfsinfo(nmp, NULL, NULL);
    }

  error = nfs_readdirrpc(nmp, np, eof, dir);

  if (error == NFSERR_BAD_COOKIE)
    {
      error = EINVAL;
      goto errout_with_semaphore;
    }

  if (!error && eof)
    {
      nfsstats.direofcache_misses++;
    }

success_with_semaphore:
  error = 0;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_decode_args
 *
 * Returned Value:
 *   None
 *
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

/*
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
*/

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
        {
          while (nfs_connect(nmp))
            {
              fvdbg("nfs_args: retrying connect\n");
            }
        }
    }
}

/****************************************************************************
 * Name: mountnfs
 *
 * Description:
 *   Common code for nfs_mount.
 *
 * Returned Value:
 *   0 on success; a positive errno value on failure.
 *
 ****************************************************************************/

int mountnfs(struct nfs_args *argp, void **handle)
{
  FAR struct nfsmount *nmp;
  struct nfsnode *np = NULL;
  struct FS3args getattr;
  struct rpc_reply_getattr resok;
  int error = 0;

  /* Create an instance of the mountpt state structure */

  nmp = (FAR struct nfsmount *)kzalloc(sizeof(struct nfsmount));
  if (!nmp)
    {
      fdbg("ERROR: Failed to allocate mountpoint structure\n");
      return ENOMEM;
    }

  /* Initialize the allocated mountpt state structure. */

  /* Initialize the semaphore that controls access.  The initial count
   * is zero, but nfs_semgive() is called at the completion of initialization,
   * incrementing the count to one.
   */

  sem_init(&nmp->nm_sem, 0, 0);     /* Initialize the semaphore that controls access */

  /* Initialize NFS */

  nfs_init();

  /* Set initial values of other fields */

  nmp->nm_flag        = argp->flags;
  nmp->nm_timeo       = NFS_TIMEO;
  nmp->nm_retry       = NFS_RETRANS;
  nmp->nm_wsize       = NFS_WSIZE;
  nmp->nm_rsize       = NFS_RSIZE;
  nmp->nm_readdirsize = NFS_READDIRSIZE;
  nmp->nm_numgrps     = NFS_MAXGRPS;
  nmp->nm_readahead   = NFS_DEFRAHEAD;
  nmp->nm_fhsize      = NFSX_V3FHMAX;
  nmp->nm_acregmin    = NFS_MINATTRTIMO;
  nmp->nm_acregmax    = NFS_MAXATTRTIMO;
  nmp->nm_acdirmin    = NFS_MINATTRTIMO;
  nmp->nm_acdirmax    = NFS_MAXATTRTIMO;
  strncpy(nmp->nm_path, argp->path, 90);
  nmp->nm_nam         = argp->addr;
  nfs_decode_args(nmp, argp);

  /* Set up the sockets and per-host congestion */

  nmp->nm_sotype  = argp->sotype;
  nmp->nm_soproto = argp->proto;

  /* For Connection based sockets (TCP,...) defer the connect until
   * the first request, in case the server is not responding.
   */

  if (nmp->nm_sotype == SOCK_DGRAM)
    {
      /* Connection-less... connect now */

      error = nfs_connect(nmp);
      if (error != 0)
        {
          fdbg("ERROR: nfs_connect failed: %d\n", error);
          goto bad;
        }
    }

  /* Create an instance of the file private data to describe the opened
   * file.
   */

  np = (struct nfsnode *)kzalloc(sizeof(struct nfsnode));
  if (!np)
    {
      fdbg("ERROR: Failed to allocate private data\n");
      error = ENOMEM;
      goto bad;
     }

  np->nfsv3_type         = NFDIR;
  np->n_open             = true;
  np->n_flag            |= NMODIFIED;
  nmp->nm_head           = np;
  nmp->nm_mounted        = true;
  nmp->nm_fh             = nmp->nm_rpcclnt->rc_fh;
  nmp->nm_fhsize         = NFSX_V2FH;
  nmp->nm_head->n_fhp    = nmp->nm_fh;
  nmp->nm_head->n_fhsize = nmp->nm_fhsize;
  nmp->nm_so             = nmp->nm_rpcclnt->rc_so;

  /* Get the file attributes */

  memset(&getattr, 0, sizeof(struct FS3args));
  memset(&resok, 0, sizeof(struct rpc_reply_getattr));
  getattr.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  getattr.fsroot.handle = nmp->nm_fh;

  error = nfs_request(nmp, NFSPROC_GETATTR, (FAR const void *)&getattr,
                      (FAR void*)&resok, sizeof(struct rpc_reply_getattr));
  if (error)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
      goto bad;
    }

  /* Save the file attributes */

  memcpy(&np->n_fattr, &resok.attr, sizeof(struct nfs_fattr));
  memcpy(&nmp->nm_fattr, &resok.attr, sizeof(struct nfs_fattr));

  /* Mounted! */

  *handle = (void*)nmp;
  nfs_semgive(nmp);

  fvdbg("Successfully mounted\n");
  return 0;

bad:
  /* Free all memory that was successfully allocated */

  if (np)
    {
      kfree(np);
    }

  if (nmp)
    {
      /* Disconnect from the server */

      nfs_disconnect(nmp);

      /* Free connection-related resources */

      sem_destroy(&nmp->nm_sem);
      if (nmp->nm_so)
        {
          kfree(nmp->nm_so);
        }
      if (nmp->nm_rpcclnt)
        {
          kfree(nmp->nm_rpcclnt);
        }
      kfree(nmp);
    }
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
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_bind(struct inode *blkdriver, const void *data, void **handle)
{
  struct nfs_args args;
  int error;

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

  error = mountnfs(&args, handle);
  return -error;
}

/****************************************************************************
 * Name: nfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nfs_unbind(void *handle, struct inode **blkdriver)
{
  struct nfsmount *nmp = (struct nfsmount *)handle;
  int error;

  fvdbg("Entry\n");

  if (!nmp)
    {
      return -EINVAL;
    }

  nfs_semtake(nmp);

  /* Umount */

  error = rpcclnt_umount(nmp->nm_rpcclnt);
  if (error)
    {
      fdbg("ERROR: rpcclnt_umount failed: %d\n", error);
    }

  /* Disconnect */

  nfs_disconnect(nmp);

  /* And free resources */

  sem_destroy(&nmp->nm_sem);
  kfree(nmp->nm_head);
  kfree(nmp->nm_so);
  kfree(nmp->nm_rpcclnt);
  kfree(nmp);

  return -error;
}

/****************************************************************************
 * Name: nfs_statfs
 *
 * Description:
 *   Return filesystem statistics
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_statfs(struct inode *mountpt, struct statfs *sbp)
{
  struct rpc_reply_fsstat sfp;
  struct nfsmount *nmp;
  int error = 0;
  uint64_t tquad;
  struct FS3args fsstat;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error < 0)
    {
      fdbg("nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Fill in the statfs info */

  memset(sbp, 0, sizeof(struct statfs));
  sbp->f_type = NFS_SUPER_MAGIC;

  if ((nmp->nm_flag & NFSMNT_GOTFSINFO) == 0)
    {
      (void)nfs_getfsinfo(nmp, NULL, NULL);
    }

  nfsstats.rpccnt[NFSPROC_FSSTAT]++;
  memset(&fsstat, 0, sizeof(struct FS3args));
  memset(&sfp, 0, sizeof(struct rpc_reply_fsstat));
  fsstat.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  fsstat.fsroot.handle = nmp->nm_fh;

  error = nfs_request(nmp, NFSPROC_FSSTAT, (FAR const void *)&fsstat,
                      (FAR void *) &sfp, sizeof(struct rpc_reply_fsstat));
  if (error)
    {
      goto errout_with_semaphore;
    }

  nmp->nm_head->n_fattr = sfp.fsstat.obj_attributes;
  sbp->f_bsize = NFS_FABLKSIZE;
  tquad = fxdr_hyper(&sfp.fsstat.sf_tbytes);
  sbp->f_blocks = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad = fxdr_hyper(&sfp.fsstat.sf_fbytes);
  sbp->f_bfree = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad = fxdr_hyper(&sfp.fsstat.sf_abytes);
  sbp->f_bavail = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad = fxdr_hyper(&sfp.fsstat.sf_tfiles);
  sbp->f_files = tquad;
  tquad = fxdr_hyper(&sfp.fsstat.sf_ffiles);
  sbp->f_ffree = tquad;
  sbp->f_namelen = NAME_MAX;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_remove
 *
 * Description:
 *   Remove a file
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_remove(struct inode *mountpt, const char *relpath)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct REMOVE3args remove;
  struct rpc_reply_remove resok;
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
      memset(&remove, 0, sizeof(struct REMOVE3args));
      memset(&resok, 0, sizeof(struct rpc_reply_remove));
      remove.object.dir.length = txdr_unsigned(np->n_fhsize);
      remove.object.dir.handle = np->n_fhp;
      remove.object.length = txdr_unsigned(64);
      strncpy(remove.object.name, relpath, 64);

      error = nfs_request(nmp, NFSPROC_REMOVE, (FAR const void *)&remove,
                          (FAR void*)&resok, sizeof(struct rpc_reply_remove));

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

    //np->n_fattr = resok.dir_wcc.after;
      np->n_flag |= NMODIFIED;
    }

   NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
   nfs_semgive(nmp);
   return -error;
}

/****************************************************************************
 * Name: nfs_mkdir
 *
 * Description:
 *   Create a directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_mkdir(struct inode *mountpt, const char *relpath, mode_t mode)
{
  struct nfsv3_sattr sp;
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct MKDIR3args mkir;
  struct rpc_reply_mkdir resok;
  int error = 0;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*) mountpt->i_private;
  np = nmp->nm_head;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != 0)
    {
      goto errout_with_semaphore;
    }

  nfsstats.rpccnt[NFSPROC_MKDIR]++;
  memset(&mkir, 0, sizeof(struct MKDIR3args));
  memset(&resok, 0, sizeof(struct rpc_reply_mkdir));
  mkir.where.dir.length = txdr_unsigned(np->n_fhsize);
  mkir.where.dir.handle = np->n_fhp;
  mkir.where.length = txdr_unsigned(64);
  strncpy(mkir.where.name, relpath, 64);

  sp.sa_modetrue = nfs_true;
  sp.sa_mode = txdr_unsigned(mode);
  sp.sa_uidfalse = 0;
  sp.sa_gidfalse = 0;
  sp.sa_sizefalse = 0;
  sp.sa_atimetype = txdr_unsigned(NFSV3SATTRTIME_DONTCHANGE);
  sp.sa_mtimetype = txdr_unsigned(NFSV3SATTRTIME_DONTCHANGE);

//memset(&sp.sa_atime, 0, sizeof(nfstime3));
//memset(&sp.sa_mtime, 0, sizeof(nfstime3));

  mkir.attributes = sp;

  error = nfs_request(nmp, NFSPROC_MKDIR, (FAR const void *)&mkir,
                      (FAR void *)&resok, sizeof(struct rpc_reply_mkdir));
  if (error)
    {
      goto errout_with_semaphore;
    }

  np->n_open        = true;
 /* np->nfsv3_type    = fxdr_unsigned(uint32_t, resok.obj_attributes.fa_type);
  np->n_fhp         = resok.fshandle.handle;
  np->n_size        = fxdr_hyper(&resok.obj_attributes.fa3_size);
  np->n_fattr       = resok.obj_attributes;
  fxdr_nfsv3time(&resok.obj_attributes.fa3_mtime, &np->n_mtime)
  np->n_ctime       = fxdr_hyper(&resok.obj_attributes.fa3_ctime);*/
  np->n_flag       |= NMODIFIED;

  NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_rmdir
 *
 * Description:
 *   Remove a directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_rmdir(struct inode *mountpt, const char *relpath)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct RMDIR3args rmdir;
  struct rpc_reply_rmdir resok;
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
      memset(&rmdir, 0, sizeof(struct RMDIR3args));
      memset(&resok, 0, sizeof(struct rpc_reply_rmdir));
      rmdir.object.dir.length = txdr_unsigned(np->n_fhsize);
      rmdir.object.dir.handle = np->n_fhp;
      rmdir.object.length = txdr_unsigned(64);
      strncpy(rmdir.object.name, relpath, 64);

      error = nfs_request(nmp, NFSPROC_RMDIR, (FAR const void *)&rmdir,
                          (FAR void *)&resok, sizeof(struct rpc_reply_rmdir));
      if (error == ENOENT)
        {
          error = 0;
        }

      if (error)
        {
          goto errout_with_semaphore;
        }

    //np->n_fattr = resok.dir_wcc.after;
      np->n_flag |= NMODIFIED;
    }

  NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_rename
 *
 * Description:
 *   Rename a file or directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_rename(struct inode *mountpt, const char *oldrelpath,
                      const char *newrelpath)
{
  struct nfsmount *nmp;
  struct nfsnode *np;
  struct RENAME3args rename;
  struct rpc_reply_rename resok;
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
      error = EACCES;
      goto errout_with_semaphore;
    }

  nfsstats.rpccnt[NFSPROC_RENAME]++;
  memset(&rename, 0, sizeof(struct RENAME3args));
  memset(&resok, 0, sizeof(struct rpc_reply_rename));
  rename.from.dir.length = txdr_unsigned(np->n_fhsize);
  rename.from.dir.handle = np->n_fhp;
  rename.from.length = txdr_unsigned(64);
  strncpy(rename.from.name, oldrelpath, 64);
  rename.to.dir.length = txdr_unsigned(np->n_fhsize);
  rename.to.dir.handle = np->n_fhp;
  rename.to.length = txdr_unsigned(64);
  strncpy(rename.to.name, newrelpath, 64);

  error = nfs_request(nmp, NFSPROC_RENAME, (FAR const void *)&rename,
                      (FAR void *)&resok, sizeof(struct rpc_reply_rename));

  /* ENOENT => 0 assuming that it is a reply to a retry. */

  if (error == ENOENT)
    {
      error = 0;
    }

  if (error)
    {
      goto errout_with_semaphore;
    }

//np->n_fattr = resok.todir_wcc.after;
  np->n_flag |= NMODIFIED;
  NFS_INVALIDATE_ATTRCACHE(np);

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_getfsinfo
 *
 * Description:
 *   Return information about root directory.  This is an internal version
 *   used only within this file.
 *
 * Returned Value:
 *   0 on success; positive errno value on failure
 *
 * Assumptions:
 *   The caller has exclusive access to the NFS mount structure
 *
 ****************************************************************************/

static int nfs_getfsinfo(struct nfsmount *nmp, const char *relpath,
                         struct stat *buf)
{
  struct rpc_reply_fsinfo fsp;
  struct FS3args fsinfo;
//uint32_t pref, max;
  int error = 0;

#warning "relpath is not used!  Additional logic will be required!"

  memset(&fsinfo, 0, sizeof(struct FS3args));
  memset(&fsp,    0, sizeof(struct rpc_reply_fsinfo));

  nfsstats.rpccnt[NFSPROC_FSINFO]++;
  fsinfo.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  fsinfo.fsroot.handle = nmp->nm_fh;

  /* Request FSINFO from the server */

  error = nfs_request(nmp, NFSPROC_FSINFO, (FAR const void *)&fsinfo,
                      (FAR void *)&fsp, sizeof(struct rpc_reply_fsinfo));
  if (error)
    {
      return error;
    }

//nmp->nm_fattr = fsp.obj_attributes;
 /* pref = fxdr_unsigned(uint32_t, fsp.fs_wtpref);
  if (pref < nmp->nm_wsize)
    {
      nmp->nm_wsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  max = fxdr_unsigned(uint32_t, fsp.fs_wtmax);
  if (max < nmp->nm_wsize)
    {
      nmp->nm_wsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize == 0)
        nmp->nm_wsize = max;
    }

  pref = fxdr_unsigned(uint32_t, fsp.fs_rtpref);
  if (pref < nmp->nm_rsize)
    {
      nmp->nm_rsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  max = fxdr_unsigned(uint32_t, fsp.fs_rtmax);
  if (max < nmp->nm_rsize)
    {
      nmp->nm_rsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize == 0)
        {
          nmp->nm_rsize = max;
        }
    }

  pref = fxdr_unsigned(uint32_t, fsp.fs_dtpref);
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
*/

  /* Verify that the caller has provided a non-NULL location to return the
   * FSINFO data.
   */

  if (buf)
    {
      buf->st_mode    = fxdr_unsigned(uint32_t, nmp->nm_fattr.fa_mode);
      buf->st_size    = fxdr_hyper(&nmp->nm_fattr.fa3_size);
      buf->st_blksize = 0;
      buf->st_blocks  = 0;
      buf->st_mtime   = fxdr_hyper(&nmp->nm_fattr.fa3_mtime);
      buf->st_atime   = fxdr_hyper(&nmp->nm_fattr.fa3_atime);
      buf->st_ctime   = fxdr_hyper(&nmp->nm_fattr.fa3_ctime);
    }

  nmp->nm_flag |= NFSMNT_GOTFSINFO;

  return 0;
}

/****************************************************************************
 * Name: nfs_fsinfo
 *
 * Description:
 *   Return information about root directory
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_fsinfo(struct inode *mountpt, const char *relpath,
                      struct stat *buf)
{
  struct nfsmount *nmp;
  int error;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error == 0)
    {
      /* Get the requested FSINFO */

      error = nfs_getfsinfo(nmp, relpath, buf);
    }

  nfs_semgive(nmp);
  return -error;
}

#ifdef COMP
/****************************************************************************
 * Name: nfs_sync
 *
 * Description: Flush out the buffer cache
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
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

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}
#endif
