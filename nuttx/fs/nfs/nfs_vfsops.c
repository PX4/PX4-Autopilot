/****************************************************************************
 * fs/nfs/nfs_vfsops.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2012 Jose Pablo Rojas Vargas. All rights reserved.
 *   Author: Jose Pablo Rojas Vargas <jrojas@nx-engineering.com>
 *           Gregory Nutt <gnutt@nuttx.org>
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
#include <time.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nfs.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-udp.h>
#include <nuttx/net/uip/uipopt.h>

#include <net/if.h>
#include <netinet/in.h>

#include "nfs.h"
#include "rpc.h"
#include "nfs_proto.h"
#include "nfs_node.h"
#include "nfs_mount.h"
#include "xdr_subs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The V3 EXCLUSIVE file creation logic is not fully supported. */

#define USE_GUARDED_CREATE    1

/* include/nuttx/fs/dirent.h has its own version of these lengths.  They must
 * match the NFS versions.
 */

#if NFSX_V3FHMAX != DIRENT_NFS_MAXHANDLE
#  error "Length of file handle in fs_dirent_s is incorrect"
#endif

#if NFSX_V3COOKIEVERF != DIRENT_NFS_VERFLEN
#  error "Length of cookie verify in fs_dirent_s is incorrect"
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

uint32_t nfs_true;
uint32_t nfs_false;
uint32_t nfs_xdrneg1;

#ifdef CONFIG_NFS_STATISTICS
struct nfsstats nfsstats;
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     nfs_filecreate(FAR struct nfsmount *nmp, struct nfsnode *np,
                   FAR const char *relpath, mode_t mode);
static int     nfs_filetruncate(FAR struct nfsmount *nmp, struct nfsnode *np);
static int     nfs_fileopen(FAR struct nfsmount *nmp, struct nfsnode *np,
                   FAR const char *relpath, int oflags, mode_t mode);
static int     nfs_open(FAR struct file *filep, const char *relpath,
                   int oflags, mode_t mode);
static int     nfs_close(FAR struct file *filep);
static ssize_t nfs_read(FAR struct file *filep, char *buffer, size_t buflen);
static ssize_t nfs_write(FAR struct file *filep, const char *buffer,
                   size_t buflen);
static int     nfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int     nfs_opendir(struct inode *mountpt, const char *relpath,
                   struct fs_dirent_s *dir);
static int     nfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir);
static int     nfs_rewinddir(FAR struct inode *mountpt,
                   FAR struct fs_dirent_s *dir);
static void    nfs_decode_args(FAR struct nfs_mount_parameters *nprmt,
                   FAR struct nfs_args *argp);
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
static int     nfs_stat(struct inode *mountpt, const char *relpath,
                   struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* nfs vfs operations. */

const struct mountpt_operations nfs_operations =
{
  nfs_open,                     /* open */
  nfs_close,                    /* close */
  nfs_read,                     /* read */
  nfs_write,                    /* write */
  NULL,                         /* seek */
  NULL,                         /* ioctl */

  NULL,                         /* sync */
  nfs_dup,                      /* dup */

  nfs_opendir,                  /* opendir */
  NULL,                         /* closedir */
  nfs_readdir,                  /* readdir */
  nfs_rewinddir,                /* rewinddir */

  nfs_bind,                     /* bind */
  nfs_unbind,                   /* unbind */
  nfs_statfs,                   /* statfs */

  nfs_remove,                   /* unlink */
  nfs_mkdir,                    /* mkdir */
  nfs_rmdir,                    /* rmdir */
  nfs_rename,                   /* rename */
  nfs_stat                      /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nfs_filecreate
 *
 * Description:
 *   Create a file.  This is part of the file open logic that is executed if
 *   the user asks to create a file.
 *
 * Returned Value:
 *   0 on success; a positive errno value on failure.
 *
 ****************************************************************************/

static int nfs_filecreate(FAR struct nfsmount *nmp, struct nfsnode *np,
                          FAR const char *relpath, mode_t mode)
{
  struct file_handle      fhandle;
  struct nfs_fattr        fattr;
  char                    filename[NAME_MAX + 1];
  FAR uint32_t           *ptr;
  uint32_t                tmp;
  int                     namelen;
  int                     reqlen;
  int                     error;

  /* Find the NFS node of the directory containing the file to be created */

  error = nfs_finddir(nmp, relpath, &fhandle, &fattr, filename);
  if (error != OK)
    {
      fdbg("ERROR: nfs_finddir returned: %d\n", error);
      return error;
    }

  /* Create the CREATE RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.create.create;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  reqlen += (int)fhandle.length;
  ptr    += uint32_increment(fhandle.length);

  /* Copy the variable-length file name */

  namelen = strlen(filename);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, filename, namelen);
  ptr    += uint32_increment(namelen);
  reqlen += uint32_alignup(namelen);

  /* Set the creation mode */

  if ((mode & O_CREAT) != 0)
    {
#ifdef USE_GUARDED_CREATE
      *ptr++  = HTONL(NFSV3CREATE_GUARDED);
#else
      *ptr++  = HTONL(NFSV3CREATE_EXCLUSIVE);
#endif
    }
  else
    {
      *ptr++  = HTONL(NFSV3CREATE_UNCHECKED);
    }
  reqlen += sizeof(uint32_t);

  /* Mode information is not provided if EXCLUSIVE creation is used.
   * in this case, we must call SETATTR after successfully creating
   * the file.
   */

#ifndef USE_GUARDED_CREATE
  if ((mode & O_CREAT) == 0)
#endif
    {
      /* Set the mode.  NOTE: Here we depend on the fact that the NuttX and NFS
       * bit settings are the same (at least for the bits of interest).
       */

      *ptr++  = nfs_true; /* True: mode value follows */
      reqlen += sizeof(uint32_t);

      tmp = mode & (NFSMODE_IWOTH | NFSMODE_IROTH | NFSMODE_IWGRP |
                    NFSMODE_IRGRP | NFSMODE_IWUSR | NFSMODE_IRUSR);
      *ptr++  = txdr_unsigned(tmp);
      reqlen += sizeof(uint32_t);

      /* Set the user ID to zero */

      *ptr++  = nfs_true;             /* True: Uid value follows */
      *ptr++  = 0;                    /* UID = 0 (nobody) */
      reqlen += 2*sizeof(uint32_t);

      /* Set the group ID to one */

      *ptr++  = nfs_true;            /* True: Gid value follows */
      *ptr++  = HTONL(1);            /* GID = 1 (nogroup) */
      reqlen += 2*sizeof(uint32_t);

      /* Set the size to zero */

      *ptr++  = nfs_true;            /* True: Size value follows */
      *ptr++  = 0;                   /* Size = 0 */
      *ptr++  = 0;
      reqlen += 3*sizeof(uint32_t);

      /* Don't change times */

      *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change atime */
      *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change mtime */
      reqlen += 2*sizeof(uint32_t);
    }

  /* Send the NFS request.  Note there is special logic here to handle version 3
   * exclusive open semantics.
   */

  do
    {
      nfs_statistics(NFSPROC_CREATE);
      error = nfs_request(nmp, NFSPROC_CREATE,
                          (FAR void *)&nmp->nm_msgbuffer.create, reqlen,
                          (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);
    }
#ifdef USE_GUARDED_CREATE
  while (0);
#else
  while (((mode & O_CREAT) != 0) && error == EOPNOTSUPP);
#endif

  /* Check for success */

  if (error == OK)
    {
      /* Parse the returned data */

      ptr = (FAR uint32_t *)&((FAR struct rpc_reply_create *)nmp->nm_iobuffer)->create;

      /* Save the file handle in the file data structure */

      tmp = *ptr++;  /* handle_follows */
      if (!tmp)
        {
          fdbg("ERROR: no file handle follows\n");
          return EINVAL;
        }

      tmp = *ptr++;
      tmp = fxdr_unsigned(uint32_t, tmp);
      DEBUGASSERT(tmp <= NFSX_V3FHMAX);

      np->n_fhsize      = (uint8_t)tmp;
      memcpy(&np->n_fhandle, ptr, tmp);
      ptr += uint32_increment(tmp);

      /* Save the attributes in the file data structure */

      tmp = *ptr;  /* handle_follows */
      if (!tmp)
        {
          fdbg("WARNING: no file attributes\n");
        }
      else
        {
          /* Initialize the file attributes */

          nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
        }

      /* Any following dir_wcc data is ignored for now */
    }

  return error;
}

/****************************************************************************
 * Name: nfs_fileopen
 *
 * Description:
 *   Truncate an open file to zero length.  This is part of the file open
 *   logic.
 *
 * Returned Value:
 *   0 on success; a positive errno value on failure.
 *
 ****************************************************************************/

static int nfs_filetruncate(FAR struct nfsmount *nmp, struct nfsnode *np)
{
  FAR uint32_t *ptr;
  int           reqlen;
  int           error;

  fvdbg("Truncating file\n");

  /* Create the SETATTR RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.setattr.setattr;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(np->n_fhsize);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &np->n_fhandle, np->n_fhsize);
  reqlen += (int)np->n_fhsize;
  ptr    += uint32_increment(np->n_fhsize);

  /* Copy the variable-length attributes */

  *ptr++  = nfs_false;                        /* Don't change mode */
  *ptr++  = nfs_false;                        /* Don't change uid */
  *ptr++  = nfs_false;                        /* Don't change gid */
  *ptr++  = nfs_true;                         /* Use the following size */
  *ptr++  = 0;                                /* Truncate to zero length */
  *ptr++  = 0;
  *ptr++  = HTONL(NFSV3SATTRTIME_TOSERVER);   /* Use the server's time */
  *ptr++  = HTONL(NFSV3SATTRTIME_TOSERVER);   /* Use the server's time */
  *ptr++  = nfs_false;                        /* No guard value */
  reqlen += 9 * sizeof(uint32_t);

  /* Perform the SETATTR RPC */

  nfs_statistics(NFSPROC_SETATTR);
  error = nfs_request(nmp, NFSPROC_SETATTR,
                      (FAR void *)&nmp->nm_msgbuffer.setattr, reqlen,
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);
  if (error != OK)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
      return error;
    }

  /* Indicate that the file now has zero length */

  np->n_size = 0;
  return OK;
}

/****************************************************************************
 * Name: nfs_fileopen
 *
 * Description:
 *   Open a file.  This is part of the file open logic that attempts to open
 *   an existing file.
 *
 * Returned Value:
 *   0 on success; a positive errno value on failure.
 *
 ****************************************************************************/

static int nfs_fileopen(FAR struct nfsmount *nmp, struct nfsnode *np,
                        FAR const char *relpath, int oflags, mode_t mode)
{
  struct file_handle fhandle;
  struct nfs_fattr   fattr;
  uint32_t           tmp;
  int                error = 0;

  /* Find the NFS node associate with the path */

  error = nfs_findnode(nmp, relpath, &fhandle, &fattr, NULL);
  if (error != OK)
    {
      fdbg("ERROR: nfs_findnode returned: %d\n", error);
      return error;
    }

  /* Check if the object is a directory */

  tmp = fxdr_unsigned(uint32_t, fattr.fa_type);
  if (tmp == NFDIR)
    {
      /* Exit with EISDIR if we attempt to open a directory */

      fdbg("ERROR: Path is a directory\n");
      return EISDIR;
    }

  /* Check if the caller has sufficient privileges to open the file */

  if ((oflags & O_WRONLY) != 0)
    {
      /* Check if anyone has priveleges to write to the file -- owner,
       * group, or other (we are probably "other" and may still not be
       * able to write).
       */

      tmp = fxdr_unsigned(uint32_t, fattr.fa_mode);
      if ((tmp & (NFSMODE_IWOTH|NFSMODE_IWGRP|NFSMODE_IWUSR)) == 0)
        {
          fdbg("ERROR: File is read-only: %08x\n", tmp);
          return EACCES;
        }
    }

  /* It would be an error if we are asked to create the file exclusively */

  if ((oflags & (O_CREAT|O_EXCL)) == (O_CREAT|O_EXCL))
    {
      /* Already exists -- can't create it exclusively */

      fdbg("ERROR: File exists\n");
      return EEXIST;
    }

  /* Initialize the file private data */
  /* Copy the file handle */

  np->n_fhsize      = (uint8_t)fhandle.length;
  memcpy(&np->n_fhandle, &fhandle.handle, fhandle.length);

  /* Save the file attributes */

  nfs_attrupdate(np, &fattr);

  /* If O_TRUNC is specified and the file is opened for writing,
   * then truncate the file.  This operation requires that the file is
   * writable, but we have already checked that. O_TRUNC without write
   * access is ignored.
   */

  if ((oflags & (O_TRUNC|O_WRONLY)) == (O_TRUNC|O_WRONLY))
    {
      /* Truncate the file to zero length.  I think we can do this with
       * the SETATTR call by setting the length to zero.
       */

      return nfs_filetruncate(nmp, np);
    }

  return OK;
}

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
  struct nfsmount *nmp;
  struct nfsnode *np;
  int error;

  /* Sanity checks */

  DEBUGASSERT(filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  nmp = (struct nfsmount*)filep->f_inode->i_private;
  DEBUGASSERT(nmp != NULL);

  /* Pre-allocate the file private data to describe the opened file. */

  np = (struct nfsnode *)kzalloc(sizeof(struct nfsnode));
  if (!np)
    {
      fdbg("ERROR: Failed to allocate private data\n");
      return -ENOMEM;
    }

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Try to open an existing file at that path */

  error = nfs_fileopen(nmp, np, relpath, oflags, mode);
  if (error != OK)
    {
      /* An error occurred while trying to open the existing file. Check if
       * the open failed because the file does not exist.  That is not
       * necessarily an error; that may only mean that we have to create the
       * file.
       */

      if (error != ENOENT)
        {
          fdbg("ERROR: nfs_findnode failed: %d\n", error);
          goto errout_with_semaphore;
        }

      /* The file does not exist. Check if we were asked to create the file.  If
       * the O_CREAT bit is set in the oflags then we should create the file if it
       * does not exist.
       */

      if ((oflags & O_CREAT) == 0)
        {
          /* Return ENOENT if the file does not exist and we were not asked
           * to create it.
           */

          fdbg("ERROR: File does not exist\n");
           error = ENOENT;
          goto errout_with_semaphore;
        }

      /* Create the file */

      error = nfs_filecreate(nmp, np, relpath, mode);
      if (error != OK)
        {
          fdbg("ERROR: nfs_filecreate failed: %d\n", error);
          goto errout_with_semaphore;
        }
    }

  /* Initialize the file private data (only need to initialize
   * non-zero elements)
   */

  np->n_crefs = 1;

  /* Attach the private data to the struct file instance */

  filep->f_priv = np;

  /* Then insert the new instance at the head of the list in the mountpoint
   * tructure. It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy.  We
   * cannot unmount the file system if this list is not empty!
   */

  np->n_next   = nmp->nm_head;
  nmp->nm_head = np;

  np->n_flags |= (NFSNODE_OPEN | NFSNODE_MODIFIED);
  nfs_semgive(nmp);
  return OK;

errout_with_semaphore:
  if (np)
    {
      kfree(np);
    }

  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_close
 *
 * Description:
 *   Close a file.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_close(FAR struct file *filep)
{
  FAR struct nfsmount *nmp;
  FAR struct nfsnode  *np;
  FAR struct nfsnode  *prev;
  FAR struct nfsnode  *curr;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (struct nfsmount*) filep->f_inode->i_private;
  np  = (struct nfsnode*) filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  /* Get exclusive access to the mount structure. */

  nfs_semtake(nmp);

  /* Decrement the reference count.  If the reference count would not
   * decrement to zero, then that is all we have to do.
   */

  if (np->n_crefs > 1)
    {
      np->n_crefs--;
      ret = OK;
    }

  /* There are no more references to the file structure.  Now we need to
   * free up all resources associated with the open file.
   *
   * First, find our file structure in the list of file structures
   * containted in the mount structure.
   */

  else
    {
      /* Assume file structure will not be found.  This should never happen. */

      ret = -EINVAL;

      for (prev = NULL, curr = nmp->nm_head;
           curr;
           prev = curr, curr = curr->n_next)
        {
          /* Check if this node is ours */

          if (np == curr)
            {
              /* Yes.. remove it from the list of file structures */

              if (prev)
                {
                  /* Remove from mid-list */

                  prev->n_next = np->n_next;
                }
              else
                {
                  /* Remove from the head of the list */

                  nmp->nm_head = np->n_next;
                }

              /* Then deallocate the file structure and return success */

              kfree(np);
              ret = OK;
              break;
            }
        }
    }

  filep->f_priv = NULL;
  nfs_semgive(nmp);
  return ret;
}

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
  FAR struct nfsmount       *nmp;
  FAR struct nfsnode        *np;
  ssize_t                    readsize;
  ssize_t                    tmp;
  ssize_t                    bytesread;
  size_t                     reqlen;
  FAR uint32_t              *ptr;
  int                        error = 0;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (struct nfsmount*)filep->f_inode->i_private;
  np  = (struct nfsnode*)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Get the number of bytes left in the file and truncate read count so that
   * it does not exceed the number of bytes left in the file.
   */

  tmp = np->n_size - filep->f_pos;
  if (buflen > tmp)
    {
      buflen = tmp;
      fvdbg("Read size truncated to %d\n", buflen);
    }

  /* Now loop until we fill the user buffer (or hit the end of the file) */

  for (bytesread = 0; bytesread < buflen; )
    {
      /* Make sure that the attempted read size does not exceed the RPC maximum */

      readsize = buflen;
      if (readsize > nmp->nm_rsize)
        {
          readsize = nmp->nm_rsize;
        }

      /* Make sure that the attempted read size does not exceed the IO buffer size */

      tmp = SIZEOF_rpc_reply_read(readsize);
      if (tmp > nmp->nm_buflen)
        {
          readsize -= (tmp - nmp->nm_buflen);
        }

      /* Initialize the request */

      ptr     = (FAR uint32_t*)&nmp->nm_msgbuffer.read.read;
      reqlen  = 0;

      /* Copy the variable length, file handle */

      *ptr++  = txdr_unsigned((uint32_t)np->n_fhsize);
      reqlen += sizeof(uint32_t);

      memcpy(ptr, &np->n_fhandle, np->n_fhsize);
      reqlen += (int)np->n_fhsize;
      ptr    += uint32_increment((int)np->n_fhsize);

      /* Copy the file offset */

      txdr_hyper((uint64_t)filep->f_pos, ptr);
      ptr += 2;
      reqlen += 2*sizeof(uint32_t);

      /* Set the readsize */

      *ptr = txdr_unsigned(readsize);
      reqlen += sizeof(uint32_t);

      /* Perform the read */

      fvdbg("Reading %d bytes\n", readsize);
      nfs_statistics(NFSPROC_READ);
      error = nfs_request(nmp, NFSPROC_READ,
                          (FAR void *)&nmp->nm_msgbuffer.read, reqlen,
                          (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);
      if (error)
        {
          fdbg("ERROR: nfs_request failed: %d\n", error);
          goto errout_with_semaphore;
        }

      /* The read was successful.  Get a pointer to the beginning of the NFS
       * response data.
       */

      ptr = (FAR uint32_t *)&((FAR struct rpc_reply_read *)nmp->nm_iobuffer)->read;

      /* Check if attributes are included in the responses */

      tmp = *ptr++;
      if (*ptr != 0)
        {
          /* Yes... just skip over the attributes for now */

          ptr += uint32_increment(sizeof(struct nfs_fattr));
        }

      /* This is followed by the count of data read.  Isn't this
       * the same as the length that is included in the read data?
       *
       * Just skip over if for now.
       */

      ptr++;

      /* Next comes an EOF indication.  Save that in tmp for now. */

      tmp = *ptr++;

      /* Then the length of the read data followed by the read data itself */

      readsize = fxdr_unsigned(uint32_t, *ptr);
      ptr++;

      /* Copy the read data into the user buffer */

      memcpy(buffer, ptr, readsize);

      /* Update the read state data */

      filep->f_pos += readsize;
      bytesread    += readsize;
      buffer       += readsize;

      /* Check if we hit the end of file */

      if (tmp != 0)
        {
          break;
        }
    }

  fvdbg("Read %d bytes\n", bytesread);
  nfs_semgive(nmp);
  return bytesread;

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

static ssize_t nfs_write(FAR struct file *filep, const char *buffer,
                         size_t buflen)
{
  struct nfsmount       *nmp;
  struct nfsnode        *np;
  ssize_t                writesize;
  ssize_t                bufsize;
  ssize_t                byteswritten;
  size_t                 reqlen;
  FAR uint32_t          *ptr;
  uint32_t               tmp;
  int                    commit = 0;
  int                    committed = NFSV3WRITE_FILESYNC;
  int                    error;

  fvdbg("Write %d bytes to offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (struct nfsmount*)filep->f_inode->i_private;
  np  = (struct nfsnode*)filep->f_priv;

  DEBUGASSERT(nmp != NULL);

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Check if the file size would exceed the range of off_t */

  if (np->n_size + buflen < np->n_size)
    {
      error = EFBIG;
      goto errout_with_semaphore;
    }

  /* Now loop until we send the entire user buffer */

  writesize = 0;
  for (byteswritten = 0; byteswritten < buflen; )
    {
      /* Make sure that the attempted write size does not exceed the RPC maximum */

      writesize = buflen;
      if (writesize > nmp->nm_wsize)
        {
          writesize = nmp->nm_wsize;
        }

      /* Make sure that the attempted read size does not exceed the IO buffer size */

      bufsize = SIZEOF_rpc_call_write(writesize);
      if (bufsize > nmp->nm_buflen)
        {
          writesize -= (bufsize - nmp->nm_buflen);
        }

      /* Initialize the request.  Here we need an offset pointer to the write
       * arguments, skipping over the RPC header.  Write is unique among the
       * RPC calls in that the entry RPC calls messasge lies in the I/O buffer
       */

      ptr     = (FAR uint32_t *)&((FAR struct rpc_call_write *)nmp->nm_iobuffer)->write;
      reqlen  = 0;

      /* Copy the variable length, file handle */

      *ptr++  = txdr_unsigned((uint32_t)np->n_fhsize);
      reqlen += sizeof(uint32_t);

      memcpy(ptr, &np->n_fhandle, np->n_fhsize);
      reqlen += (int)np->n_fhsize;
      ptr    += uint32_increment((int)np->n_fhsize);

      /* Copy the file offset */

      txdr_hyper((uint64_t)filep->f_pos, ptr);
      ptr    += 2;
      reqlen += 2*sizeof(uint32_t);

      /* Copy the count and stable values */

      *ptr++  = txdr_unsigned(buflen);
      *ptr++  = txdr_unsigned(committed);
      reqlen += 2*sizeof(uint32_t);

      /* Copy a chunk of the user data into the I/O buffer */

      *ptr++  = txdr_unsigned(buflen);
      reqlen += sizeof(uint32_t);
      memcpy(ptr, buffer, writesize);
      reqlen += uint32_alignup(writesize);

      /* Perform the write */

      nfs_statistics(NFSPROC_WRITE);
      error = nfs_request(nmp, NFSPROC_WRITE,
                          (FAR void *)nmp->nm_iobuffer, reqlen,
                          (FAR void *)&nmp->nm_msgbuffer.write, sizeof(struct rpc_reply_write));
      if (error)
        {
          fdbg("ERROR: nfs_request failed: %d\n", error);
          goto errout_with_semaphore;
        }

      /* Get a pointer to the WRITE reply data */

      ptr = (FAR uint32_t *)&nmp->nm_msgbuffer.write.write;

      /* Parse file_wcc.  First, check if WCC attributes follow. */

      tmp = *ptr++;
      if (tmp != 0)
        {
          /* Yes.. WCC attributes follow.  But we just skip over them. */

          ptr += uint32_increment(sizeof(struct wcc_attr));
        }

      /* Check if normal file attributes follow */

      tmp = *ptr++;
      if (tmp != 0)
        {
          /* Yes.. Update the cached file status in the file structure. */

          nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
          ptr += uint32_increment(sizeof(struct nfs_fattr));
        }

      /* Get the count of bytes actually written */

      tmp = fxdr_unsigned(uint32_t, *ptr);
      ptr++;

      if (tmp < 1 || tmp > writesize)
        {
           error = EIO;
           goto errout_with_semaphore;
        }

      writesize = tmp;

      /* Determine the lowest committment level obtained by any of the RPCs. */

      commit = *ptr++;
      if (committed == NFSV3WRITE_FILESYNC)
        {
          committed = commit;
        }
      else if (committed == NFSV3WRITE_DATASYNC &&
               commit == NFSV3WRITE_UNSTABLE)
        {
          committed = commit;
        }

      /* Update the read state data */

      filep->f_pos += writesize;
      byteswritten += writesize;
      buffer       += writesize;
    }

  nfs_semgive(nmp);
  return writesize;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: binfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int nfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  struct nfsmount *nmp;
  FAR struct nfsnode *np;
  int error;

  fvdbg("Dup %p->%p\n", oldp, newp);

  /* Sanity checks */

  DEBUGASSERT(oldp->f_priv != NULL && oldp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (struct nfsmount*)oldp->f_inode->i_private;
  np  = (struct nfsnode*)oldp->f_priv;

  DEBUGASSERT(nmp != NULL);

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      nfs_semgive(nmp);
      return -error;
    }

  /* Increment the reference count on the NFS node structure */

  DEBUGASSERT(np->n_crefs < 0xff);
  np->n_crefs++;

  /* And save this as the file data for the new node */

  newp->f_priv = np;

  /* Then insert the new instance at the head of the list in the mountpoint
   * tructure. It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy.  We
   * cannot unmount the file system if this list is not empty!
   */

  np->n_next   = nmp->nm_head;
  nmp->nm_head = np;

  nfs_semgive(nmp);
  return OK;
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
  struct file_handle fhandle;
  struct nfs_fattr obj_attributes;
  uint32_t objtype;
  int error;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && dir);

  /* Recover our private data from the inode instance */

  nmp = mountpt->i_private;

  /* Initialize the NFS-specific portions of dirent structure to zero */

  memset(&dir->u.nfs, 0, sizeof(struct nfsdir_s));

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Find the NFS node associate with the path */

  error = nfs_findnode(nmp, relpath, &fhandle, &obj_attributes, NULL);
  if (error != OK)
    {
      fdbg("ERROR: nfs_findnode failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* The entry is a directory */

  objtype = fxdr_unsigned(uint32_t, obj_attributes.fa_type);
  if (objtype != NFDIR)
    {
      fdbg("ERROR:  Not a directory, type=%d\n", objtype);
      error = ENOTDIR;
      goto errout_with_semaphore;
    }

  /* Save the directory information in struct fs_dirent_s so that it can be
   * used later when readdir() is called.
   */

  dir->u.nfs.nfs_fhsize = (uint8_t)fhandle.length;
  DEBUGASSERT(fhandle.length <= DIRENT_NFS_MAXHANDLE);

  memcpy(dir->u.nfs.nfs_fhandle, &fhandle.handle, DIRENT_NFS_MAXHANDLE);
  error = OK;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
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
  struct nfsmount *nmp;
  struct file_handle fhandle;
  struct nfs_fattr obj_attributes;
  uint32_t tmp;
  uint32_t *ptr;
  uint8_t *name;
  unsigned int length;
  int reqlen;
  int error = 0;

  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  nmp = mountpt->i_private;

  /* Make sure that the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Request a block directory entries, copying directory information from
   * the dirent structure.
   */

  ptr     = (FAR uint32_t*)&nmp->nm_msgbuffer.readdir.readdir;
  reqlen  = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned((uint32_t)dir->u.nfs.nfs_fhsize);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, dir->u.nfs.nfs_fhandle, dir->u.nfs.nfs_fhsize);
  reqlen += (int)dir->u.nfs.nfs_fhsize;
  ptr    += uint32_increment((int)dir->u.nfs.nfs_fhsize);

  /* Cookie and cookie verifier */

  ptr[0] = dir->u.nfs.nfs_cookie[0];
  ptr[1] = dir->u.nfs.nfs_cookie[1];
  ptr    += 2;
  reqlen += 2*sizeof(uint32_t);

  memcpy(ptr, dir->u.nfs.nfs_verifier, DIRENT_NFS_VERFLEN);
  ptr    += uint32_increment(DIRENT_NFS_VERFLEN);
  reqlen += DIRENT_NFS_VERFLEN;

  /* Number of directory entries (We currently only process one entry at a time) */

  *ptr    = txdr_unsigned(nmp->nm_readdirsize);
  reqlen += sizeof(uint32_t);

  /* And read the directory */

  nfs_statistics(NFSPROC_READDIR);
  error = nfs_request(nmp, NFSPROC_READDIR,
                      (FAR void *)&nmp->nm_msgbuffer.readdir, reqlen,
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);
  if (error != OK)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* A new group of entries was successfully read.  Process the
   * information contained in the response header.  This information
   * includes:
   *
   * 1) Attributes follow indication - 4 bytes
   * 2) Directory attributes         - sizeof(struct nfs_fattr)
   * 3) Cookie verifier              - NFSX_V3COOKIEVERF bytes
   * 4) Values follows indication    - 4 bytes
   */

  ptr = (uint32_t *)&((FAR struct rpc_reply_readdir *)nmp->nm_iobuffer)->readdir;

  /* Check if attributes follow, if 0 so Skip over the attributes */

  tmp = *ptr++;
  if (tmp != 0)
    {
      /* Attributes are not currently used */

      ptr += uint32_increment(sizeof(struct nfs_fattr));
    }

  /* Save the verification cookie */

  memcpy(dir->u.nfs.nfs_verifier, ptr, DIRENT_NFS_VERFLEN);
  ptr += uint32_increment(DIRENT_NFS_VERFLEN);

  /* Check if values follow.  If no values follow, then the EOF indication
   * will appear next.
   */

  tmp = *ptr++;
  if (tmp == 0)
    {
      /* No values follow, then the reply should consist only of a 4-byte
       * end-of-directory indication.
       */

      tmp = *ptr++;
      if (tmp != 0)
        {
          fvdbg("End of directory\n");
          error = ENOENT;
        }

      /* What would it mean if there were not data and we not at the end of
       * file?
       */

       else
         {
           fvdbg("No data but not end of directory???\n");
           error = EAGAIN;
        }

      goto errout_with_semaphore;
    }

  /* If we are not at the end of the directory listing, then a set of entries
   * will follow the header.  Each entry is of the form:
   *
   *    File ID (8 bytes)
   *    Name length (4 bytes)
   *    Name string (varaiable size but in multiples of 4 bytes)
   *    Cookie (8 bytes)
   *    next entry (4 bytes)
   */

  /* There is an entry. Skip over the file ID and point to the length */

  ptr += 2;

  /* Get the length and point to the name */

  tmp    = *ptr++;
  length = fxdr_unsigned(uint32_t, tmp);
  name   = (uint8_t*)ptr;

  /* Increment the pointer past the name (allowing for padding). ptr
   * now points to the cookie.
   */

  ptr += uint32_increment(length);

  /* Save the cookie and increment the pointer to the next entry */

  dir->u.nfs.nfs_cookie[0] = *ptr++;
  dir->u.nfs.nfs_cookie[1] = *ptr++;

  ptr++; /* Just skip over the nextentry for now */

  /* Return the name of the node to the caller */

  if (length > NAME_MAX)
    {
      length = NAME_MAX;
    }

  memcpy(dir->fd_dir.d_name, name, length);
  dir->fd_dir.d_name[length] = '\0';
  fvdbg("name: \"%s\"\n", dir->fd_dir.d_name);

  /* Get the file attributes associated with this name and return
   * the file type.
   */

  fhandle.length = (uint32_t)dir->u.nfs.nfs_fhsize;
  memcpy(&fhandle.handle, dir->u.nfs.nfs_fhandle, DIRENT_NFS_MAXHANDLE);

  error = nfs_lookup(nmp, dir->fd_dir.d_name, &fhandle, &obj_attributes, NULL);
  if (error != OK)
    {
      fdbg("nfs_lookup failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Set the dirent file type */

  tmp = fxdr_unsigned(uint32_t, obj_attributes.fa_type);
  switch (tmp)
    {
    default:
    case NFNON:        /* Unknown type */
    case NFSOCK:       /* Socket */
    case NFLNK:        /* Symbolic link */
      break;

    case NFREG:        /* Regular file */
      dir->fd_dir.d_type = DTYPE_FILE;
      break;

    case NFDIR:        /* Directory */
      dir->fd_dir.d_type = DTYPE_DIRECTORY;
      break;

    case NFBLK:        /* Block special device file */
      dir->fd_dir.d_type = DTYPE_BLK;
      break;

    case NFFIFO:       /* Named FIFO */
    case NFCHR:        /* Character special device file */
      dir->fd_dir.d_type = DTYPE_CHR;
      break;
    }
  fvdbg("type: %d->%d\n", (int)tmp, dir->fd_dir.d_type);

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_rewinddir
 *
 * Description:
 *  Reset the directory traveral logic to the first entry in the open
 *  directory.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_rewinddir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)
{
  fvdbg("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && dir != NULL);

  /* Reset the NFS-specific portions of dirent structure, retaining only the
   * file handle.
   */

  memset(&dir->u.nfs.nfs_verifier, 0, DIRENT_NFS_VERFLEN);
  dir->u.nfs.nfs_cookie[0] = 0;
  dir->u.nfs.nfs_cookie[1] = 0;
  return OK;
}

/****************************************************************************
 * Name: nfs_decode_args
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nfs_decode_args(FAR struct nfs_mount_parameters *nprmt,
                            FAR struct nfs_args *argp)
{
  int maxio;

  /* Get the selected timeout value */

  if ((argp->flags & NFSMNT_TIMEO) != 0 && argp->timeo > 0)
    {
      uint32_t tmp = ((uint32_t)argp->timeo * NFS_HZ + 5) / 10;
      if (tmp < NFS_MINTIMEO)
        {
          tmp = NFS_MINTIMEO;
        }
      else if (tmp > NFS_MAXTIMEO)
        {
          tmp = NFS_MAXTIMEO;
        }
      nprmt->timeo = tmp;
    }

  /* Get the selected retransmission count */

  if ((argp->flags & NFSMNT_RETRANS) != 0 && argp->retrans > 1)
    {
      if  (argp->retrans < NFS_MAXREXMIT)
        {
          nprmt->retry = argp->retrans;
        }
      else
        {
          nprmt->retry = NFS_MAXREXMIT;
        }
    }

  if ((argp->flags & NFSMNT_SOFT) == 0)
    {
      nprmt->retry = NFS_MAXREXMIT + 1;  /* Past clip limit */
    }

  /* Get the maximum amount of data that can be transferred in one packet */

  if ((argp->sotype == SOCK_DGRAM) != 0)
    {
      maxio = NFS_MAXDGRAMDATA;
    }
  else
    {
      fdbg("ERROR: Only SOCK_DRAM is supported\n");
      maxio = NFS_MAXDATA;
    }

  /* Get the maximum amount of data that can be transferred in one write transfer */

  if ((argp->flags & NFSMNT_WSIZE) != 0 && argp->wsize > 0)
    {
      nprmt->wsize = argp->wsize;

      /* Round down to multiple of blocksize */

      nprmt->wsize &= ~(NFS_FABLKSIZE - 1);
      if (nprmt->wsize <= 0)
        {
          nprmt->wsize = NFS_FABLKSIZE;
        }
    }

  if (nprmt->wsize > maxio)
    {
      nprmt->wsize = maxio;
    }

  if (nprmt->wsize > MAXBSIZE)
    {
      nprmt->wsize = MAXBSIZE;
    }

  /* Get the maximum amount of data that can be transferred in one read transfer */

  if ((argp->flags & NFSMNT_RSIZE) != 0 && argp->rsize > 0)
    {
      nprmt->rsize = argp->rsize;

      /* Round down to multiple of blocksize */

      nprmt->rsize &= ~(NFS_FABLKSIZE - 1);
      if (nprmt->rsize <= 0)
        {
          nprmt->rsize = NFS_FABLKSIZE;
        }
    }

  if (nprmt->rsize > maxio)
    {
      nprmt->rsize = maxio;
    }

  if (nprmt->rsize > MAXBSIZE)
    {
      nprmt->rsize = MAXBSIZE;
    }

  /* Get the maximum amount of data that can be transferred in directory transfer */

  if ((argp->flags & NFSMNT_READDIRSIZE) != 0 && argp->readdirsize > 0)
    {
      nprmt->readdirsize = argp->readdirsize;

      /* Round down to multiple of blocksize */

      nprmt->readdirsize &= ~(NFS_DIRBLKSIZ - 1);
      if (nprmt->readdirsize < NFS_DIRBLKSIZ)
        {
          nprmt->readdirsize = NFS_DIRBLKSIZ;
        }
    }
  else if (argp->flags & NFSMNT_RSIZE)
    {
      nprmt->readdirsize = nprmt->rsize;
    }

  if (nprmt->readdirsize > maxio)
    {
      nprmt->readdirsize = maxio;
    }
}

/****************************************************************************
 * Name: nfs_bind
 *
 * Description:
 *  This implements a portion of the mount operation. This function allocates
 *  and initializes the mountpoint private data and gets mount information
 *  from the NFS server.  The final binding of the private data (containing
 *  NFS server mount information) to the  mountpoint is performed by mount().
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                    FAR void **handle)
{
  FAR struct nfs_args        *argp = (FAR struct nfs_args *)data;
  FAR struct nfsmount        *nmp;
  struct rpcclnt             *rpc;
  struct rpc_call_fs          getattr;
  struct rpc_reply_getattr    resok;
  struct nfs_mount_parameters nprmt;
  uint32_t                    buflen;
  uint32_t                    tmp;
  int                         error = 0;

  DEBUGASSERT(data && handle);

  /* Set default values of the parameters.  These may be overridden by
   * settings in the argp->flags.
   */

  nprmt.timeo       = NFS_TIMEO;
  nprmt.retry       = NFS_RETRANS;
  nprmt.wsize       = NFS_WSIZE;
  nprmt.rsize       = NFS_RSIZE;
  nprmt.readdirsize = NFS_READDIRSIZE;

  nfs_decode_args(&nprmt, argp);

   /* Determine the size of a buffer that will hold one RPC data transfer.
    * First, get the maximum size of a read and a write transfer */

  buflen = SIZEOF_rpc_call_write(nprmt.wsize);
  tmp    = SIZEOF_rpc_reply_read(nprmt.rsize);

  /* The buffer size will be the maximum of those two sizes */

  if (tmp > buflen)
    {
      buflen = tmp;
    }

  /* But don't let the buffer size exceed the MSS of the socket type */

  if (buflen > UIP_UDP_MSS)
    {
      buflen = UIP_UDP_MSS;
    }

  /* Create an instance of the mountpt state structure */

  nmp = (FAR struct nfsmount *)kzalloc(SIZEOF_nfsmount(buflen));
  if (!nmp)
    {
      fdbg("ERROR: Failed to allocate mountpoint structure\n");
      return ENOMEM;
    }

  /* Save the allocated I/O buffer size */

  nmp->nm_buflen = (uint16_t)buflen;

  /* Initialize the allocated mountpt state structure. */

  /* Initialize the semaphore that controls access.  The initial count
   * is zero, but nfs_semgive() is called at the completion of initialization,
   * incrementing the count to one.
   */

  sem_init(&nmp->nm_sem, 0, 0);     /* Initialize the semaphore that controls access */

  /* Initialize NFS */

  nfs_true = txdr_unsigned(TRUE);
  nfs_false = txdr_unsigned(FALSE);
  nfs_xdrneg1 = txdr_unsigned(-1);

  rpcclnt_init();

  /* Set initial values of other fields */

  nmp->nm_timeo       = nprmt.timeo;
  nmp->nm_retry       = nprmt.retry;
  nmp->nm_wsize       = nprmt.wsize;
  nmp->nm_rsize       = nprmt.rsize;
  nmp->nm_readdirsize = nprmt.readdirsize;
  nmp->nm_fhsize      = NFSX_V3FHMAX;

  strncpy(nmp->nm_path, argp->path, 90);
  memcpy(&nmp->nm_nam, &argp->addr, argp->addrlen);

  /* Set up the sockets and per-host congestion */

  nmp->nm_sotype  = argp->sotype;

  if (nmp->nm_sotype == SOCK_DGRAM)
    {
      /* Connection-less... connect now */

      /* Create an instance of the rpc state structure */

      rpc = (struct rpcclnt *)kzalloc(sizeof(struct rpcclnt));
      if (!rpc)
        {
          fdbg("ERROR: Failed to allocate rpc structure\n");
          return ENOMEM;
        }

      fvdbg("Connecting\n");

      /* Translate nfsmnt flags -> rpcclnt flags */

      rpc->rc_path       = nmp->nm_path;
      rpc->rc_name       = &nmp->nm_nam;
      rpc->rc_sotype     = nmp->nm_sotype;
      rpc->rc_retry      = nmp->nm_retry;

      nmp->nm_rpcclnt    = rpc;

      error = rpcclnt_connect(nmp->nm_rpcclnt);
      if (error != OK)
        {
          fdbg("ERROR: nfs_connect failed: %d\n", error);
          goto bad;
        }
    }

  nmp->nm_mounted        = true;
  nmp->nm_so             = nmp->nm_rpcclnt->rc_so;
  memcpy(&nmp->nm_fh, &nmp->nm_rpcclnt->rc_fh, sizeof(nfsfh_t));

  /* Get the file attributes */

  getattr.fs.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  memcpy(&getattr.fs.fsroot.handle, &nmp->nm_fh, sizeof(nfsfh_t));

  error = nfs_request(nmp, NFSPROC_GETATTR,
                      (FAR void *)&getattr, sizeof(struct FS3args),
                      (FAR void*)&resok, sizeof(struct rpc_reply_getattr));
  if (error)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
      goto bad;
    }

  /* Save the file attributes */

  memcpy(&nmp->nm_fattr, &resok.attr, sizeof(struct nfs_fattr));

  /* Mounted! */

  *handle = (void*)nmp;
  nfs_semgive(nmp);

  fvdbg("Successfully mounted\n");
  return OK;

bad:
  if (nmp)
    {
      /* Disconnect from the server */

      rpcclnt_disconnect(nmp->nm_rpcclnt);

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
 * Name: nfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nfs_unbind(FAR void *handle, FAR struct inode **blkdriver)
{
  FAR struct nfsmount *nmp = (FAR struct nfsmount *)handle;
  int error;

  fvdbg("Entry\n");
  DEBUGASSERT(nmp);

  /* Get exclusive access to the mount structure */

  nfs_semtake(nmp);

  /* Are there any open files?  We can tell if there are open files by looking
   * at the list of file structures in the mount structure.  If this list
   * not empty, then there are open files and we cannot unmount now (or a
   * crash is sure to follow).
   */

  if (nmp->nm_head != NULL)
    {
      fdbg("ERROR;  There are open files: %p\n", nmp->nm_head);
      error = EBUSY;
      goto errout_with_semaphore;
    }

  /* No open file... Umount the file system. */

  error = rpcclnt_umount(nmp->nm_rpcclnt);
  if (error)
    {
      fdbg("ERROR: rpcclnt_umount failed: %d\n", error);
    }

  /* Disconnect from the server */

  rpcclnt_disconnect(nmp->nm_rpcclnt);

  /* And free any allocated resources */

  sem_destroy(&nmp->nm_sem);
  kfree(nmp->nm_so);
  kfree(nmp->nm_rpcclnt);
  kfree(nmp);

  return -error;

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_fsinfo
 *
 * Description:
 *   Return information about root directory.
 *
 * Returned Value:
 *   0 on success; positive errno value on failure
 *
 * Assumptions:
 *   The caller has exclusive access to the NFS mount structure
 *
 ****************************************************************************/

int nfs_fsinfo(FAR struct nfsmount *nmp)
{
  struct rpc_call_fs fsinfo;
  struct rpc_reply_fsinfo fsp;
  uint32_t pref;
  uint32_t max;
  int error = 0;

  fsinfo.fs.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  fsinfo.fs.fsroot.handle = nmp->nm_fh;

  /* Request FSINFO from the server */

  nfs_statistics(NFSPROC_FSINFO);
  error = nfs_request(nmp, NFSPROC_FSINFO,
                      (FAR void *)&fsinfo, sizeof(struct FS3args),
                      (FAR void *)&fsp, sizeof(struct rpc_reply_fsinfo));
  if (error)
    {
      return error;
    }

  /* Save the root file system attributes */

//memcpy(&nmp->nm_fattr. &fsp.obj_attributes, sizeof(struct nfs_fattr));

  pref = fxdr_unsigned(uint32_t, fsp.fsinfo.fs_wtpref);
  if (pref < nmp->nm_wsize)
    {
      nmp->nm_wsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  max = fxdr_unsigned(uint32_t, fsp.fsinfo.fs_wtmax);
  if (max < nmp->nm_wsize)
    {
      nmp->nm_wsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_wsize == 0)
        {
          nmp->nm_wsize = max;
        }
    }

  pref = fxdr_unsigned(uint32_t, fsp.fsinfo.fs_rtpref);
  if (pref < nmp->nm_rsize)
    {
      nmp->nm_rsize = (pref + NFS_FABLKSIZE - 1) & ~(NFS_FABLKSIZE - 1);
    }

  max = fxdr_unsigned(uint32_t, fsp.fsinfo.fs_rtmax);
  if (max < nmp->nm_rsize)
    {
      nmp->nm_rsize = max & ~(NFS_FABLKSIZE - 1);
      if (nmp->nm_rsize == 0)
        {
          nmp->nm_rsize = max;
        }
    }

  pref = fxdr_unsigned(uint32_t, fsp.fsinfo.fs_dtpref);
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

  return OK;
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

static int nfs_statfs(FAR struct inode *mountpt, FAR struct statfs *sbp)
{
  FAR struct nfsmount *nmp;
  FAR struct rpc_call_fs *fsstat;
  FAR struct rpc_reply_fsstat *sfp;
  int error = 0;
  uint64_t tquad;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Fill in the statfs info */

  sbp->f_type = NFS_SUPER_MAGIC;

  (void)nfs_fsinfo(nmp);

  fsstat = &nmp->nm_msgbuffer.fsstat;
  fsstat->fs.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  memcpy(&fsstat->fs.fsroot.handle, &nmp->nm_fh, sizeof(nfsfh_t));

  nfs_statistics(NFSPROC_FSSTAT);
  error = nfs_request(nmp, NFSPROC_FSSTAT,
                      (FAR void *)fsstat, sizeof(struct FS3args),
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);
  if (error)
    {
      goto errout_with_semaphore;
    }

  sfp                   = (FAR struct rpc_reply_fsstat *)nmp->nm_iobuffer;
  sbp->f_bsize          = NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_tbytes);
  sbp->f_blocks         = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_fbytes);
  sbp->f_bfree          = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_abytes);
  sbp->f_bavail         = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_tfiles);
  sbp->f_files          = tquad;
  tquad                 = fxdr_hyper(&sfp->fsstat.sf_ffiles);
  sbp->f_ffree          = tquad;
  sbp->f_namelen        = NAME_MAX;

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
  FAR struct nfsmount    *nmp;
  struct file_handle      fhandle;
  struct nfs_fattr        fattr;
  char                    filename[NAME_MAX + 1];
  FAR uint32_t           *ptr;
  int                     namelen;
  int                     reqlen;
  int                     error;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Find the NFS node of the directory containing the file to be deleted */

  error = nfs_finddir(nmp, relpath, &fhandle, &fattr, filename);
  if (error != OK)
    {
      fdbg("ERROR: nfs_finddir returned: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Create the REMOVE RPC call arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.removef.remove;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  reqlen += (int)fhandle.length;
  ptr    += uint32_increment(fhandle.length);

  /* Copy the variable-length file name */

  namelen = strlen(filename);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, filename, namelen);
  reqlen += uint32_alignup(namelen);

  /* Perform the REMOVE RPC call */

  nfs_statistics(NFSPROC_REMOVE);
  error = nfs_request(nmp, NFSPROC_REMOVE,
                      (FAR void *)&nmp->nm_msgbuffer.removef, reqlen,
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);

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
  struct nfsmount       *nmp;
  struct file_handle     fhandle;
  struct nfs_fattr       fattr;
  char                   dirname[NAME_MAX + 1];
  FAR uint32_t          *ptr;
  uint32_t               tmp;
  int                    namelen;
  int                    reqlen;
  int                    error;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*) mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Find the NFS node of the directory containing the directory to be created */

  error = nfs_finddir(nmp, relpath, &fhandle, &fattr, dirname);
  if (error != OK)
    {
      fdbg("ERROR: nfs_finddir returned: %d\n", error);
      return error;
    }

  /* Format the MKDIR call message arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.mkdir.mkdir;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  ptr    += uint32_increment(fhandle.length);
  reqlen += (int)fhandle.length;

  /* Copy the variable-length directory name */

  namelen = strlen(dirname);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, dirname, namelen);
  ptr    += uint32_increment(namelen);
  reqlen += uint32_alignup(namelen);

  /* Set the mode.  NOTE: Here we depend on the fact that the NuttX and NFS
   * bit settings are the same (at least for the bits of interest).
   */

  *ptr++  = nfs_true; /* True: mode value follows */
  reqlen += sizeof(uint32_t);

  tmp = mode & (NFSMODE_IXOTH | NFSMODE_IWOTH | NFSMODE_IROTH |
                NFSMODE_IXGRP | NFSMODE_IWGRP | NFSMODE_IRGRP |
                NFSMODE_IXUSR | NFSMODE_IWUSR | NFSMODE_IRUSR);
  *ptr++  = txdr_unsigned(tmp);
  reqlen += sizeof(uint32_t);

  /* Set the user ID to zero */

  *ptr++  = nfs_true;             /* True: Uid value follows */
  *ptr++  = 0;                    /* UID = 0 (nobody) */
  reqlen += 2*sizeof(uint32_t);

  /* Set the group ID to one */

  *ptr++  = nfs_true;            /* True: Gid value follows */
  *ptr++  = HTONL(1);            /* GID = 1 (nogroup) */
  reqlen += 2*sizeof(uint32_t);

  /* No size */

  *ptr++  = nfs_false; /* False: No size value follows */
  reqlen += sizeof(uint32_t);

  /* Don't change times */

  *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change atime */
  *ptr++  = HTONL(NFSV3SATTRTIME_DONTCHANGE); /* Don't change mtime */
  reqlen += 2*sizeof(uint32_t);

  /* Perform the MKDIR RPC */

  nfs_statistics(NFSPROC_MKDIR);
  error = nfs_request(nmp, NFSPROC_MKDIR,
                      (FAR void *)&nmp->nm_msgbuffer.mkdir, reqlen,
                      (FAR void *)&nmp->nm_iobuffer, nmp->nm_buflen);
  if (error)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
    }

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
  struct nfsmount       *nmp;
  struct file_handle     fhandle;
  struct nfs_fattr       fattr;
  char                   dirname[NAME_MAX + 1];
  FAR uint32_t          *ptr;
  int                    namelen;
  int                    reqlen;
  int                    error;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount *)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Find the NFS node of the directory containing the directory to be removed */

  error = nfs_finddir(nmp, relpath, &fhandle, &fattr, dirname);
  if (error != OK)
    {
      fdbg("ERROR: nfs_finddir returned: %d\n", error);
      return error;
    }

  /* Set up the RMDIR call message arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.rmdir.rmdir;
  reqlen = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &fhandle.handle, fhandle.length);
  reqlen += (int)fhandle.length;
  ptr    += uint32_increment(fhandle.length);

  /* Copy the variable-length directory name */

  namelen = strlen(dirname);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, dirname, namelen);
  reqlen += uint32_alignup(namelen);

  /* Perform the RMDIR RPC */

  nfs_statistics(NFSPROC_RMDIR);
  error = nfs_request(nmp, NFSPROC_RMDIR,
                          (FAR void *)&nmp->nm_msgbuffer.rmdir, reqlen,
                          (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);

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
  struct nfsmount        *nmp;
  struct file_handle      from_handle;
  struct file_handle      to_handle;
  char                    from_name[NAME_MAX+1];
  char                    to_name[NAME_MAX+1];
  struct nfs_fattr        fattr;
  FAR uint32_t           *ptr;
  int                     namelen;
  int                     reqlen;
  int                     error;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount *)mountpt->i_private;

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount returned: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Find the NFS node of the directory containing the 'from' object */

  error = nfs_finddir(nmp, oldrelpath, &from_handle, &fattr, from_name);
  if (error != OK)
    {
      fdbg("ERROR: nfs_finddir returned: %d\n", error);
      return error;
    }

  /* Find the NFS node of the directory containing the 'from' object */

  error = nfs_finddir(nmp, newrelpath, &to_handle, &fattr, to_name);
  if (error != OK)
    {
      fdbg("ERROR: nfs_finddir returned: %d\n", error);
      return error;
    }

  /* Format the RENAME RPC arguments */

  ptr    = (FAR uint32_t *)&nmp->nm_msgbuffer.renamef.rename;
  reqlen = 0;

  /* Copy the variable length, 'from' directory file handle */

  *ptr++  = txdr_unsigned(from_handle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &from_handle.handle, from_handle.length);
  reqlen += (int)from_handle.length;
  ptr    += uint32_increment(from_handle.length);

  /* Copy the variable-length 'from' object name */

  namelen = strlen(from_name);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, from_name, namelen);
  reqlen += uint32_alignup(namelen);
  ptr    += uint32_increment(namelen);

  /* Copy the variable length, 'to' directory file handle */

  *ptr++  = txdr_unsigned(to_handle.length);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, &to_handle.handle, to_handle.length);
  ptr    += uint32_increment(to_handle.length);
  reqlen += (int)to_handle.length;

  /* Copy the variable-length 'to' object name */

  namelen = strlen(to_name);

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, to_name, namelen);
  reqlen += uint32_alignup(namelen);

  /* Perform the RENAME RPC */

  nfs_statistics(NFSPROC_RENAME);
  error = nfs_request(nmp, NFSPROC_RENAME,
                      (FAR void *)&nmp->nm_msgbuffer.renamef, reqlen,
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}

/****************************************************************************
 * Name: nfs_stat
 *
 * Description:
 *   Return information about the file system object at 'relpath'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int nfs_stat(struct inode *mountpt, const char *relpath,
                    struct stat *buf)
{
  struct nfsmount   *nmp;
  struct file_handle fhandle;
  struct nfs_fattr   obj_attributes;
  uint32_t           tmp;
  uint32_t           mode;
  int                error;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  nmp = (struct nfsmount*)mountpt->i_private;
  DEBUGASSERT(nmp && buf);

  /* Check if the mount is still healthy */

  nfs_semtake(nmp);
  error = nfs_checkmount(nmp);
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Get the file handle attributes of the requested node */

  error = nfs_findnode(nmp, relpath, &fhandle, &obj_attributes, NULL);
  if (error != OK)
    {
      fdbg("ERROR: nfs_findnode failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Construct the file mode.  This is a 32-bit, encoded value containing
   * both the access mode and the file type.
   */

  tmp = fxdr_unsigned(uint32_t, obj_attributes.fa_mode);

  /* Here we exploit the fact that most mode bits are the same in NuttX
   * as in the NFSv3 spec.
   */

  mode = tmp & (NFSMODE_IXOTH|NFSMODE_IWOTH|NFSMODE_IROTH|
                NFSMODE_IXGRP|NFSMODE_IWGRP|NFSMODE_IRGRP|
                NFSMODE_IXUSR|NFSMODE_IWUSR|NFSMODE_IRUSR);

  /* Handle the cases that are not the same */

  if ((mode & NFSMODE_ISGID) != 0)
    {
      mode |= S_ISGID;
    }

  if ((mode & NFSMODE_ISUID) != 0)
    {
      mode |= S_ISUID;
    }

  /* Now OR in the file type */

  tmp = fxdr_unsigned(uint32_t, obj_attributes.fa_type);
  switch (tmp)
    {
    default:
    case NFNON:   /* Unknown type */
      break;

    case NFREG:   /* Regular file */
      mode |= S_IFREG;
      break;

    case NFDIR:   /* Directory */
      mode |= S_IFDIR;
      break;

    case NFBLK:   /* Block special device file */
      mode |= S_IFBLK;
      break;

    case NFCHR:   /* Character special device file */
      mode |= S_IFCHR;
      break;

    case NFLNK:   /* Symbolic link */
      mode |= S_IFLNK;
      break;

    case NFSOCK:  /* Socket */
      mode |= S_IFSOCK;
      break;

    case NFFIFO:  /* Named pipe */
      mode |= S_IFMT;
      break;
    }

  buf->st_mode    = mode;
  buf->st_size    = fxdr_hyper(&obj_attributes.fa_size);
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  buf->st_mtime   = fxdr_hyper(&obj_attributes.fa_mtime);
  buf->st_atime   = fxdr_hyper(&obj_attributes.fa_atime);
  buf->st_ctime   = fxdr_hyper(&obj_attributes.fa_ctime);

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}
