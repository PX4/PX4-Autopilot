/****************************************************************************
 * fs/nfs/nfs_vfsops.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
#include "nfs_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NFS_DIRHDSIZ         (sizeof (struct nfs_dirent) - (MAXNAMLEN + 1))
#define NFS_DIRENT_OVERHEAD  offsetof(struct nfs_dirent, dirent)

/* The V3 EXCLUSIVE file creation logic is not fully support. */

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

static int     nfs_filecreate(FAR struct nfsmount *nmp, struct nfsnode *np,
                   FAR const char *relpath, mode_t mode);
static int     nfs_fileopen(FAR struct nfsmount *nmp, struct nfsnode *np,
                   FAR const char *relpath, int oflags, mode_t mode);
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
static int     nfs_stat(struct inode *mountpt, const char *relpath,
                   struct stat *buf);

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
  struct CREATE3args      request;
  struct rpc_reply_create resok;
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

  ptr    = (FAR uint32_t *)&request;
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
      *ptr++  = HTONL(NFSV3CREATE_GUARDED);
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
                          (FAR const void *)&request, reqlen,
                          (FAR void *)&resok, sizeof(struct rpc_reply_create));
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

      ptr = (FAR uint32_t *)&resok.create;

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

      tmp = *ptr++;  /* handle_follows */
      if (!tmp)
        {
          fdbg("WARNING: no file attributes\n");
        }
      else
        {
          /* Initialize the file attributes */

          nfs_attrupdate(np, (FAR struct nfs_fattr *)ptr);
          ptr += uint32_increment(sizeof(struct nfs_fattr));
        }

      /* Any following dir_wcc data is ignored for now */
    }

  return error;
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

      fvdbg("Truncating file\n");
#warning "Missing logic"
      return ENOSYS;
    }

  /* Initialize the file private data */
  /* Copy the file handle */

  np->n_fhsize      = (uint8_t)fhandle.length;
  memcpy(&np->n_fhandle, &fhandle.handle, fhandle.length);

  /* Save the file attributes */

  nfs_attrupdate(np, &fattr);
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
  struct inode      *in;
  struct nfsmount   *nmp;
  struct nfsnode    *np;
  uint32_t           buflen;
  uint32_t           tmp;
  int                error;

  /* Sanity checks */

  DEBUGASSERT(filep->f_inode != NULL);

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  in  = filep->f_inode;
  nmp = (struct nfsmount*)in->i_private;

  DEBUGASSERT(nmp != NULL);

  /* Determine the size of a buffer that will hold one RPC data transfer */

  {
     /* Get the maximum size of a read and a write transfer */
 
     buflen = SIZEOF_rpc_call_write(nmp->nm_wsize);
     tmp    = SIZEOF_rpc_reply_read(nmp->nm_rsize);

     /* The buffer size will be the maximum of those two sizes */

     if (tmp > buflen)
       {
         buflen = tmp;
       }

     /* But don't let the buffer size exceed the MSS of the socket type */

#ifdef CONFIG_NFS_TCPIP
     if (buflen > UIP_TCP_MSS)
       {
         buflen = UIP_TCP_MSS;
       }
#else
     if (buflen > UIP_UDP_MSS)
       {
         buflen = UIP_UDP_MSS;
       }
#endif
  }

  /* Pre-allocate the file private data to describe the opened file. */

  np = (struct nfsnode *)kzalloc(SIZEOF_nfsnode(buflen));
  if (!np)
    {
      fdbg("ERROR: Failed to allocate private data\n");
      return -ENOMEM;
    }

  /* Save the allocated I/O buffer size */

  np->n_buflen = (uint16_t)buflen;

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

  /* Attach the private data to the struct file instance */

  filep->f_priv = np;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  np->n_next   = nmp->nm_head;
  np->n_flags |= (NFSNODE_OPEN | NFSNODE_MODIFIED);
  nmp->nm_head = np->n_next;

  nfs_semgive(nmp);
  return OK;

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

  if (np->n_type == NFREG)
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
  FAR struct nfsmount       *nmp;
  FAR struct nfsnode        *np;
  ssize_t                    readsize;
  ssize_t                    tmp;
  ssize_t                    bytesread;
  size_t                     reqlen;
  struct READ3args           request;
  FAR uint32_t              *ptr;
  int                        error = 0;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  nmp = (struct nfsmount*) filep->f_inode->i_private;
  np  = (struct nfsnode*) filep->f_priv;

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
      if (tmp > np->n_buflen)
        {
          readsize -= (tmp - np->n_buflen);
        }

      /* Initialize the request */

      ptr     = (FAR uint32_t*)&request;
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
                          (FAR const void *)&request, reqlen,
                          (FAR void *)np->n_iobuffer, np->n_buflen);
      if (error)
        {
          fdbg("ERROR: nfs_request failed: %d\n", error);
          goto errout_with_semaphore;
        }

      /* The read was successful.  Get a pointer to the beginning of the NFS
       * response data.
       */

      ptr = (FAR uint32_t *)&((FAR struct rpc_reply_read *)np->n_iobuffer)->read;

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
  struct rpc_reply_write resok;
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
      if (bufsize > np->n_buflen)
        {
          writesize -= (bufsize - np->n_buflen);
        }

      /* Initialize the request.  Here we need an offset pointer to the write
       * arguments, skipping over the RPC header.  Write is unique among the
       * RPC calls in that the entry RPC calls messasge lies in the I/O buffer
       */

      ptr     = (FAR uint32_t *)&((FAR struct rpc_call_write *)np->n_iobuffer)->write;
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
                          (FAR const void *)np->n_iobuffer, reqlen,
                          (FAR void *)&resok, sizeof(struct rpc_reply_write));
      if (error)
        {
          fdbg("ERROR: nfs_request failed: %d\n", error);
          goto errout_with_semaphore;
        }

      /* Get a pointer to the WRITE reply data */

      ptr = (FAR uint32_t *)&resok.write;

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

      /* Save the verifier if needed or if it has change*/

      if ((nmp->nm_flag & NFSMNT_HASWRITEVERF) == 0)
        {
          memcpy(nmp->nm_verf, ptr, NFSX_V3WRITEVERF);
          nmp->nm_flag |= NFSMNT_HASWRITEVERF;
        }
      else if (memcmp(ptr, nmp->nm_verf, NFSX_V3WRITEVERF) != 0)
        {
          memcpy(nmp->nm_verf, ptr, NFSX_V3WRITEVERF);
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
  uint32_t buffer[64];
  struct READDIR3args request;
  struct rpc_reply_readdir *resok;
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

  ptr     = (FAR uint32_t*)&request;
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
                      (FAR const void *)&request, reqlen,
                      (FAR void *)buffer, sizeof(buffer));
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

  resok = (struct rpc_reply_readdir *)buffer;

  /* Start with the first entry */

  ptr = (uint32_t*)&resok->readdir;

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
  nmp->nm_fhsize      = NFSX_V3FHMAX;

  strncpy(nmp->nm_path, argp->path, 90);
  memcpy(&nmp->nm_nam, &argp->addr, argp->addrlen);

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
      if (error != OK)
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

  np->n_type             = NFDIR;
  np->n_flags           |= (NFSNODE_OPEN | NFSNODE_MODIFIED);
  nmp->nm_head           = np;
  nmp->nm_mounted        = true;
  memcpy(&nmp->nm_fh, &nmp->nm_rpcclnt->rc_fh, sizeof(nfsfh_t));
  nmp->nm_fhsize         = NFSX_V3FHMAX;
  memcpy(&nmp->nm_head->n_fhandle, &nmp->nm_fh, sizeof(nfsfh_t));
  nmp->nm_head->n_fhsize = nmp->nm_fhsize;
  nmp->nm_so             = nmp->nm_rpcclnt->rc_so;

  /* Get the file attributes */

  memset(&getattr, 0, sizeof(struct FS3args));
  memset(&resok, 0, sizeof(struct rpc_reply_getattr));
  getattr.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  memcpy(&getattr.fsroot.handle, &nmp->nm_fh, sizeof(nfsfh_t));

  error = nfs_request(nmp, NFSPROC_GETATTR,
                      (FAR const void *)&getattr, sizeof(struct FS3args),
                      (FAR void*)&resok, sizeof(struct rpc_reply_getattr));
  if (error)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
      goto bad;
    }

  /* Save the file attributes */

  memcpy(&nmp->nm_fattr, &resok.attr, sizeof(struct nfs_fattr));
  nfs_attrupdate(np, &resok.attr);

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

  memcpy(&args, data, sizeof(struct nfs_args));
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
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Fill in the statfs info */

  memset(sbp, 0, sizeof(struct statfs));
  sbp->f_type = NFS_SUPER_MAGIC;

  if ((nmp->nm_flag & NFSMNT_GOTFSINFO) == 0)
    {
      (void)nfs_fsinfo(nmp);
    }

  nfs_statistics(NFSPROC_FSSTAT);
  memset(&fsstat, 0, sizeof(struct FS3args));
  memset(&sfp, 0, sizeof(struct rpc_reply_fsstat));
  fsstat.fsroot.length = txdr_unsigned(nmp->nm_fhsize);
  fsstat.fsroot.handle = nmp->nm_fh;

  error = nfs_request(nmp, NFSPROC_FSSTAT,
                      (FAR const void *)&fsstat, sizeof(struct FS3args),
                      (FAR void *) &sfp, sizeof(struct rpc_reply_fsstat));
  if (error)
    {
      goto errout_with_semaphore;
    }

  sbp->f_bsize          = NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp.fsstat.sf_tbytes);
  sbp->f_blocks         = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp.fsstat.sf_fbytes);
  sbp->f_bfree          = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp.fsstat.sf_abytes);
  sbp->f_bavail         = tquad / (uint64_t) NFS_FABLKSIZE;
  tquad                 = fxdr_hyper(&sfp.fsstat.sf_tfiles);
  sbp->f_files          = tquad;
  tquad                 = fxdr_hyper(&sfp.fsstat.sf_ffiles);
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
  struct REMOVE3args      remove;
  struct rpc_reply_remove resok;
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

  ptr    = (FAR uint32_t *)&remove;
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
                      (FAR const void *)&remove, reqlen,
                      (FAR void *)&resok, sizeof(struct rpc_reply_remove));

  /* Check if the file removal was successful */

#ifdef CONFIG_NFS_TCPIP
  if (error == ENOENT)
    {
      /* If the first reply to the remove rpc is lost, the reply to the
       * retransmitted request may be ENOENT if the file was in fact removed.
       * Therefore, we cheat and return success.
       */

      error = OK;
    }
#endif

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
  struct MKDIR3args      request;
  struct rpc_reply_mkdir resok;
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

  ptr    = (FAR uint32_t *)&request;
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
                      (FAR const void *)&request, reqlen,
                      (FAR void *)&resok, sizeof(struct rpc_reply_mkdir));
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
  struct RMDIR3args      request;
  struct rpc_reply_rmdir resok;
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

  ptr    = (FAR uint32_t *)&request;
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
                          (FAR const void *)&request, reqlen,
                          (FAR void *)&resok, sizeof(struct rpc_reply_rmdir));

  /* Check if the removal was successful */

#ifdef CONFIG_NFS_TCPIP
  if (error == ENOENT)
    {
      /* If the first reply to the remove rpc is lost, the reply to the
       * retransmitted request may be ENOENT if the file was in fact removed.
       * Therefore, we cheat and return success.
       */
          error = 0;
    }
#endif

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
  struct RENAME3args      request;
  struct rpc_reply_rename resok;
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

  ptr    = (FAR uint32_t *)&request;
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
                      (FAR const void *)&request, reqlen,
                      (FAR void *)&resok, sizeof(struct rpc_reply_rename));

  /* Check if the rename was successful */

#ifdef CONFIG_NFS_TCPIP
  if (error == ENOENT)
    {
      /* If the first reply to the remove rpc is lost, the reply to the
       * retransmitted request may be ENOENT if the file was in fact removed.
       * Therefore, we cheat and return success.
       */

      error = 0;
    }
#endif

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
  if (error != OK)
    {
      fdbg("ERROR: nfs_checkmount failed: %d\n", error);
      goto errout_with_semaphore;
    }

  /* Force stale buffer cache information to be flushed. */

  /* Check if the has been modified in any way */

  if ((np->n_flags & NFSNODE_MODIFIED) != 0)
    {
      //error = VOP_FSYNC(vp, cred, waitfor, p);
    }

errout_with_semaphore:
  nfs_semgive(nmp);
  return -error;
}
#endif
