/****************************************************************************
 * fs/nfs/nfs_util.c
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/time.h>

#include <stdint.h>
#include <queue.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/dirent.h>

#include "rpc.h"
#include "nfs.h"
#include "nfs_proto.h"
#include "nfs_mount.h"
#include "nfs_node.h"
#include "xdr_subs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int nfs_pathsegment(FAR const char **path, FAR char *buffer,
                                  FAR char *terminator)
{
  FAR const char *src = *path;
  FAR char *dest = buffer;
  int nbytes = 0;
  char ch;

  /* Loop until the name is successfully parsed or an error occurs */

  for (;;)
    {
      /* Get the next byte from the path */

      ch = *src++;

      /* Check if this the last byte in this segment name */

      if (ch == '\0' || ch == '/')
        {
          /* This logic just suppors "//" sequences in the path name */

          if (ch == '\0' || nbytes > 0 )
            {
              /* NULL terminate the parsed path segment */

              *dest        = '\0';

              /* Return next path and the terminating character */

              *terminator = ch;
              *path       = src;
              return OK;
            }

          /* Just skip over any leading '/' characters */
        }
      else if (nbytes >= NAME_MAX)
        {
          fdbg("File name segment is too long: %d\n", *path);
          return EFBIG;
        }
      else
        {
          /* Save next character in the accumulated name */

          *dest++ = ch;
          nbytes++;
        }
    }
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nfs_semtake
 ****************************************************************************/

void nfs_semtake(struct nfsmount *nmp)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&nmp->nm_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/****************************************************************************
 * Name: nfs_semgive
 ****************************************************************************/

void nfs_semgive(struct nfsmount *nmp)
{
  sem_post(&nmp->nm_sem);
}

/****************************************************************************
 * Name: nfs_checkmount
 *
 * Desciption: Check if the mountpoint is still valid.
 *
 *   The caller should hold the mountpoint semaphore
 *
 ****************************************************************************/

int nfs_checkmount(struct nfsmount *nmp)
{
  struct nfsnode *file;

  /* If the nm_mounted flag is false, then we have already handled the loss
   * of the mount.
   */

  DEBUGASSERT(nmp);
  if (!nmp->nm_mounted)
    {
      /* Make sure that this is flagged in every opened file */

      for (file = nmp->nm_head; file; file = file->n_next)
        {
          file->n_flags &= ~NFSNODE_OPEN;
        }

      return -ENODEV;
    }

  return 0;
}

/****************************************************************************
 * Name: nfs_request
 *
 * Desciption:
 *   Perform the NFS request. On successful receipt, it verifies the NFS level of the 
 *   returned values.
 *
 * Return Value:
 *   Zero on success; a positive errno value on failure.
 *
 ****************************************************************************/

int nfs_request(struct nfsmount *nmp, int procnum,
                FAR void *request, size_t reqlen,
                FAR void *response, size_t resplen)
{
  struct rpcclnt *clnt = nmp->nm_rpcclnt;
  struct nfs_reply_header replyh;
  int error;

tryagain:
  error = rpcclnt_request(clnt, procnum, NFS_PROG, NFS_VER3,
                          request, reqlen, response, resplen);
  if (error != 0)
    {
      fdbg("ERROR: rpcclnt_request failed: %d\n", error);
      return error;
    }

  memcpy(&replyh, response, sizeof(struct nfs_reply_header));

  if (replyh.nfs_status != 0)
    {
      if (fxdr_unsigned(uint32_t, replyh.nfs_status) > 32)
        {
          error = EOPNOTSUPP;
        }
      else
        {
          /* NFS_ERRORS are the same as NuttX errno values */

          error = fxdr_unsigned(uint32_t, replyh.nfs_status);
        }

      return error;
    }

  if (replyh.rpc_verfi.authtype != 0)
    {
      error = fxdr_unsigned(int, replyh.rpc_verfi.authtype);

      if (error == EAGAIN)
        {
          error = 0;
          goto tryagain;
        }

      fdbg("ERROR: NFS error %d from server\n", error);
      return error;
    }

  fvdbg("NFS_SUCCESS\n");
  return OK;
}

/****************************************************************************
 * Name: nfs_lookup
 *
 * Desciption:
 *   Given a directory file handle, and the path to file in the directory,
 *   return the file handle of the path and attributes of both the file and
 *   the directory containing the file.
 *
 *   NOTE:  The LOOKUP call differs from other RPC messages in that the
 *   call message is variable length, depending upon the size of the path
 *   name.
 *
 ****************************************************************************/

int nfs_lookup(struct nfsmount *nmp, FAR const char *filename,
               FAR struct file_handle *fhandle,
               FAR struct nfs_fattr *obj_attributes,
               FAR struct nfs_fattr *dir_attributes)
{
  FAR uint32_t *ptr;
  uint32_t value;
  int reqlen;
  int namelen;
  int error = 0;

  DEBUGASSERT(nmp && filename && fhandle);

  /* Get the length of the string to be sent */

  namelen = strlen(filename);
  if (namelen > NAME_MAX)
    {
      fdbg("Length of the string is too big: %d\n", namelen);
      return E2BIG;
    }

  /* Initialize the request */

  ptr     = (FAR uint32_t *)&nmp->nm_msgbuffer.lookup.lookup;
  reqlen  = 0;

  /* Copy the variable length, directory file handle */

  *ptr++  = txdr_unsigned(fhandle->length);
  reqlen += sizeof(uint32_t);
  
  memcpy(ptr, &fhandle->handle, fhandle->length);
  reqlen += fhandle->length;
  ptr    += uint32_increment(fhandle->length);

  /* Copy the variable-length file name */

  *ptr++  = txdr_unsigned(namelen);
  reqlen += sizeof(uint32_t);

  memcpy(ptr, filename, namelen);
  reqlen += uint32_alignup(namelen);

  /* Request LOOKUP from the server */

  nfs_statistics(NFSPROC_LOOKUP);
  error = nfs_request(nmp, NFSPROC_LOOKUP,
                      (FAR void *)&nmp->nm_msgbuffer.lookup, reqlen,
                      (FAR void *)nmp->nm_iobuffer, nmp->nm_buflen);

  if (error)
    {
      fdbg("ERROR: nfs_request failed: %d\n", error);
      return error;
    }

  /* Return the data to the caller's buffers.  NOTE:  Here we ignore the
   * the exact layout of the rpc_reply_lookup structure.  File handles
   * may differ in size whereas struct rpc_reply_lookup uses a fixed size.
   */

  ptr = (FAR uint32_t *)&((FAR struct rpc_reply_lookup *)nmp->nm_iobuffer)->lookup;

  /* Get the length of the file handle */

  value = *ptr++;
  value = fxdr_unsigned(uint32_t, value);
  if (value > NFSX_V3FHMAX)
    {
      fdbg("ERROR: Bad file handle length: %d\n", value);
      return EIO;
    }

  /* Return the file handle */

  fhandle->length = value;
  memcpy(&fhandle->handle, ptr, value);
  ptr += uint32_increment(value);

  /* Check if there are object attributes and, if so, copy them to the user
   * buffer
   */

  value = *ptr++;
  if (value)
    {
      if (obj_attributes)
        {
          memcpy(obj_attributes, ptr, sizeof(struct nfs_fattr));
        }
      ptr += uint32_increment(sizeof(struct nfs_fattr));
    }

  /* Check if there are directory attributes and, if so, copy them to the
   * user buffer
   */

  value = *ptr++;
  if (value && dir_attributes)
    {
      memcpy(dir_attributes, ptr, sizeof(struct nfs_fattr));
    }

  return OK;
}

/****************************************************************************
 * Name: nfs_findnode
 *
 * Desciption:
 *   Given a path to something that may or may not be in the file system,
 *   return the handle of the directory entry of the requested object.
 *
 * Return Value:
 *   Zero on success; a positive errno value on failure.
 *
 ****************************************************************************/

int nfs_findnode(struct nfsmount *nmp, FAR const char *relpath,
                 FAR struct file_handle *fhandle, FAR struct nfs_fattr *obj_attributes,
                 FAR struct nfs_fattr *dir_attributes)
{
  FAR const char *path = relpath;
  char            buffer[NAME_MAX+1];
  char            terminator;
  uint32_t         tmp;
  int             error;

  /* Start with the file handle of the root directory.  */

  fhandle->length = nmp->nm_fhsize;
  memcpy(&fhandle->handle, &nmp->nm_fh, nmp->nm_fhsize);

  /* If no path was provided, then the root directory must be exactly what
   * the caller is looking for.
   */

  if (*path == '\0' || strlen(path) == 0)
    {
      /* Return the root directory attributes */

      if (obj_attributes)
        {
          memcpy(obj_attributes, &nmp->nm_fattr, sizeof(struct nfs_fattr));
        }

      if (dir_attributes)
        {
          memcpy(dir_attributes, &nmp->nm_fattr, sizeof(struct nfs_fattr));
        }

      return OK;
    }

  /* This is not the root directory. Loop until the directory entry corresponding
   * to the path is found.
   */

  for (;;)
    {
      /* Extract the next path segment name. */

      error = nfs_pathsegment(&path, buffer, &terminator);
      if (error != OK)
        {
          /* The filename segment contains is too long. */

          fdbg("nfs_pathsegment of \"%s\" failed after \"%s\": %d\n",
               relpath, buffer, error);
          return error;
        }

      /* Look-up this path segment */

      error = nfs_lookup(nmp, buffer, fhandle, obj_attributes, dir_attributes);
      if (error != OK)
        {
          fdbg("nfs_lookup of \"%s\" failed at \"%s\": %d\n",
                relpath, buffer, error);
          return error;
        }

      /* If the terminator character in the path was the end of the string
       * then we have successfully found the directory entry that describes
       * the path.
       */

      if (!terminator)
        {
          /* Return success meaning that the description the matching
           * directory entry is in fhandle, obj_attributes, and dir_attributes.
           */

          return OK;
        }

      /* No.. then we have found one of the intermediate directories on
       * the way to the final path target.  In this case, make sure
       * the thing that we found is, indeed, a directory.
       */

      tmp = fxdr_unsigned(uint32_t, obj_attributes->fa_type);
      if (tmp != NFDIR)
        {
          /* Ooops.. we found something else */

          fdbg("ERROR: Intermediate segment \"%s\" of \'%s\" is not a directory\n",
               buffer, path);
          return ENOTDIR;
        }
    }
}

/****************************************************************************
 * Name: nfs_finddir
 *
 * Desciption:
 *   Given a path to something that may or may not be in the file system,
 *   return the handle of the entry of the directory containing the requested
*    object.
 *
 * Return Value:
 *   Zero on success; a positive errno value on failure.
 *
 ****************************************************************************/

int nfs_finddir(struct nfsmount *nmp, FAR const char *relpath,
                FAR struct file_handle *fhandle,
                FAR struct nfs_fattr *attributes, FAR char *filename)
{
  FAR const char  *path = relpath;
  uint32_t         tmp;
  char             terminator;
  int              error;

  /* Verify that a path was provided */

  if (*path == '\0' || strlen(path) == 0)
    {
      /* Return the root directory attributes */

      return ENOENT;
    }

  /* Start with the file handle of the root directory.  */

  fhandle->length = nmp->nm_fhsize;
  memcpy(&fhandle->handle, &nmp->nm_fh, nmp->nm_fhsize);
  memcpy(attributes, &nmp->nm_fattr, sizeof(struct nfs_fattr));

  /* Loop until the directory entry containing the path is found. */

  for (;;)
    {
      /* Extract the next path segment name. */

      error = nfs_pathsegment(&path, filename, &terminator);
      if (error != OK)
        {
          /* The filename segment contains is too long. */

          fdbg("nfs_pathsegment of \"%s\" failed after \"%s\": %d\n",
               relpath, filename, error);
          return error;
        }

      /* If the terminator character in the path was the end of the string
       * then we have successfully found the directory that contains the name
       * of interest.
       */

      if (!terminator)
        {
          /* Return success meaning that the description of the directory
           * containing the object is in fhandle and attributes.
           */

          return OK;
        }

      /* Look-up the next path segment */

      error = nfs_lookup(nmp, filename, fhandle, attributes, NULL);
      if (error != OK)
        {
          fdbg("nfs_lookup of \"%s\" failed at \"%s\": %d\n",
                relpath, filename, error);
          return error;
        }

      /* Make sure the thing that we found is, indeed, a directory. */

      tmp = fxdr_unsigned(uint32_t, attributes->fa_type);
      if (tmp != NFDIR)
        {
          /* Ooops.. we found something else */

          fdbg("ERROR: Intermediate segment \"%s\" of \'%s\" is not a directory\n",
               filename, path);
          return ENOTDIR;
        }
    }
}

/****************************************************************************
 * Name: nfs_attrupdate
 *
 * Desciption:
 *   Update file attributes on write or after the file is modified.
 *
 * Return Value:
 *   None.
 *
 ****************************************************************************/

void nfs_attrupdate(FAR struct nfsnode *np, FAR struct nfs_fattr *attributes)
{
  /* Save a few of the files attribute values in file structur (host order) */

  np->n_type   = fxdr_unsigned(uint32_t, attributes->fa_type);
  np->n_size   = fxdr_hyper(&attributes->fa_size);
  fxdr_nfsv3time(&attributes->fa_mtime, &np->n_mtime)
  np->n_ctime  = fxdr_hyper(&attributes->fa_ctime);
}
