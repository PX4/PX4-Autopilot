/****************************************************************************
 * fs/fs_internal.h
 *
 *   Copyright (C) 2007, 2009, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __FS_FS_INTERNAL_H
#define __FS_FS_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <dirent.h>

#include <nuttx/fs/fs.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FSNODEFLAG_TYPE_MASK      0x00000003
#define   FSNODEFLAG_TYPE_DRIVER  0x00000000
#define   FSNODEFLAG_TYPE_BLOCK   0x00000001
#define   FSNODEFLAG_TYPE_MOUNTPT 0x00000002
#define FSNODEFLAG_DELETED        0x00000004

#define INODE_IS_DRIVER(i) \
  (((i)->i_flags & FSNODEFLAG_TYPE_MASK) == FSNODEFLAG_TYPE_DRIVER)
#define INODE_IS_BLOCK(i) \
  (((i)->i_flags & FSNODEFLAG_TYPE_BLOCK) == FSNODEFLAG_TYPE_BLOCK)
#define INODE_IS_MOUNTPT(i) \
  (((i)->i_flags & FSNODEFLAG_TYPE_MOUNTPT) == FSNODEFLAG_TYPE_MOUNTPT)

#define INODE_SET_DRIVER(i) \
  ((i)->i_flags &= ~FSNODEFLAG_TYPE_MASK)
#define INODE_SET_BLOCK(i) \
  ((i)->i_flags = (((i)->i_flags & ~FSNODEFLAG_TYPE_MASK) | FSNODEFLAG_TYPE_BLOCK))
#define INODE_SET_MOUNTPT(i) \
  ((i)->i_flags = (((i)->i_flags & ~FSNODEFLAG_TYPE_MASK) | FSNODEFLAG_TYPE_MOUNTPT))

/* Mountpoint fd_flags values */

#define DIRENTFLAGS_PSEUDONODE 1

#define DIRENT_SETPSEUDONODE(f) do (f) |= DIRENTFLAGS_PSEUDONODE; while (0)
#define DIRENT_ISPSEUDONODE(f) (((f) & DIRENTFLAGS_PSEUDONODE) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback used by foreach_inode to traverse all inodes in the pseudo-
 * file system.
 */

typedef int (*foreach_inode_t)(FAR struct inode *node,
                               FAR char dirpath[PATH_MAX],
                               FAR void *arg);

/****************************************************************************
 * Global Variables
 ****************************************************************************/

extern FAR struct inode *root_inode;

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

/* fs_inode.c ***************************************************************/
/****************************************************************************
 * Name: inode_semtake
 *
 * Description:
 *   Get exclusive access to the in-memory inode tree (tree_sem).
 *
 ****************************************************************************/

EXTERN void inode_semtake(void);

/****************************************************************************
 * Name: inode_semgive
 *
 * Description:
 *   Relinquish exclusive access to the in-memory inode tree (tree_sem).
 *
 ****************************************************************************/

EXTERN void inode_semgive(void);

/****************************************************************************
 * Name: inode_search
 *
 * Description:
 *   Find the inode associated with 'path' returning the inode references
 *   and references to its companion nodes.
 *
 * Assumptions:
 *   The caller holds the tree_sem
 *
 ****************************************************************************/

EXTERN FAR struct inode *inode_search(FAR const char **path,
                                      FAR struct inode **peer,
                                      FAR struct inode **parent,
                                      FAR const char **relpath);

/****************************************************************************
 * Name: inode_free
 *
 * Description:
 *   Free resources used by an inode
 *
 ****************************************************************************/

EXTERN void inode_free(FAR struct inode *node);

/****************************************************************************
 * Name: inode_nextname
 *
 * Description:
 *   Given a path with node names separated by '/', return the next node
 *   name.
 *
 ****************************************************************************/

EXTERN const char *inode_nextname(FAR const char *name);

/* fs_inodereserver.c *******************************************************/
/****************************************************************************
 * Name: inode_reserve
 *
 * Description:
 *   Reserve an (initialized) inode the pseudo file system.
 *
 *   NOTE: Caller must hold the inode semaphore
 *
 * Input parameters:
 *   path - The path to the inode to create
 *   inode - The location to return the inode pointer
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on failure:
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

EXTERN int inode_reserve(FAR const char *path, FAR struct inode **inode);

/* fs_inoderemove.c *********************************************************/
/****************************************************************************
 * Name: inode_remove
 *
 * Description:
 *   Remove a node from the in-memory, inode tree
 *
 *   NOTE: Caller must hold the inode semaphore
 *
 ****************************************************************************/

EXTERN int inode_remove(FAR const char *path);

/* fs_inodefind.c ***********************************************************/
/****************************************************************************
 * Name: inode_find
 *
 * Description:
 *   This is called from the open() logic to get a reference to the inode
 *   associated with a path.
 *
 ****************************************************************************/

EXTERN FAR struct inode *inode_find(FAR const char *path, const char **relpath);

/* fs_inodeaddref.c *********************************************************/

EXTERN void inode_addref(FAR struct inode *inode);

/* fs_inoderelease.c ********************************************************/

EXTERN void inode_release(FAR struct inode *inode);

/* fs_foreachinode.c ********************************************************/
/****************************************************************************
 * Name: foreach_inode
 *
 * Description:
 *   Visit each inode in the pseudo-file system.  The traversal is terminated
 *   when the callback 'handler' returns a non-zero value, or when all of
 *   the inodes have been visited.
 *
 *   NOTE 1: Use with caution... The pseudo-file system is locked throughout
 *   the traversal.
 *   NOTE 2: The search algorithm is recursive and could, in principle, use
 *   an indeterminant amount of stack space.  This will not usually be a
 *   real work issue.
 *
 ****************************************************************************/

EXTERN int foreach_inode(foreach_inode_t handler, FAR void *arg);

/* fs_files.c ***************************************************************/
/****************************************************************************
 * Name: files_initialize
 *
 * Description:
 *   This is called from the FS initialization logic to configure the files.
 *
 ****************************************************************************/

EXTERN void weak_function files_initialize(void);

/****************************************************************************
 * Name: files_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode instance.
 *   Returns the file descriptor == index into the files array.
 *
 ****************************************************************************/

EXTERN int  files_allocate(FAR struct inode *inode, int oflags, off_t pos, int minfd);

/****************************************************************************
 * Name: files_close
 *
 * Description:
 *   Close an inode (if open)
 *
 * Assumuptions:
 *   Caller holds the list semaphore because the file descriptor will be freed.
 *
 ****************************************************************************/

EXTERN int  files_close(int filedes);

/****************************************************************************
 * Name: files_release
 *
 * Assumuptions:
 *   Similar to files_close().  Called only from open() logic on error
 *   conditions.
 *
 ****************************************************************************/

EXTERN void files_release(int filedes);

/* fs_findblockdriver.c *****************************************************/
/****************************************************************************
 * Name: find_blockdriver
 *
 * Description:
 *   Return the inode of the block driver specified by 'pathname'
 *
 * Inputs:
 *   pathname - the full path to the block driver to be located
 *   mountflags - if MS_RDONLY is not set, then driver must support write
 *     operations (see include/sys/mount.h)
 *   ppinode - address of the location to return the inode reference
 *
 * Return:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - pathname or pinode is NULL
 *   ENOENT  - No block driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not a block driver
 *   EACCESS - The MS_RDONLY option was not set but this driver does not
 *     support write access
 *
 ****************************************************************************/

EXTERN int find_blockdriver(FAR const char *pathname, int mountflags,
                            FAR struct inode **ppinode);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_FS_INTERNAL_H */
