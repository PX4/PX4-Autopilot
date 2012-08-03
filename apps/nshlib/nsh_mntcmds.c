/****************************************************************************
 * apps/nshlib/nsh_mntcmds.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <sys/mount.h>
#include <sys/statfs.h>

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_handler
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_READABLE)
#ifndef CONFIG_NSH_DISABLE_MOUNT
static int mount_handler(FAR const char *mountpoint,
                         FAR struct statfs *statbuf, FAR void *arg)
{
  FAR struct nsh_vtbl_s *vtbl = (FAR struct nsh_vtbl_s *)arg;
  FAR const char *fstype;

  DEBUGASSERT(mountpoint && statbuf && vtbl);

  /* Get the file system type */

  switch (statbuf->f_type)
    {
#ifdef CONFIG_FS_FAT
      case MSDOS_SUPER_MAGIC:
        fstype = "vfat";
        break;
#endif

#ifdef CONFIG_FS_ROMFS
      case ROMFS_MAGIC:
        fstype = "romfs";
        break;
#endif

#ifdef CONFIG_APPS_BINDIR
      case BINFS_MAGIC:
        fstype = "bindir";
        break;
#endif

#ifdef CONFIG_FS_NXFFS
      case NXFFS_MAGIC:
        fstype = "nxffs";
        break;
#endif

#ifdef CONFIG_NFS
      case NFS_SUPER_MAGIC:
        fstype = "nfs";
        break;
#endif

      default:
        fstype = "Unrecognized";
        break;
    }

  nsh_output(vtbl, "  %s type %s\n", mountpoint, fstype);
  return OK;
}
#endif
#endif

/****************************************************************************
 * Name: mount_show
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_READABLE)
#ifndef CONFIG_NSH_DISABLE_MOUNT
static inline int mount_show(FAR struct nsh_vtbl_s *vtbl, FAR const char *progname)
{
  return foreach_mountpoint(mount_handler, (FAR void *)vtbl);
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_mount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_READABLE)
#ifndef CONFIG_NSH_DISABLE_MOUNT
int cmd_mount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *source;
  char *target;
  char *filesystem = 0;
  bool badarg = false;
  int ret;

  /* The mount command behaves differently if no parameters are provided */

  if (argc < 2)
    {
      return mount_show(vtbl, argv[0]);
    }

  /* Get the mount options */

  int option;
  while ((option = getopt(argc, argv, ":t:")) != ERROR)
    {
      switch (option)
        {
          case 't':
            filesystem = optarg;
            break;

          case ':':
            nsh_output(vtbl, g_fmtargrequired, argv[0]);
            badarg = true;
            break;

          case '?':
          default:
            nsh_output(vtbl, g_fmtarginvalid, argv[0]);
            badarg = true;
            break;
        }
    }

  /* If a bad argument was encountered, then return without processing the command */

  if (badarg)
    {
      return ERROR;
    }

  /* There are two required arguments after the options */

  if (optind + 2 < argc)
    {
      nsh_output(vtbl, g_fmttoomanyargs, argv[0]);
      return ERROR;
    }
  else if (optind + 2 > argc)
    {
      nsh_output(vtbl, g_fmtargrequired, argv[0]);
      return ERROR;
    }

  /* The source and target paths might be relative to the current
   * working directory.
   */

  source = nsh_getfullpath(vtbl, argv[optind]);
  if (!source)
    {
      return ERROR;
    }

  target = nsh_getfullpath(vtbl, argv[optind+1]);
  if (!target)
    {
      nsh_freefullpath(source);
      return ERROR;
    }

  /* Perform the mount */

  ret = mount(source, target, filesystem, 0, NULL);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mount", NSH_ERRNO);
    }

  nsh_freefullpath(source);
  nsh_freefullpath(target);
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_nfsmount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && \
    defined(CONFIG_NET) && defined(CONFIG_NFS)
#ifndef CONFIG_NSH_DISABLE_NFSMOUNT
int cmd_nfsmount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct nfs_args data;
  FAR char *address;
  FAR char *lpath;
  FAR char *rpath;
  bool badarg = false;
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 *sin;
  struct in6_addr inaddr;
#else
  FAR struct sockaddr_in *sin;
  struct in_addr inaddr;
#endif
  int ret;

  /* If a bad argument was encountered, then return without processing the
   * command.
   */

  if (badarg)
    {
      return ERROR;
    }

  /* The fist argument on the command line should be the NFS server IP address
   * in standard IPv4 (or IPv6) dot format.
   */

  address = argv[1];
  if (!address)
    {
      return ERROR;
    }

  /* The local mount point path (lpath) might be relative to the current working
   * directory.
   */

  lpath = nsh_getfullpath(vtbl, argv[2]);
  if (!lpath)
    {
      return ERROR;
    }

  /* Get the remote mount point path */

  rpath = argv[3];

   /* Convert the IP address string into its binary form */

#ifdef CONFIG_NET_IPv6
  ret = inet_pton(AF_INET6, address, &inaddr);
#else
  ret = inet_pton(AF_INET, address, &inaddr);
#endif
  if (ret != 1)
    {
      nsh_freefullpath(lpath);
      return ERROR;
    }

  /* Place all of the NFS arguements into the nfs_args structure */

  memset(&data, 0, sizeof(data));

#ifdef CONFIG_NET_IPv6
  sin                  = (FAR struct sockaddr_in6 *)&data.addr;
  sin->sin_family      = AF_INET6;
  sin->sin_port        = htons(NFS_PMAPPORT);
  memcpy(&sin->sin6_addr, &inaddr, sizeof(struct in6_addr));
  data.addrlen         = sizeof(struct sockaddr_in6);
#else
  sin                  = (FAR struct sockaddr_in *)&data.addr;
  sin->sin_family      = AF_INET;
  sin->sin_port        = htons(NFS_PMAPPORT);
  sin->sin_addr        = inaddr;
  data.addrlen         = sizeof(struct sockaddr_in);
#endif

  data.sotype          = SOCK_DGRAM;
  data.path            = rpath;
  data.flags           = 0;       /* 0=Use all defaults */

  /* Perform the mount */

  ret = mount(NULL, lpath, "nfs", 0, (FAR void *)&data);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mount", NSH_ERRNO);
    }

  /* We no longer need the allocated mount point path */

  nsh_freefullpath(lpath);
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_umount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_READABLE)
#ifndef CONFIG_NSH_DISABLE_UMOUNT
int cmd_umount(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {
      /* Perform the umount */

      ret = umount(fullpath);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "umount", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif
