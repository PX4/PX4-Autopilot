/****************************************************************************
 * apps/nshlib/nsh_fscmds.c
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
#include <stdint.h>
#include <stdbool.h>

#if CONFIG_NFILE_DESCRIPTORS > 0
# include <sys/stat.h>
# include <fcntl.h>
# if !defined(CONFIG_DISABLE_MOUNTPOINT)
#   ifdef CONFIG_FS_READABLE /* Need at least one filesytem in configuration */
#     include <sys/mount.h>
#     include <nuttx/ramdisk.h>
#   endif
#   ifdef CONFIG_FS_FAT
#     include <nuttx/fs/mkfatfs.h>
#   endif
#   ifdef CONFIG_NFS
#     include <sys/socket.h>
#     include <netinet/in.h>
#     include <nuttx/fs/nfs.h>
#   endif
#endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <dirent.h>
#include <limits.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define LSFLAGS_SIZE          1
#define LSFLAGS_LONG          2
#define LSFLAGS_RECURSIVE     4

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*direntry_handler_t)(FAR struct nsh_vtbl_s *, const char *,
                                  struct dirent *, void *);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Common buffer for file I/O.  Note the use of this common buffer precludes
 * multiple copies of NSH running concurrently.  It should be allocated per
 * NSH instance and retained in the "vtbl" as is done for the telnet
 * connection.
 */

static char g_iobuffer[IOBUFFERSIZE];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: trim_dir
 ****************************************************************************/

static void trim_dir(char *arg)
{
 /* Skip any trailing '/' characters (unless it is also the leading '/') */

 int len = strlen(arg) - 1;
 while (len > 0 && arg[len] == '/')
   {
      arg[len] = '\0';
      len--;
   }
}

/****************************************************************************
 * Name: nsh_getdirpath
 ****************************************************************************/

static char *nsh_getdirpath(const char *path, const char *file)
{
  /* Handle the case where all that is left is '/' */

  if (strcmp(path, "/") == 0)
    {
      sprintf(g_iobuffer, "/%s", file);
    }
  else
    {
      sprintf(g_iobuffer, "%s/%s", path, file);
    }

  g_iobuffer[PATH_MAX] = '\0';
  return strdup(g_iobuffer);
}

/****************************************************************************
 * Name: foreach_direntry
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int foreach_direntry(FAR struct nsh_vtbl_s *vtbl, const char *cmd, const char *dirpath,
                            direntry_handler_t handler, void *pvarg)
{
  DIR *dirp;
  int ret = OK;

  /* Trim trailing '/' from directory names */

#ifdef CONFIG_NSH_FULLPATH
  trim_dir(arg);
#endif

  /* Open the directory */

  dirp = opendir(dirpath);

  if (!dirp)
    {
      /* Failed to open the directory */

      nsh_output(vtbl, g_fmtnosuch, cmd, "directory", dirpath);
      return ERROR;
    }

  /* Read each directory entry */

  for (;;)
    {
      struct dirent *entryp = readdir(dirp);
      if (!entryp)
        {
          /* Finished with this directory */

          break;
        }

      /* Call the handler with this directory entry */

      if (handler(vtbl, dirpath, entryp, pvarg) <  0)
        {
          /* The handler reported a problem */

          ret = ERROR;
          break;
        }
    }

  closedir(dirp);
  return ret;
}
#endif

/****************************************************************************
 * Name: ls_specialdir
 ****************************************************************************/

static inline int ls_specialdir(const char *dir)
{
  /* '.' and '..' directories are not listed like normal directories */

  return (strcmp(dir, ".")  == 0 || strcmp(dir, "..") == 0);
}

/****************************************************************************
 * Name: ls_handler
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_handler(FAR struct nsh_vtbl_s *vtbl, const char *dirpath, struct dirent *entryp, void *pvarg)
{
  unsigned int lsflags = (unsigned int)pvarg;
  int ret;

  /* Check if any options will require that we stat the file */

  if ((lsflags & (LSFLAGS_SIZE|LSFLAGS_LONG)) != 0)
    {
      struct stat buf;
      char *fullpath = nsh_getdirpath(dirpath, entryp->d_name);

      /* Yes, stat the file */

      ret = stat(fullpath, &buf);
      free(fullpath);
      if (ret != 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, "ls", "stat", NSH_ERRNO);
          return ERROR;
        }

      if ((lsflags & LSFLAGS_LONG) != 0)
        {
          char details[] = "----------";
          if (S_ISDIR(buf.st_mode))
            {
              details[0]='d';
            }
          else if (S_ISCHR(buf.st_mode))
            {
              details[0]='c';
            }
          else if (S_ISBLK(buf.st_mode))
            {
              details[0]='b';
            }

          if ((buf.st_mode & S_IRUSR) != 0)
            {
              details[1]='r';
            }

          if ((buf.st_mode & S_IWUSR) != 0)
            {
              details[2]='w';
            }

          if ((buf.st_mode & S_IXUSR) != 0)
            {
              details[3]='x';
            }

          if ((buf.st_mode & S_IRGRP) != 0)
            {
              details[4]='r';
            }

          if ((buf.st_mode & S_IWGRP) != 0)
            {
              details[5]='w';
            }

          if ((buf.st_mode & S_IXGRP) != 0)
            {
              details[6]='x';
            }

          if ((buf.st_mode & S_IROTH) != 0)
            {
              details[7]='r';
            }

          if ((buf.st_mode & S_IWOTH) != 0)
            {
              details[8]='w';
            }

          if ((buf.st_mode & S_IXOTH) != 0)
            {
              details[9]='x';
            }

          nsh_output(vtbl, " %s", details);
        }

      if ((lsflags & LSFLAGS_SIZE) != 0)
        {
          nsh_output(vtbl, "%8d", buf.st_size);
        }
    }

  /* then provide the filename that is common to normal and verbose output */

#ifdef CONFIG_NSH_FULLPATH
  nsh_output(vtbl, " %s/%s", arg, entryp->d_name);
#else
  nsh_output(vtbl, " %s", entryp->d_name);
#endif

  if (DIRENT_ISDIRECTORY(entryp->d_type) && !ls_specialdir(entryp->d_name))
    {
      nsh_output(vtbl, "/\n");
    }
  else
    {
      nsh_output(vtbl, "\n");
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: ls_recursive
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
static int ls_recursive(FAR struct nsh_vtbl_s *vtbl, const char *dirpath,
                        struct dirent *entryp, void *pvarg)
{
  int ret = OK;

  /* Is this entry a directory (and not one of the special directories, . and ..)? */

  if (DIRENT_ISDIRECTORY(entryp->d_type) && !ls_specialdir(entryp->d_name))
    {
      /* Yes.. */

      char *newpath;
      newpath = nsh_getdirpath(dirpath, entryp->d_name);

      /* List the directory contents */

      nsh_output(vtbl, "%s:\n", newpath);

      /* Traverse the directory  */

      ret = foreach_direntry(vtbl, "ls", newpath, ls_handler, pvarg);
      if (ret == 0)
        {
          /* Then recurse to list each directory within the directory */

          ret = foreach_direntry(vtbl, "ls", newpath, ls_recursive, pvarg);
          free(newpath);
        }
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: cat_common
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_CAT
static int cat_common(FAR struct nsh_vtbl_s *vtbl, FAR const char *cmd,
                      FAR const char *filename)
{
  char buffer[IOBUFFERSIZE];
  int fd;
  int ret = OK;

  /* Open the file for reading */

  fd = open(filename, O_RDONLY);
  if (fd < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, cmd, "open", NSH_ERRNO);
      return ERROR;
    }

  /* And just dump it byte for byte into stdout */

  for (;;)
    {
      int nbytesread = read(fd, buffer, IOBUFFERSIZE);

      /* Check for read errors */

      if (nbytesread < 0)
        {
          int errval = errno;

          /* EINTR is not an error (but will stop stop the cat) */

#ifndef CONFIG_DISABLE_SIGNALS
          if (errval == EINTR)
            {
              nsh_output(vtbl, g_fmtsignalrecvd, cmd);
            }
          else
#endif
            {
              nsh_output(vtbl, g_fmtcmdfailed, cmd, "read", NSH_ERRNO_OF(errval));
            }

          ret = ERROR;
          break;
        }

      /* Check for data successfully read */

      else if (nbytesread > 0)
        {
          int nbyteswritten = 0;

          while (nbyteswritten < nbytesread)
            {
              ssize_t n = nsh_write(vtbl, buffer, nbytesread);
              if (n < 0)
                {
                  int errval = errno;

                  /* EINTR is not an error (but will stop stop the cat) */

 #ifndef CONFIG_DISABLE_SIGNALS
                  if (errval == EINTR)
                    {
                      nsh_output(vtbl, g_fmtsignalrecvd, cmd);
                    }
                  else
#endif
                    {
                      nsh_output(vtbl, g_fmtcmdfailed, cmd, "write", NSH_ERRNO);
                    }

                  ret = ERROR;
                  break;
                }
              else
                {
                  nbyteswritten += n;
                }
            }
        }

      /* Otherwise, it is the end of file */

      else
        {
          break;
        }
    }

   (void)close(fd);
   return ret;
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_cat
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_CAT
int cmd_cat(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath;
  int i;
  int ret = OK;

  /* Loop for each file name on the command line */

  for (i = 1; i < argc && ret == OK; i++)
    {
      /* Get the fullpath to the file */

      fullpath = nsh_getfullpath(vtbl, argv[i]);
      if (!fullpath)
        {
          ret = ERROR;
        }
      else
        {
          /* Dump the file to the console */

          ret = cat_common(vtbl, argv[0], fullpath);

          /* Free the allocated full path */

          nsh_freefullpath(fullpath);
        }
    }

  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_dmesg
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_SYSLOG) && !defined(CONFIG_NSH_DISABLE_DMESG)
int cmd_dmesg(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return cat_common(vtbl, argv[0], "/dev/syslog");
}
#endif

/****************************************************************************
 * Name: cmd_cp
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_CP
int cmd_cp(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct stat buf;
  char *srcpath  = NULL;
  char *destpath = NULL;
  char *allocpath = NULL;
  int oflags = O_WRONLY|O_CREAT|O_TRUNC;
  int rdfd;
  int wrfd;
  int ret = ERROR;

  /* Get the full path to the source file */

  srcpath = nsh_getfullpath(vtbl, argv[1]);
  if (!srcpath)
    {
      goto errout;
    }

  /* Open the source file for reading */

  rdfd = open(srcpath, O_RDONLY);
  if (rdfd < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "open", NSH_ERRNO);
      goto errout_with_srcpath;
    }

  /* Get the full path to the destination file or directory */

  destpath = nsh_getfullpath(vtbl, argv[2]);
  if (!destpath)
    {
      goto errout_with_rdfd;
    }

  /* Check if the destination is a directory */

  ret = stat(destpath, &buf);
  if (ret == 0)
    {
      /* Something exists here... is it a directory? */

      if (S_ISDIR(buf.st_mode))
        {
          /* Yes, it is a directory. Remove any trailing '/' characters from the path */

          trim_dir(argv[2]);

          /* Construct the full path to the new file */

          allocpath = nsh_getdirpath(argv[2], basename(argv[1]) );
          if (!allocpath)
            {
              nsh_output(vtbl, g_fmtcmdoutofmemory, argv[0]);
              goto errout_with_destpath;
            }

          /* Open then dest for writing */

          nsh_freefullpath(destpath);
          destpath = allocpath;
        }
      else if (!S_ISREG(buf.st_mode))
        {
          /* Maybe it is a driver? */

          oflags = O_WRONLY;
        }
    }

  /* Now open the destination */

  wrfd = open(destpath, oflags, 0666);
  if (wrfd < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "open", NSH_ERRNO);
      goto errout_with_allocpath;
    }

  /* Now copy the file */

  for (;;)
    {
      int nbytesread;
      int nbyteswritten;

      do
        {
          nbytesread = read(rdfd, g_iobuffer, IOBUFFERSIZE);
          if (nbytesread == 0)
            {
              /* End of file */

              ret = OK;
              goto errout_with_wrfd;
            }
          else if (nbytesread < 0)
            {
              /* EINTR is not an error (but will still stop the copy) */

#ifndef CONFIG_DISABLE_SIGNALS
              if (errno == EINTR)
                {
                  nsh_output(vtbl, g_fmtsignalrecvd, argv[0]);
                }
              else
#endif
                {
                  /* Read error */

                  nsh_output(vtbl, g_fmtcmdfailed, argv[0], "read", NSH_ERRNO);
                }
              goto errout_with_wrfd;
            }
        }
      while (nbytesread <= 0);

      do
        {
          nbyteswritten = write(wrfd, g_iobuffer, nbytesread);
          if (nbyteswritten >= 0)
            {
              nbytesread -= nbyteswritten;
            }
          else
            {
              /* EINTR is not an error (but will still stop the copy) */

#ifndef CONFIG_DISABLE_SIGNALS
              if (errno == EINTR)
                {
                  nsh_output(vtbl, g_fmtsignalrecvd, argv[0]);
                }
              else
#endif
                {
                 /* Read error */

                  nsh_output(vtbl, g_fmtcmdfailed, argv[0], "write", NSH_ERRNO);
                }
              goto errout_with_wrfd;
            }
        }
      while (nbytesread > 0);
    }

errout_with_wrfd:
  close(wrfd);

errout_with_allocpath:
  if (allocpath)
    {
      free(allocpath);
    }

errout_with_destpath:
  if (destpath && !allocpath)
    {
      nsh_freefullpath(destpath);
    }

errout_with_rdfd:
  close(rdfd);

errout_with_srcpath:
  if (srcpath)
    {
      nsh_freefullpath(srcpath);
    }
errout:
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_losetup
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_DISABLE_MOUNTPOINT)
#ifndef CONFIG_NSH_DISABLE_LOSETUP
int cmd_losetup(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *loopdev  = NULL;
  char *filepath = NULL;
  bool  teardown = false;
  bool  readonly = false;
  off_t offset   = 0;
  bool  badarg   = false;
  int   ret      = ERROR;
  int   option;

  /* Get the losetup options:  Two forms are supported:
   *
   *   losetup -d <loop-device>
   *   losetup [-o <offset>] [-r] <loop-device> <filename>
   *
   * NOTE that the -o and -r options are accepted with the -d option, but
   * will be ignored.
   */

  while ((option = getopt(argc, argv, "d:o:r")) != ERROR)
    {
      switch (option)
        {
        case 'd':
          loopdev  = nsh_getfullpath(vtbl, optarg);
          teardown = true;
          break;

        case 'o':
          offset = atoi(optarg);
          break;

        case 'r':
          readonly = true;
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
      goto errout_with_paths;
    }

  /* If this is not a tear down operation, then additional command line
   * parameters are required.
   */

  if (!teardown)
    {
      /* There must be two arguments on the command line after the options */

      if (optind + 1 < argc)
        {
          loopdev = nsh_getfullpath(vtbl, argv[optind]);
          optind++;

          filepath = nsh_getfullpath(vtbl, argv[optind]);
          optind++;
        }
      else
        {
          nsh_output(vtbl, g_fmtargrequired, argv[0]);
          goto errout_with_paths;
        }
    }

  /* There should be nothing else on the command line */

  if (optind < argc)
   {
     nsh_output(vtbl, g_fmttoomanyargs, argv[0]);
     goto errout_with_paths;
   }

  /* Perform the teardown operation */

  if (teardown)
    {
      /* Tear down the loop device. */

      ret = loteardown(loopdev);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "loteardown", NSH_ERRNO_OF(-ret));
          goto errout_with_paths;
        }
    }
  else
    {
      /* Set up the loop device */

      ret = losetup(loopdev, filepath, 512, offset, readonly);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "losetup", NSH_ERRNO_OF(-ret));
          goto errout_with_paths;
        }
    }

  /* Free memory */

errout_with_paths:
  if (loopdev)
    {
      free(loopdev);
    }

  if (filepath)
    {
      free(filepath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_ls
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_LS
int cmd_ls(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  const char *relpath;
  unsigned int lsflags = 0;
  char *fullpath;
  bool badarg = false;
  int ret;

  /* Get the ls options */

  int option;
  while ((option = getopt(argc, argv, "lRs")) != ERROR)
    {
      switch (option)
        {
          case 'l':
            lsflags |= (LSFLAGS_SIZE|LSFLAGS_LONG);
            break;

          case 'R':
            lsflags |= LSFLAGS_RECURSIVE;
            break;

          case 's':
            lsflags |= LSFLAGS_SIZE;
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

  /* There may be one argument after the options */

  if (optind + 1 < argc)
    {
      nsh_output(vtbl, g_fmttoomanyargs, argv[0]);
      return ERROR;
    }
  else if (optind >= argc)
    {
#ifndef CONFIG_DISABLE_ENVIRON
      relpath = nsh_getcwd();
#else
      nsh_output(vtbl, g_fmtargrequired, argv[0]);
      return ERROR;
#endif
    }
  else
    {
      relpath = argv[optind];
    }

  /* Get the fullpath to the directory */

  fullpath = nsh_getfullpath(vtbl, relpath);
  if (!fullpath)
    {
      return ERROR;
    }

  /* List the directory contents */

  nsh_output(vtbl, "%s:\n", fullpath);
  ret = foreach_direntry(vtbl, "ls", fullpath, ls_handler, (void*)lsflags);
  if (ret == OK && (lsflags & LSFLAGS_RECURSIVE) != 0)
    {
      /* Then recurse to list each directory within the directory */

      ret = foreach_direntry(vtbl, "ls", fullpath, ls_recursive, (void*)lsflags);
    }
  nsh_freefullpath(fullpath);
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_mkdir
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
#ifndef CONFIG_NSH_DISABLE_MKDIR
int cmd_mkdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {
      ret = mkdir(fullpath, 0777);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mkdir", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_mkfatfs
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_FAT)
#ifndef CONFIG_NSH_DISABLE_MKFATFS
int cmd_mkfatfs(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct fat_format_s fmt = FAT_FORMAT_INITIALIZER;
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {  
      ret = mkfatfs(fullpath, &fmt);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mkfatfs", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_mkfifo
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifndef CONFIG_NSH_DISABLE_MKFIFO
int cmd_mkfifo(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {
      ret = mkfifo(fullpath, 0777);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "mkfifo", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_mkrd
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
#ifndef CONFIG_NSH_DISABLE_MKRD
int cmd_mkrd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  const char *fmt;
  uint8_t *buffer;
  uint32_t nsectors;
  bool badarg = false;
  int sectsize = 512;
  int minor = 0;
  int ret;

  /* Get the mount options */

  int option;
  while ((option = getopt(argc, argv, ":m:s:")) != ERROR)
    {
      switch (option)
        {
          case 'm':
            minor = atoi(optarg);
            if (minor < 0 || minor > 255)
              {
                nsh_output(vtbl, g_fmtargrange, argv[0]);
                badarg = true;
              }
            break;

          case 's':
            sectsize = atoi(optarg);
            if (minor < 0 || minor > 16384)
              {
                nsh_output(vtbl, g_fmtargrange, argv[0]);
                badarg = true;
              }
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

  /* There should be exactly on parameter left on the command-line */

  if (optind == argc-1)
    {
      nsectors = (uint32_t)atoi(argv[optind]);
    }
  else if (optind >= argc)
    {
      fmt = g_fmttoomanyargs;
      goto errout_with_fmt;
    }
  else
    {
      fmt = g_fmtargrequired;
      goto errout_with_fmt;
    }

  /* Allocate the memory backing up the ramdisk */

  buffer = (uint8_t*)malloc(sectsize * nsectors);
  if (!buffer)
    {
      fmt = g_fmtcmdoutofmemory;
      goto errout_with_fmt;
    }

#ifdef CONFIG_DEBUG_VERBOSE
  memset(buffer, 0, sectsize * nsectors);
#endif
  dbg("RAMDISK at %p\n", buffer);

  /* Then register the ramdisk */

  ret = ramdisk_register(minor, buffer, nsectors, sectsize, true);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "ramdisk_register", NSH_ERRNO_OF(-ret));
      free(buffer);
      return ERROR;
    }
  return ret;

errout_with_fmt:
  nsh_output(vtbl, fmt, argv[0]);
  return ERROR;
}
#endif
#endif

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
 * Name: cmd_mv
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
#ifndef CONFIG_NSH_DISABLE_MV
int cmd_mv(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *oldpath;
  char *newpath;
  int ret;

  /* Get the full path to the old and new file paths */

  oldpath = nsh_getfullpath(vtbl, argv[1]);
  if (!oldpath)
    {
      return ERROR;
    }

  newpath = nsh_getfullpath(vtbl, argv[2]);
  if (!newpath)
    {
      nsh_freefullpath(newpath);
      return ERROR;
    }

  /* Perform the mount */

  ret = rename(oldpath, newpath);
  if (ret < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, argv[0], "rename", NSH_ERRNO);
    }

  /* Free the file paths */

  nsh_freefullpath(oldpath);
  nsh_freefullpath(newpath);
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_nfsmount
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && \
    defined(CONFIG_FS_READABLE) && defined(CONFIG_NET) && defined(CONFIG_NFS)
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
 * Name: cmd_rm
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
#ifndef CONFIG_NSH_DISABLE_RM
int cmd_rm(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {
      ret = unlink(fullpath);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "unlink", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: cmd_rmdir
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0 && defined(CONFIG_FS_WRITABLE)
#ifndef CONFIG_NSH_DISABLE_RMDIR
int cmd_rmdir(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  char *fullpath = nsh_getfullpath(vtbl, argv[1]);
  int ret = ERROR;

  if (fullpath)
    {
      ret = rmdir(fullpath);
      if (ret < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, argv[0], "rmdir", NSH_ERRNO);
        }
      nsh_freefullpath(fullpath);
    }
  return ret;
}
#endif
#endif

/****************************************************************************
 * Name: nsh_script
 ****************************************************************************/

#if  CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !defined(CONFIG_NSH_DISABLESCRIPT)
int nsh_script(FAR struct nsh_vtbl_s *vtbl, const char *cmd, const char *path)
{
  char *fullpath;
  FILE *stream;
  char *buffer;
  char *pret;
  int ret = ERROR;

  /* The path to the script may be relative to the current working directory */

  fullpath = nsh_getfullpath(vtbl, path);
  if (!fullpath)
    {
      return ERROR;
    }

  /* Get a reference to the common input buffer */

  buffer = nsh_linebuffer(vtbl);
  if (buffer)
    {
      /* Open the file containing the script */

      stream = fopen(fullpath, "r");
      if (!stream)
        {
          nsh_output(vtbl, g_fmtcmdfailed, cmd, "fopen", NSH_ERRNO);
          nsh_freefullpath(fullpath);
          return ERROR;
        }

      /* Loop, processing each command line in the script file (or
       * until an error occurs)
       */

      do
        {
          /* Get the next line of input from the file */

          fflush(stdout);
          pret = fgets(buffer, CONFIG_NSH_LINELEN, stream);
          if (pret)
            {
              /* Parse process the command.  NOTE:  this is recursive...
               * we got to cmd_sh via a call to nsh_parse.  So some
               * considerable amount of stack may be used.
               */

              ret = nsh_parse(vtbl, buffer);
            }
        }
      while (pret && ret == OK);
      fclose(stream);
    }

  nsh_freefullpath(fullpath);
  return ret;
}
#endif

/****************************************************************************
 * Name: cmd_sh
 ****************************************************************************/

#if  CONFIG_NFILE_DESCRIPTORS > 0 && CONFIG_NFILE_STREAMS > 0 && !defined(CONFIG_NSH_DISABLESCRIPT)
#ifndef CONFIG_NSH_DISABLE_SH
int cmd_sh(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  return nsh_script(vtbl, argv[0], argv[1]);
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
