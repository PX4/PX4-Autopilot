/****************************************************************************
 * examples/mount/mount_main.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>

#include "mount.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define TEST_USE_STAT         1
#define TEST_SHOW_DIRECTORIES 1
#define TEST_USE_STATFS       1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_mntdir[]         = "/mnt";
static const char g_target[]         = "/mnt/fs";
static const char g_filesystemtype[] = "vfat";

static const char g_testdir1[]       = "/mnt/fs/TestDir";
static const char g_testdir2[]       = "/mnt/fs/NewDir1";
static const char g_testdir3[]       = "/mnt/fs/NewDir2";
static const char g_testdir4[]       = "/mnt/fs/NewDir3";
#ifdef CONFIG_EXAMPLES_MOUNT_DEVNAME
static const char g_testfile1[]      = "/mnt/fs/TestDir/TestFile.txt";
#endif
static const char g_testfile2[]      = "/mnt/fs/TestDir/WrTest1.txt";
static const char g_testfile3[]      = "/mnt/fs/NewDir1/WrTest2.txt";
static const char g_testfile4[]      = "/mnt/fs/NewDir3/Renamed.txt";
static const char g_testmsg[]        = "This is a write test";

static int        g_nerrors          = 0;

static char       g_namebuffer[256];

/****************************************************************************
 * Public Data
 ****************************************************************************/

       const char g_source[]         = MOUNT_DEVNAME;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef TEST_USE_STAT
static void show_stat(const char *path, struct stat *ps)
{
  printf("%s stat:\n", path);
  printf("\tmode        : %08x\n", ps->st_mode);
  if (S_ISREG(ps->st_mode))
    {
      printf("\ttype        : File\n");
    }
  else if (S_ISDIR(ps->st_mode))
    {
      printf("\ttype        : Directory\n");
    }
  else if (S_ISCHR(ps->st_mode))
    {
      printf("\ttype        : Character driver\n");
    }
  else if (S_ISBLK(ps->st_mode))
    {
      printf("\ttype        : Block driver\n");
    }
  else
    {
      printf("\ttype        : Unknown\n");
    }

  printf("\tsize        : %d (bytes)\n",  ps->st_size);
  printf("\tblock size  : %d (bytes)\n",  ps->st_blksize);
  printf("\tsize        : %d (blocks)\n", ps->st_blocks);
  printf("\taccess time : %d\n", ps->st_atime);
  printf("\tmodify time : %d\n", ps->st_mtime);
  printf("\tchange time : %d\n", ps->st_ctime);
}
#endif

/****************************************************************************
 * Name: show_statfs
 ****************************************************************************/

#ifdef TEST_USE_STATFS
static void show_statfs(const char *path)
{
  struct statfs buf;
  int ret;

  /* Try stat() against a file or directory.  It should fail with expectederror */

  printf("show_statfs: Try statfs(%s)\n", path);
  ret = statfs(path, &buf);
  if (ret == 0)
    {
      printf("show_statfs: statfs(%s) succeeded\n", path);
      printf("\tFS Type           : %0x\n", buf.f_type);
      printf("\tBlock size        : %d\n", buf.f_bsize);
      printf("\tNumber of blocks  : %d\n", buf.f_blocks);
      printf("\tFree blocks       : %d\n", buf.f_bfree);
      printf("\tFree user blocks  : %d\n", buf.f_bavail);
      printf("\tNumber file nodes : %d\n", buf.f_files);
      printf("\tFree file nodes   : %d\n", buf.f_ffree);
      printf("\tFile name length  : %d\n", buf.f_namelen);
    }
  else
    {
      printf("show_statfs: ERROR statfs(%s) failed with errno=%d\n",
             path, errno);
      g_nerrors++;
    }
}
#else
# define show_statfs(p)
#endif

/****************************************************************************
 * Name: show_directories
 ****************************************************************************/

#ifdef TEST_SHOW_DIRECTORIES
static void show_directories(const char *path, int indent)
{
  DIR *dirp;
  struct dirent *direntry;
  int i;

  dirp = opendir(path);
  if ( !dirp )
    {
      printf("show_directories: ERROR opendir(\"%s\") failed with errno=%d\n",
             path, errno);
      g_nerrors++;
      return;
    }

  for (direntry = readdir(dirp); direntry; direntry = readdir(dirp))
    {
      for (i = 0; i < 2*indent; i++)
        {
          putchar(' ');
        }
      if (DIRENT_ISDIRECTORY(direntry->d_type))
        {
          char *subdir;
          printf("%s/\n", direntry->d_name);
          sprintf(g_namebuffer, "%s/%s", path, direntry->d_name);
          subdir = strdup(g_namebuffer);
          show_directories( subdir, indent + 1);
          free(subdir);
        }
      else
        {
          printf("%s\n", direntry->d_name);
        }
    }

  closedir(dirp);
}
#else
# define show_directories(p,i)
#endif

/****************************************************************************
 * Name: fail_read_open
 ****************************************************************************/
#ifdef CONFIG_EXAMPLES_MOUNT_DEVNAME
static void fail_read_open(const char *path, int expectederror)
{
  int fd;

  printf("fail_read_open: Try open(%s) for reading\n", path);

  fd = open(path, O_RDONLY);
  if (fd >= 0)
    {
      printf("fail_read_open: ERROR open(%s) succeeded\n", path);
      g_nerrors++;
      close(fd);
    }
  else if (errno != expectederror)
    {
      printf("fail_read_open: ERROR open(%s) failed with errno=%d (expected %d)\n",
             path, errno, expectederror);
      g_nerrors++;
    }
}
#endif

/****************************************************************************
 * Name: read_test_file
 ****************************************************************************/

static void read_test_file(const char *path)
{
  char buffer[128];
  int  nbytes;
  int  fd;

  /* Read a test file that is already on the test file system image */

  printf("read_test_file: opening %s for reading\n", path);

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("read_test_file: ERROR failed to open %s, errno=%d\n",
             path, errno);
      g_nerrors++;
    }
  else
    {
      memset(buffer, 0, 128);
      nbytes = read(fd, buffer, 128);
      if (nbytes < 0)
        {
          printf("read_test_file: ERROR failed to read from %s, errno=%d\n",
                 path, errno);
          g_nerrors++;
        }
      else
        {
          buffer[127]='\0';
          printf("read_test_file: Read \"%s\" from %s\n", buffer, path);
        }
      close(fd);
    }
}

/****************************************************************************
 * Name: write_test_file
 ****************************************************************************/

static void write_test_file(const char *path)
{
  int fd;

  /* Write a test file into a pre-existing file on the test file system */

  printf("write_test_file: opening %s for writing\n", path);

  fd = open(path, O_WRONLY|O_CREAT|O_TRUNC, 0644);
  if (fd < 0)
    {
      printf("write_test_file: ERROR failed to open %s for writing, errno=%d\n",
             path, errno);
      g_nerrors++;
    }
  else
    {
      int nbytes = write(fd, g_testmsg, strlen(g_testmsg));
      if (nbytes < 0)
        {
          printf("write_test_file: ERROR failed to write to %s, errno=%d\n",
                 path, errno);
          g_nerrors++;
        }
      else
        {
          printf("write_test_file: wrote %d bytes to %s\n", nbytes, path);
        }
      close(fd);
    }
}

/****************************************************************************
 * Name: fail_mkdir
 ****************************************************************************/

static void fail_mkdir(const char *path, int expectederror)
{
  int ret;

  /* Try mkdir() against a file or directory.  It should fail with expectederror */

  printf("fail_mkdir: Try mkdir(%s)\n", path);

  ret = mkdir(path, 0666);
  if (ret == 0)
    {
      printf("fail_mkdir: ERROR mkdir(%s) succeeded\n", path);
      g_nerrors++;
    }
  else if (errno != expectederror)
    {
      printf("fail_mkdir: ERROR mkdir(%s) failed with errno=%d (expected %d)\n",
             path, errno, expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_mkdir
 ****************************************************************************/

static void succeed_mkdir(const char *path)
{
  int ret;

  printf("succeed_mkdir: Try mkdir(%s)\n", path);

  ret = mkdir(path, 0666);
  if (ret != 0)
    {
      printf("succeed_mkdir: ERROR mkdir(%s) failed with errno=%d\n",
             path, errno);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: fail_rmdir
 ****************************************************************************/

static void fail_rmdir(const char *path, int expectederror)
{
  int ret;

  /* Try rmdir() against a file or directory.  It should fail with expectederror */

  printf("fail_rmdir: Try rmdir(%s)\n", path);

  ret = rmdir(path);
  if (ret == 0)
    {
      printf("fail_rmdir: ERROR rmdir(%s) succeeded\n", path);
      g_nerrors++;
    }
  else if (errno != expectederror)
    {
      printf("fail_rmdir: ERROR rmdir(%s) failed with errno=%d (expected %d)\n",
             path, errno, expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_rmdir
 ****************************************************************************/

static void succeed_rmdir(const char *path)
{
  int ret;

  printf("succeed_rmdir: Try rmdir(%s)\n", path);

  ret = rmdir(path);
  if (ret != 0)
    {
      printf("succeed_rmdir: ERROR rmdir(%s) failed with errno=%d\n",
             path, errno);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: fail_unlink
 ****************************************************************************/

static void fail_unlink(const char *path, int expectederror)
{
  int ret;

  /* Try unlink() against a file or directory.  It should fail with expectederror */

  printf("fail_unlink: Try unlink(%s)\n", path);

  ret = unlink(path);
  if (ret == 0)
    {
      printf("fail_unlink: ERROR unlink(%s) succeeded\n", path);
      g_nerrors++;
    }
  else if (errno != expectederror)
    {
      printf("fail_unlink: ERROR unlink(%s) failed with errno=%d (expected %d)\n",
             path, errno, expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_unlink
 ****************************************************************************/

static void succeed_unlink(const char *path)
{
  int ret;

  /* Try unlink() against the test file.  It should succeed. */

  printf("succeed_unlink: Try unlink(%s)\n", path);

  ret = unlink(path);
  if (ret != 0)
    {
      printf("succeed_unlink: ERROR unlink(%s) failed with errno=%d\n",
             path, errno);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: fail_rename
 ****************************************************************************/

static void fail_rename(const char *oldpath, const char *newpath, int expectederror)
{
  int ret;

  /* Try rename() against a file or directory.  It should fail with expectederror */

  printf("fail_rename: Try rename(%s->%s)\n", oldpath, newpath);

  ret = rename(oldpath, newpath);
  if (ret == 0)
    {
      printf("fail_rename: ERROR rename(%s->%s) succeeded\n",
             oldpath, newpath);
      g_nerrors++;
    }
  else if (errno != expectederror)
    {
      printf("fail_rename: ERROR rename(%s->%s) failed with errno=%d (expected %d)\n",
             oldpath, newpath, errno, expectederror);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: succeed_rename
 ****************************************************************************/

static void succeed_rename(const char *oldpath, const char *newpath)
{
  int ret;

  printf("succeed_rename: Try rename(%s->%s)\n", oldpath, newpath);

  ret = rename(oldpath, newpath);
  if (ret != 0)
    {
      printf("succeed_rename: ERROR rename(%s->%s) failed with errno=%d\n",
             oldpath, newpath, errno);
      g_nerrors++;
    }
}

/****************************************************************************
 * Name: fail_stat
 ****************************************************************************/

#ifdef TEST_USE_STAT
static void fail_stat(const char *path, int expectederror)
{
  struct stat buf;
  int ret;

  /* Try stat() against a file or directory.  It should fail with expectederror */

  printf("fail_stat: Try stat(%s)\n", path);

  ret = stat(path, &buf);
  if (ret == 0)
    {
      printf("fail_stat: ERROR stat(%s) succeeded\n", path);
      show_stat(path, &buf);
      g_nerrors++;
    }
  else if (errno != expectederror)
    {
      printf("fail_stat: ERROR stat(%s) failed with errno=%d (expected %d)\n",
             path, errno, expectederror);
      g_nerrors++;
    }
}
#else
# define fail_stat(p,e);
#endif

/****************************************************************************
 * Name: succeed_stat
 ****************************************************************************/

#ifdef TEST_USE_STAT
static void succeed_stat(const char *path)
{
  struct stat buf;
  int ret;

  printf("succeed_stat: Try stat(%s)\n", path);

  ret = stat(path, &buf);
  if (ret != 0)
    {
      printf("succeed_stat: ERROR stat(%s) failed with errno=%d\n",
             path, errno);
      g_nerrors++;
    }
  else
    {
      printf("succeed_stat: stat(%s) succeeded\n", path);
      show_stat(path, &buf);
    }
}
#else
#define succeed_stat(p)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mount_main
 ****************************************************************************/

int mount_main(int argc, char *argv[])
{
  int  ret;

#ifndef CONFIG_EXAMPLES_MOUNT_DEVNAME
  /* Create a RAM disk for the test */

  ret = create_ramdisk();
  if (ret < 0)
    {
      printf("mount_main: ERROR failed to create RAM disk\n");
      return 1;
    }
#endif

  /* Mount the test file system (see arch/sim/src/up_deviceimage.c */

  printf("mount_main: mounting %s filesystem at target=%s with source=%s\n",
         g_filesystemtype, g_target, g_source);

  ret = mount(g_source, g_target, g_filesystemtype, 0, NULL);
  printf("mount_main: mount() returned %d\n", ret);

  if (ret == 0)
    {
      show_statfs(g_mntdir);
      show_statfs(g_target);

#ifdef CONFIG_EXAMPLES_MOUNT_DEVNAME
      /* Read a test file that is already on the test file system image */

      show_directories("", 0);
      succeed_stat(g_testfile1);
      show_statfs(g_testfile1);
      read_test_file(g_testfile1);
#else
      /* Create the test directory that would have been on the canned filesystem */

      succeed_mkdir(g_testdir1);
      show_directories("", 0);
      succeed_stat(g_testdir1);
      show_statfs(g_testdir1);
#endif

      /* Write a test file into a pre-existing directory on the test file system */

      fail_stat(g_testfile2, ENOENT);
      write_test_file(g_testfile2);
      show_directories("", 0);
      succeed_stat(g_testfile2);
      show_statfs(g_testfile2);

      /* Read the file that we just wrote */

      read_test_file(g_testfile2);

      /* Try rmdir() against a file on the directory.  It should fail with ENOTDIR */
#ifdef CONFIG_EXAMPLES_MOUNT_DEVNAME
      fail_rmdir(g_testfile1, ENOTDIR);
#endif

      /* Try rmdir() against the test directory.  It should fail with ENOTEMPTY */

      fail_rmdir(g_testdir1, ENOTEMPTY);

      /* Try unlink() against the test directory.  It should fail with EISDIR */

      fail_unlink(g_testdir1, EISDIR);

      /* Try unlink() against the test file1.  It should succeed. */
#ifdef CONFIG_EXAMPLES_MOUNT_DEVNAME
      succeed_unlink(g_testfile1);
      fail_stat(g_testfile1, ENOENT);
      show_directories("", 0);
#endif

      /* Attempt to open testfile1 should fail with ENOENT */
#ifdef CONFIG_EXAMPLES_MOUNT_DEVNAME
      fail_read_open(g_testfile1, ENOENT);
#endif
      /* Try rmdir() against the test directory.  It should still fail with ENOTEMPTY */

      fail_rmdir(g_testdir1, ENOTEMPTY);

      /* Try mkdir() against the test file2.  It should fail with EEXIST. */

      fail_mkdir(g_testfile2, EEXIST);

      /* Try unlink() against the test file2.  It should succeed. */

      succeed_unlink(g_testfile2);
      show_directories("", 0);
      fail_stat(g_testfile2, ENOENT);

      /* Try mkdir() against the test dir1.  It should fail with EEXIST. */

      fail_mkdir(g_testdir1, EEXIST);

      /* Try rmdir() against the test directory.  mkdir should now succeed. */

      succeed_rmdir(g_testdir1);
      show_directories("", 0);
      fail_stat(g_testdir1, ENOENT);

      /* Try mkdir() against the test dir2.  It should succeed */

      succeed_mkdir(g_testdir2);
      show_directories("", 0);
      succeed_stat(g_testdir2);
      show_statfs(g_testdir2);

      /* Try mkdir() against the test dir2.  It should fail with EXIST */

      fail_mkdir(g_testdir2, EEXIST);

      /* Write a test file into a new directory on the test file system */

      fail_stat(g_testfile3, ENOENT);
      write_test_file(g_testfile3);
      show_directories("", 0);
      succeed_stat(g_testfile3);
      show_statfs(g_testfile3);

      /* Read the file that we just wrote */

      read_test_file(g_testfile3);

      /* Use mkdir() to create test dir3.  It should succeed */

      fail_stat(g_testdir3, ENOENT);
      succeed_mkdir(g_testdir3);
      show_directories("", 0);
      succeed_stat(g_testdir3);
      show_statfs(g_testdir3);

      /* Try rename() on the root directory. Should fail with EXDEV*/

      fail_rename(g_target, g_testdir4, EXDEV);

      /* Try rename() to an existing directory.  Should fail with EEXIST */

      fail_rename(g_testdir2, g_testdir3, EEXIST);

      /* Try rename() to a non-existing directory.  Should succeed */

      fail_stat(g_testdir4, ENOENT);
      succeed_rename(g_testdir3, g_testdir4);
      show_directories("", 0);
      fail_stat(g_testdir3, ENOENT);
      succeed_stat(g_testdir4);
      show_statfs(g_testdir4);

      /* Try rename() of file.  Should work. */

      fail_stat(g_testfile4, ENOENT);
      succeed_rename(g_testfile3, g_testfile4);
      show_directories("", 0);
      fail_stat(g_testfile3, ENOENT);
      succeed_stat(g_testfile4);
      show_statfs(g_testfile4);

      /* Make sure that we can still read the renamed file */

      read_test_file(g_testfile4);

      /* Unmount the file system */

      printf("mount_main: Try unmount(%s)\n", g_target);

      ret = umount(g_target);
      if (ret != 0)
        {
          printf("mount_main: ERROR umount() failed, errno %d\n", errno);
          g_nerrors++;
        }

      printf("mount_main: %d errors reported\n", g_nerrors);
    }

  fflush(stdout);
  return 0;
}
