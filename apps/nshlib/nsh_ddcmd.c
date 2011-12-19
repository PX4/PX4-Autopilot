/****************************************************************************
 * apps/nshlib/nsh_ddcmd.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <sys/stat.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs.h>
#include "nsh.h"

#if CONFIG_NFILE_DESCRIPTORS > 0 && !defined(CONFIG_NSH_DISABLE_DD)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* If no sector size is specified wity BS=, then the following default value
 * is used.
 */

#define DEFAULT_SECTSIZE 512

/* At present, piping of input and output are not support, i.e., both of=
 * and if= arguments are required.
 */

#undef CAN_PIPE_FROM_STD

/* Function pointer calls are only need if block drivers are supported
 * (or, rather, if mount points are supported in the file system)
 */

#ifndef CONFIG_DISABLE_MOUNTPOINT
#  define DD_INFD         ((dd)->inf.fd)
#  define DD_INHANDLE     ((dd)->inf.handle)
#  define DD_OUTFD        ((dd)->outf.fd)
#  define DD_OUTHANDLE    ((dd)->outf.handle)
#  define DD_READ(dd)     ((dd)->infread(dd))
#  define DD_WRITE(dd)    ((dd)->outfwrite(dd))
#  define DD_INCLOSE(dd)  ((dd)->infclose(dd))
#  define DD_OUTCLOSE(dd) ((dd)->outfclose(dd))
#else
#  define DD_INFD         ((dd)->infd)
#  undef  DD_INHANDLE
#  define DD_OUTFD        ((dd)->outfd)
#  undef  DD_OUTHANDLE
#  define DD_READ(dd)     dd_readch(dd)
#  define DD_WRITE(dd)    dd_writech(dd)
#  define DD_INCLOSE(dd)  dd_infclosech(dd)
#  define DD_OUTCLOSE(dd) dd_outfclosech(dd)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dd_s
{
  FAR struct nsh_vtbl_s *vtbl;

#ifndef CONFIG_DISABLE_MOUNTPOINT
  union
  {
    FAR void *handle;  /* BCH lib handle for block device*/
    int fd;            /* File descriptor of the character device */
  } inf;
#else
  int infd;            /* File descriptor of the input device */ 
#endif

#ifndef CONFIG_DISABLE_MOUNTPOINT
  union
  {
    FAR void *handle;  /* BCH lib handle for block device*/
    int fd;            /* File descriptor of the character device */
  } outf;
#else
  int outfd;           /* File descriptor of the output device */ 
#endif

  uint32_t nsectors;   /* Number of sectors to transfer */
  uint32_t sector;     /* The current sector number */
  uint32_t skip;       /* The number of sectors skipped on input */
  bool     eof;        /* true:  The of the input or output file has been hit */
  uint16_t sectsize;   /* Size of one sector */
  uint16_t nbytes;     /* Number of valid bytes in the buffer */
  uint8_t *buffer;     /* Buffer of data to write to the output file */

  /* Function pointers to handle differences between block and character devices */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  int  (*infread)(struct dd_s *dd);
  void (*infclose)(struct dd_s *dd);
  int  (*outfwrite)(struct dd_s *dd);
  void (*outfclose)(struct dd_s *dd);
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_dd[] = "dd";

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dd_outfcloseblk
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static void dd_outfcloseblk(struct dd_s *dd)
{
  (void)bchlib_teardown(DD_OUTHANDLE);
}
#endif


/****************************************************************************
 * Name: dd_outfclosech
 ****************************************************************************/

static void dd_outfclosech(struct dd_s *dd)
{
  (void)close(DD_OUTFD);
}

/****************************************************************************
 * Name: dd_infcloseblk
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static void dd_infcloseblk(struct dd_s *dd)
{
  (void)bchlib_teardown(DD_INHANDLE);
}
#endif

/****************************************************************************
 * Name: dd_infclosech
 ****************************************************************************/

static void dd_infclosech(struct dd_s *dd)
{
  (void)close(DD_INFD);
}

/****************************************************************************
 * Name: dd_writeblk
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static int dd_writeblk(struct dd_s *dd)
{
  ssize_t nbytes;
  off_t   offset = (dd->sector - dd->skip) * dd->sectsize;

  /* Write the sector at the specified offset */

  nbytes = bchlib_write(DD_OUTHANDLE, (char*)dd->buffer, offset, dd->sectsize);
  if (nbytes < 0)
    {
      /* bchlib_write return -EFBIG on attempts to write past the end of
       * the device.
       */

      if (nbytes == -EFBIG)
        {
          dd->eof = true; /* Set end-of-file */
        }
      else
        {
          FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
          nsh_output(vtbl, g_fmtcmdfailed, g_dd, "bshlib_write", NSH_ERRNO_OF(-nbytes));
          return ERROR;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: dd_writech
 ****************************************************************************/

static int dd_writech(struct dd_s *dd)
{
  uint8_t *buffer = dd->buffer;
  uint16_t written ;
  ssize_t nbytes;

  /* Is the out buffer full (or is this the last one)? */

  written = 0;
  do
    {
      nbytes = write(DD_OUTFD, buffer, dd->sectsize - written);
      if (nbytes < 0)
        {
           FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
           nsh_output(vtbl, g_fmtcmdfailed, g_dd, "write", NSH_ERRNO_OF(-nbytes));
           return ERROR;
        }

      written += nbytes;
      buffer  += nbytes;
    }
  while (written < dd->sectsize);

  return OK;
}

/****************************************************************************
 * Name: dd_readblk
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static int dd_readblk(struct dd_s *dd)
{
  ssize_t nbytes;
  off_t   offset = dd->sector * dd->sectsize;

  nbytes = bchlib_read(DD_INHANDLE, (char*)dd->buffer, offset, dd->sectsize);
  if (nbytes < 0)
    {
      FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
      nsh_output(vtbl, g_fmtcmdfailed, g_dd, "bshlib_read", NSH_ERRNO_OF(-nbytes));
      return ERROR;
    }

  /* bchlib_read return 0 on attempts to write past the end of the device. */

  dd->nbytes = nbytes;
  dd->eof    = (nbytes == 0);
  return OK;
}
#endif

/****************************************************************************
 * Name: dd_readch
 ****************************************************************************/

static int dd_readch(struct dd_s *dd)
{
  uint8_t *buffer = dd->buffer;
  ssize_t nbytes;

  dd->nbytes = 0;
  do
    {
      nbytes = read(DD_INFD, buffer, dd->sectsize - dd->nbytes);
      if (nbytes < 0)
        {
           FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
           nsh_output(vtbl, g_fmtcmdfailed, g_dd, "read", NSH_ERRNO_OF(-nbytes));
           return ERROR;
        }

      dd->nbytes += nbytes;
      buffer     += nbytes;
    }
  while (dd->nbytes < dd->sectsize && nbytes > 0);

  dd->eof |= (dd->nbytes == 0);
  return OK;
}

/****************************************************************************
 * Name: dd_infopen
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static int dd_filetype(const char *filename)
{
  struct stat sb;
  int ret;

  /* Get the type of the file */

  ret = stat(filename, &sb);
  if (ret < 0)
    {
      return ERROR;  /* Return -1 on failure */
    }

  return S_ISBLK(sb.st_mode); /* Return true(1) if block, false(0) if char */
}
#endif

/****************************************************************************
 * Name: dd_infopen
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static inline int dd_infopen(const char *name, struct dd_s *dd)
{
  FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
  int ret;
  int type;

  /* Get the type of the input file */

  type = dd_filetype(name);
  if (type < 0)
    {
      nsh_output(vtbl, g_fmtcmdfailed, g_dd, "stat", NSH_ERRNO_OF(-type));
      return type;
    }

  /* Open the input file */

  if (!type)
    {
      DD_INFD = open(name, O_RDONLY);
      if (DD_INFD < 0)
        {
          nsh_output(vtbl, g_fmtcmdfailed, g_dd, "open", NSH_ERRNO);
          return ERROR;
        }

      dd->infread  = dd_readch;  /* Character oriented read */
      dd->infclose = dd_infclosech;
    }
  else
    {
      ret = bchlib_setup(name, true, &DD_INHANDLE);
      if (ret < 0)
        {
          return ERROR;
        }

      dd->infread  = dd_readblk;
      dd->infclose = dd_infcloseblk;
    }
  return OK;
}
#else
static inline int dd_infopen(const char *name, struct dd_s *dd)
{
  DD_INFD = open(name, O_RDONLY);
  if (DD_INFD < 0)
    {
      FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
      nsh_output(vtbl, g_fmtcmdfailed, g_dd, "open", NSH_ERRNO);
      return ERROR;
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: dd_outfopen
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static inline int dd_outfopen(const char *name, struct dd_s *dd)
{
  int type;
  int ret = OK;

  /* Get the type of the output file */

  type = dd_filetype(name);

  /* Open the block driver for input */

  if (type == true)
    {
      ret = bchlib_setup(name, true, &DD_OUTHANDLE);
      if (ret < 0)
        {
          return ERROR;
        }

      dd->outfwrite = dd_writeblk;  /* Block oriented write */
      dd->outfclose = dd_outfcloseblk;
    }

  /* Otherwise, the file is character oriented or does not exist */

  else
    {
      DD_OUTFD = open(name, O_WRONLY|O_CREAT|O_TRUNC, 0644);
      if (DD_OUTFD < 0)
        {
          FAR struct nsh_vtbl_s *vtbl = dd->vtbl;
          nsh_output(vtbl, g_fmtcmdfailed, g_dd, "open", NSH_ERRNO);
          return ERROR;
        }

      dd->outfwrite = dd_writech;  /* Character oriented write */
      dd->outfclose = dd_outfclosech;
    }
  return OK;
}
#else
static inline int dd_outfopen(const char *name, struct dd_s *dd)
{
  DD_OUTFD = open(name, O_WRONLY|O_CREAT|O_TRUNC, 0644);
  if (DD_OUTFD < 0)
    {
      nsh_output(dd->vtbl, g_fmtcmdfailed, g_dd, "open", NSH_ERRNO);
      return ERROR;
    }
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cmd_dd
 ****************************************************************************/

int cmd_dd(FAR struct nsh_vtbl_s *vtbl, int argc, char **argv)
{
  struct dd_s dd;
  char *infile = NULL;
  char *outfile = NULL;
  int ret = ERROR;
  int i;

  /* Initialize the dd structure */

  memset(&dd, 0, sizeof(struct dd_s));
  dd.vtbl      = vtbl;              /* For nsh_output */
  dd.sectsize  = DEFAULT_SECTSIZE;  /* Sector size if 'bs=' not provided */
  dd.nsectors  = 0xffffffff;        /* MAX_UINT32 */

  /* If no IF= option is provided on the command line, then read
   * from stdin.
   */

#ifdef CAN_PIPE_FROM_STD
  DD_INFD      = 0;       /* stdin */
#ifndef CONFIG_NSH_DISABLE_DD
  dd.infread   = readch;  /* Character oriented read */
  dd.infclose  = noclose; /* Don't close stdin */
#endif
#endif
  /* If no OF= option is provided on the command line, then write
   * to stdout.
   */

#ifdef CAN_PIPE_FROM_STD
  DD_OUTDF     = 1;       /* stdout */
#ifndef CONFIG_NSH_DISABLE_DD
  dd.outfwrite = writech; /* Character oriented write */
  dd.outfclose = noclose; /* Don't close stdout */
#endif
#endif

  /* Parse command line parameters */

  for (i = 1; i < argc; i++)
    {
      if (strncmp(argv[i], "if=", 3) == 0)
        {
          infile = nsh_getfullpath(vtbl, &argv[i][3]);
        }
      else if (strncmp(argv[i], "of=", 3) == 0)
        {
          outfile = nsh_getfullpath(vtbl, &argv[i][3]);
        }
      else if (strncmp(argv[i], "bs=", 3) == 0)
        {
          dd.sectsize = atoi(&argv[i][3]);
        }
      else if (strncmp(argv[i], "count=", 6) == 0)
        {
          dd.nsectors = atoi(&argv[i][6]);
        }
      else if (strncmp(argv[i], "skip=", 5) == 0)
        {
          dd.skip = atoi(&argv[i][5]);
        }
    }

#ifndef CAN_PIPE_FROM_STD
  if (!infile || !outfile)
    {
      nsh_output(vtbl, g_fmtargrequired, g_dd);
      goto errout_with_paths;
    }
#endif

  if (dd.skip < 0 || dd.skip > dd.nsectors)
    {
      nsh_output(vtbl, g_fmtarginvalid, g_dd);
      goto errout_with_paths;
    }

  /* Allocate the I/O buffer */

  dd.buffer = malloc(dd.sectsize);
  if (!dd.buffer)
    {
      nsh_output(vtbl, g_fmtcmdoutofmemory, g_dd);
      goto errout_with_paths;
    }

  /* Open the input file */

  ret = dd_infopen(infile, &dd);
  if (ret < 0)
    {
      goto errout_with_paths;
    }

  /* Open the output file */

  ret = dd_outfopen(outfile, &dd);
  if (ret < 0)
    {
      goto errout_with_inf;
    }

  /* Then perform the data transfer */

  dd.sector = 0;
  while (!dd.eof && dd.nsectors > 0)
    {
      /* Read one sector from from the input */

      ret = DD_READ(&dd);
      if (ret < 0)
        {
          goto errout_with_outf;
        }

      /* Has the incoming data stream ended? */

      if (!dd.eof)
        {
          /* Pad with zero if necessary (at the end of file only) */

          for (i = dd.nbytes; i < dd.sectsize; i++)
            {
              dd.buffer[i] = 0;
            }

          /* Write one sector to the output file */

          if (dd.sector >= dd.skip)
            {
              ret = DD_WRITE(&dd);
              if (ret < 0)
                {
                  goto errout_with_outf;
                }

              /* Decrement to show that a sector was written */

              dd.nsectors--;
            }

          /* Increment the sector number */

          dd.sector++;
        }
    }
  ret = OK;

errout_with_outf:
  DD_INCLOSE(&dd);
errout_with_inf:
  DD_OUTCLOSE(&dd);
  free(dd.buffer);
errout_with_paths:
  if (infile)
    {
      free(infile);
    }
  if (outfile)
    {
      free(outfile);
    }
  return ret;
}

#endif /* CONFIG_NFILE_DESCRIPTORS && !CONFIG_NSH_DISABLE_DD */

