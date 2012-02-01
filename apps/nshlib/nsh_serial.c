/****************************************************************************
 * apps/nshlib/nsh_serial.c
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <apps/readline.h>

#include "nsh.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Are we using the NuttX console for I/O?  Or some other character device? */

#ifdef CONFIG_NSH_CONDEV
#  define INFD(p)      ((p)->ss_confd)
#  define INSTREAM(p)  ((p)->ss_constream)
#  define OUTFD(p)     ((p)->ss_confd)
#  define OUTSTREAM(p) ((p)->ss_constream)
#else
#  define INFD(p)      0
#  define INSTREAM(p)  stdin
#  define OUTFD(p)     1
#  define OUTSTREAM(p) stdout
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct serial_s
{
  /* NSH front-end call table */

  struct nsh_vtbl_s ss_vtbl;

  /* NSH input/output streams */

#ifdef CONFIG_NSH_CONDEV
  int    ss_confd;     /* Console I/O file descriptor */
#endif
  int    ss_outfd;     /* Output file descriptor (possibly redirected) */
#ifdef CONFIG_NSH_CONDEV
  FILE  *ss_constream; /* Console I/O stream (possibly redirected) */
#endif
  FILE  *ss_outstream; /* Output stream */

  /* Line input buffer */

  char   ss_line[CONFIG_NSH_LINELEN];
};

struct serialsave_s
{
  int    ss_outfd;     /* Re-directed output file descriptor */
  FILE  *ss_outstream; /* Re-directed output stream */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static FAR struct nsh_vtbl_s *nsh_consoleclone(FAR struct nsh_vtbl_s *vtbl);
static void nsh_consolerelease(FAR struct nsh_vtbl_s *vtbl);
#endif
static ssize_t nsh_consolewrite(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes);
static int nsh_consoleoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);
static FAR char *nsh_consolelinebuffer(FAR struct nsh_vtbl_s *vtbl);
static void nsh_consoleredirect(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save);
static void nsh_consoleundirect(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save);
static void nsh_consoleexit(FAR struct nsh_vtbl_s *vtbl);

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
 * Name: nsh_allocstruct
 ****************************************************************************/

static inline FAR struct serial_s *nsh_allocstruct(void)
{
  struct serial_s *pstate = (struct serial_s *)zalloc(sizeof(struct serial_s));
  if (pstate)
    {
      /* Initialize the call table */

#ifndef CONFIG_NSH_DISABLEBG
      pstate->ss_vtbl.clone      = nsh_consoleclone;
      pstate->ss_vtbl.release    = nsh_consolerelease;
#endif
      pstate->ss_vtbl.write      = nsh_consolewrite;
      pstate->ss_vtbl.output     = nsh_consoleoutput;
      pstate->ss_vtbl.linebuffer = nsh_consolelinebuffer;
      pstate->ss_vtbl.redirect   = nsh_consoleredirect;
      pstate->ss_vtbl.undirect   = nsh_consoleundirect;
      pstate->ss_vtbl.exit       = nsh_consoleexit;

      /* (Re-) open the console input device */

#ifdef CONFIG_NSH_CONDEV
      pstate->ss_confd           = open(CONFIG_NSH_CONDEV, O_RDWR);
      if (pstate->ss_confd < 0)
        {
          free(pstate);
          return NULL;
        }

      /* Create a standard C stream on the console device */

      pstate->ss_constream = fdopen(pstate->ss_confd, "r+");
      if (!pstate->ss_constream)
        {
          close(pstate->ss_confd);
          free(pstate);
          return NULL;
        }
#endif

      /* Initialize the output stream */

      pstate->ss_outfd           = OUTFD(pstate);
      pstate->ss_outstream       = OUTSTREAM(pstate);
    }
  return pstate;
}

/****************************************************************************
 * Name: nsh_openifnotopen
 ****************************************************************************/

static int nsh_openifnotopen(struct serial_s *pstate)
{
  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.
   */

  if (!pstate->ss_outstream)
    {
      pstate->ss_outstream = fdopen(pstate->ss_outfd, "w");
      if (!pstate->ss_outstream)
        {
          return ERROR;
        }
    }
  return 0;
}

/****************************************************************************
 * Name: nsh_closeifnotclosed
 *
 * Description:
 *   Close the output stream if it is not the standard output stream.
 *
 ****************************************************************************/

static void nsh_closeifnotclosed(struct serial_s *pstate)
{
  if (pstate->ss_outstream == OUTSTREAM(pstate))
    {
      fflush(OUTSTREAM(pstate));
      pstate->ss_outfd = OUTFD(pstate);
    }
  else
    {
      if (pstate->ss_outstream)
        {
          fflush(pstate->ss_outstream);
          fclose(pstate->ss_outstream);
        }
      else if (pstate->ss_outfd >= 0 && pstate->ss_outfd != OUTFD(pstate))
        {
          close(pstate->ss_outfd);
        }

      pstate->ss_outfd     = -1;
      pstate->ss_outstream = NULL;
    }
}

/****************************************************************************
 * Name: nsh_consolewrite
 *
 * Description:
 *   write a buffer to the remote shell window.
 *
 *   Currently only used by cat.
 *
 ****************************************************************************/

static ssize_t nsh_consolewrite(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  ssize_t ret;

  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.  The
   * actual open will then occur with the first output from the new task.
   */

  if (nsh_openifnotopen(pstate) != 0)
   {
     return (ssize_t)ERROR;
   }

  /* Write the data to the output stream */

  ret = fwrite(buffer, 1, nbytes, pstate->ss_outstream);
  if (ret < 0)
    {
      dbg("[%d] Failed to send buffer: %d\n", pstate->ss_outfd, errno);
    }
  return ret;
}

/****************************************************************************
 * Name: nsh_consoleoutput
 *
 * Description:
 *   Print a string to the currently selected stream.
 *
 ****************************************************************************/

static int nsh_consoleoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  va_list ap;
  int     ret;

  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.  The
   * actual open will then occur with the first output from the new task.
   */

  if (nsh_openifnotopen(pstate) != 0)
   {
     return ERROR;
   }
 
  va_start(ap, fmt);
  ret = vfprintf(pstate->ss_outstream, fmt, ap);
  va_end(ap);
 
  return ret;
}

/****************************************************************************
 * Name: nsh_consolelinebuffer
 *
 * Description:
 *   Return a reference to the current line buffer
 *
 ****************************************************************************/

static FAR char *nsh_consolelinebuffer(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  return pstate->ss_line;
}

/****************************************************************************
 * Name: nsh_consoleclone
 *
 * Description:
 *   Make an independent copy of the vtbl
 *
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static FAR struct nsh_vtbl_s *nsh_consoleclone(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct serial_s *pclone = nsh_allocstruct();
  return &pclone->ss_vtbl;
}
#endif

/****************************************************************************
 * Name: nsh_consolerelease
 *
 * Description:
 *   Release the cloned instance
 *
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static void nsh_consolerelease(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;

  /* Close the output stream */

  nsh_closeifnotclosed(pstate);

  /* Close the console stream */

#ifdef CONFIG_NSH_CONDEV
  (void)fclose(pstate->ss_constream);
#endif

  /* Then release the vtable container */

  free(pstate);
}
#endif

/****************************************************************************
 * Name: nsh_consoleredirect
 *
 * Description:
 *   Set up for redirected output.  This function is called from nsh_parse()
 *   in two different contexts:
 *
 *   1) Redirected background commands of the form:  command > xyz.text &
 *
 *      In this case:
 *      - vtbl: A newly allocated and initialized instance created by
 *        nsh_consoleclone,
 *      - fd:- The file descriptor of the redirected output
 *      - save: NULL
 *
 *      nsh_consolerelease() will perform the clean-up when the clone is
 *      destroyed.
 *        
 *   2) Redirected foreground commands of the form:  command > xyz.txt
 *
 *      In this case:
 *      - vtbl: The current state structure,
 *      - fd: The file descriptor of the redirected output
 *      - save: Where to save the re-directed registers.
 *
 *      nsh_consoleundirect() will perform the clean-up after the redirected
 *      command completes.
 *
 ****************************************************************************/

static void nsh_consoleredirect(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save)
{
  FAR struct serial_s     *pstate = (FAR struct serial_s *)vtbl;
  FAR struct serialsave_s *ssave  = (FAR struct serialsave_s *)save;

  /* Case 1: Redirected foreground commands */

  if (ssave)
    {
      /* pstate->ss_outstream and ss_outfd refer refer to the
       * currently opened output stream.  If the output stream is open, flush
       * any pending output.
       */

      if (pstate->ss_outstream)
        {
          fflush(pstate->ss_outstream);          
        }

      /* Save the current fd and stream values.  These will be restored
       * when nsh_consoleundirect() is called.
       */

      ssave->ss_outfd     = pstate->ss_outfd;
      ssave->ss_outstream = pstate->ss_outstream;
    }
  else
    {
      /* nsh_consoleclone() set pstate->ss_outfd and ss_outstream to refer
       * to standard out.  We just want to leave these alone and overwrite
       * them with the fd for the re-directed stream.
       */
    }

  /* In either case, set the fd of the new, re-directed output and nullify
   * the output stream (it will be fdopen'ed if it is used).
   */

  pstate->ss_outfd     = fd;
  pstate->ss_outstream = NULL;
}

/****************************************************************************
 * Name: nsh_consoleundirect
 *
 * Description:
 *   Set up for redirected output
 *
 ****************************************************************************/

static void nsh_consoleundirect(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save)
{
  FAR struct serial_s *pstate = (FAR struct serial_s *)vtbl;
  FAR struct serialsave_s *ssave  = (FAR struct serialsave_s *)save;

  nsh_closeifnotclosed(pstate);
  pstate->ss_outfd     = ssave->ss_outfd;
  pstate->ss_outstream = ssave->ss_outstream;
}

/****************************************************************************
 * Name: nsh_consoleexit
 *
 * Description:
 *   Exit the shell task
 *
 ****************************************************************************/

static void nsh_consoleexit(FAR struct nsh_vtbl_s *vtbl)
{
  exit(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_consolemain
 ****************************************************************************/

int nsh_consolemain(int argc, char *argv[])
{
  FAR struct serial_s *pstate = nsh_allocstruct();
  DEBUGASSERT(pstate);
  int ret;

  /* If we are using a USB console, then we will have to wait for the USB to
   * be connected/
   */

#ifdef HAVE_USB_CONSOLE
  DEBUGASSERT(nsh_usbconsole() == OK);
#endif

  /* Present a greeting */

  fputs(g_nshgreeting, pstate->ss_outstream);
  fflush(pstate->ss_outstream);

  /* Execute the startup script */

#ifdef CONFIG_NSH_ROMFSETC
  (void)nsh_script(&pstate->ss_vtbl, "init", NSH_INITPATH);
#endif

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* Display the prompt string */

      fputs(g_nshprompt, pstate->ss_outstream);
      fflush(pstate->ss_outstream);

      /* Get the next line of input */

      ret = readline(pstate->ss_line, CONFIG_NSH_LINELEN,
                     INSTREAM(pstate), OUTSTREAM(pstate));
      if (ret > 0)
        {
          /* Parse process the command */

          (void)nsh_parse(&pstate->ss_vtbl, pstate->ss_line);
          fflush(pstate->ss_outstream);
        }

      /* Readline normally returns the number of characters read,
       * but will return 0 on end of file or a negative value
       * if an error occurs.  Either will cause the session to
       * terminate.
       */

      else
        {
          fprintf(pstate->ss_outstream, g_fmtcmdfailed, "readline", NSH_ERRNO);
          return 1;
        }
    }

  return OK;
}
