/****************************************************************************
 * apps/nshlib/nsh_consolemain.c
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

#include <stdio.h>
#include <assert.h>

#include <apps/readline.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_consolemain
 *
 * Description:
 *   This interfaces maybe to called or started with task_start to start a
 *   single an NSH instance that operates on stdin and stdout (/dev/console).
 *   This function does not return.
 *
 * Input Parameters:
 *   Standard task start-up arguements.  These are not used.  argc may be
 *   zero and argv may be NULL.
 *
 * Returned Values:
 *   This function does not normally return.  exit() is usually called to
 *   terminate the NSH session.  This function will return in the event of
 *   an error.  In that case, a nonzero value is returned (1).
 *  
 ****************************************************************************/

int nsh_consolemain(int argc, char *argv[])
{
  FAR struct console_stdio_s *pstate = nsh_newconsole();
  DEBUGASSERT(pstate);
  int ret;

  /* If we are using a USB serial console, then we will have to wait for the
   * USB to be connected to the host.
   */

#ifdef HAVE_USB_CONSOLE
  ret = nsh_usbconsole();
  DEBUGASSERT(ret == OK);
#endif

  /* Present a greeting */

  fputs(g_nshgreeting, pstate->cn_outstream);
  fflush(pstate->cn_outstream);

  /* Execute the startup script */

#ifdef CONFIG_NSH_ROMFSETC
  (void)nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
#endif

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* Display the prompt string */

      fputs(g_nshprompt, pstate->cn_outstream);
      fflush(pstate->cn_outstream);

      /* Get the next line of input */

      ret = readline(pstate->cn_line, CONFIG_NSH_LINELEN,
                     INSTREAM(pstate), OUTSTREAM(pstate));
      if (ret > 0)
        {
          /* Parse process the command */

          (void)nsh_parse(&pstate->cn_vtbl, pstate->cn_line);
          fflush(pstate->cn_outstream);
        }

      /* Readline normally returns the number of characters read,
       * but will return 0 on end of file or a negative value
       * if an error occurs.  Either will cause the session to
       * terminate.
       */

      else
        {
          fprintf(pstate->cn_outstream, g_fmtcmdfailed, "readline", NSH_ERRNO_OF(-ret));
          nsh_exit(&pstate->cn_vtbl, 1);
        }
    }

  /* Clean up */

  nsh_exit(&pstate->cn_vtbl, 0);

  /* We do not get here, but this is necessary to keep some compilers happy */

  return OK;
}
