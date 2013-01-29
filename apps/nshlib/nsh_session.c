/****************************************************************************
 * apps/nshlib/nsh_session.c
 *
 *   Copyright (C) 2007-2009, 2011-2013 Gregory Nutt. All rights reserved.
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
#include <stdlib.h>

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
 * Name: nsh_session
 *
 * Description:
 *   This is the common session logic or any NSH session.  This function
 *   return when an error reading from the input stream occurs, presumably
 *   signaling the end of the session.
 *
 *   This function:
 *   - Executes the NSH logic script
 *   - Presents a greeting
 *   - Then provides a prompt then gets and processes the command line.
 *   - This continues until an error occurs, then the session returns.
 *
 * Input Parameters:
 *   pstate - Abstracts the underlying session.
 *
 * Returned Values:
 *   EXIT_SUCESS or EXIT_FAILURE is returned.
 *  
 ****************************************************************************/

int nsh_session(FAR struct console_stdio_s *pstate)
{
  int ret;

  DEBUGASSERT(pstate);

  /* Present a greeting */

  fputs(g_nshgreeting, pstate->cn_outstream);
  fflush(pstate->cn_outstream);

  /* Execute the login script */

#ifdef CONFIG_NSH_ROMFSRC
  (void)nsh_loginscript(&pstate->cn_vtbl);
#endif

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* For the case of debugging the USB console... dump collected USB trace data */

#ifdef CONFIG_NSH_USBDEV_TRACE
      nsh_usbtrace();
#endif

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
          fprintf(pstate->cn_outstream, g_fmtcmdfailed, "nsh_session", 
                  "readline", NSH_ERRNO_OF(-ret));
          return ret == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
        }
    }

  /* We do not get here, but this is necessary to keep some compilers happy.
   * But others will complain that this code is not reachable.
   */

  return EXIT_SUCCESS;
}
