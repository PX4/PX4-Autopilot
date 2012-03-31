/****************************************************************************
 * apps/nshlib/nsh_telnetd.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <assert.h>
#include <debug.h>

#include <apps/netutils/telnetd.h>

#include "nsh.h"
#include "nsh_console.h"

#ifdef CONFIG_NSH_TELNET

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
 * Name: nsh_telnetmain
 ****************************************************************************/

int nsh_telnetmain(int argc, char *argv[])
{
  FAR struct console_stdio_s *pstate = nsh_newconsole();
  DEBUGASSERT(pstate != NULL);

  dbg("Session [%d] Started\n", getpid());

  /* Present a greeting */

  fputs(g_nshgreeting, pstate->cn_outstream);
  fflush(pstate->cn_outstream);

  /* Execute the startup script */

#if defined(CONFIG_NSH_ROMFSETC) && !defined(CONFIG_NSH_CONSOLE)
  (void)nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
#endif

  /* Then enter the command line parsing loop */

  for (;;)
    {
      /* Display the prompt string */

      fputs(g_nshprompt, pstate->cn_outstream);
      fflush(pstate->cn_outstream);

      /* Get the next line of input from the Telnet client */

      if (fgets(pstate->cn_line, CONFIG_NSH_LINELEN, INSTREAM(pstate)) != NULL)
        {
          /* Parse process the received Telnet command */

          (void)nsh_parse(&pstate->cn_vtbl, pstate->cn_line);
          fflush(pstate->cn_outstream);
        }
      else
        {
          fprintf(pstate->cn_outstream, g_fmtcmdfailed, "nsh_telnetmain",
                  "fgets", NSH_ERRNO);
          nsh_exit(&pstate->cn_vtbl, 1);
        }
    }

  /* Clean up */

  nsh_exit(&pstate->cn_vtbl, 0);

  /* We do not get here, but this is necessary to keep some compilers happy */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_telnetstart
 *
 * Description:
 *   nsh_telnetstart() starts the Telnet daemon that will allow multiple
 *   NSH connections via Telnet.  This function returns immediately after
 *   the daemon has been started.
 *
 * Input Parameters:
 *   None.  All of the properties of the Telnet daemon are controlled by
 *   NuttX configuration setting.
 *
 * Returned Values:
 *   Zero if the Telnet daemon was successfully started.  A negated errno
 *   value will be returned on failure.
 *  
 ****************************************************************************/

int nsh_telnetstart(void)
{
  struct telnetd_config_s config;
  int ret;

  /* Configure the telnet daemon */

  config.d_port      = HTONS(CONFIG_NSH_TELNETD_PORT);
  config.d_priority  = CONFIG_NSH_TELNETD_DAEMONPRIO;
  config.d_stacksize = CONFIG_NSH_TELNETD_DAEMONSTACKSIZE;
  config.t_priority  = CONFIG_NSH_TELNETD_CLIENTPRIO;
  config.t_stacksize = CONFIG_NSH_TELNETD_CLIENTSTACKSIZE;
  config.t_entry     = nsh_telnetmain;

  /* Start the telnet daemon */

  vdbg("Starting the Telnet daemon\n");
  ret = telnetd_start(&config);
  if (ret < 0)
    {
      dbg("Failed to tart the Telnet daemon: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_NSH_TELNET */
