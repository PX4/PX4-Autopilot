/****************************************************************************
 * apps/nshlib/nsh_telnetd.c
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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
#include <string.h>

#include <apps/netutils/telnetd.h>

#include "nsh.h"
#include "nsh_console.h"

#ifdef CONFIG_NSH_TELNET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NSH_TELNET_LOGIN

#  define TELNET_IAC              255
#  define TELNET_WILL             251
#  define TELNET_WONT             252
#  define TELNET_DO               253
#  define TELNET_DONT             254
#  define TELNET_USE_ECHO         1
#  define TELNET_NOTUSE_ECHO      0

#endif /* CONFIG_NSH_TELNET_LOGIN */

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
 * Name: nsh_telnetecho
 ****************************************************************************/

#ifdef CONFIG_NSH_TELNET_LOGIN
void nsh_telnetecho(struct console_stdio_s *pstate, uint8_t is_use)
{
  uint8_t optbuf[4];
  optbuf[0] = TELNET_IAC;
  optbuf[1] = (is_use == TELNET_USE_ECHO) ? TELNET_WILL : TELNET_DO;
  optbuf[2] = 1;
  optbuf[3] = 0;
  fputs((char *)optbuf, pstate->cn_outstream);
  fflush(pstate->cn_outstream);
}
#endif

/****************************************************************************
 * Name: nsh_telnetlogin
 ****************************************************************************/

#ifdef CONFIG_NSH_TELNET_LOGIN
int nsh_telnetlogin(struct console_stdio_s *pstate)
{
  char username[16];
  char password[16];
  uint8_t i;

  /* Present the NSH Telnet greeting */

  fputs(g_telnetgreeting, pstate->cn_outstream);
  fflush(pstate->cn_outstream);

  /* Loop for the configured number of retries */

  for(i = 0; i < CONFIG_NSH_TELNET_FAILCOUNT; i++)
    {
      /* Ask for the login username */

      fputs(g_userprompt, pstate->cn_outstream);
      fflush(pstate->cn_outstream);
      if (fgets(pstate->cn_line, CONFIG_NSH_LINELEN, INSTREAM(pstate)) != NULL)
        {
          strcpy(username, pstate->cn_line);
          username[strlen(pstate->cn_line) - 1] = 0;
        }

      /* Ask for the login password */
  
      fputs(g_passwordprompt, pstate->cn_outstream);
      fflush(pstate->cn_outstream);
      nsh_telnetecho(pstate, TELNET_NOTUSE_ECHO);
      if (fgets(pstate->cn_line, CONFIG_NSH_LINELEN, INSTREAM(pstate)) != NULL)
        {
          /* Verify the username and password */

          strcpy(password,pstate->cn_line);
          password[strlen(pstate->cn_line) - 1] = 0;

          if (strcmp(password, CONFIG_NSH_TELNET_PASSWORD) == 0 &&
              strcmp(username, CONFIG_NSH_TELNET_USERNAME) == 0)
            {
              fputs(g_loginsuccess, pstate->cn_outstream);
              fflush(pstate->cn_outstream);
              nsh_telnetecho(pstate, TELNET_USE_ECHO);
              return OK;
            }
          else
            {
              fputs(g_badcredentials, pstate->cn_outstream);
              fflush(pstate->cn_outstream);
            }
        }

      nsh_telnetecho(pstate, TELNET_USE_ECHO);
    }

  /* Too many failed login attempts */

  fputs(g_loginfailure, pstate->cn_outstream);
  fflush(pstate->cn_outstream);
  return -1;
}
#endif /* CONFIG_NSH_TELNET_LOGIN */

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

  /* Login User and Password Check */

#ifdef CONFIG_NSH_TELNET_LOGIN
  if (nsh_telnetlogin(pstate) != OK)
    {
      nsh_exit(&pstate->cn_vtbl, 1);
      return -1; /* nsh_exit does not return */
    }
#endif /* CONFIG_NSH_TELNET_LOGIN */

  /* The following logic mostly the same as the login in nsh_session.c.  It
   * differs only in that gets() is called to get the command instead of
   * readline().
   */

  /* Present the NSH greeting */

  fputs(g_nshgreeting, pstate->cn_outstream);
  fflush(pstate->cn_outstream);

  /* Execute the startup script.  If standard console is also defined, then
   * we will not bother with the initscript here (although it is safe to
   * call nshinitscript multiple times).
   */

#if defined(CONFIG_NSH_ROMFSETC) && !defined(CONFIG_NSH_CONSOLE)
  (void)nsh_initscript(&pstate->cn_vtbl);
#endif

  /* Execute the login script */

#ifdef CONFIG_NSH_ROMFSRC
  (void)nsh_loginscript(&pstate->cn_vtbl);
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
 *   The task ID of the Telnet daemon was successfully started.  A negated
 *   errno value will be returned on failure.
 *  
 ****************************************************************************/

int nsh_telnetstart(void)
{
  struct telnetd_config_s config;
  int ret;

  /* Initialize any USB tracing options that were requested.  If standard
   * console is also defined, then we will defer this step to the standard
   * console.
   */

#if defined(CONFIG_NSH_USBDEV_TRACE) && !defined(CONFIG_NSH_CONSOLE)
  usbtrace_enable(TRACE_BITSET);
#endif

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
