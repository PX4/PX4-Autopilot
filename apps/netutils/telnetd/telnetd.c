/****************************************************************************
 * netutils/telnetd/telnetd.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This is a leverage of similar logic from uIP:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <debug.h>

#include <apps/netutils/telnetd.h>
#include <apps/netutils/uiplib.h>

#include "shell.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define ISO_nl       0x0a
#define ISO_cr       0x0d

#define STATE_NORMAL 0
#define STATE_IAC    1
#define STATE_WILL   2
#define STATE_WONT   3
#define STATE_DO     4
#define STATE_DONT   5
#define STATE_CLOSE  6

#define TELNET_IAC   255
#define TELNET_WILL  251
#define TELNET_WONT  252
#define TELNET_DO    253
#define TELNET_DONT  254

/* Configurable settings */

#ifndef CONFIG_NETUTILS_IOBUFFER_SIZE
# define CONFIG_NETUTILS_IOBUFFER_SIZE 512
#endif

#ifndef CONFIG_NETUTILS_CMD_SIZE
# define CONFIG_NETUTILS_CMD_SIZE 40
#endif

/* As threads are created to handle each request, a stack must be allocated
 * for the thread.  Use a default if the user provided no stacksize.
 */

#ifndef CONFIG_NETUTILS_TELNETDSTACKSIZE
# define CONFIG_NETUTILS_TELNETDSTACKSIZE 4096
#endif

/* Enabled dumping of all input/output buffers */

#undef CONFIG_NETUTILS_TELNETD_DUMPBUFFER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct telnetd_s
{
  int     tn_sockfd;
  char    tn_iobuffer[CONFIG_NETUTILS_IOBUFFER_SIZE];
  char    tn_cmd[CONFIG_NETUTILS_CMD_SIZE];
  uint8_t tn_bufndx;
  uint8_t tn_state;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnetd_dumpbuffer
 *
 * Description:
 *   Dump a buffer of data (debug only)
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_TELNETD_DUMPBUFFER
static inline void telnetd_dumpbuffer(FAR const char *msg, FAR const char *buffer, unsigned int nbytes)
{
  /* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_NET have to be
  * defined or the following does nothing.
  */
    
  nvdbgdumpbuffer(msg, (FAR const uint8_t*)buffer, nbytes);
}
#else
# define telnetd_dumpbuffer(msg,buffer,nbytes)
#endif

/****************************************************************************
 * Name: telnetd_putchar
 *
 * Description:
 *   Add another parsed character to the TELNET command string
 *
 ****************************************************************************/

static void telnetd_putchar(struct telnetd_s *pstate, uint8_t ch)
{
  /* Ignore carriage returns */

  if (ch == ISO_cr)
  {
    return;
  }

  /* Add all other characters to the cmd buffer */

  pstate->tn_cmd[pstate->tn_bufndx] = ch;

  /* If a newline was added or if the buffer is full, then process it now */

  if (ch == ISO_nl || pstate->tn_bufndx == (CONFIG_NETUTILS_CMD_SIZE - 1))
    {
      if (pstate->tn_bufndx > 0)
        {
          pstate->tn_cmd[pstate->tn_bufndx] = '\0';
        }

      telnetd_dumpbuffer("TELNET CMD", pstate->tn_cmd, strlen(pstate->tn_cmd));
      shell_input(pstate, pstate->tn_cmd);
      pstate->tn_bufndx = 0;
    }
  else
    {
      pstate->tn_bufndx++;
      vdbg("Add '%c', bufndx=%d\n", ch, pstate->tn_bufndx);
    }
}

/****************************************************************************
 * Name: telnetd_sendopt
 *
 * Description:
 *
 ****************************************************************************/

static void telnetd_sendopt(struct telnetd_s *pstate, uint8_t option, uint8_t value)
{
  uint8_t optbuf[4];
  optbuf[0] = TELNET_IAC;
  optbuf[1] = option;
  optbuf[2] = value;
  optbuf[3] = 0;

  telnetd_dumpbuffer("Send optbuf", optbuf, 4);
  if (send(pstate->tn_sockfd, optbuf, 4, 0) < 0)
    {
      dbg("[%d] Failed to send TELNET_IAC\n", pstate->tn_sockfd);
    }
}

/****************************************************************************
 * Name: telnetd_receive
 *
 * Description:
 *   Process a received TELENET buffer
 *
 ****************************************************************************/

static int telnetd_receive(struct telnetd_s *pstate, size_t len)
{
  char *ptr = pstate->tn_iobuffer;
  uint8_t ch;

  while (len > 0)
    {
      ch = *ptr++;
      len--;

      vdbg("ch=%02x state=%d\n", ch, pstate->tn_state);
      switch (pstate->tn_state)
        {
          case STATE_IAC:
            if (ch == TELNET_IAC)
              {
                telnetd_putchar(pstate, ch);
                pstate->tn_state = STATE_NORMAL;
             }
            else
              {
                switch (ch)
                  {
                    case TELNET_WILL:
                      pstate->tn_state = STATE_WILL;
                      break;

                    case TELNET_WONT:
                      pstate->tn_state = STATE_WONT;
                      break;

                    case TELNET_DO:
                      pstate->tn_state = STATE_DO;
                      break;

                    case TELNET_DONT:
                      pstate->tn_state = STATE_DONT;
                      break;

                    default:
                      pstate->tn_state = STATE_NORMAL;
                      break;
                  }
              }
            break;

          case STATE_WILL:
            /* Reply with a DONT */

            telnetd_sendopt(pstate, TELNET_DONT, ch);
            pstate->tn_state = STATE_NORMAL;
            break;

          case STATE_WONT:
            /* Reply with a DONT */

            telnetd_sendopt(pstate, TELNET_DONT, ch);
            pstate->tn_state = STATE_NORMAL;
            break;

          case STATE_DO:
            /* Reply with a WONT */

            telnetd_sendopt(pstate, TELNET_WONT, ch);
            pstate->tn_state = STATE_NORMAL;
            break;

          case STATE_DONT:
            /* Reply with a WONT */

            telnetd_sendopt(pstate, TELNET_WONT, ch);
            pstate->tn_state = STATE_NORMAL;
            break;

          case STATE_NORMAL:
            if (ch == TELNET_IAC)
              {
                pstate->tn_state = STATE_IAC;
              }
            else
              {
                telnetd_putchar(pstate, ch);
              }
            break;
        }
    }
  return OK;
}

/****************************************************************************
 * Name: telnetd_handler
 *
 * Description:
 *   Each time a new connection to port 23 is made, a new thread is created
 *   that begins at this entry point.  There should be exactly one argument
 *   and it should be the socket descriptor (+1).
 *
 ****************************************************************************/

static void *telnetd_handler(void *arg)
{
  struct telnetd_s *pstate = (struct telnetd_s *)malloc(sizeof(struct telnetd_s));
  int               sockfd = (int)arg;
  int               ret    = ERROR;

  dbg("[%d] Started\n", sockfd);

  /* Verify that the state structure was successfully allocated */

  if (pstate)
    {
      /* Initialize the thread state structure */

      memset(pstate, 0, sizeof(struct telnetd_s));
      pstate->tn_sockfd = sockfd;
      pstate->tn_state  = STATE_NORMAL;

      /* Start up the shell */

      shell_init(pstate);
      shell_start(pstate);

      /* Loop processing each TELNET command */
      do
        {
          /* Read a buffer of data from the TELNET client */

          ret = recv(pstate->tn_sockfd, pstate->tn_iobuffer, CONFIG_NETUTILS_IOBUFFER_SIZE, 0);
          if (ret > 0)
            {

              /* Process the received TELNET data */

              telnetd_dumpbuffer("Received buffer", pstate->tn_iobuffer, ret);
              ret = telnetd_receive(pstate, ret);
            }
        }
      while (ret >= 0 && pstate->tn_state != STATE_CLOSE);
      dbg("[%d] ret=%d tn_state=%d\n", sockfd, ret, pstate->tn_state);

      /* End of command processing -- Clean up and exit */

      free(pstate);
    }

  /* Exit the task */

  dbg("[%d] Exitting\n", sockfd);
  close(sockfd);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnetd_init
 *
 * Description:
 *   This is the main processing thread for telnetd.  It never returns
 *   unless an error occurs
 *
 ****************************************************************************/

void telnetd_init(void)
{
  /* Execute telnetd_handler on each connection to port 23 */

  uip_server(HTONS(23), telnetd_handler, CONFIG_NETUTILS_TELNETDSTACKSIZE);
}

/****************************************************************************
 * Name: shell_prompt
 *
 * Description:
 *   Print a prompt to the shell window.
 *
 *   This function can be used by the shell back-end to print out a prompt
 *   to the shell window.
 *
 ****************************************************************************/

void shell_prompt(void *handle, char *str)
{
  struct telnetd_s *pstate = (struct telnetd_s *)handle;
  int len = strlen(str);

  strncpy(pstate->tn_iobuffer, str, len);
  telnetd_dumpbuffer("Shell prompt", pstate->tn_iobuffer, len);
  if (send(pstate->tn_sockfd, pstate->tn_iobuffer, len, 0) < 0)
    {
      dbg("[%d] Failed to send prompt\n", pstate->tn_sockfd);
    }
}

/****************************************************************************
 * Name: shell_output
 *
 * Description:
 *   Print a string to the shell window.
 *
 *   This function is implemented by the shell GUI / telnet server and
 *   can be called by the shell back-end to output a string in the
 *   shell window. The string is automatically appended with a linebreak.
 *
 ****************************************************************************/

void shell_output(void *handle, const char *fmt, ...)
{
  struct telnetd_s *pstate = (struct telnetd_s *)handle;
  unsigned len;
  va_list ap;

  va_start(ap, fmt);
  vsnprintf(pstate->tn_iobuffer, CONFIG_NETUTILS_IOBUFFER_SIZE, fmt, ap);
  va_end(ap);

  len = strlen(pstate->tn_iobuffer);
  if (len < CONFIG_NETUTILS_IOBUFFER_SIZE - 2)
    {
      pstate->tn_iobuffer[len]   = ISO_cr;
      pstate->tn_iobuffer[len+1] = ISO_nl;
      pstate->tn_iobuffer[len+2] = '\0';
    }

  telnetd_dumpbuffer("Shell output", pstate->tn_iobuffer, len+2);
  if (send(pstate->tn_sockfd, pstate->tn_iobuffer, len+2, 0) < 0)
    {
      dbg("[%d] Failed to send response\n", pstate->tn_sockfd);
    }
}

/****************************************************************************
 * Name: shell_quit
 *
 * Description:
 *   Quit the shell
 *
 ****************************************************************************/

void shell_quit(void *handle, char *str)
{
  struct telnetd_s *pstate = (struct telnetd_s *)handle;
  pstate->tn_state = STATE_CLOSE;
}

