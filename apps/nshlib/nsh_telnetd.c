/****************************************************************************
 * apps/nshlib/nsh_telnetd.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <apps/netutils/uiplib.h>
#if defined(CONFIG_NSH_DHCPC)
#  include <apps/netutils/resolv.h>
#  include <apps/netutils/dhcpc.h>
#endif

#include "nsh.h"

#ifdef CONFIG_NSH_TELNET

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

#ifdef CONFIG_NSH_TELNETD_DUMPBUFFER
# define nsh_telnetdump(vtbl,msg,buf,nb) nsh_dumpbuffer(vtbl,msg,buf,nb)
#else
# define nsh_telnetdump(vtbl,msg,buf,nb)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct telnetio_s
{
  sem_t   tio_sem;
  int     tio_sockfd;
  uint8_t tio_bufndx;
  uint8_t tio_state;
  char    tio_inbuffer[CONFIG_NSH_IOBUFFER_SIZE];
};

struct redirect_s
{
  int    rd_fd;      /* Re-direct file descriptor */
  FILE  *rd_stream;  /* Re-direct stream */
};

struct telnetsave_s
{
  bool ts_redirected;
  union
    {
      struct telnetio_s *tn;
      struct redirect_s  rd;
    } u;
};

struct telnetd_s
{
  struct nsh_vtbl_s tn_vtbl;
  uint16_t          tn_sndlen;
  bool              tn_redirected;
  union
    {
      struct telnetio_s *tn;
      struct redirect_s  rd;
    } u;
  char tn_outbuffer[CONFIG_NSH_IOBUFFER_SIZE];
  char tn_cmd[CONFIG_NSH_LINELEN];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static void tio_semtake(struct telnetio_s *tio);
static FAR struct nsh_vtbl_s *nsh_telnetclone(FAR struct nsh_vtbl_s *vtbl);
#endif
static void nsh_telnetrelease(FAR struct nsh_vtbl_s *vtbl);
static ssize_t nsh_telnetwrite(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes);
static int nsh_telnetoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);
static int nsh_redirectoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...);
static FAR char *nsh_telnetlinebuffer(FAR struct nsh_vtbl_s *vtbl);
static void nsh_telnetredirect(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save);
static void nsh_telnetundirect(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save);
static void nsh_telnetexit(FAR struct nsh_vtbl_s *vtbl);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tio_semtake
 ****************************************************************************/

static void tio_semtake(struct telnetio_s *tio)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&tio->tio_sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: tio_semgive
 ****************************************************************************/

#define tio_semgive(tio) ASSERT(sem_post(&tio->tio_sem) == 0)

/****************************************************************************
 * Name: nsh_allocstruct
 ****************************************************************************/

static FAR struct telnetd_s *nsh_allocstruct(void)
{
  struct telnetd_s *pstate = (struct telnetd_s *)zalloc(sizeof(struct telnetd_s));
  if (pstate)
    {
#ifndef CONFIG_NSH_DISABLEBG
      pstate->tn_vtbl.clone      = nsh_telnetclone;
      pstate->tn_vtbl.release    = nsh_telnetrelease;
#endif
      pstate->tn_vtbl.write      = nsh_telnetwrite;
      pstate->tn_vtbl.output     = nsh_telnetoutput;
      pstate->tn_vtbl.linebuffer = nsh_telnetlinebuffer;
      pstate->tn_vtbl.redirect   = nsh_telnetredirect;
      pstate->tn_vtbl.undirect   = nsh_telnetundirect;
      pstate->tn_vtbl.exit       = nsh_telnetexit;
    }
  return pstate;
}

/****************************************************************************
 * Name: nsh_openifnotopen
 ****************************************************************************/

static int nsh_openifnotopen(struct telnetd_s *pstate)
{
  struct redirect_s *rd = &pstate->u.rd;

  /* The stream is open in a lazy fashion.  This is done because the file
   * descriptor may be opened on a different task than the stream.
   */

  if (!rd->rd_stream)
    {
      rd->rd_stream = fdopen(rd->rd_fd, "w");
      if (!rd->rd_stream)
        {
          return ERROR;
        }
    }
  return 0;
}

/****************************************************************************
 * Name: nsh_closeifnotclosed
 ****************************************************************************/

static void nsh_closeifnotclosed(struct telnetd_s *pstate)
{
  struct redirect_s *rd = &pstate->u.rd;

  if (rd->rd_stream == stdout)
    {
      fflush(stdout);
      rd->rd_fd = 1;
    }
  else
    {
      if (rd->rd_stream)
        {
          fflush(rd->rd_stream);
          fclose(rd->rd_stream);
        }
      else if (rd->rd_fd >= 0 && rd->rd_fd != 1)
        {
          close(rd->rd_fd);
        }

      rd->rd_fd     = -1;
      rd->rd_stream = NULL;
    }
}

/****************************************************************************
 * Name: nsh_putchar
 *
 * Description:
 *   Add another parsed character to the TELNET command string
 *
 * Assumption:
 *   Caller holds TIO semaphore
 *
 ****************************************************************************/

static void nsh_putchar(struct telnetd_s *pstate, uint8_t ch)
{
  struct telnetio_s *tio = pstate->u.tn;

  /* Ignore carriage returns */

  if (ch == ISO_cr)
  {
    return;
  }

  /* Add all other characters to the cmd buffer */

  pstate->tn_cmd[tio->tio_bufndx] = ch;

  /* If a newline was added or if the buffer is full, then process it now */

  if (ch == ISO_nl || tio->tio_bufndx == (CONFIG_NSH_LINELEN - 1))
    {
      pstate->tn_cmd[tio->tio_bufndx] = '\0';
      nsh_telnetdump(&pstate->tn_vtbl, "TELNET CMD",
                     (uint8_t*)pstate->tn_cmd, strlen(pstate->tn_cmd));
      nsh_parse(&pstate->tn_vtbl, pstate->tn_cmd);
      tio->tio_bufndx = 0;
    }
  else
    {
      tio->tio_bufndx++;
      vdbg("Add '%c', bufndx=%d\n", ch, tio->tio_bufndx);
    }
}

/****************************************************************************
 * Name: nsh_sendopt
 *
 * Description:
 *
 ****************************************************************************/

static void nsh_sendopt(struct telnetd_s *pstate, uint8_t option, uint8_t value)
{
  struct telnetio_s *tio = pstate->u.tn;
  uint8_t optbuf[4];
  optbuf[0] = TELNET_IAC;
  optbuf[1] = option;
  optbuf[2] = value;
  optbuf[3] = 0;

  nsh_telnetdump(&pstate->tn_vtbl, "Send optbuf", optbuf, 4);
  tio_semtake(tio); /* Only one call to send at a time */
  if (send(tio->tio_sockfd, optbuf, 4, 0) < 0)
    {
      dbg("[%d] Failed to send TELNET_IAC: %d\n", tio->tio_sockfd, errno);
    }
  tio_semgive(tio);
}

/****************************************************************************
 * Name: nsh_flush
 *
 * Description:
 *   Dump the buffered output info.
 *
 ****************************************************************************/

static void nsh_flush(FAR struct telnetd_s *pstate)
{
  struct telnetio_s *tio = pstate->u.tn;

  if (pstate->tn_sndlen > 0)
    {
      nsh_telnetdump(&pstate->tn_vtbl, "Shell output",
                     (uint8_t*)pstate->tn_outbuffer, pstate->tn_sndlen);
      tio_semtake(tio); /* Only one call to send at a time */
      if (send(tio->tio_sockfd, pstate->tn_outbuffer, pstate->tn_sndlen, 0) < 0)
        {
          dbg("[%d] Failed to send response: %d\n", tio->tio_sockfd, errno);
        }
      tio_semgive(tio);
    }
  pstate->tn_sndlen = 0;
}

/****************************************************************************
 * Name: nsh_receive
 *
 * Description:
 *   Process a received TELENET buffer
 *
 ****************************************************************************/

static int nsh_receive(struct telnetd_s *pstate, size_t len)
{
  struct telnetio_s *tio = pstate->u.tn;
  char              *ptr = tio->tio_inbuffer;
  uint8_t ch;

  while (len > 0)
    {
      ch = *ptr++;
      len--;

      vdbg("ch=%02x state=%d\n", ch, tio->tio_state);
      switch (tio->tio_state)
        {
          case STATE_IAC:
            if (ch == TELNET_IAC)
              {
                nsh_putchar(pstate, ch);
                tio->tio_state = STATE_NORMAL;
             }
            else
              {
                switch (ch)
                  {
                    case TELNET_WILL:
                      tio->tio_state = STATE_WILL;
                      break;

                    case TELNET_WONT:
                      tio->tio_state = STATE_WONT;
                      break;

                    case TELNET_DO:
                      tio->tio_state = STATE_DO;
                      break;

                    case TELNET_DONT:
                      tio->tio_state = STATE_DONT;
                      break;

                    default:
                      tio->tio_state = STATE_NORMAL;
                      break;
                  }
              }
            break;

          case STATE_WILL:
            /* Reply with a DONT */

            nsh_sendopt(pstate, TELNET_DONT, ch);
            tio->tio_state = STATE_NORMAL;
            break;

          case STATE_WONT:
            /* Reply with a DONT */

            nsh_sendopt(pstate, TELNET_DONT, ch);
            tio->tio_state = STATE_NORMAL;
            break;

          case STATE_DO:
            /* Reply with a WONT */

            nsh_sendopt(pstate, TELNET_WONT, ch);
            tio->tio_state = STATE_NORMAL;
            break;

          case STATE_DONT:
            /* Reply with a WONT */

            nsh_sendopt(pstate, TELNET_WONT, ch);
            tio->tio_state = STATE_NORMAL;
            break;

          case STATE_NORMAL:
            if (ch == TELNET_IAC)
              {
                tio->tio_state = STATE_IAC;
              }
            else
              {
                nsh_putchar(pstate, ch);
              }
            break;
        }
    }
  return OK;
}

/****************************************************************************
 * Name: nsh_connection
 *
 * Description:
 *   Each time a new connection to port 23 is made, a new thread is created
 *   that begins at this entry point.  There should be exactly one argument
 *   and it should be the socket descriptor (+1).
 *
 ****************************************************************************/

static void *nsh_connection(void *arg)
{
  struct telnetd_s  *pstate = nsh_allocstruct();
  struct telnetio_s *tio    = (struct telnetio_s *)zalloc(sizeof(struct telnetio_s));
  struct nsh_vtbl_s *vtbl   = &pstate->tn_vtbl;
  int                sockfd = (int)arg;
  int                ret    = ERROR;

  dbg("[%d] Started\n", sockfd);

  /* Verify that the state structure was successfully allocated */

  if (pstate && tio)
    {
      /* Initialize the thread state structure */

      sem_init(&tio->tio_sem, 0, 1);
      tio->tio_sockfd = sockfd;
      tio->tio_state  = STATE_NORMAL;
      pstate->u.tn    = tio;

      /* Output a greeting */

      nsh_output(vtbl, g_nshgreeting);

      /* Execute the startup script */

#if defined(CONFIG_NSH_ROMFSETC) && !defined(CONFIG_NSH_CONSOLE)
     (void)nsh_script(vtbl, "init", NSH_INITPATH);
#endif

      /* Loop processing each TELNET command */

      do
        {
          /* Display the prompt string */

          nsh_output(vtbl, g_nshprompt);
          nsh_flush(pstate);

          /* Read a buffer of data from the TELNET client */

          ret = recv(tio->tio_sockfd, tio->tio_inbuffer,
                     CONFIG_NSH_IOBUFFER_SIZE, 0);
          if (ret > 0)
            {

              /* Process the received TELNET data */

              nsh_telnetdump(vtbl, "Received buffer",
                             (uint8_t*)tio->tio_inbuffer, ret);
              ret = nsh_receive(pstate, ret);
            }
        }
      while (ret >= 0 && tio->tio_state != STATE_CLOSE);
      dbg("[%d] ret=%d tn.tio_state=%d\n", sockfd, ret, tio->tio_state);

      /* End of command processing -- Clean up and exit */
    }

  /* Exit the task */

  if (pstate)
    {
      free(pstate);
    }

  if (tio)
    {
      sem_destroy(&tio->tio_sem);
      free(tio);
    }

  dbg("[%d] Exitting\n", sockfd);
  close(sockfd);
  return NULL;
}

/****************************************************************************
 * Name: nsh_telnetwrite
 *
 * Description:
 *   write a buffer to the remote shell window.
 *
 *   Currently only used by cat.
 *
 ****************************************************************************/

static ssize_t nsh_telnetwrite(FAR struct nsh_vtbl_s *vtbl, FAR const void *buffer, size_t nbytes)
{
  struct telnetd_s  *pstate = (struct telnetd_s *)vtbl;
  struct telnetio_s *tio    = pstate->u.tn;
  ssize_t            ret    = nbytes;

  /* Flush anything already in the output buffer */

  nsh_flush(pstate);

  /* Then write the user buffer */

  nsh_telnetdump(&pstate->tn_vtbl, "Buffer output",(uint8_t*)buffer, nbytes);

  tio_semtake(tio); /* Only one call to send at a time */
  ret = send(tio->tio_sockfd, buffer, nbytes, 0);
  if (ret < 0)
    {
      dbg("[%d] Failed to send buffer: %d\n", tio->tio_sockfd, errno);
    }

  tio_semgive(tio);
  return ret;
}

/****************************************************************************
 * Name: nsh_telnetoutput
 *
 * Description:
 *   Print a string to the remote shell window.
 *
 *   This function is implemented by the shell GUI / telnet server and
 *   can be called by the shell back-end to output a string in the
 *   shell window. The string is automatically appended with a linebreak.
 *
 ****************************************************************************/

static int nsh_telnetoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...)
{
  struct telnetd_s  *pstate = (struct telnetd_s *)vtbl;
  int                nbytes = pstate->tn_sndlen;
  int                len;
  va_list            ap;

  /* Put the new info into the buffer.  Here we are counting on the fact that
   * no output strings will exceed CONFIG_NSH_LINELEN!
   */

  va_start(ap, fmt);
  vsnprintf(&pstate->tn_outbuffer[nbytes],
            (CONFIG_NSH_IOBUFFER_SIZE - 1) - nbytes, fmt, ap);
  va_end(ap);

  /* Get the size of the new string just added and the total size of
   * buffered data
   */

  len     = strlen(&pstate->tn_outbuffer[nbytes]);
  nbytes += len;

  /* Expand any terminating \n to \r\n */

  if (nbytes < (CONFIG_NSH_IOBUFFER_SIZE - 2) &&
      pstate->tn_outbuffer[nbytes-1] == '\n')
    {
      pstate->tn_outbuffer[nbytes-1] = ISO_cr;
      pstate->tn_outbuffer[nbytes]   = ISO_nl;
      pstate->tn_outbuffer[nbytes+1] = '\0';
      nbytes++;
    }
  pstate->tn_sndlen = nbytes;

  /* Flush to the network if the buffer does not have room for one more
   * maximum length string.
   */

  if (nbytes > CONFIG_NSH_IOBUFFER_SIZE - CONFIG_NSH_LINELEN)
    {
      nsh_flush(pstate);
    }

  return len;
}

/****************************************************************************
 * Name: nsh_redirectoutput
 *
 * Description:
 *   Print a string to the currently selected stream.
 *
 ****************************************************************************/

static int nsh_redirectoutput(FAR struct nsh_vtbl_s *vtbl, const char *fmt, ...)
{
  FAR struct telnetd_s *pstate = (FAR struct telnetd_s *)vtbl;
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
  ret = vfprintf(pstate->u.rd.rd_stream, fmt, ap);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: nsh_telnetlinebuffer
 *
 * Description:
 *   Return a reference to the current line buffer
 *
* ****************************************************************************/

static FAR char *nsh_telnetlinebuffer(FAR struct nsh_vtbl_s *vtbl)
{
  struct telnetd_s *pstate = (struct telnetd_s *)vtbl;
  return pstate->tn_cmd;
}

/****************************************************************************
 * Name: nsh_telnetclone
 *
 * Description:
 *   Make an independent copy of the vtbl
 *
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static FAR struct nsh_vtbl_s *nsh_telnetclone(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct telnetd_s  *pstate = (FAR struct telnetd_s *)vtbl;
  FAR struct telnetd_s  *pclone = nsh_allocstruct();
  FAR struct nsh_vtbl_s *ret    = NULL;

  if (pclone)
    {
      if (pstate->tn_redirected)
        {
          pclone->tn_redirected  = true;
          pclone->tn_vtbl.output = nsh_redirectoutput;
          pclone->u.rd.rd_fd     = pstate->u.rd.rd_fd;
          pclone->u.rd.rd_stream = NULL;
        }
      else
        {
          pclone->u.tn = pstate->u.tn;
        }
      ret = &pclone->tn_vtbl;
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: nsh_telnetrelease
 *
 * Description:
 *   Release the cloned instance
 *
 ****************************************************************************/

#ifndef CONFIG_NSH_DISABLEBG
static void nsh_telnetrelease(FAR struct nsh_vtbl_s *vtbl)
{
  FAR struct telnetd_s *pstate = (FAR struct telnetd_s *)vtbl;

  if (pstate->tn_redirected)
    {
      nsh_closeifnotclosed(pstate);
    }
  else
    {
      nsh_flush(pstate);
    }
  free(pstate);
}
#endif

/****************************************************************************
 * Name: nsh_telnetredirect
 *
 * Description:
 *   Set up for redirected output.  This function is called from nsh_parse()
 *   in two different contexts:
 *
 *   1) Redirected background commands of the form:  command > xyz.text &
 *
 *      In this case:
 *      - vtbl: A newly allocated and initialized instance created by
 *        nsh_telnetclone,
 *      - fd:- The file descriptor of the redirected output
 *      - save: NULL
 *
 *      nsh_telnetrelease() will perform the clean-up when the clone is
 *      destroyed.
 *        
 *   2) Redirected foreground commands of the form:  command > xyz.txt
 *
 *      In this case:
 *      - vtbl: The current state structure,
 *      - fd: The file descriptor of the redirected output
 *      - save: Where to save the re-directed registers.
 *
 *      nsh_telnetundirect() will perform the clean-up after the redirected
 *      command completes.
 *
 ****************************************************************************/

static void nsh_telnetredirect(FAR struct nsh_vtbl_s *vtbl, int fd, FAR uint8_t *save)
{
  FAR struct telnetd_s    *pstate = (FAR struct telnetd_s *)vtbl;
  FAR struct telnetsave_s *ssave  = (FAR struct telnetsave_s *)save;

  if (pstate->tn_redirected)
    {
       (void)nsh_openifnotopen(pstate);
       fflush(pstate->u.rd.rd_stream);
       if (!ssave)
         {
           fclose(pstate->u.rd.rd_stream);
         }
    }

  if (ssave)
    {
      ssave->ts_redirected = pstate->tn_redirected;
      memcpy(&ssave->u.rd, &pstate->u.rd, sizeof(struct redirect_s));
    }

  pstate->tn_redirected  = true;
  pstate->u.rd.rd_fd     = fd;
  pstate->u.rd.rd_stream = NULL;  
}

/****************************************************************************
 * Name: nsh_telnetundirect
 *
 * Description:
 *   Set up for redirected output
 *
 ****************************************************************************/

static void nsh_telnetundirect(FAR struct nsh_vtbl_s *vtbl, FAR uint8_t *save)
{
  FAR struct telnetd_s *pstate = (FAR struct telnetd_s *)vtbl;
  FAR struct telnetsave_s *ssave  = (FAR struct telnetsave_s *)save;

  if (pstate->tn_redirected)
    {
      nsh_closeifnotclosed(pstate);
    }

  pstate->tn_redirected = ssave->ts_redirected;
  memcpy(&pstate->u.rd, &ssave->u.rd, sizeof(struct redirect_s));
}

/****************************************************************************
 * Name: nsh_telnetexit
 *
 * Description:
 *   Quit the shell instance
 *
 ****************************************************************************/

static void nsh_telnetexit(FAR struct nsh_vtbl_s *vtbl)
{
  struct telnetd_s *pstate = (struct telnetd_s *)vtbl;
  pstate->u.tn->tio_state = STATE_CLOSE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_telnetmain
 *
 * Description:
 *   This is the main processing thread for telnetd.  It never returns
 *   unless an error occurs
 *
 ****************************************************************************/

int nsh_telnetmain(int argc, char *argv[])
{
  /* Execute nsh_connection() on each connection to port 23 */

  uip_server(HTONS(23), nsh_connection, CONFIG_NSH_STACKSIZE);
  return OK;
}

#endif /* CONFIG_NSH_TELNET */
