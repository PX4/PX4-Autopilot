/****************************************************************************
 * netutils/thttpd/thttpd.c
 * Tiny HTTP Server
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995,1998,1999,2000,2001 by Jef Poskanzer <jef@mail.acme.com>.
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/compiler.h>
#include <nuttx/binfmt/symtab.h>
#include <apps/netutils/thttpd.h>

#include "config.h"
#include "fdwatch.h"
#include "libhttpd.h"
#include "thttpd_alloc.h"
#include "thttpd_strings.h"
#include "timers.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAXPATHLEN
#  define MAXPATHLEN 64
#endif

/* The connection states */

#define CNST_FREE      0
#define CNST_READING   1
#define CNST_SENDING   2
#define CNST_LINGERING 3

#define SPARE_FDS      2
#define AVAILABLE_FDS  (CONFIG_NSOCKET_DESCRIPTORS - SPARE_FDS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct connect_s
{
  struct connect_s *next;
  int conn_state;
  httpd_conn *hc;
  time_t active_at;
  Timer *wakeup_timer;
  Timer *linger_timer;
  off_t end_offset;            /* The final offset+1 of the file to send */
  off_t offset;                /* The current offset into the file to send */
  bool eof;                    /* Set true when length==0 read from file */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static httpd_server *hs;
static struct connect_s *free_connections;
static struct connect_s *connects;
static struct fdwatch_s *fw;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void shut_down(void);
static int  handle_newconnect(struct timeval *tv, int listen_fd);
static void handle_read(struct connect_s *conn, struct timeval *tv);
static void handle_send(struct connect_s *conn, struct timeval *tv);
static void handle_linger(struct connect_s *conn, struct timeval *tv);
static void finish_connection(struct connect_s *conn, struct timeval *tv);
static void clear_connection(struct connect_s *conn, struct timeval *tv);
static void really_clear_connection(struct connect_s *conn);
static void idle(ClientData client_data, struct timeval *nowP);
static void linger_clear_connection(ClientData client_data, struct timeval *nowP);
static void occasional(ClientData client_data, struct timeval *nowP);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void shut_down(void)
{
  int cnum;

  for (cnum = 0; cnum < AVAILABLE_FDS; ++cnum)
    {
      if (connects[cnum].conn_state != CNST_FREE)
        {
          httpd_close_conn(connects[cnum].hc);
        }

      if (connects[cnum].hc != NULL)
        {
          httpd_destroy_conn(connects[cnum].hc);
          httpd_free((void *)connects[cnum].hc);
          connects[cnum].hc = NULL;
        }
    }

  if (hs)
    {
      httpd_server *ths = hs;
      hs = NULL;
      if (ths->listen_fd != -1)
        {
          fdwatch_del_fd(fw, ths->listen_fd);
        }
      httpd_terminate(ths);
    }

  tmr_destroy();
  httpd_free((void *)connects);
}

static int handle_newconnect(struct timeval *tv, int listen_fd)
{
  struct connect_s *conn;
  ClientData client_data;

  /* This loops until the accept() fails, trying to start new connections as 
   * fast as possible so we don't overrun the listen queue.
   */

  nvdbg("New connection(s) on listen_fd %d\n", listen_fd);
  for (;;)
    {
      /* Get the next free connection from the free list */

      conn = free_connections;
      
      /* Are there any free connections? */

      if (!conn)
        {
          /* Out of connection slots.  Run the timers, then the  existing
           * connections, and maybe we'll free up a slot  by the time we get
           * back here.
           */

          ndbg("No free connections\n");
          tmr_run(tv);
          return -1;
        }

      /* Make the httpd_conn if necessary */

      if (!conn->hc)
        {
          conn->hc = NEW(httpd_conn, 1);
          if (conn->hc == NULL)
            {
              ndbg("out of memory allocating an httpd_conn\n");
              exit(1);
            }

          conn->hc->initialized = 0;
        }

      /* Get the connection */

      switch (httpd_get_conn(hs, listen_fd, conn->hc))
        {
          /* Some error happened.  Run the timers, then the  existing
           * connections.  Maybe the error will clear.
           */

        case GC_FAIL:
          tmr_run(tv);
          return -1;

          /* No more connections to accept for now */

        case GC_NO_MORE:
          return 0;

        default:
          break;
        }

      nvdbg("New connection fd %d\n", conn->hc->conn_fd);
 
      /* Remove the connection entry from the free list */

      conn->conn_state        = CNST_READING;
      free_connections        = conn->next;
      conn->next              = NULL;

      client_data.p           = conn;
      conn->active_at         = tv->tv_sec;
      conn->wakeup_timer      = NULL;
      conn->linger_timer      = NULL;
      conn->offset            = 0;
 
      /* Set the connection file descriptor to no-delay mode */

      httpd_set_ndelay(conn->hc->conn_fd);
      fdwatch_add_fd(fw, conn->hc->conn_fd, conn);
    }
}

static void handle_read(struct connect_s *conn, struct timeval *tv)
{
  ClientData client_data;
  httpd_conn *hc = conn->hc;
  off_t actual;
  int sz;

  /* Is there room in our buffer to read more bytes? */

  if (hc->read_idx >= hc->read_size)
    {
      if (hc->read_size > CONFIG_THTTPD_MAXREALLOC)
        {
          BADREQUEST("MAXREALLOC");
          goto errout_with_400;
        }
      httpd_realloc_str(&hc->read_buf, &hc->read_size, hc->read_size + CONFIG_THTTPD_REALLOCINCR);
    }

  /* Read some more bytes */

  sz = read(hc->conn_fd, &(hc->read_buf[hc->read_idx]), hc->read_size - hc->read_idx);
  if (sz == 0)
    {
      BADREQUEST("EOF");
      goto errout_with_400;
    }

  if (sz < 0)
    {
      /* Ignore EINTR and EAGAIN.  Also ignore EWOULDBLOCK.  At first glance
       * you would think that connections returned by fdwatch as readable
       * should never give an EWOULDBLOCK; however, this apparently can
       * happen if a packet gets garbled.
       */

      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
        {
          return;
        }
 
      ndbg("read(fd=%d) failed: %d\n", hc->conn_fd, errno);
      BADREQUEST("read");
      goto errout_with_400;
    }

  hc->read_idx += sz;
  conn->active_at = tv->tv_sec;

  /* Do we have a complete request yet? */

  switch (httpd_got_request(hc))
    {
    case GR_NO_REQUEST:
      return;
    case GR_BAD_REQUEST:
     BADREQUEST("httpd_got_request");
     goto errout_with_400;
    }

  /* Yes.  Try parsing and resolving it */

  if (httpd_parse_request(hc) < 0)
    {
      goto errout_with_connection;
    }

  /* Start the connection going */

  if (httpd_start_request(hc, tv) < 0)
    {
      /* Something went wrong.  Close down the connection */

      goto errout_with_connection;
    }

  /* Set up the file offsets to read */

  conn->eof            = false;
  if (hc->got_range)
    {
      conn->offset     = hc->range_start;
      conn->end_offset = hc->range_end + 1;
    }
  else
    {
      conn->offset     = 0;
      if (hc->bytes_to_send < 0)
        {
          conn->end_offset = 0;
        }
      else
        {
          conn->end_offset = hc->bytes_to_send;
        }
    }

  /* Check if it's already handled */

  if (hc->file_fd < 0)
    {
      /* No file descriptor means someone else is handling it */

      conn->offset = hc->bytes_sent;
      goto errout_with_connection;
    }

  if (conn->offset >= conn->end_offset)
    {
      /* There's nothing to send */

      goto errout_with_connection;
    }

  /* Seek to the offset of the next byte to send */

  actual = lseek(hc->file_fd, conn->offset, SEEK_SET);
  if (actual != conn->offset)
    {
       ndbg("fseek to %d failed: offset=%d errno=%d\n", conn->offset, actual, errno);
       BADREQUEST("lseek");
       goto errout_with_400;
    }

  /* We have a valid connection and a file to send to it */

  conn->conn_state = CNST_SENDING;
  client_data.p    = conn;
  fdwatch_del_fd(fw, hc->conn_fd);
  return;

errout_with_400:
  BADREQUEST("errout");
  httpd_send_err(hc, 400, httpd_err400title, "", httpd_err400form, "");

errout_with_connection:
  finish_connection(conn, tv);
  return;
}

static inline int read_buffer(struct connect_s *conn)
{
  httpd_conn *hc = conn->hc;
  ssize_t nread = 0;

  if (hc->buflen < CONFIG_THTTPD_IOBUFFERSIZE && !conn->eof)
    {
      nread = read(hc->file_fd, &hc->buffer[hc->buflen],
                   CONFIG_THTTPD_IOBUFFERSIZE - hc->buflen);
      if (nread == 0)
        {
          /* Reading zero bytes means we are at the end of file */

          conn->end_offset = conn->offset;
          conn->eof        = true;
        }
      else if (nread > 0)
        {
          hc->buflen      += nread;
        }
    }
  return nread;
}

static void handle_send(struct connect_s *conn, struct timeval *tv)
{
  httpd_conn *hc = conn->hc;
  int nwritten;
  int nread;

  /* Read until the entire file is sent -- this could take awhile!! */

  while (conn->offset < conn->end_offset)
    {
      nvdbg("offset: %d end_offset: %d bytes_sent: %d\n",
            conn->offset, conn->end_offset, conn->hc->bytes_sent);

      /* Fill the rest of the response buffer with file data */

      nread = read_buffer(conn);
      if (nread < 0)
        {
          ndbg("File read error: %d\n", errno);
          goto errout_clear_connection;
        }
      nvdbg("Read %d bytes, buflen %d\n", nread, hc->buflen);

      /* Send the buffer */

      if (hc->buflen > 0)
        {
          /* httpd_write does not return until all bytes have been sent
           * (or an error occurs).
           */
 
          nwritten = httpd_write(hc->conn_fd, hc->buffer, hc->buflen);
          if (nwritten < 0)
            {
              ndbg("Error sending %s: %d\n", hc->encodedurl, errno);
              goto errout_clear_connection;
            }

          /* We wrote one full buffer of data (httpd_write does not
           * return until the full buffer is written (or an error occurs).
           */

          conn->active_at       = tv->tv_sec;
          hc->buflen            = 0;

          /* And update how much of the file we wrote */

          conn->offset         += nwritten;
          conn->hc->bytes_sent += nwritten;
          nvdbg("Wrote %d bytes\n", nwritten);
        }
    }

  /* The file transfer is complete -- finish the connection */

  nvdbg("Finish connection\n");
  finish_connection(conn, tv);
  return;

errout_clear_connection:
  ndbg("Clear connection\n");
  clear_connection(conn, tv);
  return;
}

static void handle_linger(struct connect_s *conn, struct timeval *tv)
{
  httpd_conn *hc = conn->hc;
  int ret;

    /* In lingering-close mode we just read and ignore bytes.  An error or EOF 
   * ends things, otherwise we go until a timeout 
   */

  ret = read(conn->hc->conn_fd, hc->buffer, CONFIG_THTTPD_IOBUFFERSIZE);
  if (ret < 0 && (errno == EINTR || errno == EAGAIN))
    {
      return;
    }

  if (ret <= 0)
    {
      really_clear_connection(conn);
    }
}

static void finish_connection(struct connect_s *conn, struct timeval *tv)
{
  /* If we haven't actually sent the buffered response yet, do so now */

  httpd_write_response(conn->hc);

  /* And clear */

  clear_connection(conn, tv);
}

static void clear_connection(struct connect_s *conn, struct timeval *tv)
{
  ClientData client_data;

  if (conn->wakeup_timer != NULL)
    {
      tmr_cancel(conn->wakeup_timer);
      conn->wakeup_timer = 0;
    }

  /* This is our version of Apache's lingering_close() routine, which is
   * their version of the often-broken SO_LINGER socket option.  For why
   * this is necessary, see http://www.apache.org/docs/misc/fin_wait_2.html
   * What we do is delay the actual closing for a few seconds, while reading
   * any bytes that come over the connection.  However, we don't want to do
   * this unless it's necessary, because it ties up a connection slot and
   * file descriptor which means our maximum connection-handling rateis
   * lower.  So, elsewhere we set a flag when we detect the few
   * circumstances that make a lingering close necessary.  If the flag isn't 
   * set we do the real close now.
   */

  if (conn->conn_state == CNST_LINGERING)
    {
      /* If we were already lingering, shut down for real */

      tmr_cancel(conn->linger_timer);
      conn->linger_timer      = NULL;
      conn->hc->should_linger = false;
    }
  else if (conn->hc->should_linger)
    {
      fdwatch_del_fd(fw, conn->hc->conn_fd);
      conn->conn_state = CNST_LINGERING;
      fdwatch_add_fd(fw, conn->hc->conn_fd, conn);
      client_data.p = conn;

      conn->linger_timer = tmr_create(tv, linger_clear_connection, client_data,
                                      CONFIG_THTTPD_LINGER_MSEC, 0);
      if (conn->linger_timer != NULL)
        {
          return;
        }
      ndbg("tmr_create(linger_clear_connection) failed\n");
    }

  /* Either we are done lingering, we shouldn't linger, or we failed to setup the linger */

  really_clear_connection(conn);
}

static void really_clear_connection(struct connect_s *conn)
{
  fdwatch_del_fd(fw, conn->hc->conn_fd);
  httpd_close_conn(conn->hc);
  if (conn->linger_timer != NULL)
    {
      tmr_cancel(conn->linger_timer);
      conn->linger_timer = 0;
    }

  /* Put the connection structure back on the free list */

  conn->conn_state  = CNST_FREE;
  conn->next        = free_connections;
  free_connections  = conn;
}

static void idle(ClientData client_data, struct timeval *nowP)
{
  int cnum;
  struct connect_s *conn;

  for (cnum = 0; cnum < AVAILABLE_FDS; ++cnum)
    {
      conn = &connects[cnum];
      switch (conn->conn_state)
        {
        case CNST_READING:
          if (nowP->tv_sec - conn->active_at >= CONFIG_THTTPD_IDLE_READ_LIMIT_SEC)
            {
              ndbg("%s connection timed out reading\n", httpd_ntoa(&conn->hc->client_addr));
              httpd_send_err(conn->hc, 408, httpd_err408title, "",
                             httpd_err408form, "");
              finish_connection(conn, nowP);
            }
          break;

        case CNST_SENDING:
          if (nowP->tv_sec - conn->active_at >= CONFIG_THTTPD_IDLE_SEND_LIMIT_SEC)
            {
              ndbg("%s connection timed out sending\n", httpd_ntoa(&conn->hc->client_addr));
              clear_connection(conn, nowP);
            }
          break;
        }
    }
}

static void linger_clear_connection(ClientData client_data, struct timeval *nowP)
{
  struct connect_s *conn;

  nvdbg("Clear connection\n");
  conn = (struct connect_s *) client_data.p;
  conn->linger_timer = NULL;
  really_clear_connection(conn);
}

static void occasional(ClientData client_data, struct timeval *nowP)
{
  tmr_cleanup();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: thttpd_main
 *
 * Description:
 *   This function is the entrypoint into the THTTPD server.  It does not
 *   return.  It may be called, the normal mechanism for starting the server
 *   is:
 *
 *   1) Set is g_thttpdsymtab and g_thttpdnsymbols.  The user is required
 *      to provide a symbol table to use for binding CGI programs (if CGI
 *      is enabled.  See examples/nxflat and examples/thttpd for examples of
 *      how such a symbol table may be created.
 *   2) Call task_create() to start thttpd_main()
 *
 ****************************************************************************/

int thttpd_main(int argc, char **argv)
{
  int num_ready;
  int cnum;
  FAR struct connect_s *conn;
  FAR httpd_conn *hc;
  httpd_sockaddr sa;
  struct timeval tv;
#ifdef CONFIG_THTTPD_DIR
  int ret;
#endif

  nvdbg("THTTPD started\n");

  /* Setup host address */

#ifdef  CONFIG_NET_IPv6
#  error "IPv6 support not yet implemented"
#else
  sa.sin_family      = AF_INET;
  sa.sin_port        = HTONS(CONFIG_THTTPD_PORT);
  sa.sin_addr.s_addr = HTONL(CONFIG_THTTPD_IPADDR);
#endif

  /* Initialize the fdwatch package to handle all of the configured
   * socket descriptors
   */

  fw = fdwatch_initialize(CONFIG_NSOCKET_DESCRIPTORS);
  if (!fw)
    {
      ndbg("fdwatch initialization failure\n");
      exit(1);
    }

  /* Switch directories again if requested */

#ifdef CONFIG_THTTPD_DATADIR
  if (chdir(CONFIG_THTTPD_DATADIR) < 0)
    {
      ndbg("chdir to %s: %d\n", CONFIG_THTTPD_DATADIR, errno);
      exit(1);
    }
#endif

  /* Initialize the timer package */

  tmr_init();

  /* Initialize the HTTP layer */

  nvdbg("Calling httpd_initialize()\n");
  hs = httpd_initialize(&sa);
  if (!hs)
    {
      ndbg("httpd_initialize() failed\n");
      exit(1);
    }

  /* Set up the occasional timer */

  if (tmr_create(NULL, occasional, JunkClientData, CONFIG_THTTPD_OCCASIONAL_MSEC * 1000L, 1) == NULL)
    {
      ndbg("tmr_create(occasional) failed\n");
      exit(1);
    }

  /* Set up the idle timer */

  if (tmr_create(NULL, idle, JunkClientData, 5 * 1000L, 1) == NULL)
    {
      ndbg("tmr_create(idle) failed\n");
      exit(1);

    }

  /* Initialize our connections table */

  connects = NEW(struct connect_s, AVAILABLE_FDS);
  if (connects == NULL)
    {
      ndbg("Out of memory allocating a struct connect_s\n");
      exit(1);
    }

  for (cnum = 0; cnum < AVAILABLE_FDS; ++cnum)
    {
      connects[cnum].conn_state  = CNST_FREE;
      connects[cnum].next        = &connects[cnum + 1];
      connects[cnum].hc          = NULL;
    }

  connects[AVAILABLE_FDS-1].next = NULL;      /* End of link list */
  free_connections               = connects;  /* Beginning of the link list */

  if (hs != NULL)
    {
      if (hs->listen_fd != -1)
        {
          fdwatch_add_fd(fw, hs->listen_fd, NULL);
        }
    }

  /* Main loop */

  nvdbg("Entering the main loop\n");
  (void)gettimeofday(&tv, NULL);
  for(;;)
    {
      /* Do the fd watch */

      num_ready = fdwatch(fw, tmr_mstimeout(&tv));
      if (num_ready < 0)
        {
          if (errno == EINTR || errno == EAGAIN)
            {
              /* Not errors... try again */

              continue;
            }

          ndbg("fdwatch failed: %d\n", errno);
          exit(1);
        }

      (void)gettimeofday(&tv, NULL);

      if (num_ready == 0)
        {
          /* No fd's are ready - run the timers */

          tmr_run(&tv);
          continue;
        }

      /* Is it a new connection? */

      if (fdwatch_check_fd(fw, hs->listen_fd))
        {
          if (!handle_newconnect(&tv, hs->listen_fd))
            {
              /* Go around the loop and do another fdwatch, rather than 
               * dropping through and processing existing connections.  New
               * connections always get priority.
               */

              continue;
            }
        }

      /* Find the connections that need servicing */

      while ((conn = (struct connect_s*)fdwatch_get_next_client_data(fw)) != (struct connect_s*)-1)
        {
          if (conn)
            {
              hc = conn->hc;
              if (fdwatch_check_fd(fw, hc->conn_fd))
                {
                  nvdbg("Handle conn_state %d\n", conn->conn_state);
                  switch (conn->conn_state)
                    {
                      case CNST_READING:
                        {
                          handle_read(conn, &tv);

                          /* If a GET request was received and a file is ready to
                           * be sent, then fall through to send the file.
                           */

                          if (conn->conn_state != CNST_SENDING)
                            { 
                              break;
                            }
                        }

                      case CNST_SENDING:
                        {
                          /* Send a file -- this really should be performed on a
                           * separate thread to keep the serve from locking up during
                           * the write.
                           */

                          handle_send(conn, &tv);
                        }
                        break;

                      case CNST_LINGERING:
                        {
                          /* Linger close the connection */

                          handle_linger(conn, &tv);
                        }
                        break;
                    }
                }
            }
        }
      tmr_run(&tv);
    }

  /* The main loop terminated */

  shut_down();
  ndbg("Exiting\n");
  exit(0);
}

#endif /* CONFIG_THTTPD */

