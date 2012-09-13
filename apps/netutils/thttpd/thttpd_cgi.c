/****************************************************************************
 * netutils/thttpd/thttpd_cgi.c
 * CGI support
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file libhttpd.c in the original THTTPD package:
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/symtab.h>
#include <nuttx/binfmt.h>
#include <apps/netutils/thttpd.h>

#include "config.h"
#include "libhttpd.h"
#include "thttpd_alloc.h"
#include "thttpd_strings.h"
#include "timers.h"
#include "fdwatch.h"

#if defined(CONFIG_THTTPD) && defined(CONFIG_THTTPD_CGI_PATTERN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_THTTPD_CGIDUMP will dump the contents of each transfer to and from the CGI task. */

#ifdef CONFIG_THTTPD_CGIDUMP
#  define cgi_dumpbuffer(m,a,n) lib_dumpbuffer(m,(FAR const uint8_t*)a,n)
#else
#  define cgi_dumpbuffer(m,a,n)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum cgi_outbuffer_e
{
  CGI_OUTBUFFER_READHEADER = 0,  /* Reading header from HTTP client */
  CGI_OUTBUFFER_HEADERREAD,      /* Header has been read */
  CGI_OUTBUFFER_HEADERSENT,      /* Header has been sent to the CGI program */
  CGI_OUTBUFFER_READDATA,        /* Transferring data from CGI to client */
  CGI_OUTBUFFER_DONE,            /* Finished */
};

struct cgi_outbuffer_s
{
  enum cgi_outbuffer_e state;    /* State of the transfer */
  char  *buffer;                 /* Allocated I/O buffer */
  size_t size;                   /* Size of the allocation */
  size_t len;                    /* Amount of valid data in the allocated buffer */
};

struct cgi_inbuffer_s
{
  int    contentlength;			/* Size of content to send to CGI task */
  int    nbytes;				/* Number of bytes sent */
  char   buffer[CONFIG_THTTPD_CGIINBUFFERSIZE]; /* Fixed size input buffer */
};

struct cgi_conn_s
{
  /* Descriptors */

  int    connfd;                 /* Socket connect to CGI client */
  int    rdfd;                   /* Pipe read fd */
  int    wrfd;                   /* Pipe write fd */

  /* Buffering */

  struct cgi_outbuffer_s outbuf; /* Dynamically sized output buffer */
  struct cgi_inbuffer_s inbuf;   /* Fixed size input buffer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void create_environment(httpd_conn *hc);
static char **make_argp(httpd_conn *hc);
static inline int cgi_interpose_input(struct cgi_conn_s *cc);
static inline int cgi_interpose_output(struct cgi_conn_s *cc);
static int  cgi_child(int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Used to hold off the main task until the CGI tasks have been configured */

static sem_t g_cgisem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* semaphore helpers */

static inline void cgi_semtake(void)
{
  while (sem_wait(&g_cgisem) != 0)
    {
      /* The only case that an error should occr here is if the wait was\
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void cgi_semgive(void)
{
  sem_post(&g_cgisem);
}

/* Set up environment variables. Be real careful here to avoid
 * letting malicious clients overrun a buffer.  We don't have
 * to worry about freeing stuff since we're a sub-task.
 */

static void create_environment(httpd_conn *hc)
{
  char *cp;
  char buf[256];

  setenv("PATH", CONFIG_THTTPD_CGI_PATH, TRUE);
#ifdef CGI_LD_LIBRARY_PATH
  setenv("LD_LIBRARY_PATH", CGI_LD_LIBRARY_PATH, TRUE);
#endif /* CGI_LD_LIBRARY_PATH */

  setenv("SERVER_SOFTWARE",  CONFIG_THTTPD_SERVER_SOFTWARE, TRUE);

  /* If vhosting, use that server-name here. */
#ifdef CONFIG_THTTPD_VHOST
  if (hc->vhostname)
    {
      cp = hc->vhostname;
    }
  else
#endif
    {
      cp = hc->hs->hostname;
    }

  if (cp)
    {
      setenv("SERVER_NAME", cp, TRUE);
    }

  setenv("GATEWAY_INTERFACE", "CGI/1.1", TRUE);
  setenv("SERVER_PROTOCOL", hc->protocol, TRUE);

  (void)snprintf(buf, sizeof(buf), "%d", (int)CONFIG_THTTPD_PORT);
  setenv("SERVER_PORT", buf, TRUE);

  setenv("REQUEST_METHOD", httpd_method_str(hc->method), TRUE);

  if (hc->pathinfo[0] != '\0')
    {
      char *cp2;
      size_t l;

      (void)snprintf(buf, sizeof(buf), "/%s", hc->pathinfo);
      setenv("PATH_INFO", buf, TRUE);

      l = strlen(httpd_root) + strlen(hc->pathinfo) + 1;
      cp2 = NEW(char, l);
      if (cp2)
        {
          (void)snprintf(cp2, l, "%s%s", httpd_root, hc->pathinfo);
          setenv("PATH_TRANSLATED", cp2, TRUE);
        }
    }

  (void)snprintf(buf, sizeof(buf), "/%s",strcmp(hc->origfilename, ".") == 0 ? "" : hc->origfilename);
  setenv("SCRIPT_NAME", buf, TRUE);

  if (hc->query[0] != '\0')
    {
      setenv("QUERY_STRING", hc->query, TRUE);
    }

  setenv("REMOTE_ADDR", httpd_ntoa(&hc->client_addr), TRUE);
  if (hc->referer[0] != '\0')
    {
      setenv("HTTP_REFERER", hc->referer, TRUE);
    }

  if (hc->useragent[0] != '\0')
    {
      setenv("HTTP_USER_AGENT", hc->useragent, TRUE);
    }

  if (hc->accept[0] != '\0')
    {
      setenv("HTTP_ACCEPT", hc->accept, TRUE);
    }

  if (hc->accepte[0] != '\0')
    {
      setenv("HTTP_ACCEPT_ENCODING", hc->accepte, TRUE);
    }

  if (hc->acceptl[0] != '\0')
    {
      setenv("HTTP_ACCEPT_LANGUAGE", hc->acceptl, TRUE);
    }

  if (hc->cookie[0] != '\0')
    {
      setenv("HTTP_COOKIE", hc->cookie, TRUE);
    }

  if (hc->contenttype[0] != '\0')
    {
      setenv("CONTENT_TYPE", hc->contenttype, TRUE);
    }

  if (hc->hdrhost[0] != '\0')
    {
      setenv("HTTP_HOST", hc->hdrhost, TRUE);
    }

  if (hc->contentlength != -1)
    {
      (void)snprintf(buf, sizeof(buf), "%lu", (unsigned long)hc->contentlength);
      setenv("CONTENT_LENGTH", buf, TRUE);
    }

  if (hc->remoteuser[0] != '\0')
    {
      setenv("REMOTE_USER", hc->remoteuser, TRUE);
    }

  if (hc->authorization[0] != '\0')
    {
      setenv("AUTH_TYPE", "Basic", TRUE);
    }

  /* We only support Basic auth at the moment. */

  if (getenv("TZ") != NULL)
    {
      setenv("TZ", getenv("TZ"), TRUE);
    }

  setenv("CGI_PATTERN", CONFIG_THTTPD_CGI_PATTERN, TRUE);
}

/* Set up argument vector */

static FAR char **make_argp(httpd_conn *hc)
{
  FAR char **argp;
  int argn;
  char *cp1;
  char *cp2;

  /* By allocating an arg slot for every character in the query, plus one
   * for the filename and one for the NULL, we are guaranteed to have
   * enough.  We could actually use strlen/2.
   */

  argp = NEW(char *, strlen(hc->query) + 2);
  if (!argp)
    {
      return NULL;
    }

  argp[0] = strrchr(hc->expnfilename, '/');
  if (argp[0])
    {
      ++argp[0];
    }
  else
    {
      argp[0] = hc->expnfilename;
    }
  argn = 1;

  /* According to the CGI spec at http://hoohoo.ncsa.uiuc.edu/cgi/cl.html,
   * "The server should search the query information for a non-encoded =
   * character to determine if the command line is to be used, if it finds
   * one, the command line is not to be used."
   */

  if (strchr(hc->query, '=') == NULL)
    {
      for (cp1 = cp2 = hc->query; *cp2 != '\0'; ++cp2)
        {
          if (*cp2 == '+')
            {
              *cp2 = '\0';
              httpd_strdecode(cp1, cp1);
              argp[argn++] = cp1;
              cp1 = cp2 + 1;
            }
        }

      if (cp2 != cp1)
        {
          httpd_strdecode(cp1, cp1);
          argp[argn++] = cp1;
        }
    }

  argp[argn] = NULL;
  return argp;
}

/* Data is available from the client socket. This routine is used only for POST
 * requests.  It reads the data from the client and sends it to the child thread.
 */

static inline int cgi_interpose_input(struct cgi_conn_s *cc)
{
  ssize_t nbytes_read;
  ssize_t nbytes_written;

  nllvdbg("nbytes: %d contentlength: %d\n", cc->inbuf.nbytes, cc->inbuf.contentlength);
  if (cc->inbuf.nbytes < cc->inbuf.contentlength)
    {
      do
        {
          nbytes_read = read(cc->connfd, cc->inbuf.buffer,
            MIN(CONFIG_THTTPD_CGIINBUFFERSIZE, cc->inbuf.contentlength - cc->inbuf.nbytes));
          nllvdbg("nbytes_read: %d\n", nbytes_read);
          if (nbytes_read < 0)
            {
              if (errno != EINTR)
                {
                  nlldbg("read failed: %d\n", errno);
                  return 1;
                }
            }
        }
      while (nbytes_read < 0);

      if (nbytes_read > 0)
        {
          nbytes_written = httpd_write(cc->wrfd, cc->inbuf.buffer, nbytes_read);
          nllvdbg("nbytes_written: %d\n", nbytes_written);
          if (nbytes_written != nbytes_read)
            {
              nlldbg("httpd_write failed\n");
              return 1;
            }
          cgi_dumpbuffer("Sent to CGI:", cc->inbuf.buffer, nbytes_written);
        }

      cc->inbuf.nbytes += nbytes_read;
    }

  if (cc->inbuf.nbytes >= cc->inbuf.contentlength)
    {
      /* Special hack to deal with broken browsers that send a LF or CRLF
       * after POST data, causing TCP resets - we just read and discard up
       * to 2 bytes.  Unfortunately this doesn't fix the problem for CGIs
       * which avoid the interposer task due to their POST data being
       * short.  Creating an interposer task for all POST CGIs is
       * unacceptably expensive.  The eventual fix will come when interposing
       * gets integrated into the main loop as a tasklet instead of a task.
       */

      /* Turn on no-delay mode in case we previously cleared it. */

      httpd_set_ndelay(cc->connfd);

      /* And read up to 2 bytes. */

      (void)read(cc->connfd, cc->inbuf.buffer, CONFIG_THTTPD_CGIINBUFFERSIZE);
      return 1;
    }
  return 0;
}

/* This routine is used for parsed-header CGIs.  The idea here is that the
 * CGI can return special headers such as "Status:" and "Location:" which
 * change the return status of the response.  Since the return status has to
 * be the very first line written out, we have to accumulate all the headers
 * and check for the special ones before writing the status.  Then we write
 * out the saved headers and proceed to echo the rest of the response.
 */

static inline int cgi_interpose_output(struct cgi_conn_s *cc)
{
  ssize_t nbytes_read;
  char *br = NULL;
  int status;
  const char *title;
  char *cp;

  /* Make sure the connection is in blocking mode.  It should already be
   * blocking, but we might as well be sure.
   */

  httpd_clear_ndelay(cc->connfd);

  /* Loop while there are things we can do without waiting for more input */

  nllvdbg("state: %d\n", cc->outbuf.state);
  switch (cc->outbuf.state)
    {
      case CGI_OUTBUFFER_READHEADER:
        {
          /* Slurp in all headers as they become available from the client. */

          do
            {
              /* Read until we successfully read data or until an error occurs.
               * EAGAIN is not an error, but it is still cause to return.
               */

              nbytes_read = read(cc->rdfd, cc->inbuf.buffer, CONFIG_THTTPD_CGIINBUFFERSIZE);
              nllvdbg("Read %d bytes from fd %d\n", nbytes_read, cc->rdfd);

              if (nbytes_read < 0)
                {
                  if (errno != EINTR)
                    {
                      if (errno != EAGAIN)
                        {
                          nlldbg("read: %d\n", errno);
                        }
                      return 1;
                    }
                }
              else
                {
                  cgi_dumpbuffer("Received from CGI:", cc->inbuf.buffer, nbytes_read);
                }
            }
          while (nbytes_read < 0);

          /* Check for end-of-file */

          if (nbytes_read <= 0)
            {
              nllvdbg("End-of-file\n");
              br               = &(cc->outbuf.buffer[cc->outbuf.len]);
              cc->outbuf.state = CGI_OUTBUFFER_HEADERREAD;
            }
          else
            {
              /* Accumulate more header data */

              httpd_realloc_str(&cc->outbuf.buffer, &cc->outbuf.size, cc->outbuf.len + nbytes_read);
              (void)memcpy(&(cc->outbuf.buffer[cc->outbuf.len]), cc->inbuf.buffer, nbytes_read);
              cc->outbuf.len                   += nbytes_read;
              cc->outbuf.buffer[cc->outbuf.len] = '\0';
              nllvdbg("Header bytes accumulated: %d\n", cc->outbuf.len);

              /* Check for end of header */

              if ((br = strstr(cc->outbuf.buffer, "\r\n\r\n")) != NULL ||
                  (br = strstr(cc->outbuf.buffer, "\012\012")) != NULL)
                {
                  nllvdbg("End-of-header\n");
                  cc->outbuf.state = CGI_OUTBUFFER_HEADERREAD;
                }
              else
                {
                  /* All of the headers have not yet been read ... Return.  We
                   * will be called again when more data is available in the pipe
                   * connected to the CGI task.
                   */

                  return 0;
                }
            }
        }

        /* Otherwise, fall through and parse status in the HTTP headers */

      case CGI_OUTBUFFER_HEADERREAD:
        {
          /* If there were no headers, bail. */

          if (cc->outbuf.buffer[0] == '\0')
            {
              cc->outbuf.state = CGI_OUTBUFFER_DONE;
              return 1;
            }

          /* Figure out the status.  Look for a Status: or Location: header; else if 
           * there's an HTTP header line, get it from there; else default to 200.
           */

          status = 200;
          if (strncmp(cc->outbuf.buffer, "HTTP/", 5) == 0)
            {
              cp     = cc->outbuf.buffer;
              cp    += strcspn(cp, " \t");
              status = atoi(cp);
            }

          if ((cp = strstr(cc->outbuf.buffer, "Status:")) != NULL &&
              cp < br && (cp == cc->outbuf.buffer || *(cp - 1) == '\012'))
            {
              cp    += 7;
              cp    += strspn(cp, " \t");
              status = atoi(cp);
            }

          if ((cp = strstr(cc->outbuf.buffer, "Location:")) != NULL &&
              cp < br && (cp == cc->outbuf.buffer || *(cp - 1) == '\012'))
            {
              status = 302;
            }

          /* Write the status line. */

          nllvdbg("Status: %d\n", status);
          switch (status)
            {
            case 200:
              title = ok200title;
              break;

            case 302:
              title = err302title;
              break;

            case 304:
              title = err304title;
              break;

            case 400:
              BADREQUEST("status");
              title = httpd_err400title;
              break;

#ifdef CONFIG_THTTPD_AUTH_FILE
            case 401:
              title = err401title;
              break;
#endif

            case 403:
              title = err403title;
              break;

            case 404:
              title = err404title;
             break;

            case 408:
              title = httpd_err408title;
              break;

            case 500:
              INTERNALERROR("status");
              title = err500title;
              break;

            case 501:
              NOTIMPLEMENTED("status");
              title = err501title;
              break;

            case 503:
              title = httpd_err503title;
              break;

            default:
              title = "Something";
              break;
            }

          (void)snprintf(cc->inbuf.buffer, CONFIG_THTTPD_CGIINBUFFERSIZE, "HTTP/1.0 %d %s\r\n", status, title);
          (void)httpd_write(cc->connfd, cc->inbuf.buffer, strlen(cc->inbuf.buffer));

          /* Write the saved cc->outbuf.buffer to the client. */

          (void)httpd_write(cc->connfd, cc->outbuf.buffer, cc->outbuf.len);
        }

        /* Then set up to read the data following the header from the CGI program and
         * pass it back to the client. We return now; we will be called again when
         * data is available on the pipe.
         */

        cc->outbuf.state = CGI_OUTBUFFER_READDATA;
        return 0;

      case CGI_OUTBUFFER_READDATA:
        {
          /* Read data from the pipe. */

          do
            {
              /* Read until we successfully read data or until an error occurs.
               * EAGAIN is not an error, but it is still cause to return.
               */

              nbytes_read = read(cc->rdfd, cc->inbuf.buffer, CONFIG_THTTPD_CGIINBUFFERSIZE);
              nllvdbg("Read %d bytes from fd %d\n", nbytes_read, cc->rdfd);

              if (nbytes_read < 0)
                {
                  if (errno != EINTR)
                    {
                      if (errno != EAGAIN)
                        {
                          nlldbg("read: %d\n", errno);
                        }
                      return 1;
                    }
                }
              else
                {
                  cgi_dumpbuffer("Received from CGI:", cc->inbuf.buffer, nbytes_read);
                }
            }
          while (nbytes_read < 0);

          /* Check for end of file */

          if (nbytes_read == 0)
            {
              nllvdbg("End-of-file\n");
              cc->outbuf.state = CGI_OUTBUFFER_DONE;
              return 1;
            }
          else
            {
               /* Forward the data from the CGI program to the client */

             (void)httpd_write(cc->connfd, cc->inbuf.buffer, nbytes_read);
            }
        }
        break;

      case CGI_OUTBUFFER_DONE:
      default:
        return 1;
    }
  return 0;
}

/* CGI child task. */

static int cgi_child(int argc, char **argv)
{
  FAR httpd_conn *hc = (FAR httpd_conn*)strtoul(argv[1], NULL, 16);
#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
  ClientData client_data;
#endif
  FAR char **argp;
  FAR struct cgi_conn_s *cc;
  FAR struct fdwatch_s *fw;
  FAR char  *directory;
  FAR char  *dupname;
  bool       indone;
  bool       outdone;
  int        child;
  int        pipefd[2];
  int        nbytes;
  int        fd;
  int        ret;
  int        err = 1;

  /* Use low-level debug out (because the low-level output may survive closing
   * all file descriptors
   */

  nllvdbg("Started: %s\n", argv[1]);

  /* Allocate memory and initialize memory for interposing */

  cc = (FAR struct cgi_conn_s*)httpd_malloc(sizeof(struct cgi_conn_s));
  if (!cc)
    {
      nlldbg("cgi_conn allocation failed\n");
      close(hc->conn_fd);
      goto errout;
    }

  cc->connfd = hc->conn_fd;
  cc->wrfd   = -1;
  cc->rdfd   = -1;
  memset(&cc->outbuf, 0, sizeof(struct cgi_outbuffer_s));

  /* Update all of the environment variable settings, these will be inherited
   * by the CGI task.
   */

  create_environment(hc);

  /* Make the argument vector. */

  argp = make_argp(hc);

  /* Close all file descriptors EXCEPT for stdin, stdout, stderr and
   * hc->conn_fd.  We'll keep stderr open for error reporting; stdin and
   * stdout will be closed later by dup2().  Keeping stdin and stdout open
   * now prevents re-use of fd=0 and 1 by pipe().
   */

  nllvdbg("Closing descriptors\n");
  for (fd = 3; fd < (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS); fd++)
    {
       /* Keep hc->conn_fd open for obvious reasons */

       if (fd != hc->conn_fd)
         {
           close(fd);
         }
    }

  /* Create pipes that will be interposed between the CGI task's stdin or
   * stdout and the socket.
   *
   * Setup up the STDIN pipe - a pipe to transfer data received on the
   * socket to the CGI program.
   */

  nllvdbg("Create STDIN pipe\n");
  ret = pipe(pipefd);
  if (ret < 0)
    {
      nlldbg("STDIN pipe: %d\n", errno);
      goto errout_with_cgiconn;
    }
  else
    {
      /* Then map the receiving end the pipe to stdin, save the sending end, and
       * closing the original receiving end
       */

      ret = dup2(pipefd[0], 0);

      cc->wrfd = pipefd[1];
      close(pipefd[0]);

      if (ret < 0)
        {
          nlldbg("STDIN dup2: %d\n", errno);
          goto errout_with_descriptors;
        }
    }

  /* Set up the STDOUT pipe - a pipe to transfer data received from the CGI program
   * to the client.
   */

  if (ret == 0)
    {
      nllvdbg("Create STDOUT pipe\n");
      ret = pipe(pipefd);
      if (ret < 0)
        {
          nlldbg("STDOUT pipe: %d\n", errno);
          goto errout_with_descriptors;
        }
      else
        {
          /* Then map the sending end the pipe to stdout, save the receiving end, and
           * closing the original sending end
           */

          ret = dup2(pipefd[1], 1);

          cc->rdfd = pipefd[0];
          close(pipefd[1]);
 
          if (ret < 0)
            {
              nlldbg("STDOUT dup2: %d\n", errno);
              goto errout_with_descriptors;
            }
        }
    }

  /* chdir to the directory containing the binary. This isn't in the CGI 1.1
   * spec, but it's what  other HTTP servers do.
   */

  dupname = httpd_strdup(hc->expnfilename);
  if (dupname)
    {
      directory = dirname(dupname);
      if (directory)
        {
          (void)chdir(directory); /* ignore errors */
        }
      httpd_free(dupname);
    }

  /* Allocate memory for output buffering */

  httpd_realloc_str(&cc->outbuf.buffer, &cc->outbuf.size, CONFIG_THTTPD_CGIOUTBUFFERSIZE);
  if (!cc->outbuf.buffer)
    {
      nlldbg("hdr allocation failed\n");
      goto errout_with_descriptors;
    }

  /* Create fdwatch structures */

  fw = fdwatch_initialize(2);
  if (!fw)
    {
      nlldbg("fdwatch allocation failed\n");
      goto errout_with_outbuffer;
    }

  /* Run the CGI program. */

  nllvdbg("Starting CGI: %s\n", hc->expnfilename);
  child = exec(hc->expnfilename, (FAR const char **)argp, g_thttpdsymtab, g_thttpdnsymbols);
  if (child < 0)
    {
      /* Something went wrong. */

      nlldbg("execve %s: %d\n", hc->expnfilename, errno);
      goto errout_with_watch;
   }

  /* Schedule a kill for the child task in case it runs too long. */

#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
  client_data.i = child;
  if (tmr_create(NULL, cgi_kill, client_data, CONFIG_THTTPD_CGI_TIMELIMIT * 1000L, 0) == NULL)
    {
      nlldbg("tmr_create(cgi_kill child) failed\n");
      goto errout_with_watch;
    }
#endif

  /* Add the read descriptors to the watch */

  fdwatch_add_fd(fw, cc->connfd, NULL);
  fdwatch_add_fd(fw, cc->rdfd, NULL);

  /* Send any data that is already buffer to the CGI task */

  nbytes = hc->read_idx - hc->checked_idx;
  nllvdbg("nbytes: %d contentlength: %d\n", nbytes, hc->contentlength);
  if (nbytes > 0)
    {
      if (httpd_write(cc->wrfd, &(hc->read_buf[hc->checked_idx]), nbytes) != nbytes)
        {
          nlldbg("httpd_write failed\n");
          return 1;
        }
    }

  cc->inbuf.contentlength = hc->contentlength;
  cc->inbuf.nbytes        = nbytes;

  /* Then perform the interposition */

  indone  = false;
  outdone = false;

  nllvdbg("Interposing\n");
  cgi_semgive();  /* Not safe to reference hc after this point */
  do
    {
      (void)fdwatch(fw, 1000);

      /* Check for incoming data from the remote client to the CGI task */

      if (!indone && fdwatch_check_fd(fw, cc->connfd))
        {
          /* Transfer data from the client to the CGI program (POST) */

          nllvdbg("Interpose input\n");
          indone = cgi_interpose_input(cc);
          if (indone)
            {
              fdwatch_del_fd(fw, cc->connfd);
            }
        }

      /* Check for outgoing data from the CGI task to the remote client */

      if (fdwatch_check_fd(fw, cc->rdfd))
        {
          /* Handle receipt of headers and CGI program response (GET) */

          nllvdbg("Interpose output\n");
          outdone = cgi_interpose_output(cc);
        }

      /* No outgoing data... is the child task still running?   Use kill()
       * kill() with signal number == 0 does not actually send a signal, but
       * can be used to check if the target task exists.  If the task exists
       * but is hung, then you might enable CONFIG_THTTPD_CGI_TIMELIMIT to
       * kill the task.  However, killing the task could cause other problems
       * (consider resetting the microprocessor instead).
       */

      else if (kill(child, 0) != 0)
        {
          nllvdbg("CGI no longer running: %d\n", errno);
          outdone = true;
        }
  }
  while (!outdone);
  err = 0;

  /* Get rid of watch structures */

errout_with_watch:
  fdwatch_uninitialize(fw);

  /* Free output buffer memory */

errout_with_outbuffer:
  httpd_free(cc->outbuf.buffer);

  /* Close all descriptors */

errout_with_descriptors:
  close(cc->wrfd);
  close(cc->rdfd);

errout_with_cgiconn:
  close(cc->connfd);
  httpd_free(cc);

errout:
  nllvdbg("Return %d\n", err);
  if (err != 0)
    {
      INTERNALERROR("errout");
      httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
      httpd_write_response(hc);
      cgi_semgive();
    }
  return err;
}

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

int cgi(httpd_conn *hc)
{
  char arg[16];
  char *argv[2];
  pid_t child;
  int   retval = ERROR;

  /* Set up a semaphore to hold off the make THTTPD thread until the CGI
   * threads are configured (basically until the file descriptors are all
   * dup'ed and can be closed by the main thread).
   */

  sem_init(&g_cgisem, 0, 0);

  if (hc->method == METHOD_GET || hc->method == METHOD_POST)
    {
#ifdef CONFIG_THTTPD_CGILIMIT
      if (hc->hs->cgi_count >= CONFIG_THTTPD_CGILIMIT)
        {
          httpd_send_err(hc, 503, httpd_err503title, "", httpd_err503form,
                         hc->encodedurl);
          goto errout_with_sem;
        }
#endif
      ++hc->hs->cgi_count;
      httpd_clear_ndelay(hc->conn_fd);

      /* Start the child task.  We use a trampoline task here so that we can
       * safely muck with the file descriptors before actually started the CGI
       * task.
       */

      snprintf(arg, 16, "%p", hc); /* task_create doesn't handle binary arguments. */
      argv[0] = arg;
      argv[1] = NULL;

#ifndef CONFIG_CUSTOM_STACK
      child = task_create("CGI child", CONFIG_THTTPD_CGI_PRIORITY,
                          CONFIG_THTTPD_CGI_STACKSIZE,
                          (main_t)cgi_child, (const char **)argv);
#else
      child = task_create("CGI child", CONFIG_THTTPD_CGI_PRIORITY,
                          (main_t)cgi_child, (const char **)argv);
#endif
      if (child < 0)
        {
          ndbg("task_create: %d\n", errno);
          INTERNALERROR("task_create");
          httpd_send_err(hc, 500, err500title, "", err500form, hc->encodedurl);
          goto errout_with_sem;
        }

      ndbg("Started CGI task %d for file '%s'\n", child, hc->expnfilename);

      /* Wait for the CGI threads to become initialized */

      cgi_semtake();

      hc->bytes_sent    = CONFIG_THTTPD_CGI_BYTECOUNT;
      hc->should_linger = false;
    }
  else
    {
      NOTIMPLEMENTED("CGI");
      httpd_send_err(hc, 501, err501title, "", err501form, httpd_method_str(hc->method));
      goto errout_with_sem;
    }

  /* Successfully started */

  retval = OK;

errout_with_sem:
  sem_destroy(&g_cgisem);
  return retval;
}

#if CONFIG_THTTPD_CGI_TIMELIMIT > 0
static void cgi_kill(ClientData client_data, struct timeval *nowP)
{
  pid_t pid = (pid_t)client_data.i;

  /* task_delete() is a very evil API.  It can leave memory stranded! */

  nlldbg("Killing CGI child: %d\n", pid);
  if (task_delete(pid) != 0)
    {
      nlldbg("task_delete() failed: %d\n", errno);
    }
}
#endif

#endif /* CONFIG_THTTPD && CONFIG_THTTPD_CGI_PATTERN */

