/****************************************************************************
 * netutils/thttpd/timers.c
 * FD watcher routines for poll()
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1999,2000 by Jef Poskanzer <jef@mail.acme.com>.
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

#include <stdint.h>
#include <stdlib.h>
#include <debug.h>
#include <poll.h>
#include <debug.h>

#include "config.h"
#include "thttpd_alloc.h"
#include "fdwatch.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Debug output from this file is normally suppressed.  If enabled, be aware
 * that output to stdout will interfere with CGI programs (you could use the
 * the low-level debug (lldbg) functions which probably do not use stdout
 */

#ifdef CONFIG_THTTPD_FDWATCH_DEBUG
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define fwdbg(format, arg...)    ndbg(format, ##arg)
#    define fwlldbg(format, arg...)  nlldbg(format, ##arg)
#    define fwvdbg(format, arg...)   nvdbg(format, ##arg)
#    define fwllvdbg(format, arg...) nllvdbg(format, ##arg)
#  else
#    define fwdbg    ndbg
#    define fwlldbg  nlldbg
#    define fwvdbg   nvdbg
#    define fwllvdbg nllvdbg
#  endif
#else
#  ifdef CONFIG_CPP_HAVE_VARARGS
#    define fwdbg(x...)
#    define fwlldbg(x...)
#    define fwvdbg(x...)
#    define fwllvdbg(x...)
#  else
#    define fwdbg    (void)
#    define fwlldbg  (void)
#    define fwvdbg   (void)
#    define fwllvdbg (void)
#  endif
#endif

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_THTTPD_FDWATCH_DEBUG
static void fdwatch_dump(const char *msg, FAR struct fdwatch_s *fw)
{
  int i;
 
  fwvdbg("%s\n", msg);
  fwvdbg("nwatched: %d nfds: %d\n", fw->nwatched, fw->nfds);
  for (i = 0; i < fw->nwatched; i++)
  {
    fwvdbg("%2d. pollfds: {fd: %d events: %02x revents: %02x} client: %p\n",
           i+1, fw->pollfds[i].fd, fw->pollfds[i].events,
           fw->pollfds[i].revents, fw->client[i]);
  }
  fwvdbg("nactive: %d next: %d\n", fw->nactive, fw->next);
  for (i = 0; i < fw->nactive; i++)
  {
    fwvdbg("%2d. %d active\n", i, fw->ready[i]);
  }
}
#else
#  define fdwatch_dump(m,f)
#endif

static int fdwatch_pollndx(FAR struct fdwatch_s *fw, int fd)
{
  int pollndx;

  /* Get the index associated with the fd */

  for (pollndx = 0; pollndx < fw->nwatched; pollndx++)
    {
      if (fw->pollfds[pollndx].fd == fd)
        {
          fwvdbg("pollndx: %d\n", pollndx);
          return pollndx;
        }
    }

  fwdbg("No poll index for fd %d: %d\n", fd);
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Initialize the fdwatch data structures.  Returns -1 on failure. */

struct fdwatch_s *fdwatch_initialize(int nfds)
{
  FAR struct fdwatch_s *fw;

  /* Allocate the fdwatch data structure */

  fw = (struct fdwatch_s*)zalloc(sizeof(struct fdwatch_s));
  if (!fw)
    {
      fwdbg("Failed to allocate fdwatch\n");
      return NULL;
    }

  /* Initialize the fdwatch data structures. */

  fw->nfds = nfds;

  fw->client = (void**)httpd_malloc(sizeof(void*) * nfds);
  if (!fw->client)
    {
      goto errout_with_allocations;
    }

  fw->pollfds = (struct pollfd*)httpd_malloc(sizeof(struct pollfd) * nfds);
  if (!fw->pollfds)
    {
      goto errout_with_allocations;
    }

  fw->ready = (uint8_t*)httpd_malloc(sizeof(uint8_t) * nfds);
  if (!fw->ready)
    {
      goto errout_with_allocations;
    }

  fdwatch_dump("Initial state:", fw);
  return fw;

errout_with_allocations:
  fdwatch_uninitialize(fw);
  return NULL;
}

/* Uninitialize the fwdatch data structure */

void fdwatch_uninitialize(struct fdwatch_s *fw)
{
  if (fw)
    {
      fdwatch_dump("Uninitializing:", fw);
      if (fw->client)
        {
          httpd_free(fw->client);
        }

      if (fw->pollfds)
        {
          httpd_free(fw->pollfds);
        }

      if (fw->ready)
        {
          httpd_free(fw->ready);
        }

      httpd_free(fw);
    }
}

/* Add a descriptor to the watch list.  rw is either FDW_READ or FDW_WRITE.  */

void fdwatch_add_fd(struct fdwatch_s *fw, int fd, void *client_data)
{
  fwvdbg("fd: %d client_data: %p\n", fd, client_data);
  fdwatch_dump("Before adding:", fw);

  if (fw->nwatched >= fw->nfds)
    {
      fwdbg("too many fds\n");
      return;
    }

  /* Save the new fd at the end of the list */

  fw->pollfds[fw->nwatched].fd     = fd;
  fw->pollfds[fw->nwatched].events = POLLIN;
  fw->client[fw->nwatched]         = client_data;

  /* Increment the count of watched descriptors */

  fw->nwatched++;
  fdwatch_dump("After adding:", fw);
}

/* Remove a descriptor from the watch list. */

void fdwatch_del_fd(struct fdwatch_s *fw, int fd)
{
  int pollndx;

  fwvdbg("fd: %d\n", fd);
  fdwatch_dump("Before deleting:", fw);

  /* Get the index associated with the fd */

  pollndx = fdwatch_pollndx(fw, fd);
  if (pollndx >= 0)
    {
      /* Decrement the number of fds in the poll table */

      fw->nwatched--;

      /* Replace the deleted one with the one at the the end
       * of the list.
       */

      if (pollndx != fw->nwatched)
        {
          fw->pollfds[pollndx] = fw->pollfds[fw->nwatched];
          fw->client[pollndx]  = fw->client[fw->nwatched];
        }
    }
   fdwatch_dump("After deleting:", fw);
}

/* Do the watch.  Return value is the number of descriptors that are ready,
 * or 0 if the timeout expired, or -1 on errors.  A timeout of INFTIM means
 * wait indefinitely.
 */

int fdwatch(struct fdwatch_s *fw, long timeout_msecs)
{
  int ret;
  int i;

  /* Wait for activity on any of the desciptors.  When poll() returns, ret
   * will hold the number of descriptors with activity (or zero on a timeout
   * or <0 on an error.
   */

  fdwatch_dump("Before waiting:", fw);
  fwvdbg("Waiting... (timeout %d)\n", timeout_msecs);
  fw->nactive = 0;
  fw->next    = 0;
  ret         = poll(fw->pollfds, fw->nwatched, (int)timeout_msecs);
  fwvdbg("Awakened: %d\n", ret);

  /* Look through all of the descriptors and make a list of all of them than
   * have activity.
   */

  if (ret > 0)
    {
      for (i = 0; i < fw->nwatched; i++)
        {
          /* Is there activity on this descriptor? */

          if (fw->pollfds[i].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL))
            {
              /* Yes... save it in a shorter list */

              fwvdbg("pollndx: %d fd: %d revents: %04x\n",
                    i, fw->pollfds[i].fd, fw->pollfds[i].revents);

              fw->ready[fw->nactive++] = fw->pollfds[i].fd;
              if (fw->nactive == ret)
                {
                  /* We have all of them, break out early */

                  break;
                }
            }
        }
    }

  /* Return the number of descriptors with activity */

  fwvdbg("nactive: %d\n", fw->nactive);
  fdwatch_dump("After wakeup:", fw);
  return ret;
}

/* Check if a descriptor was ready. */

int fdwatch_check_fd(struct fdwatch_s *fw, int fd)
{
  int pollndx;

  fwvdbg("fd: %d\n", fd);
  fdwatch_dump("Checking:", fw);

  /* Get the index associated with the fd */

  pollndx = fdwatch_pollndx(fw, fd);
  if (pollndx >= 0 && (fw->pollfds[pollndx].revents & POLLERR) == 0)
    {
      return fw->pollfds[pollndx].revents & (POLLIN | POLLHUP | POLLNVAL);
    }

  fwvdbg("POLLERR fd: %d\n", fd);
  return 0;
}

void *fdwatch_get_next_client_data(struct fdwatch_s *fw)
{
  fdwatch_dump("Before getting client data:", fw);
  if (fw->next >= fw->nwatched)
    {
      fwvdbg("All client data returned: %d\n", fw->next);
      return (void*)-1;
    }

  fwvdbg("client_data[%d]: %p\n", fw->next, fw->client[fw->next]);
  return fw->client[fw->next++];
}

#endif /* CONFIG_THTTPD */

