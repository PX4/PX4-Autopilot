/****************************************************************************
 * httpd_cgi.c
 * Web server script interface
 * Author: Adam Dunkels <adam@sics.se>
 *
 * Copyright (c) 2001-2006, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>

#include <nuttx/compiler.h>

#include <nuttx/net/uip/uip.h>
#include <apps/netutils/httpd.h>

#include "httpd_cgi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nullfunction(struct httpd_state *pstate, char *ptr);
#ifdef CONFIG_NETUTILS_HTTPDNETSTATS
static void net_stats(struct httpd_state *pstate, char *ptr);
#endif

#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
static void file_stats(struct httpd_state *pstate, char *ptr);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
HTTPD_CGI_CALL(file, "file-stats", file_stats);
#endif
#ifdef CONFIG_NETUTILS_HTTPDNETSTATS
HTTPD_CGI_CALL(net, "net-stats", net_stats);
#endif

static const struct httpd_cgi_call *calls[] =
{
#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
  &file,
#endif
#ifdef CONFIG_NETUTILS_HTTPDNETSTATS
  &net,
#endif
  NULL
};

static const char closed[] =   /*  "CLOSED",*/
  {0x43, 0x4c, 0x4f, 0x53, 0x45, 0x44, 0};
static const char syn_rcvd[] = /*  "SYN-RCVD",*/
  {0x53, 0x59, 0x4e, 0x2d, 0x52, 0x43, 0x56, 0x44,  0};
static const char syn_sent[] = /*  "SYN-SENT",*/
  {0x53, 0x59, 0x4e, 0x2d, 0x53, 0x45, 0x4e, 0x54,  0};
static const char established[] = /*  "ESTABLISHED",*/
  {0x45, 0x53, 0x54, 0x41, 0x42, 0x4c, 0x49, 0x53, 0x48, 0x45, 0x44, 0};
static const char fin_wait_1[] = /*  "FIN-WAIT-1",*/
  {0x46, 0x49, 0x4e, 0x2d, 0x57, 0x41, 0x49, 0x54, 0x2d, 0x31, 0};
static const char fin_wait_2[] = /*  "FIN-WAIT-2",*/
  {0x46, 0x49, 0x4e, 0x2d, 0x57, 0x41, 0x49, 0x54, 0x2d, 0x32, 0};
static const char closing[] = /*  "CLOSING",*/
  {0x43, 0x4c, 0x4f, 0x53, 0x49, 0x4e, 0x47, 0};
static const char time_wait[] = /*  "TIME-WAIT,"*/
  {0x54, 0x49, 0x4d, 0x45, 0x2d, 0x57, 0x41, 0x49, 0x54, 0};
static const char last_ack[] = /*  "LAST-ACK"*/
  {0x4c, 0x41, 0x53, 0x54, 0x2d, 0x41, 0x43, 0x4b, 0};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nullfunction
 ****************************************************************************/

static void nullfunction(struct httpd_state *pstate, char *ptr)
{
}

/****************************************************************************
 * Name: net_stats
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_HTTPDNETSTATS
static void net_stats(struct httpd_state *pstate, char *ptr)
{
  char buffer[16];
  int i;

  for (i = 0; i < sizeof(uip_stat) / sizeof(uip_stats_t); i++)
    {
      snprintf(buffer, 16, "%5u\n", ((uip_stats_t *)&uip_stat)[i]);
      send(pstate->ht_sockfd, buffer, strlen(buffer), 0);
    }
}
#endif

/****************************************************************************
 * Name: file_stats
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_HTTPDFILESTATS
static void file_stats(struct httpd_state *pstate, char *ptr)
{
  char buffer[16];
  char *pcount = strchr(ptr, ' ') + 1;
  snprintf(buffer, 16, "%5u", httpd_fs_count(pcount));
  (void)send(pstate->ht_sockfd, buffer, strlen(buffer), 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

httpd_cgifunction httpd_cgi(char *name)
{
  const struct httpd_cgi_call **f;

  /* Find the matching name in the table, return the function. */

  for(f = calls; *f != NULL; ++f)
    {
      if(strncmp((*f)->name, name, strlen((*f)->name)) == 0)
        {
          return (*f)->function;
        }
    }
  return nullfunction;
}
