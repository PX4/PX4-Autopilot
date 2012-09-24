/****************************************************************************
 * net/uip/uip_tcpconn.c
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Large parts of this file were leveraged from uIP logic:
 *
 *   Copyright (c) 2001-2003, Adam Dunkels.
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
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>

#include "uip_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all uIP TCP connections. */

static struct uip_conn g_tcp_connections[CONFIG_NET_TCP_CONNS];

/* A list of all free TCP connections */

static dq_queue_t g_free_tcp_connections;

/* A list of all connected TCP connections */

static dq_queue_t g_active_tcp_connections;

/* Last port used by a TCP connection connection. */

static uint16_t g_last_tcp_port;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_selectport()
 *
 * Description:
 *   If the portnumber is zero; select an unused port for the connection.
 *   If the portnumber is non-zero, verify that no other connection has
 *   been created with this port number.
 *
 * Input Parameters:
 *   portno -- the selected port number in host order. Zero means no port
 *     selected.
 *
 * Return:
 *   0 on success, negated errno on failure:
 *
 *   EADDRINUSE
 *     The given address is already in use.
 *   EADDRNOTAVAIL
 *     Cannot assign requested address (unlikely)
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int uip_selectport(uint16_t portno)
{
  if (portno == 0)
    {
      /* No local port assigned. Loop until we find a valid listen port number
       * that is not being used by any other connection. NOTE the following loop
       * is assumed to terminate but could not if all 32000-4096+1 ports are
       * in used (unlikely).
       */

      do
        {
          /* Guess that the next available port number will be the one after
           * the last port number assigned.
           */
          portno = ++g_last_tcp_port;

          /* Make sure that the port number is within range */

          if (g_last_tcp_port >= 32000)
            {
              g_last_tcp_port = 4096;
            }
        }
      while (uip_tcplistener(htons(g_last_tcp_port)));
    }
  else
    {
      /* A port number has been supplied.  Verify that no other TCP/IP
       * connection is using this local port.
       */

      if (uip_tcplistener(portno))
        {
          /* It is in use... return EADDRINUSE */

          return -EADDRINUSE;
        }
    }

  /* Return the selected or verified port number */

  return portno;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpinit()
 *
 * Description:
 *   Initialize the TCP/IP connection structures.  Called only once and only
 *   from the UIP layer at startup in normal user mode.
 *
 ****************************************************************************/

void uip_tcpinit(void)
{
  int i;

  /* Initialize the queues */

  dq_init(&g_free_tcp_connections);
  dq_init(&g_active_tcp_connections);

  /* Now initialize each connection structure */

  for (i = 0; i < CONFIG_NET_TCP_CONNS; i++)
    {
      /* Mark the connection closed and move it to the free list */

      g_tcp_connections[i].tcpstateflags = UIP_CLOSED;
      dq_addlast(&g_tcp_connections[i].node, &g_free_tcp_connections);
    }

  g_last_tcp_port = 1024;
}

/****************************************************************************
 * Name: uip_tcpalloc()
 *
 * Description:
 *   Find a free TCP/IP connection structure and allocate it
 *   for use.  This is normally something done by the implementation of the
 *   socket() API but is also called from the interrupt level when a TCP
 *   packet is received while "listening"
 *
 ****************************************************************************/

struct uip_conn *uip_tcpalloc(void)
{
  struct uip_conn *conn;
  uip_lock_t flags;

  /* Because this routine is called from both interrupt level and
   * and from user level, we have not option but to disable interrupts
   * while accessing g_free_tcp_connections[];
   */

  flags = uip_lock();

  /* Return the entry from the head of the free list */

  conn = (struct uip_conn *)dq_remfirst(&g_free_tcp_connections);

#if 0 /* Revisit */
  /* Is the free list empty? */

  if (!conn)
    {
      /* As a fallback, check for connection structures in the TIME_WAIT
       * state.  If no CLOSED connections are found, then take the oldest
       */

      struct uip_conn *tmp = g_active_tcp_connections.head;
      while (tmp)
        {
          /* Is this connectin in the UIP_TIME_WAIT state? */

          if (tmp->tcpstateflags == UIP_TIME_WAIT)
            {
              /* Is it the oldest one we have seen so far? */

              if (!conn || tmp->timer > conn->timer)
                {
                  /* Yes.. remember it */

                  conn = tmp;
                }
            }

          /* Look at the next active connection */

          tmp = tmp->node.flink;
        }

      /* If we found one, remove it from the active connection list */

      dq_rem(&conn->node, &g_active_tcp_connections);
    }
#endif

  uip_unlock(flags);

  /* Mark the connection allocated */

  if (conn)
    {
      conn->tcpstateflags = UIP_ALLOCATED;
    }

  return conn;
}

/****************************************************************************
 * Name: uip_tcpfree()
 *
 * Description:
 *   Free a connection structure that is no longer in use. This should be
 *   done by the implementation of close()
 *
 ****************************************************************************/

void uip_tcpfree(struct uip_conn *conn)
{
#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  struct uip_readahead_s *readahead;
#endif
  uip_lock_t flags;

  /* Because g_free_tcp_connections is accessed from user level and interrupt
   * level, code, it is necessary to keep interrupts disabled during this
   * operation.
   */

  DEBUGASSERT(conn->crefs == 0);
  flags = uip_lock();

  /* UIP_ALLOCATED means that that the connection is not in the active list
   * yet.
   */

  if (conn->tcpstateflags != UIP_ALLOCATED)
    {
      /* Remove the connection from the active list */

      dq_rem(&conn->node, &g_active_tcp_connections);
    }

  /* Release any read-ahead buffers attached to the connection */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  while ((readahead = (struct uip_readahead_s *)sq_remfirst(&conn->readahead)) != NULL)
    {
      uip_tcpreadaheadrelease(readahead);
    }
#endif

  /* Remove any backlog attached to this connection */

#ifdef CONFIG_NET_TCPBACKLOG
  if (conn->backlog)
    {
      uip_backlogdestroy(conn);
    }

  /* If this connection is, itself, backlogged, then remove it from the
   * parent connection's backlog list.
   */

  if (conn->blparent)
    {
      uip_backlogdelete(conn->blparent, conn);
    }
#endif

  /* Mark the connection available and put it into the free list */

  conn->tcpstateflags = UIP_CLOSED;
  dq_addlast(&conn->node, &g_free_tcp_connections);
  uip_unlock(flags);
}

/****************************************************************************
 * Name: uip_tcpactive()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided TCP/IP header
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level
 *
 ****************************************************************************/

struct uip_conn *uip_tcpactive(struct uip_tcpip_hdr *buf)
{
  struct uip_conn *conn      = (struct uip_conn *)g_active_tcp_connections.head;
  in_addr_t        srcipaddr = uip_ip4addr_conv(buf->srcipaddr);  

  while (conn)
    {
      /* Find an open connection matching the tcp input */

      if (conn->tcpstateflags != UIP_CLOSED &&
          buf->destport == conn->lport && buf->srcport == conn->rport &&
          uip_ipaddr_cmp(srcipaddr, conn->ripaddr))
        {
          /* Matching connection found.. break out of the loop and return a
           * reference to it.
           */

          break;
        }

      /* Look at the next active connection */

      conn = (struct uip_conn *)conn->node.flink;
    }

  return conn;
}

/****************************************************************************
 * Name: uip_nexttcpconn()
 *
 * Description:
 *   Traverse the list of active TCP connections
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level (or with
 *   interrupts disabled).
 *
 ****************************************************************************/

struct uip_conn *uip_nexttcpconn(struct uip_conn *conn)
{
  if (!conn)
    {
      return (struct uip_conn *)g_active_tcp_connections.head;
    }
  else
    {
      return (struct uip_conn *)conn->node.flink;
    }
}

/****************************************************************************
 * Name: uip_tcplistener()
 *
 * Description:
 *   Given a local port number (in network byte order), find the TCP
 *   connection that listens on this this port.
 *
 *   Primary uses: (1) to determine if a port number is available, (2) to
 *   To idenfity the socket that will accept new connections on a local port.
 *
 ****************************************************************************/

struct uip_conn *uip_tcplistener(uint16_t portno)
{
  struct uip_conn *conn;
  int i;

  /* Check if this port number is in use by any active UIP TCP connection */
 
  for (i = 0; i < CONFIG_NET_TCP_CONNS; i++)
    {
      conn = &g_tcp_connections[i];
      if (conn->tcpstateflags != UIP_CLOSED && conn->lport == portno)
        {
          /* The portnumber is in use, return the connection */

          return conn;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name: uip_tcpaccept()
 *
 * Description:
 *    Called when uip_interupt matches the incoming packet with a connection
 *    in LISTEN. In that case, this function will create a new connection and
 *    initialize it to send a SYNACK in return.
 *
 * Assumptions:
 *   This function is called from UIP logic at interrupt level
 *
 ****************************************************************************/

struct uip_conn *uip_tcpaccept(struct uip_tcpip_hdr *buf)
{
  struct uip_conn *conn = uip_tcpalloc();
  if (conn)
    {
      /* Fill in the necessary fields for the new connection. */

      conn->rto           = UIP_RTO;
      conn->timer         = UIP_RTO;
      conn->sa            = 0;
      conn->sv            = 4;
      conn->nrtx          = 0;
      conn->lport         = buf->destport;
      conn->rport         = buf->srcport;
      uip_ipaddr_copy(conn->ripaddr, uip_ip4addr_conv(buf->srcipaddr));
      conn->tcpstateflags = UIP_SYN_RCVD;

      uip_tcpinitsequence(conn->sndseq);
      conn->unacked       = 1;

      /* rcvseq should be the seqno from the incoming packet + 1. */

      memcpy(conn->rcvseq, buf->seqno, 4);

      /* Initialize the list of TCP read-ahead buffers */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
      sq_init(&conn->readahead);
#endif

      /* And, finally, put the connection structure into the active list.
       * Interrupts should already be disabled in this context.
       */

      dq_addlast(&conn->node, &g_active_tcp_connections);
    }
  return conn;
}

/****************************************************************************
 * Name: uip_tcpbind()
 *
 * Description:
 *   This function implements the UIP specific parts of the standard TCP
 *   bind() operation.
 *
 * Return:
 *   0 on success or -EADDRINUSE on failure
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int uip_tcpbind(struct uip_conn *conn, const struct sockaddr_in6 *addr)
#else
int uip_tcpbind(struct uip_conn *conn, const struct sockaddr_in *addr)
#endif
{
  uip_lock_t flags;
  int port;

  /* Verify or select a local port */

  flags = uip_lock();
  port = uip_selectport(ntohs(addr->sin_port));
  uip_unlock(flags);

  if (port < 0)
    {
      return port;
    }

  /* Save the local address in the connection structure.  Note that the requested
   * local IP address is saved but not used.  At present, only a single network
   * interface is supported, the IP address is not of importance.
   */

  conn->lport = addr->sin_port;

#if 0 /* Not used */
#ifdef CONFIG_NET_IPv6
  uip_ipaddr_copy(conn->lipaddr, addr->sin6_addr.in6_u.u6_addr16);
#else
  uip_ipaddr_copy(conn->lipaddr, addr->sin_addr.s_addr);
#endif
#endif

  return OK;
}

/****************************************************************************
 * Name: uip_tcpconnect()
 *
 * Description:
 *   This function implements the UIP specific parts of the standard
 *   TCP connect() operation:  It connects to a remote host using TCP.
 *
 *   This function is used to start a new connection to the specified
 *   port on the specied host. It uses the connection structure that was
 *   allocated by a preceding socket() call.  It sets the connection to
 *   the SYN_SENT state and sets the retransmission timer to 0. This will
 *   cause a TCP SYN segment to be sent out the next time this connection
 *   is periodically processed, which usually is done within 0.5 seconds
 *   after the call to uip_tcpconnect().
 *
 * Assumptions:
 *   This function is called from normal user level code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int uip_tcpconnect(struct uip_conn *conn, const struct sockaddr_in6 *addr)
#else
int uip_tcpconnect(struct uip_conn *conn, const struct sockaddr_in *addr)
#endif
{
  uip_lock_t flags;
  int port;

  /* The connection is expected to be in the UIP_ALLOCATED state.. i.e., 
   * allocated via up_tcpalloc(), but not yet put into the active connections
   * list.
   */

  if (!conn || conn->tcpstateflags != UIP_ALLOCATED)
    {
      return -EISCONN;
    }

  /* If the TCP port has not alread been bound to a local port, then select
   * one now.
   */

  flags = uip_lock();
  port = uip_selectport(ntohs(conn->lport));
  uip_unlock(flags);

  if (port < 0)
    {
      return port;
    }

  /* Initialize and return the connection structure, bind it to the port number */

  conn->tcpstateflags = UIP_SYN_SENT;
  uip_tcpinitsequence(conn->sndseq);

  conn->initialmss = conn->mss = UIP_TCP_MSS;
  conn->unacked    = 1;    /* TCP length of the SYN is one. */
  conn->nrtx       = 0;
  conn->timer      = 1;    /* Send the SYN next time around. */
  conn->rto        = UIP_RTO;
  conn->sa         = 0;
  conn->sv         = 16;   /* Initial value of the RTT variance. */
  conn->lport      = htons((uint16_t)port);

  /* The sockaddr port is 16 bits and already in network order */

  conn->rport = addr->sin_port;

  /* The sockaddr address is 32-bits in network order. */

  uip_ipaddr_copy(conn->ripaddr, addr->sin_addr.s_addr);

  /* Initialize the list of TCP read-ahead buffers */

#if CONFIG_NET_NTCP_READAHEAD_BUFFERS > 0
  sq_init(&conn->readahead);
#endif

  /* And, finally, put the connection structure into the active
   * list. Because g_active_tcp_connections is accessed from user level and
   * interrupt level, code, it is necessary to keep interrupts disabled during
   * this operation.
   */

  flags = uip_lock();
  dq_addlast(&conn->node, &g_active_tcp_connections);
  uip_unlock(flags);

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
