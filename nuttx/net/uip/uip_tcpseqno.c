/****************************************************************************
 * net/uip/uip_tcpseqno.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
#include <debug.h>

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

/* g_tcpsequence is used to generate initial TCP sequence numbers */

static uint32_t g_tcpsequence;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpsetsequence
 *
 * Description:
 *   Set the TCP/IP sequence number
 *
 * Assumptions:
 *   This function may called from the interrupt level
 *
 ****************************************************************************/

void uip_tcpsetsequence(FAR uint8_t *seqno, uint32_t value)
{
  /* Copy the sequence number in network (big-endian) order */

  *seqno++ =  value >> 24;
  *seqno++ = (value >> 16) & 0xff;
  *seqno++ = (value >>  8) & 0xff;
  *seqno   =  value        & 0xff;
}

/****************************************************************************
 * Name: uip_tcpgetsequence
 *
 * Description:
 *   Get the TCP/IP sequence number
 *
 * Assumptions:
 *   This function may called from the interrupt level
 *
 ****************************************************************************/

uint32_t uip_tcpgetsequence(FAR uint8_t *seqno)
{
  uint32_t value;
 
  /* Combine the sequence number from network (big-endian) order */

   value = (uint32_t)seqno[0] << 24 |
           (uint32_t)seqno[1] << 16 |
           (uint32_t)seqno[2] <<  8 |
           (uint32_t)seqno[3];
   return value;
}

/****************************************************************************
 * Name: uip_tcpaddsequence
 *
 * Description:
 *   Add the length to get the next TCP sequence number.
 *
 * Assumptions:
 *   This function may called from the interrupt level
 *
 ****************************************************************************/

uint32_t uip_tcpaddsequence(FAR uint8_t *seqno, uint16_t len)
{
  return uip_tcpgetsequence(seqno) + (uint32_t)len;
}

/****************************************************************************
 * Name: uip_tcpinitsequence
 *
 * Description:
 *   Set the (initial) the TCP/IP sequence number when a TCP connection is
 *   established.
 *
 * Assumptions:
 *   This function may called from the interrupt level
 *
 ****************************************************************************/

void uip_tcpinitsequence(FAR uint8_t *seqno)
{
  uip_tcpsetsequence(seqno, g_tcpsequence);
}

/****************************************************************************
 * Name: uip_tcpnextsequence
 *
 * Description:
 *   Increment the TCP/IP sequence number
 *
 * Assumptions:
 *   This function is called from the interrupt level
 *
 ****************************************************************************/

void uip_tcpnextsequence(void)
{
  g_tcpsequence++;
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
