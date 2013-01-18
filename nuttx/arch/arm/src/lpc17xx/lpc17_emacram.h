/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_emacram.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_EMACRAM_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_EMACRAM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Default, no-EMAC Case ************************************************************/
/* Assume that all of AHB SRAM will be available for heap. If this is not true, then
 * LPC17_BANK0_HEAPSIZE will be undefined and redefined below.
 */

#undef LPC17_BANK0_HEAPBASE
#undef LPC17_BANK0_HEAPSIZE
#ifdef LPC17_HAVE_BANK0
#  define LPC17_BANK0_HEAPBASE LPC17_SRAM_BANK0
#  define LPC17_BANK0_HEAPSIZE LPC17_BANK0_SIZE
#endif

/* Is networking enabled?  Is the LPC17xx Ethernet device enabled? Does this chip have
 * and Ethernet controlloer?  Yes... then we will replace the above default definitions.
 */

#if defined(CONFIG_NET) && defined(CONFIG_LPC17_ETHERNET) && LPC17_NETHCONTROLLERS > 0

/* EMAC RAM Configuration ***********************************************************/
/* Is AHB SRAM available? */

#ifndef LPC17_HAVE_BANK0
#  error "AHB SRAM Bank0 is not available for EMAC RAM"
#endif

/* Number of Tx descriptors */

#ifndef CONFIG_NET_NTXDESC
#  define CONFIG_NET_NTXDESC 18
#endif

/* Number of Rx descriptors */

#ifndef CONFIG_NET_NRXDESC
#  define CONFIG_NET_NRXDESC 18
#endif

/* Size of the region at the beginning of AHB SRAM 0 set set aside for the EMAC.
 * This size must fit within AHB SRAM Bank 0 and also be a multiple of 32-bit
 * words.
 */

#ifndef CONFIG_NET_EMACRAM_SIZE
#  define CONFIG_NET_EMACRAM_SIZE LPC17_BANK0_SIZE
#endif

#if CONFIG_NET_EMACRAM_SIZE > LPC17_BANK0_SIZE
#  error "EMAC RAM size cannot exceed the size of AHB SRAM Bank 0"
#endif

#if (CONFIG_NET_EMACRAM_SIZE & 3) != 0
#  error "EMAC RAM size must be in multiples of 32-bit words"
#endif

/* Determine is there is any meaningful space left at the end of AHB Bank 0 that
 * could be added to the heap.
 */

#undef LPC17_BANK0_HEAPBASE
#undef LPC17_BANK0_HEAPSIZE
#if CONFIG_NET_EMACRAM_SIZE < (LPC17_BANK0_SIZE-128)
#  define LPC17_BANK0_HEAPBASE (LPC17_SRAM_BANK0 + CONFIG_NET_EMACRAM_SIZE)
#  define LPC17_BANK0_HEAPSIZE (LPC17_BANK0_SIZE - CONFIG_NET_EMACRAM_SIZE)
#endif

/* Memory at the beginning of AHB SRAM, Bank 0 is set aside for EMAC Tx and Rx
 * descriptors.  The position is not controllable, only the size of the region
 * is controllable.
 */

#define LPC17_EMACRAM_BASE   LPC17_SRAM_BANK0
#define LPC17_EMACRAM_SIZE   CONFIG_NET_EMACRAM_SIZE

/* Descriptor Memory Layout *********************************************************/
/* EMAC DMA RAM and descriptor definitions.  The configured number of descriptors
 * will determine the organization and the size of the descriptor and status tables.
 * There is a complex interaction between the maximum packet size (CONFIG_NET_BUFSIZE)
 * and the number of Rx and Tx descriptors that can be suppored (CONFIG_NET_NRXDESC
 * and CONFIG_NET_NTXDESC): Small buffers -> more packets.  This is something that
 * needs to be tuned for you system.
 *
 * For a 16Kb SRAM region, here is the relationship:
 *
 *  16384 <= ntx * (pktsize + 8 + 4) + nrx * (pktsize + 8 + 8)
 *
 * If ntx == nrx and pktsize == 424, then you could have
 * ntx = nrx = 18.
 *
 * An example with all of the details:
 *
 * NTXDESC=18 NRXDESC=18 CONFIG_NET_EMACRAM_SIZE=16Kb CONFIG_NET_BUFSIZE=420:
 *   LPC17_TXDESCTAB_SIZE = 18*8 = 144
 *   LPC17_TXSTATTAB_SIZE = 18*4 =  72
 *   LPC17_TXTAB_SIZE     = 216
 *
 *   LPC17_RXDESCTAB_SIZE = 16*8 = 144
 *   LPC17_RXSTATTAB_SIZE = 16*8 = 144
 *   LPC17_TXTAB_SIZE     = 288
 *
 *   LPC17_DESCTAB_SIZE   = 504
 *   LPC17_DESC_BASE      = LPC17_SRAM_BANK0 + 0x00004000 - 504
 *                        = LPC17_SRAM_BANK0 + 0x00003e08
 *   LPC17_TXDESC_BASE    = LPC17_SRAM_BANK0 + 0x00003e08
 *   LPC17_TXSTAT_BASE    = LPC17_SRAM_BANK0 + 0x00003e98
 *   LPC17_RXDESC_BASE    = LPC17_SRAM_BANK0 + 0x00003ee0
 *   LPC17_RXSTAT_BASE    = LPC17_SRAM_BANK0 + 0x00003f70
 *
 *   LPC17_PKTMEM_BASE    = LPC17_SRAM_BANK0
 *   LPC17_PKTMEM_SIZE    = 0x00004000-504 = 0x00003e40
 *   LPC17_PKTMEM_END     = LPC17_SRAM_BANK0 + 0x00003e08

 *   LPC17_MAXPACKET_SIZE = ((420 + 3 + 2) & ~3) = 424
 *   LPC17_NTXPKTS        = 18
 *   LPC17_NRXPKTS        = 18

 *   LPC17_TXBUFFER_SIZE  = 18 * 424 = 0x00001dd0
 *   LPC17_RXBUFFER_SIZE  = 18 * 424 = 0x00001dd0
 *   LPC17_BUFFER_SIZE    = 0x00003ba0

 *   LPC17_BUFFER_BASE    = LPC17_SRAM_BANK0
 *   LPC17_TXBUFFER_BASE  = LPC17_SRAM_BANK0
 *   LPC17_RXBUFFER_BASE  = LPC17_SRAM_BANK0 + 0x00001dd0
 *   LPC17_BUFFER_END     = LPC17_SRAM_BANK0 + 0x00003ba0
 *
 *   Then the check LPC17_BUFFER_END < LPC17_PKTMEM_END passes. The amount of
 *   unused memory is small: 0x00003e08-0x00003ba0 or about 616 bytes -- not
 *   enough for two more packets.
 *
 * [It is also possible, with some effort, to reclaim any unused
 *  SRAM for the use in the heap.  But that has not yet been pursued.]
 */

#define LPC17_TXDESCTAB_SIZE (CONFIG_NET_NTXDESC*LPC17_TXDESC_SIZE)
#define LPC17_TXSTATTAB_SIZE (CONFIG_NET_NTXDESC*LPC17_TXSTAT_SIZE)
#define LPC17_TXTAB_SIZE     (LPC17_TXDESCTAB_SIZE+LPC17_TXSTATTAB_SIZE)

#define LPC17_RXDESCTAB_SIZE (CONFIG_NET_NRXDESC*LPC17_RXDESC_SIZE)
#define LPC17_RXSTATTAB_SIZE (CONFIG_NET_NRXDESC*LPC17_RXSTAT_SIZE)
#define LPC17_RXTAB_SIZE     (LPC17_RXDESCTAB_SIZE+LPC17_RXSTATTAB_SIZE)

#define LPC17_DESCTAB_SIZE   (LPC17_TXTAB_SIZE+LPC17_RXTAB_SIZE)

/* Descriptor table memory organization.  Descriptor tables are packed at
 * the end of AHB SRAM, Bank 0.  The beginning of bank 0 is reserved for
 * packet memory.
 */

#define LPC17_DESC_BASE      (LPC17_EMACRAM_BASE+LPC17_EMACRAM_SIZE-LPC17_DESCTAB_SIZE)
#define LPC17_TXDESC_BASE    LPC17_DESC_BASE
#define LPC17_TXSTAT_BASE    (LPC17_TXDESC_BASE+LPC17_TXDESCTAB_SIZE)
#define LPC17_RXDESC_BASE    (LPC17_TXSTAT_BASE+LPC17_TXSTATTAB_SIZE)
#define LPC17_RXSTAT_BASE    (LPC17_RXDESC_BASE + LPC17_RXDESCTAB_SIZE)

/* Now carve up the beginning of SRAM for packet memory.  The size of a
 * packet buffer is related to the size of the MTU.  We'll round sizes up
 * to multiples of 256 bytes.
 */

#define LPC17_PKTMEM_BASE     LPC17_EMACRAM_BASE
#define LPC17_PKTMEM_SIZE     (LPC17_EMACRAM_SIZE-LPC17_DESCTAB_SIZE)
#define LPC17_PKTMEM_END      (LPC17_EMACRAM_BASE+LPC17_PKTMEM_SIZE)

#define LPC17_MAXPACKET_SIZE  ((CONFIG_NET_BUFSIZE + CONFIG_NET_GUARDSIZE + 3) & ~3)
#define LPC17_NTXPKTS         CONFIG_NET_NTXDESC
#define LPC17_NRXPKTS         CONFIG_NET_NRXDESC

#define LPC17_TXBUFFER_SIZE   (LPC17_NTXPKTS * LPC17_MAXPACKET_SIZE)
#define LPC17_RXBUFFER_SIZE   (LPC17_NRXPKTS * LPC17_MAXPACKET_SIZE)
#define LPC17_BUFFER_SIZE     (LPC17_TXBUFFER_SIZE + LPC17_RXBUFFER_SIZE)

#define LPC17_BUFFER_BASE     LPC17_PKTMEM_BASE
#define LPC17_TXBUFFER_BASE   LPC17_BUFFER_BASE
#define LPC17_RXBUFFER_BASE   (LPC17_TXBUFFER_BASE + LPC17_TXBUFFER_SIZE)
#define LPC17_BUFFER_END      (LPC17_BUFFER_BASE + LPC17_BUFFER_SIZE)

#if LPC17_BUFFER_END > LPC17_PKTMEM_END
#  error "Packet memory overlaps descriptor tables"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* CONFIG_NET && CONFIG_LPC17_ETHERNET && LPC17_NETHCONTROLLERS > 0*/
#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_EMACRAM_H */
