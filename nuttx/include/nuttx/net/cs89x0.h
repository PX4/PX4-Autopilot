/****************************************************************************
 * include/nuttx/net/cs89x0.h
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_CS89x0_H
#define __INCLUDE_NUTTX_NET_CS89x0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <wdog.h>
 
/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure returns driver statistics (if enabled) */

#ifdef CONFIG_C89x0_STATISTICS
struct cs89x0_statistics_s
{
  uint32_t tx_packets;
  uint32_t tx_errors;
  uint32_t tx_carriererrors;
  uint32_t tx_heartbeaterrors;
  uint32_t tx_windowerrors;
  uint32_t tx_abortederrors;
  uint32_t rx_missederrors;
  uint32_t rx_packets;
  uint32_t rx_errors;
  uint32_t rx_lengtherrors;
  uint32_t rx_crcerrors;
  uint32_t rx_frameerrors;
  uint32_t rx_dropped;
  uint32_t rx_missederrors;
  uint32_t collisions;
};
#endif

/* This structure encapsulates all state information for a single hardware
 * interface. It includes values that must be provided by the user to in
 * to describe details of the CS89x00 implementation on a particular board.
 * An instance if this structure is passed to cs89x00 to instantiate the
 * driver.
 *
 * This structure also includes internal driver state information that should
 * be of no concern to the caller of cs89x0_initialize().  These fields must
 * be zeroed.
 */

struct cs89x0_driver_s
{
  /* User-provided CS89x00 platform-specific implementation details.  The
   * caller of cs89x0_initialize() must provide all of these values.
   */

  FAR void *cs_base;           /* CS89x0 region base address */
#ifdef CONFIG_CS89x0_MEMMODE
  FAR void *cs_ppbase;         /* CS89x0 page packet base address */
#endif
  uint8_t   cs_irq;            /* CS89x00 IRQ number */

  /* Driver internal state fields.  These must be zeroed by before the
   * instance of this structure is passed to cs89x0_initialize
   */
#ifdef CONFIG_CS89x0_XMITEARLY
  uint8_t   txstart;           /* Bits 6-7 of TxCMD controls Tx race */
#endif
  bool      cs_memmode;        /* true:memory mode false: I/O mode */
  bool      cs_bifup;          /* true:ifup false:ifdown */
  WDOG_ID   cs_txpoll;         /* TX poll timer */
  WDOG_ID   cs_txtimeout;      /* TX timeout timer */
#ifdef CONFIG_CS89x0_XMITEARLY
  uint32_t  cs_txunderrun;     /* Count of Tx underrun errors */
#endif

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s cs_dev;  /* Interface understood by uIP */

  /* Driver statistics */

#ifdef CONFIG_C89x0_STATISTICS
  struct cs89x0_statistics_s cs_stats;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: cs89x0_initialize
 *
 * Description:
 *   Initialize the Ethernet driver
 *
 * Parameters:
 *   impl - decribes the implementation of the cs89x00 implementation.
 *     This reference is retained so so must remain stable throughout the
 *     life of the driver instance.
 *   devno - Identifies the device number.  This must be a number between
 *     zero CONFIG_CS89x0_NINTERFACES and the same devno must not be
 *     initialized twice.  The associated network device will be referred
 *     to with the name "eth" followed by this number (eth0, eth1, etc).
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

/* Initialize the CS89x0 chip and driver */

EXTERN int cs89x0_initialize(FAR const cs89x0_driver_s *cs89x0, int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_CS89x0_H */
