/****************************************************************************
 * include/nuttx/enc28j60.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __INCLUDE_NUTTX_ENC28J60_H
#define __INCLUDE_NUTTX_ENC28J60_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
 
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ENC28J60 Configuration Settings:
 *
 * CONFIG_NET_ENC28J60 - Enabled ENC28J60 support
 * CONFIG_ENC28J60_SPIMODE - Controls the SPI mode
 * CONFIG_ENC28J60_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ENC28J60_NINTERFACES - Specifies the number of physical ENC28J60
 *   devices that will be supported.
 * CONFIG_ENC28J60_STATS - Collect network statistics
 * CONFIG_ENC28J60_HALFDUPPLEX - Default is full duplex
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure returns driver statistics (if enabled) */

#ifdef CONFIG_ENC28J60_STATS
struct enc_stats_s
{
  uint8_t  maxpktcnt;         /* Max. number of buffered RX packets */
  uint32_t txrequests;        /* Number of TX packets queued */
  uint32_t txifs;             /* TXIF completion events */
  uint32_t txabrts;           /* TXIF completions with ESTAT.TXABRT */
  uint32_t txerifs;           /* TXERIF error events */
  uint32_t txtimeouts;        /* S/W detected TX timeouts */
  uint32_t pktifs;            /* PKTIF RX completion events */
  uint32_t rxnotok;           /* PKTIF without RXSTAT_OK */
  uint32_t rxpktlen;          /* PKTIF with bad pktlen */
  uint32_t rxerifs;           /* RXERIF error evernts */
};
#endif

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
 * Function: enc_initialize
 *
 * Description:
 *   Initialize the Ethernet driver.  The ENC28J60 device is assumed to be
 *   in the post-reset state upon entry to this function.
 *
 * Parameters:
 *   spi   - A reference to the platform's SPI driver for the ENC28J60
 *   devno - If more than one ENC28J60 is supported, then this is the
 *           zero based number that identifies the ENC28J60;
 *   irq   - The fully configured GPIO IRQ that ENC28J60 interrupts will be
 *           asserted on.  This driver will attach and entable this IRQ.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct spi_dev_s; /* see nuttx/spi.h */
EXTERN int enc_initialize(FAR struct spi_dev_s *spi, unsigned int devno,
                          unsigned int irq);

/****************************************************************************
 * Function: enc_stats
 *
 * Description:
 *   Return accumulated ENC28J60 statistics.  Statistics are cleared after
 *   being returned.
 *
 * Parameters:
 *   devno - If more than one ENC28J60 is supported, then this is the
 *           zero based number that identifies the ENC28J60;
 *   stats - The user-provided location to return the statistics.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_ENC28J60_STATS
EXTERN int enc_stats(unsigned int devno, struct enc_stats_s *stats);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ENC28J60_H */
