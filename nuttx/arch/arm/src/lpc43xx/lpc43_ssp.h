/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_ssp.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_SSP_H
#define __ARCH_ARM_SRC_LPC43XX_SSP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi.h>
#include "chip/lpc43_ssp.h"

#if defined(CONFIG_LPC43_SSP0) || defined(CONFIG_LPC43_SSP1)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* This header file defines interfaces to common SSP logic.  To use this common SSP
 * logic on your board:
 *
 * 1. Provide logic in lpc43_boardinitialize() to configure SSP chip select pins.
 * 2. Provide lpc43_ssp0/1select() and lpc43_ssp0/1status() functions in your
 *    board-specific logic.  These functions will perform chip selection
 *    and status operations using GPIOs in the way your board is configured.
 * 3. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *    lpc43_ssp0/1cmddata() functions in your board-specific logic.  These
 *    functions will perform cmd/data selection operations using GPIOs in the
 *    way your board is configured.
 * 4. Your low level board initialization logic should call lpc43_sspinitialize.
 * 5. The handle returned by lpc43_sspinitialize() may then be used to bind the
 *    SSP driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *    for example, will bind the SPI driver to the SPI MMC/SD driver).
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: lpc43_sspinitialize
 *
 * Description:
 *   Initialize the selected SSP port (0=SSP0, 1=SSP1)
 *
 * Input Parameter:
 *   port - Port number (0=SSP0, 1=SSP1)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *lpc43_sspinitialize(int port)

/************************************************************************************
 * Name:  lpc43_ssp0/1select, lpc43_ssp0/1status, and lpc43_ssp0/1cmddata
 *
 * Description:
 *   These functions must be provided in your board-specific logic.  The
 *   lpc43_ssp0/1select functions will perform chip selection and the
 *   lpc43_ssp0/1status will perform status operations using GPIOs in the way your
 *   board is configured.
 *
 *   If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, then
 *   lpc43_ssp0/1cmddata must also be provided.  This functions performs cmd/data
 *   selection operations using GPIOs in the way your board is configured.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC43_SSP0
EXTERN void  lpc43_ssp0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lpc43_ssp0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lpc43_ssp0cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

#ifdef CONFIG_LPC43_SSP1
EXTERN void  lpc43_ssp1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t lpc43_ssp1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int lpc43_ssp1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
#endif

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Flush and discard any words left in the RX fifo.  This can be called
 *   from ssp0/1select after a device is deselected (if you worry about such
 *   things).
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_LPC43_SSP0) || defined(CONFIG_LPC43_SSP1)
EXTERN void ssp_flush(FAR struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: lpc43_spi/ssp0/1register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD drvier when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s) must
 *   must be implemented.  These functiosn implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_LPC43_SSP0
EXTERN int lpc43_ssp0register(FAR struct spi_dev_s *dev,
                              spi_mediachange_t callback, void *arg);
#endif

#ifdef CONFIG_LPC43_SSP1
EXTERN int lpc43_ssp1register(FAR struct spi_dev_s *dev,
                              spi_mediachange_t callback, void *arg);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC43_SSP0/1 */
#endif /* __ARCH_ARM_SRC_LPC43XX_SSP_H */
