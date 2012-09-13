/************************************************************************************
 * arch/z80/src/ez80/ez80f91_spi.h
 * arch/z80/src/chip/ez80f91_spi.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_Z80_SRC_EZ80_EZ80F91_SPI_H
#define __ARCH_Z80_SRC_EZ80_EZ80F91_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/spi.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* SPIC Registers  *****************************************************************/

/* Provided in ez80f91.h */

/* SPIC Register Bit Definitions  **************************************************/

/* Baud Rate Generator (BRG) H/L Register Definitions
 *
 * No bit definitions -- These two 8-bit registers set the 16-bit BRG divider value
 */

/* SPI Control (CTL} Register Definitions */

#define SPI_CTL_IRQEN    (1 << 7) /* Bit 7: 1=SPI system interrupt is enabled */
#define SPI_CTL_SPIEN    (1 << 5) /* Bit 5: 1=SPI is enabled */
#define SPI_CTL_MASTEREN (1 << 4) /* Bit 4: 1=SPI operates as a master */
#define SPI_CTL_CPOL     (1 << 3) /* Bit 3: 1=Master SCK pin idles in a high (1) state */
#define SPI_CTL_CPHA     (1 << 2) /* Bit 2: 1=SS remains Low to transfer any number of data bytes */

/* SR Register Definitions */

#define SPI_SR_SPIF      (1 << 7) /* Bit x: 1=SPI data transfer is finished */
#define SPI_SR_WCOL      (1 << 6) /* Bit x: 1=SPI write collision is detected*/
#define SPI_SR_MODF      (1 << 4) /* Bit x: 1=Mode fault (multimaster conflict) is detected */

/* RBR/TSR Register Definitions */

/* No definitions: 8-bit SPI receive/transmit data */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif /* __cplusplus */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* The external functions, ez80_spiselect, ez80_spistatus, ans ez80_spicmddata must
 * be provided by board-specific logic.  These are implementations of the select,
 * status, and cmddata methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi.h).  All other methods (including up_spiinitialize()) are
 * provided by common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide ez80_spiselect() and ez80_spistatus() functions in your board-specific 
 *      logic.  This function will perform chip selection and status operations using
 *      GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide the
 *      ez80_spiscmddata() function in your board-specific logic.  This function will
 *      perform cmd/data selection operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a call to up_spiinitialize() in your low level initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 */

EXTERN void ez80_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t ez80_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
EXTERN int ez80_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_Z80_SRC_EZ80_EZ80F91_SPI_H */
