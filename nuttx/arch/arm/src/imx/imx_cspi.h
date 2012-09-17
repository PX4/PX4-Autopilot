/************************************************************************************
 * arch/arm/src/imx/imx_cspi.h
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

#ifndef __ARCH_ARM_IMX_CSPI_H
#define __ARCH_ARM_IMX_CSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <nuttx/spi.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CSPI Register Offsets ************************************************************/

#define CSPI_RXD_OFFSET          0x0000 /* Receive Data Register */
#define CSPI_TXD_OFFSET          0x0004 /* Transmit Data Register */
#define CSPI_CTRL_OFFSET         0x0008 /* Control Register */
#define CSPI_INTCS_OFFSET        0x000c /* Interrupt Control/Status Register */
#define CSPI_TEST_OFFSET         0x0010 /* Test Register */
#define CSPI_SPCR_OFFSET         0x0014 /* Sample Period Control Register */
#define CSPI_DMA_OFFSET          0x0018 /* DMA Control Register */
#define CSPI_RESET_OFFSET        0x001c /* Soft Reset Register */

/* CSPI Register Addresses **********************************************************/

/* CSPI1 */

#define IMX_CSPI1_RXD            (IMX_CSPI1_VBASE + CSPI_RXD_OFFSET)
#define IMX_CSPI1_TXD            (IMX_CSPI1_VBASE + CSPI_TXD_OFFSET)
#define IMX_CSPI1_CTRL           (IMX_CSPI1_VBASE + CSPI_CTRL_OFFSET)
#define IMX_CSPI1_INTCS          (IMX_CSPI1_VBASE + CSPI_INTCS_OFFSET)
#define IMX_CSPI1_SPITEST        (IMX_CSPI1_VBASE + CSPI_TEST_OFFSET)
#define IMX_CSPI1_SPISPCR        (IMX_CSPI1_VBASE + CSPI_SPCR_OFFSET)
#define IMX_CSPI1_SPIDMA         (IMX_CSPI1_VBASE + CSPI_DMA_OFFSET)
#define IMX_CSPI1_SPIRESET       (IMX_CSPI1_VBASE + CSPI_RESET_OFFSET)

/* CSPI1 */

#define IMX_CSPI2_RXD            (IMX_CSPI2_VBASE + CSPI_RXD_OFFSET)
#define IMX_CSPI2_TXD            (IMX_CSPI2_VBASE + CSPI_TXD_OFFSET)
#define IMX_CSPI2_CTRL           (IMX_CSPI2_VBASE + CSPI_CTRL_OFFSET)
#define IMX_CSPI2_INTCS          (IMX_CSPI2_VBASE + CSPI_INTCS_OFFSET)
#define IMX_CSPI2_SPITEST        (IMX_CSPI2_VBASE + CSPI_TEST_OFFSET)
#define IMX_CSPI2_SPISPCR        (IMX_CSPI2_VBASE + CSPI_SPCR_OFFSET)
#define IMX_CSPI2_SPIDMA         (IMX_CSPI2_VBASE + CSPI_DMA_OFFSET)
#define IMX_CSPI2_SPIRESET       (IMX_CSPI2_VBASE + CSPI_RESET_OFFSET)

/* CSPI Register Bit Definitions ****************************************************/

/* CSPI Control Register */

#define CSPI_CTRL_DATARATE_SHIFT 13
#define CSPI_CTRL_DATARATE_MASK    (7 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV4             (0 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV8             (1 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV16            (2 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV32            (3 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV64            (4 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV128           (5 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV256           (6 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DIV512           (7 << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_CTRL_DRCTL_SHIFT      11
#define CSPI_CTRL_DRCTL_MASK       (3 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_DRCTL_IGNRDY     (0 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_DRCTL_FALLING    (1 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_DRCTL_ACTVLOW    (2 << CSPI_CTRL_DRCTL_SHIFT)
#define CSPI_CTRL_MODE             (1 << 10)
#define CSPI_CTRL_SPIEN            (1 << 9)
#define CSPI_CTRL_XCH              (1 << 8)
#define CSPI_CTRL_SSPOL            (1 << 7)
#define CSPI_CTRL_SSCTL            (1 << 6)
#define CSPI_CTRL_PHA              (1 << 5)
#define CSPI_CTRL_POL              (1 << 4)
#define CSPI_CTRL_BITCOUNT_SHIFT   0
#define CSPI_CTRL_BITCOUNT_MASK    (15 << CSPI_CTRL_BITCOUNT_SHIFT)

/* CSPI Interrrupt Control/Status Register */

#define CSPI_INTCS_TE              (1 << 0)  /* Bit  0: TXFIFO Empty Status */
#define CSPI_INTCS_TH              (1 << 1)  /* Bit  1: TXFIFO Half Status */
#define CSPI_INTCS_TF              (1 << 2)  /* Bit  2: TXFIFO Full Status */
#define CSPI_INTCS_RR              (1 << 3)  /* Bit  3: RXFIFO Data Ready Status */
#define CSPI_INTCS_RH              (1 << 4)  /* Bit  4: RXFIFO Half Status */
#define CSPI_INTCS_RF              (1 << 5)  /* Bit  5: RXFIFO Full Status */
#define CSPI_INTCS_RO              (1 << 6)  /* Bit  6: RXFIFO Overflow */
#define CSPI_INTCS_BO              (1 << 7)  /* Bit  7: Bit Count Overflow */
#define CSPI_INTCS_TEEN            (1 << 8)  /* Bit  8: TXFIFO Empty Interrupt Enable */
#define CSPI_INTCS_THEN            (1 << 9)  /* Bit  9: TXFIFO Half Interrupt Enable */
#define CSPI_INTCS_TFEN            (1 << 10) /* Bit 10: TXFIFO Full Interrupt Enable */
#define CSPI_INTCS_RREN            (1 << 11) /* Bit 11: RXFIFO Data Ready Interrupt Enable */
#define CSPI_INTCS_RHEN            (1 << 12) /* Bit 12: RXFIFO Half Interrupt Enable */
#define CSPI_INTCS_RFEN            (1 << 13) /* Bit 13: RXFIFO Full Interrupt Enable */
#define CSPI_INTCS_ROEN            (1 << 14) /* BIT 14: RXFIFO Overflow Interrupt Enable */
#define CSPI_INTCS_BOEN            (1 << 15) /* Bit 15: Bit Count Overflow Interrupt Enable */

#define CSPI_INTCS_ALLINTS         0x0000ff00

/* CSPI Sample Period Control Register */

#define CSPI_SPCR_WAIT_SHIFT       0
#define CSPI_SPCR_WAIT_MASK        (0x7ff << CSPI_CTRL_DATARATE_SHIFT)
#define CSPI_SPCR_CSRC             (1 << 15) /* Bit 15: 1:32768 or 32 kHz clock source */

/* CSPI DMA Control Register */

#define CSPI_DMA_RHDMA             (1 <<  4) /* Bit  4: RXFIFO Half Status */
#define CSPI_DMA_RFDMA             (1 <<  5) /* Bit  5: RXFIFO Full Status */
#define CSPI_DMA_TEDMA             (1 <<  6) /* Bit  6: TXFIFO Empty Status */
#define CSPI_DMA_THDMA             (1 <<  7) /* Bit  7: TXFIFO Half Status */
#define CSPI_DMA_RHDEN             (1 << 12) /* Bit 12: Enable RXFIFO Half DMA Request */
#define CSPI_DMA_RFDEN             (1 << 13) /* Bit 13: Enables RXFIFO Full DMA Request */
#define CSPI_DMA_TEDEN             (1 << 14) /* Bit 14: Enable TXFIFO Empty DMA Request */
#define CSPI_DMA_THDEN             (1 << 15) /* Bit 15: Enable TXFIFO Half DMA Request */

/* Soft Reset Register */

#define CSPI_RESET_START           (1 << 0)  /* Bit  0: Execute soft reset */

/************************************************************************************
 * Inline Functions
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

/* The external functions, imx_spiselect, imx_spistatus, and imx_cmddaa must be
 * provided by board-specific logic.  These are implementations of the select and
 * status methods of the SPI interface defined by struct spi_ops_s (see
 * include/nuttx/spi.h).  All other methods (including up_spiinitialize()) are
 * provided by common logic.  To use this common SPI logic on your board:
 *
 *   1. Provide imx_spiselect() and imx_spistatus() functions in your board-specific 
 *      logic.  This function will perform chip selection and status operations using
 *      GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration, provide the
 *      imx_spicmddata() function in your board-specific logic.  This function will
 *      perform cmd/data selection operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a call to up_spiinitialize() in your low level initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 */

EXTERN void imx_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
EXTERN uint8_t imx_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
EXTERN int imx_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_IMX_CSPI_H */
