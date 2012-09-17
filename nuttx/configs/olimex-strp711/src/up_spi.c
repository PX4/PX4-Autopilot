/****************************************************************************
 * config/olimex-strp711/src/up_spi.c
 *
 *   Copyright (C) 2008-2010,2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "str71x_internal.h"

#if defined(CONFIG_STR71X_BSPI0) || defined(CONFIG_STR71X_BSPI1)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_STR714X_BSPI0_TXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI0_TXFIFO_DEPTH 8
#endif

#ifndef CONFIG_STR714X_BSPI0_RXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI0_RXFIFO_DEPTH 8
#endif

#ifndef CONFIG_STR714X_BSPI1_TXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI1_TXFIFO_DEPTH 8
#endif

#ifndef CONFIG_STR714X_BSPI1_RXFIFO_DEPTH
#  define CONFIG_STR714X_BSPI1_RXFIFO_DEPTH 8
#endif

#if defined(CONFIG_STR71X_UART3) && defined (CONFIG_STR71X_BSPI0)
#  warning "BSPI0 GPIO usage conflicts with UART3"
#endif

#if defined(CONFIG_STR71X_IC21) && defined (CONFIG_STR71X_BSPI0)
#  warning "BSPI0 GPIO usage conflicts with IC21"
#endif

#if defined(CONFIG_STR71X_HDLC) && defined (CONFIG_STR71X_BSPI1)
#  warning "BSPI1 GPIO usage conflicts with HDLC"
#endif

/****************************************************************************
 * On the Olimex-STR-STR-P711, BSPI0 is not connected on board, but is
 * available on a header for use in the prototyping area.  BSPI connects
 * to the MMC/SD card slot.
 *
 * GPIO pin configurations (STR710/STR711,2,5).
 * BSP0:
 *   PIN     NORMAL  ALTERNATE  Olimex-STR-STR-P711 Connection
 *   123/52  P0.0    S0.MISO *  UEXT-3 (Not connected on board)
 *   124/53  P0.1    S0.MOSI *  UEXT-4  " " "       " "" "   "
 *   125/54  P0.2    S0.SCLK ** UEXT-5  " " "       " "" "   "
 *   126/55  P0.3   ~SO.SS   ** UEXT-6  " " "       " "" "   "
 *
 *  * Programming the AF function selects UART3 by default.  BSPI must be
 *    enabled with the SPI_EN bit in the BOOTCR register
 * ** Programming the AF function selects I2C1 by default.  BSPI must be
 *    enabled with the SPI_EN bit in the BOOTCR register
 *
 * BSP1
 *   PIN     NORMAL  ALTERNATE Olimex-STR-STR-P711 Connection
 *   127/56  P0.4    S1.MISO   SD_CARDBOT DAT0/D0
 *   140/60  P0.5    S1.MOSI   SD_CARDBOT CMD/DI
 *   141/61  P0.6    S1.SCLK   SD_CARDBOT CLK/SCLK
 *   142/62  P0.7   ~S1.SS     SD_CARDBOT CD/DAT/CS
 *
 * Two GPIO pins also connect to the MMC/SD slot:
 *
 *   PIN     NORMAL  ALTERNATE Olimex-STR-STR-P711 Connection
 *   106/46  P1.10   USB clock MMC/SD write protect (WP)
 *   111/49  P1.15   HDLC xmit MMC/SD card present (CP)
 *
 ****************************************************************************/

/* SPI0 *********************************************************************/

#define BSPI0_GPIO0_MISO (1 << 0)
#define BSPI0_GPIO0_MOSI (1 << 1)
#define BSPI0_GPIO0_SCLK (1 << 2)
#define BSPI0_GPIO0_SS   (1 << 3) /* Not used */

#define BSPI0_GPIO0_ALT  (BSPI0_GPIO0_MISO|BSPI0_GPIO0_MOSI|BSPI0_GPIO0_SCLK)

/* ENC28J60 Module
 *
 * The ENC28J60 module does not come on the Olimex-STR-P711, but this describes
 * how I have connected it. NOTE that the ENC28J60 requires an external interrupt
 * (XTI) pin.  The only easily accessible XTI pins are on SPI0/1 so you can't have
 * both SPI0 and 1 together with this configuration.
 *
 * STR-P711 PIN            PIN CONFIGURATION ENC28J60 CONNECTION  
 * ----------------------- ----------------- -----------------------
 * P0.3/S0.SS/I1.SDA       P0.3 output      CON5 1 J8-1 NET CS 
 * P0.2/S0.SCLK/I1.SCL     SCLK0                 2    2 SCK
 * P0.0/S0.MOSI/U3.RX      MOSI0                 3    3 MOSI
 * P0.1/S0.MISO/U3.TX      MISO0                 4    4 MISO
 * GND                     GND                   5    5 GND 
 * 3.3V                    3.3V                 10 J9-1 3V3
 * NC                      NC                    9    2 WOL
 * P0.6/S1.SCLK            P0.6 input            8    3 NET INT
 * NC                      NC                    7    4 CLKOUT
 * P0.4/S1.MISO            P0.4 output           6    5 NET RST
 */

#ifdef CONFIG_ENC28J60

/* UART3, I2C cannot be used with SPI0.  The GPIOs selected for the ENC28J60
 * interrupt conflict with BSPI1
 */

#  ifdef CONFIG_STR71X_BSPI1
#    warning "CONFIG_STR71X_BSPI1 cannot be used in this configuration"
#  endif

/* ENC28J60 additional pins
 *
 * NOTE: The ENC28J60 is a 3.3V part; however, it was designed to be
 * easily integrated into 5V systems. The SPI CS, SCK and SI inputs,
 * as well as the RESET pin, are all 5V tolerant. On the other hand,
 * if the host controller is operated at 5V, it quite likely will
 * not be within specifications when its SPI and interrupt inputs
 * are driven by the 3.3V CMOS outputs on the ENC28J60. A
 * unidirectional level translator would be necessary.
 */

#  define ENC_GPIO0_CS       (1 << 3)
#  define ENC_GPIO0_NETRST   (1 << 4)
#  define ENC_GPIO0_NETINT   (1 << 6)

#  define ENC_GPIO0_INTTL    (0)
#  define ENC_GPIO0_INCMOS   ENC_GPIO0_NETINT
#  define ENC_GPIO0_OUTPP    (ENC_GPIO0_CS|ENC_GPIO0_NETRST)
#  define ENC_GPIO0_ALL      (ENC_GPIO0_CS|ENC_GPIO0_NETINT|ENC_GPIO0_NETRST)

#  define BSPI0_GPIO0_INTTL  ENC_GPIO0_INTTL
#  define BSPI0_GPIO0_INCMOS ENC_GPIO0_INCMOS
#  define BSPI0_GPIO0_OUTPP  ENC_GPIO0_OUTPP
#  define BSPI0_GPIO0_ALL    (BSPI0_GPIO0_ALT|ENC_GPIO0_ALL)

#else
#  define BSPI0_GPIO0_INTTL  (0)
#  define BSPI0_GPIO0_INCMOS (0)
#  define BSPI0_GPIO0_OUTPP  (0)
#  define BSPI0_GPIO0_ALL    BSPI0_GPIO0_ALT
#endif

/* SPI1 *********************************************************************/

#define BSPI1_GPIO0_MISO (1 << 4)
#define BSPI1_GPIO0_MOSI (1 << 5)
#define BSPI1_GPIO0_SCLK (1 << 6)
#define BSPI1_GPIO0_SS   (1 << 7) /* Not used */

#define BSPI1_GPIO0_ALT  (BSPI1_GPIO0_MISO|BSPI1_GPIO0_MOSI|BSPI1_GPIO0_SCLK)

/* MMC/SD Pin Usage:
 *
 * STR-P711 PIN MMC/SD USAGE     PIN CONFIGURATION
 * ------------ ---------------- -----------------------
 * P0.7/S1.SS   1     CD/DAT3/CS P0.7 output
 * P0.5/S1.MOSI 2     CMD/DI     MOSI1
 * ---          3     VSS1       ---
 * ---          4     VDD        ---
 * P0.6/S1.SCLK 5     CLK/SCLK   SLCK1
 * ---          6     VSS2       ---
 * P0.4/S1.MISO 7     DAT0/D0    MISO1
 * ---          8     DAT1/RES   (Pulled up)
 * ---          9     DAT2/RES   (Pulled up)
 *            
 * P1.10/USBCLK 10/14 WP         P1.10 input
 * P1.15/HTXD   13/15 CP         P1.15 input
 *
 * Use of SPI1 doesn't conflict with anything.  WP conflicts USB; CP conflicts
 * with HTXD. 
 */

/* MMC/SD additional pins */

#define MMCSD_GPIO0_CS     (1 << 7)
#define MMCSD_GPIO0_INTTL  (0)
#define MMCSD_GPIO0_INCMOS (0)
#define MMCSD_GPIO0_OUTPP  MMCSD_GPIO0_CS
#define MMCSD_GPIO0_ALL    MMCSD_GPIO0_CS

#define MMCSD_GPIO1_WPIN   (1 << 10)
#define MMCSD_GPIO1_CPIN   (1 << 15)
#define MMCSD_GPIO1_INTTL  (MMCSD_GPIO1_WPIN|MMCSD_GPIO1_CPIN)
#define MMCSD_GPIO1_INCMOS (0)
#define MMCSD_GPIO1_OUTPP  (0)
#define MMCSD_GPIO1_ALL    (MMCSD_GPIO1_WPIN|MMCSD_GPIO1_CPIN)

#define BSPI1_GPIO0_INTTL   MMCSD_GPIO0_INTTL
#define BSPI1_GPIO0_INCMOS  MMCSD_GPIO0_INCMOS
#define BSPI1_GPIO0_OUTPP   MMCSD_GPIO0_OUTPP
#define BSPI1_GPIO0_ALL    (BSPI1_GPIO0_ALT|MMCSD_GPIO0_ALL)

#define BSPI1_GPIO1_INTTL  MMCSD_GPIO1_INTTL
#define BSPI1_GPIO1_INCMOS MMCSD_GPIO1_INCMOS
#define BSPI1_GPIO1_OUTPP  MMCSD_GPIO1_OUTPP
#define BSPI1_GPIO1_ALL    MMCSD_GPIO1_ALL

/* Configuration register settings ******************************************/

#if CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 1
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE1
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 2
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE12
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 3
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE13
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 4
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE14
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 5
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE15
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 6
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE16
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 7
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE17
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 8
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE18
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 9
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE19
#elif CONFIG_STR714X_BSPI0_RXFIFO_DEPTH == 10
#  define STR71X_BSPI0_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE110
#else
#  error "Invaid RX FIFO depth setting"
#endif

#define STR71X_BSPI0_CSR1DISABLE STR71X_BSPI0_CSR1RXFIFODEPTH
#define STR71X_BSPI0_CSR1ENABLE  (STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR|STR71X_BSPI0_CSR1RXFIFODEPTH)

#if CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 1
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE1
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 2
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE12
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 3
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE13
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 4
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE14
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 5
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE15
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 6
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE16
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 7
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE17
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 8
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE18
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 9
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE19
#elif CONFIG_STR714X_BSPI0_TXFIFO_DEPTH == 10
#  define STR71X_BSPI0_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE110
#else
#  error "Invaid TX FIFO depth setting"
#endif

#define STR71X_BSPI0_CSR2VALUE STR71X_BSPI0_CSR1TXFIFODEPTH

#if CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 1
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE1
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 2
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE12
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 3
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE13
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 4
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE14
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 5
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE15
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 6
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE16
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 7
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE17
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 8
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE18
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 9
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE19
#elif CONFIG_STR714X_BSPI1_RXFIFO_DEPTH == 10
#  define STR71X_BSPI1_CSR1RXFIFODEPTH STR71X_BSPICSR1_RFE110
#else
#  error "Invaid RX FIFO depth setting"
#endif

#define STR71X_BSPI1_CSR1DISABLE STR71X_BSPI1_CSR1RXFIFODEPTH
#define STR71X_BSPI1_CSR1ENABLE  (STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR|STR71X_BSPI1_CSR1RXFIFODEPTH)

#if CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 1
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE1
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 2
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE12
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 3
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE13
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 4
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE14
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 5
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE15
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 6
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE16
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 7
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE17
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 8
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE18
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 9
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE19
#elif CONFIG_STR714X_BSPI1_TXFIFO_DEPTH == 10
#  define STR71X_BSPI1_CSR1TXFIFODEPTH STR71X_BSPICSR2_TFE110
#else
#  error "Invaid TX FIFO depth setting"
#endif

#define STR71X_BSPI1_CSR2VALUE STR71X_BSPI1_CSR1TXFIFODEPTH

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NOTE: As implemented here, this driver will support only one device per
 * SPI bus:  Only one chip select, csbit, per bus; no locking, not mode or
 * bits-per-word settings.  To support multiple devices per but, spi_select
 * would also require some logic changes.
 */

struct str71x_spidev_s
{
  struct spi_dev_s spidev;      /* Externally visible part of the SPI interface */
  bool             initialized; /* Initialize port only once! */
  uint32_t         spibase;     /* BSPIn base address */
  uint16_t         csbit;       /* BSPIn SS bit int GPIO0 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint16_t spi_getreg(FAR struct str71x_spidev_s *priv, uint8_t offset);
static inline void   spi_putreg(FAR struct str71x_spidev_s *priv, uint8_t offset, uint16_t value);
static inline void spi_drain(FAR struct str71x_spidev_s *priv);

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int    spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static void   spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static uint8_t  spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
#ifdef CONFIG_SPI_CMDDATA
static int    spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd);
#endif
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void   spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t buflen);
static void   spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .status            = spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = spi_cmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
  .registercallback  = 0,                 /* Not implemented */
};

#ifdef CONFIG_STR71X_BSPI0
static struct str71x_spidev_s g_spidev0 =
{
  .spidev  = { &g_spiops },
  .spibase = STR71X_BSPI0_BASE,
  .csbit   = MMCSD_GPIO0_CS
};
#endif

#ifdef CONFIG_STR71X_BSPI1
static struct str71x_spidev_s g_spidev1 =
{
  .spidev  = { &g_spiops },
  .spibase = STR71X_BSPI1_BASE,
  .csbit   = ENC_GPIO0_CS
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint16_t spi_getreg(FAR struct str71x_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline void spi_putreg(FAR struct str71x_spidev_s *priv, uint8_t offset, uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_drain
 *
 * Description:
 *   Drain any bytes left in the fifos.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_drain(FAR struct str71x_spidev_s *priv)
{
#if CONFIG_STR714X_BSPI0_TXFIFO_DEPTH > 1
  /* Wait while the TX FIFO is full */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) != 0);
#else
  /* Wait until the TX FIFO is empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFE) == 0);
#endif
  /* Write 0xff to the TX FIFO */

  spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, 0xff00);

  /* Wait for the TX FIFO empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFNE) != 0);

  /* Wait for the RX FIFO not empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) == 0);

  /* Then read and discard bytes until the RX FIFO is empty */

  do
    {
      (void)spi_getreg(priv, STR71X_BSPI_RXR_OFFSET);
    }
  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) != 0);
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  /* Not implemented */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI slave select.  The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  uint16_t reg16;

  DEBUGASSERT(priv && priv->spibase);

  reg16 = spi_getreg(priv, STR71X_GPIO_PD_OFFSET);
  if (selected)
    {
      /* Enable slave select (low enables) */

      reg16 &= ~priv->csbit;
      spi_putreg(priv, STR71X_GPIO_PD_OFFSET, reg16);
    }
  else
    {
      /* Disable slave select (low enables) */

      reg16 |= priv->csbit;
      spi_putreg(priv, STR71X_GPIO_PD_OFFSET, reg16);

      /* And drain the FIFOs */

      spi_drain(priv);
    }
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  uint32_t divisor;
  uint32_t cr1;

  DEBUGASSERT(priv && priv->spibase);

  /* The BSPI clock is determined by divider the APB1 clock (PCLK1).
   *
   * Eg. PCLK1 = 32MHz, frequency = 20000000:
   *     correct divisor is 2.1, calculated value is 2.
   */

  divisor = (STR71X_PCLK1 + (frequency >> 1)) / frequency;

  /* The divisor must be an even number and contrained to the range of
   * 5 (master mode, or 7 for slave mode) and 255.  These bits must
   * be configured BEFORE  the BSPE or MSTR bits.. i.e., before the SPI
   * is put into master mode.
   */

  divisor <<= 1;   /* The full, even divisor */
  if (divisor < 6)
    {
      divisor = 6;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  /* The BSPI must be disable when the following setting is made. */

  cr1 = spi_getreg(priv, STR71X_BSPI_CSR1_OFFSET);
  cr1 &= ~(STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR);
  spi_putreg(priv, STR71X_BSPI_CSR1_OFFSET, cr1);
  spi_putreg(priv, STR71X_BSPI_CLK_OFFSET, (uint16_t)divisor);

  /* Now we can enable the BSP in master mode */

  cr1 |= (STR71X_BSPICSR1_BSPE|STR71X_BSPICSR1_MSTR);
  spi_putreg(priv, STR71X_BSPI_CSR1_OFFSET, cr1);

  return STR71X_PCLK1 / divisor;
}

/****************************************************************************
 * Name: spi_status
 *
 * Description:
 *   Get SPI/MMC status
 *
 * Input Parameters:
 *   dev -   Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines
 *
 ****************************************************************************/

static uint8_t spi_status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  uint8_t ret = 0;
  uint16_t reg16 = getreg16(STR71X_GPIO1_PD);

  if ((reg16 & MMCSD_GPIO1_WPIN) != 0)
    {
      ret |= SPI_STATUS_WRPROTECTED;
    }

  if ((reg16 & MMCSD_GPIO1_CPIN) != 0)
    {
      ret |= SPI_STATUS_PRESENT;
    }

  return ret;
}

/****************************************************************************
 * Name: spi_cmddata
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

 #ifdef CONFIG_SPI_CMDDATA
static int spi_cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
#  error "spi_cmddata not implemented"
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;

  DEBUGASSERT(priv && priv->spibase);

#if CONFIG_STR714X_BSPI0_TXFIFO_DEPTH > 1
  /* Wait while the TX FIFO is full */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) != 0);
#else
  /* Wait until the TX FIFO is empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFE) == 0);
#endif

  /* Write the byte to the TX FIFO */

  spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, wd << 8);

  /* Wait for the RX FIFO not empty */

  while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) == 0);

  /* Get the received value from the RX FIFO and return it */

  return (uint8_t)(spi_getreg(priv, STR71X_BSPI_RXR_OFFSET) >> 8);
}

/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   buflen - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t buflen)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  FAR const uint8_t *ptr = (FAR const uint8_t *)buffer;
  uint16_t csr2;

  DEBUGASSERT(priv && priv->spibase);

  /* Loop while thre are bytes remaining to be sent */

  while (buflen > 0)
    {
      /* While the TX FIFO is not full and there are bytes left to send */

      while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) == 0 && buflen > 0)
        {
          /* Send the data */

          spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, ((uint16_t)*ptr) << 8);
          ptr++;
          buflen--;
        }
    }

  /* Then discard all card responses until the RX & TX FIFOs are emptied. */

  do
    {
      /* Is there anything in the RX fifo? */

      csr2 = spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET);
      if ((csr2 & STR71X_BSPICSR2_RFNE) != 0)
        {
          /* Yes.. Read and discard */

          (void)spi_getreg(priv, STR71X_BSPI_RXR_OFFSET);
        }

      /* There is a race condition where TFNE may go false just before
       * RFNE goes true and this loop terminates prematurely.  The nasty little
       * delay in the following solves that (it could probably be tuned to
       * improve performance).
       */

      else if ((csr2 & STR71X_BSPICSR2_TFNE) != 0)
        {
          up_udelay(100);
          csr2 = spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET);
        }
    }
  while ((csr2 & STR71X_BSPICSR2_RFNE) != 0 || (csr2 & STR71X_BSPICSR2_TFNE) == 0);
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   buflen - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t buflen)
{
  FAR struct str71x_spidev_s *priv = (FAR struct str71x_spidev_s *)dev;
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;
  uint32_t fifobytes = 0;

  DEBUGASSERT(priv && priv->spibase);

  /* While there is remaining to be sent (and no synchronization error has occurred) */

  while (buflen || fifobytes)
    {
      /* Fill the transmit FIFO with 0xff...
       * Write 0xff to the data register while (1) the TX FIFO is
       * not full, (2) we have not exceeded the depth of the TX FIFO,
       * and (3) there are more bytes to be sent.
       */

      while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_TFF) == 0 &&
             (fifobytes < CONFIG_STR714X_BSPI0_TXFIFO_DEPTH) && buflen > 0)
        {
          spi_putreg(priv, STR71X_BSPI_TXR_OFFSET, 0xff00);
          buflen--;
          fifobytes++;
        }

      /* Now, read the RX data from the RX FIFO while the RX FIFO is not empty */

      while ((spi_getreg(priv, STR71X_BSPI_CSR2_OFFSET) & STR71X_BSPICSR2_RFNE) != 0)
        {
          *ptr++ = (uint8_t)(spi_getreg(priv, STR71X_BSPI_RXR_OFFSET) >> 8);
          fifobytes--;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port.  This function could get called
 *   multiple times for each STR7 devices that needs an SPI reference.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct spi_dev_s *ret;
  irqstate_t flags;
  uint16_t reg16;

  flags = irqsave();
#ifdef CONFIG_STR71X_BSPI0
  if (port == 0)
    {
      /* Check if this port has already been initialized */

      if (!g_spidev0.initialized)
        {
          /* The default, alternate functionality of the GPIO0 pin selections is
           * UART3/I2C1.  In order to have BSP0 functionality, we also have to
           * set the BSPI0 enable bit in the PCU BOOTCR register.
           */

          reg16 = getreg16(STR71X_PCU_BOOTCR);
          reg16 |= STR71X_PCUBOOTCR_BSPIOEN;
          putreg16(reg16, STR71X_PCU_BOOTCR);

          /* Configure all GPIO pins to their appropriate function:
           *
           * PC0=1 PC1=1 PC2=1: Alternate function, push-pull
           * PC0=1 PC1=0 PC2=0: In, TTL
           * PC0=0 PC1=1 PC2=0: In, CMOS
           * PC0=1 PC1=0 PC2=1: Output, push pull 
           */

          reg16  = getreg16(STR71X_GPIO0_PC0);
          reg16 &= ~BSPI0_GPIO0_ALL;
          reg16 |= (BSPI0_GPIO0_ALT|BSPI0_GPIO0_INTTL|BSPI0_GPIO0_OUTPP);
          putreg16(reg16, STR71X_GPIO0_PC0);

          reg16  = getreg16(STR71X_GPIO0_PC1);
          reg16 &= ~BSPI0_GPIO0_ALL;
          reg16 |= (BSPI0_GPIO0_ALT|BSPI0_GPIO0_INCMOS);
          putreg16(reg16, STR71X_GPIO0_PC1);

          reg16  = getreg16(STR71X_GPIO0_PC2);
          reg16 &= ~BSPI0_GPIO0_ALL;
          reg16 |= (BSPI0_GPIO0_ALT|BSPI0_GPIO0_OUTPP);
          putreg16(reg16, STR71X_GPIO0_PC2);

          /* Start with enc28j60 de-selected (active low) and in
           * reset (also active low)
           */

#ifdef CONFIG_ENC28J60
          reg16  = getreg16(STR71X_GPIO0_PD);
          reg16 |= (ENC_GPIO0_CS | ENC_GPIO0_NETRST);
          putreg16(reg16, STR71X_GPIO0_PD);
#endif

          /* Set the clock divider to the maximum */

          putreg16(255, STR71X_BSPI0_CLK);

          /* Set FIFO sizes and disable the BSP1.  It won't be enabled
           * until the frequency is set.
           */

          putreg16(STR71X_BSPI0_CSR1DISABLE, STR71X_BSPI0_CSR1);
          putreg16(STR71X_BSPI0_CSR2VALUE, STR71X_BSPI0_CSR2);

          /* Configure GPIO1 pins for ENC28J60 inputs and outputs.
           *
           * PC0=1 PC1=0 PC2=0: In, TTL
           * PC0=0 PC1=1 PC2=0: In, CMOS
           * PC0=1 PC1=0 PC2=1: Output, push pull 
           */

#ifdef BSPI0_GPIO1_ALL
          reg16  = getreg16(STR71X_GPIO1_PC0);
          reg16 &= ~BSPI0_GPIO1_ALL;
          reg16 |= (BSPI0_GPIO1_INTTL|BSPI0_GPIO1_OUTPP);
          putreg16(reg16, STR71X_GPIO1_PC0);

          reg16  = getreg16(STR71X_GPIO1_PC1);
          reg16 &= ~BSPI0_GPIO1_ALL;
          reg16 |= BSPI0_GPIO0_INCMOS;
          putreg16(reg16, STR71X_GPIO1_PC1);

          reg16  = getreg16(STR71X_GPIO1_PC2);
          reg16 &= ~BSPI0_GPIO1_ALL;
          reg16 |= BSPI0_GPIO0_OUTPP;
          putreg16(reg16, STR71X_GPIO1_PC2);
#endif
          g_spidev0.initialized = true;
        }

      /* Return the SPI device reference */

      ret = &g_spidev0.spidev;
    }
  else
#endif
#ifdef CONFIG_STR71X_BSPI1
  if (port == 1)
    {
      /* Check if this port has already been initialized */

      if (!g_spidev1.initialized)
        {
          /* Configure all GPIO pins to their alternate function EXCEPT for
           * the CS pin .. we will configure that as an push-pull output
           * and control the chip select as a normal GPIO.
           *
           * PC0=1 PC1=1 PC2=1: Alternate function, push-pull
           * PC0=1 PC1=0 PC2=0: In, TTL
           * PC0=0 PC1=1 PC2=0: In, CMOS
           * PC0=1 PC1=0 PC2=1: Output, push pull 
           */

          reg16  = getreg16(STR71X_GPIO0_PC0);
          reg16 &= ~BSPI1_GPIO0_ALL;
          reg16 |= (BSPI1_GPIO0_ALT|BSPI1_GPIO0_INTTL|BSPI1_GPIO0_OUTPP);
          putreg16(reg16, STR71X_GPIO0_PC0);

          reg16  = getreg16(STR71X_GPIO0_PC1);
          reg16 &= ~BSPI1_GPIO0_ALL;
          reg16 |= (BSPI1_GPIO0_ALT|BSPI1_GPIO0_INCMOS);
          putreg16(reg16, STR71X_GPIO0_PC1);

          reg16  = getreg16(STR71X_GPIO0_PC2);
          reg16 &= ~BSPI1_GPIO0_ALL;
          reg16 |= (BSPI1_GPIO0_ALT|BSPI1_GPIO0_OUTPP);
          putreg16(reg16, STR71X_GPIO0_PC2);

          /* Start with MMC/SD disabled */

          reg16  = getreg16(STR71X_GPIO0_PD);
          reg16 |= MMCSD_GPIO0_CS;
          putreg16(reg16, STR71X_GPIO0_PD);

          /* Set the clock divider to the maximum */

          putreg16(255, STR71X_BSPI1_CLK);

          /* Set FIFO sizes and disable the BSP1.  It won't be enabled
           * until the frequency is set.
           */

          putreg16(STR71X_BSPI1_CSR1DISABLE, STR71X_BSPI1_CSR1);
          putreg16(STR71X_BSPI1_CSR2VALUE, STR71X_BSPI1_CSR2);

          /* Configure GPIO1 pins for WP/CP input
           *
           * PC0=1 PC1=0 PC2=0: In, TTL
           * PC0=0 PC1=1 PC2=0: In, CMOS
           * PC0=1 PC1=0 PC2=1: Output, push pull 
           */

#ifdef BSPI1_GPIO1_ALL
          reg16  = getreg16(STR71X_GPIO1_PC0);
          reg16 &= ~BSPI1_GPIO1_ALL;
          reg16 |= (BSPI1_GPIO1_INTTL|BSPI1_GPIO1_OUTPP);
          putreg16(reg16, STR71X_GPIO1_PC0);

          reg16  = getreg16(STR71X_GPIO1_PC1);
          reg16 &= ~BSPI1_GPIO1_ALL;
          reg16 |= BSPI1_GPIO0_INCMOS;
          putreg16(reg16, STR71X_GPIO1_PC1);

          reg16  = getreg16(STR71X_GPIO1_PC2);
          reg16 &= ~BSPI1_GPIO1_ALL;
          reg16 |= BSPI1_GPIO0_OUTPP;
          putreg16(reg16, STR71X_GPIO1_PC2);
#endif
          g_spidev1.initialized = true;
        }

      /* Return the SPI device reference */

      ret = &g_spidev1.spidev;
    }
  else
#endif
    {
      ret = NULL;
    }
  irqrestore(flags);
  return ret;
}

#endif /* CONFIG_STR71X_BSPI0 || CONFIG_STR71X_BSPI1 */
