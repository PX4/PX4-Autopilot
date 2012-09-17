/****************************************************************************
 * configs/olimex-strp711/src/up_enc28j60.c
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
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

/*
 * ENC28J60 Module
 *
 * The ENC28J60 module does not come on the Olimex-STR-P711, but this
 * describes how I have connected it.  NOTE that the ENC28J60 requires an
 * external interrupt (XTI) pin.  The only easily accessible XTI pins are on
 * SPI0/1 so you can't have both SPI0 and 1 together with this configuration.
 *
 * Module CON5     QFN ENC2860 Description
 * --------------- -------------------------------------------------------
 * 1  J8-1 NET CS   5  ~CS    Chip select input pin for SPI interface (active low)
 * 2     2 SCK      4  SCK    Clock in pin for SPI interface
 * 3     3 MOSI     3  SI     Data in pin for SPI interface
 * 4     4 MISO     2  SO     Data out pin for SPI interface
 * 5     5 GND      -- ---    ---
 * 10 J9-1 3V3      -- ---    ---
 * 9     2 WOL      1  ~WOL   Unicast WOL filter
 * 8     3 NET INT  28 ~INT   Interrupt output pin (active low)
 * 7     4 CLKOUT   27 CLKOUT Programmable clock output pin
 * 6     5 NET RST  6  ~RESET Active-low device Reset input
 *
 * For the Olimex STR-P711, the ENC28J60 module is placed on SPI0 and uses
 * P0.3 for CS, P0.6 for an interrupt, and P0.4 as a reset:
 *
 * Module CON5     Olimex STR-P711 Connection
 * --------------- -------------------------------------------------------
 * 1  J8-1 NET CS   SPI0-2     P0.3 output P0.3/S0.SS/I1.SDA
 * 2     2 SCK      SPI0-5     SCLK0       P0.2/S0.SCLK/I1.SCL
 * 3     3 MOSI     SPI0-3     MOSI0       P0.0/S0.MOSI/U3.RX
 * 4     4 MISO     SPI0-4     MISO0       P0.1/S0.MISO/U3.TX
 * 5     5 GND      SPI0-1     GND
 * 10 J9-1 3V3      SPI0-6     3.3V
 * 9     2 WOL      NC
 * 8     3 NET INT  SPI1-5     P0.6 XTI 11 P0.6/S1.SCLK
 * 7     4 CLKOUT   NC
 * 6     5 NET RST  SPI1-4     P0.4 output P0.4/S1.MISO
 *
 * UART3, I2C cannot be used with SPI0.  The GPIOs selected for the ENC28J60
 * interrupt conflict with TMR1.
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <nuttx/net/enc28j60.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "str71x_internal.h"

#ifdef CONFIG_ENC28J60

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* We assume that the ENC28J60 is on SPI0 */

#ifndef CONFIG_STR71X_BSPI0
# error "Need CONFIG_STR71X_BSPI0 in the configuration"
#endif

#ifndef CONFIG_STR71X_XTI
# error "Need CONFIG_STR71X_XTI in the configuration"
#endif

/* UART3, I2C cannot be used with SPI0.  The GPIOs selected for the ENC28J60
 * interrupt conflict with BSPI1.
 */

#ifdef CONFIG_STR71X_UART3
# error "CONFIG_STR71X_UART3 cannot be used in this configuration"
#endif

#ifdef CONFIG_STR71X_I2C1
# error "CONFIG_STR71X_I2C1 cannot be used in this configuration"
#endif

#ifdef CONFIG_STR71X_BSP1
# error "CONFIG_STR71X_BSP1 cannot be used in this configuration"
#endif

/* SPI Assumptions **********************************************************/

#define ENC28J60_SPI_PORTNO 0                  /* On SPI0 */
#define ENC28J60_DEVNO      0                  /* Only one ENC28J60 */
#define ENC28J60_IRQ        STR71X_IRQ_PORT0p6 /* XTI Line 11: P0.6 */

/* ENC28J60 additional pins *************************************************
 *
 * NOTE: The ENC28J60 is a 3.3V part; however, it was designed to be
 * easily integrated into 5V systems. The SPI CS, SCK and SI inputs,
 * as well as the RESET pin, are all 5V tolerant. On the other hand,
 * if the host controller is operated at 5V, it quite likely will
 * not be within specifications when its SPI and interrupt inputs
 * are driven by the 3.3V CMOS outputs on the ENC28J60. A
 * unidirectional level translator would be necessary.
 */

#  define ENC_GPIO0_CS       (1 << 3) /* Chip select (P0.3) */
#  define ENC_GPIO0_NETRST   (1 << 4) /* Reset (P0.4) */
#  define ENC_GPIO0_NETINT   (1 << 6) /* Interrupt (P0.6) */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(FAR struct enc_lower_s *lower, xcpt_t handler);
static void up_enable(FAR struct enc_lower_s *lower);
static void up_disable(FAR struct enc_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ENC28J60 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the ENC28J60 GPIO interrupt.
 */

static const struct enc_lower_s g_enclower =
{
  .attach  = up_attach,
  .enable  = up_enable,
  .disable = up_disable
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct enc_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler)
{
  return irq_attach(ENC28J60_IRQ, handler)
}

static void up_enable(FAR const struct enc_lower_s *lower)
{
  up_enable_irq(ENC28J60_IRQ);
}

static void up_disable(FAR const struct enc_lower_s *lower)
{
  up_disable_irq(ENC28J60_IRQ);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
  FAR struct spi_dev_s *spi;
  uint16_t reg16;
  int ret;

  /* Get the SPI port */

  spi = up_spiinitialize(ENC28J60_SPI_PORTNO);
  if (!spi)
    {
      nlldbg("Failed to initialize SPI port %d\n", ENC28J60_SPI_PORTNO);
      return;
    }

  /* Configure the XTI for the ENC28J60 interrupt.  */

  ret = str71x_xticonfig(ENC28J60_IRQ, false);
  if (ret < 0)
    {
      nlldbg("Failed configure interrupt for IRQ %d: %d\n", ENC28J60_IRQ, ret);
      return;
    }

  /* Take ENC28J60 out of reset (active low)*/

  reg16  = getreg16(STR71X_GPIO0_PD);
  reg16 &= ~ENC_GPIO0_NETRST;
  putreg16(reg16, STR71X_GPIO0_PD);

  /* Bind the SPI port to the ENC28J60 driver */

  ret = enc_initialize(spi, &g_enclower, ENC28J60_DEVNO);
  if (ret < 0)
    {
      nlldbg("Failed to bind SPI port %d ENC28J60 device %d: %d\n",
             ENC28J60_SPI_PORTNO, ENC28J60_DEVNO, ret);
      return;
    }

  nllvdbg("Bound SPI port %d to ENC28J60 device %d\n",
        ENC28J60_SPI_PORTNO, ENC28J60_DEVNO);
}
#endif /* CONFIG_ENC28J60 */
