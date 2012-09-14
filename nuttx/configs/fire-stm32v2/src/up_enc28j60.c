/****************************************************************************
 * configs/fire-stm32v2/src/up_enc28j60.c
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
 ****************************************************************************/

/* 2MBit SPI FLASH OR ENC28J60
 *
 * --- ------ -------------- -----------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -----------------------------------------------------
 *
 * 29  PA4    PA4-SPI1-NSS   10Mbit ENC28J60, SPI 2M FLASH
 * 30  PA5    PA5-SPI1-SCK   2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 31  PA6    PA6-SPI1-MISO  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 32  PA7    PA7-SPI1-MOSI  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
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
#include "fire-internal.h"

#ifdef CONFIG_ENC28J60

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* ENC28J60
 *
 * --- ------ -------------- -----------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -----------------------------------------------------
 *
 * 29  PA4    PA4-SPI1-NSS   10Mbit ENC28J60, SPI 2M FLASH
 * 30  PA5    PA5-SPI1-SCK   2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 31  PA6    PA6-SPI1-MISO  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 32  PA7    PA7-SPI1-MOSI  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 98  PE1    PE1-FSMC_NBL1  2.4" TFT + Touchscreen, 10Mbit EN28J60 Reset
 * 4   PE5    (no name)      10Mbps ENC28J60 Interrupt
 */

/* ENC28J60 is on SPI1 */

#ifndef CONFIG_STM32_SPI1
# error "Need CONFIG_STM32_SPI1 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define ENC28J60_SPI_PORTNO 1   /* On SPI1 */
#define ENC28J60_DEVNO      0   /* Only one ENC28J60 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct enc_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                   handler;  /* ENC28J60 interrupt handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler);
static void up_enable(FAR const struct enc_lower_s *lower);
static void up_disable(FAR const struct enc_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The ENC28J60 normal provides interrupts to the MCU via a GPIO pin.  The
 * following structure provides an MCU-independent mechanixm for controlling
 * the ENC28J60 GPIO interrupt.
 */

static struct stm32_lower_s g_enclower =
{
  .lower =
  {
    .attach  = up_attach,
    .enable  = up_enable,
    .disable = up_disable
  },
  .handler = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: struct enc_lower_s methods
 ****************************************************************************/

static int up_attach(FAR const struct enc_lower_s *lower, xcpt_t handler)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  return OK;
}

static void up_enable(FAR const struct enc_lower_s *lower)
{
  FAR struct stm32_lower_s *priv = (FAR struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  (void)stm32_gpiosetevent(GPIO_ENC28J60_INTR, false, true, true, priv->handler);
}

static void up_disable(FAR const struct enc_lower_s *lower)
{
  (void)stm32_gpiosetevent(GPIO_ENC28J60_INTR, false, true, true, NULL);
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
  int ret;

  /* Assumptions:
   * 1) ENC28J60 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in boot-up.
   */

  spi = up_spiinitialize(ENC28J60_SPI_PORTNO);
  if (!spi)
    {
      nlldbg("Failed to initialize SPI port %d\n", ENC28J60_SPI_PORTNO);
      return;
    }

  /* Take ENC28J60 out of reset (active low)*/

  stm32_gpiowrite(GPIO_ENC28J60_RESET, true);

  /* Bind the SPI port to the ENC28J60 driver */

  ret = enc_initialize(spi, &g_enclower.lower, ENC28J60_DEVNO);
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
