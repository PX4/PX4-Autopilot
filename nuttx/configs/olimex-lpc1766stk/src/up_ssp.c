/************************************************************************************
 * configs/olimex-lpc1766stk/src/up_ssp.c
 * arch/arm/src/board/up_ssp.c
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
#ifdef CONFIG_SPI_CALLBACK
#include <nuttx/irq.h>
#endif

#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "lpc17_gpio.h"
#include "lpc17_ssp.h"
#include "lpc1766stk_internal.h"

#if defined(CONFIG_LPC17_SSP0) || defined(CONFIG_LPC17_SSP1)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#  ifndef CONFIG_GPIO_IRQ
#    warning "CONFIG_GPIO_IRQ is required to support CONFIG_SPI_CALLBACK"
#  endif
#endif

/* Debug ********************************************************************/
/* The following enable debug output from this file (needs CONFIG_DEBUG too).
 * 
 * CONFIG_SSP_DEBUG - Define to enable basic SSP debug
 * CONFIG_SSP_VERBOSE - Define to enable verbose SSP debug
 */

#ifdef CONFIG_SSP_DEBUG
#  define sspdbg  lldbg
#  ifdef CONFIG_SSP_VERBOSE
#    define sspvdbg lldbg
#  else
#    define sspvdbg(x...)
#  endif
#else
#  undef CONFIG_SSP_VERBOSE
#  define sspdbg(x...)
#  define sspvdbg(x...)
#endif

/* Dump GPIO registers */

#ifdef CONFIG_SSP_VERBOSE
#  define ssp_dumpssp0gpio(m) lpc17_dumpgpio(LPC1766STK_LCD_CS, m)
#  define ssp_dumpssp1gpio(m) lpc17_dumpgpio(LPC1766STK_MMC_CS, m)
#else
#  define ssp_dumpssp0gpio(m)
#  define ssp_dumpssp1gpio(m)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This structure describes on media change callback */

#ifdef CONFIG_SPI_CALLBACK
struct lpc17_mediachange_s
{
  spi_mediachange_t callback; /* The media change callback */
  FAR void          *arg;     /* Callback argument */
};
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Registered media change callback */

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_LPC17_SSP0
static struct lpc17_mediachange_s g_ssp0callback;
#endif
#ifdef CONFIG_LPC17_SSP1
static struct lpc17_mediachange_s g_ssp1callback;
#endif
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ssp_cdirqsetup
 *
 * Description:
 *   Setup to receive a card detection interrupt
 *
 ************************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
static void ssp_cdirqsetup(int irq, xcpt_t irqhandler)
{
  irqstate_t flags;

  /* Disable interrupts until we are done */

  flags = irqsave();

  /* Configure the interrupt.  Either attach and enable the new
   * interrupt or disable and detach the old interrupt handler.
   */

  if (irqhandler)
    {
      /* Attach then enable the new interrupt handler */

      (void)irq_attach(irq, irqhandler);
      up_enable_irq(irq);
    }
  else
    {
      /* Disable then then detach the the old interrupt handler */

      up_disable_irq(irq);
      (void)irq_detach(irq);
    }
}
#endif

/************************************************************************************
 * Name: ssp0/1_cdinterrupt
 *
 * Description:
 *   Handle card detection interrupt
 *
 ************************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_LPC17_SSP0
static int ssp0_cdinterrupt(int irq, FAR void *context)
{
  /* Invoke the media change callback */

  if (g_ssp0callback.callback)
    {
      g_ssp0callback.callback(g_ssp0callback.arg);
    }
  return OK;
}
#endif

#ifdef CONFIG_LPC17_SSP1
static int ssp1_cdinterrupt(int irq, FAR void *context)
{
  /* Invoke the media change callback */

  if (g_ssp1callback.callback)
    {
      g_ssp1callback.callback(g_ssp1callback.arg);
    }
  return OK;
}
#endif
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC1766-STK.
 *
 ************************************************************************************/

void weak_function lpc17_sspinitialize(void)
{
  /* Configure the SSP0 chip select GPIOs.  Only the Nokia LCD is connected to SSP0 */

#ifdef CONFIG_LPC17_SSP0
  ssp_dumpssp0gpio("BEFORE SSP0 Initialization");
  lpc17_configgpio(LPC1766STK_LCD_CS);
  ssp_dumpssp0gpio("AFTER SSP0 Initialization");
#endif

  /* Configure SSP1 chip select GPIOs.  Only the SD/MMC card slot is connected to SSP1 */

#ifdef CONFIG_LPC17_SSP1
  ssp_dumpssp0gpio("BEFORE SSP1 Initialization");
  lpc17_configgpio(LPC1766STK_MMC_CS);

  /* Also configure the SD/MMC power GPIO (but leave power off).  This really has
   * nothing to do with SSP, but does belong with other SD/MMC GPIO configuration
   * settings.
   */

  lpc17_configgpio(LPC1766STK_MMC_PWR);
  ssp_dumpssp0gpio("AFTER SSP1 Initialization");
#endif

#ifdef CONFIG_SPI_CALLBACK
  /* If there were any CD detect pins for the LPC1766-STK, this is where
   * they would be configured.
   */
#endif
}

/************************************************************************************
 * Name:  lpc17_ssp0/ssp1select and lpc17_ssp0/ssp1status
 *
 * Description:
 *   The external functions, lpc17_ssp0/ssp1select and lpc17_ssp0/ssp1status 
 *   must be provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common LPC17xx logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in lpc17_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide lpc17_ssp0/ssp1select() and lpc17_ssp0/ssp1status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_SSP0
void  lpc17_ssp0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  sspdbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  if (devid == SPIDEV_DISPLAY)
    {
      /* Assert/de-assert the CS pin to the card */

      ssp_dumpssp0gpio("lpc17_ssp0select() Entry");
      lpc17_gpiowrite(LPC1766STK_LCD_CS, !selected);
      ssp_dumpssp0gpio("lpc17_ssp0select() Exit");
    }
}

uint8_t lpc17_ssp0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  sspdbg("Returning nothing\n");
  return 0;
}
#endif

#ifdef CONFIG_LPC17_SSP1
void  lpc17_ssp1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  sspdbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  if (devid == SPIDEV_MMCSD)
    {
      /* Assert/de-assert the CS pin to the card */

      ssp_dumpssp1gpio("lpc17_ssp1select() Entry");
      lpc17_gpiowrite(LPC1766STK_MMC_CS, !selected);
      ssp_dumpssp1gpio("lpc17_ssp1select() Exit");
    }
}

uint8_t lpc17_ssp1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  sspdbg("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}
#endif

/************************************************************************************
 * Name: lpc17_ssp0/1register
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
 ************************************************************************************/

#ifdef CONFIG_SPI_CALLBACK
#ifdef CONFIG_LPC17_SSP0
  /* If there were any CD detect pins on the LPC1766-STK, this is how the
   * would be configured.
   */

int lpc17_ssp0register(FAR struct spi_dev_s *dev, spi_mediachange_t callback, void *arg)
{
  /* Save the callback information */

#if 0
  g_ssp0callback.callback = callback;
  g_ssp0callback.arg      = arg;

  /* Setup the interrupt */

  spi_cdirqsetup(LPC1766STK_SPICD_IRQ, ssp0_cdinterrupt);
#endif
  return OK;
}
#endif

#ifdef CONFIG_LPC17_SSP1
int lpc17_ssp1register(FAR struct spi_dev_s *dev, spi_mediachange_t callback, void *arg)
{
  /* Save the callback information */

#if 0
  g_ssp1callback.callback = callback;
  g_ssp1callback.arg      = arg;

  /* Setup the interrupt */

  spi_cdirqsetup(LPC1766STK_SPICD_IRQ, ssp1_cdinterrupt);
#endif
  return OK;
}
#endif
#endif

#endif /* CONFIG_LPC17_SSP0 || CONFIG_LPC17_SSP1 */
