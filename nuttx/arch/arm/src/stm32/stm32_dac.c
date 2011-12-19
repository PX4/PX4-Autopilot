/************************************************************************************
 * arch/arm/src/stm32/stm32_dac.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32_internal.h"
#include "stm32_dac.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Up to 2 DAC interfaces are supported */

#if STM32_NDAC < 2
#  undef CONFIG_STM32_DAC2
#  undef CONFIG_STM32_DAC2_DMA
#  undef CONFIG_STM32_DAC2_TIMER
#  undef CONFIG_STM32_DAC2_TIMER_FREQUENCY
#endif

#if STM32_NDAC < 1
#  undef CONFIG_STM32_DAC1
#  undef CONFIG_STM32_DAC1_DMA
#  undef CONFIG_STM32_DAC1_TIMER
#  undef CONFIG_STM32_DAC1_TIMER_FREQUENCY
#endif

#if defined(CONFIG_STM32_DAC1) || defined(CONFIG_STM32_DAC2)

/* DMA configuration. */

#if defined(CONFIG_STM32_DAC1_DMA) || defined(CONFIG_STM32_DAC2_DMA)
# if defined(CONFIG_STM32_STM32F10XX)
#   ifndef CONFIG_STM32_DMA2
#     warning "STM32 F1 DAC DMA support requires CONFIG_STM32_DMA2"
#     undef CONFIG_STM32_DAC1_DMA
#     undef CONFIG_STM32_DAC2_DMA
#   endif
# elif defined(CONFIG_STM32_STM32F40XX)
#   ifndef CONFIG_STM32_DMA1
#     warning "STM32 F4 DAC DMA support requires CONFIG_STM32_DMA1"
#     undef CONFIG_STM32_DAC1_DMA
#     undef CONFIG_STM32_DAC2_DMA
#   endif
# else
#   warning "No DAC DMA information for this STM32 family"
#   undef CONFIG_STM32_DAC1_DMA
#   undef CONFIG_STM32_DAC2_DMA
# endif
#endif

/* If DMA is selected, then a timer and output frequency must also be
 * provided to support the DMA transfer.  The DMA transfer could be
 * supported by and EXTI trigger, but this feature is not currently
 * supported by the driver.
 */

#ifdef CONFIG_STM32_DAC1_DMA
#  if !defined(CONFIG_STM32_DAC1_TIMER)
#    warning "A timer number must be specificed in CONFIG_STM32_DAC1_TIMER"
#    undef CONFIG_STM32_DAC1_DMA
#    undef CONFIG_STM32_DAC1_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC1_TIMER_FREQUENCY)
#    warning "A timer frequency must be specificed in CONFIG_STM32_DAC1_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC1_DMA
#    undef CONFIG_STM32_DAC1_TIMER
#  endif
#endif

#ifdef CONFIG_STM32_DAC2_DMA
#  if !defined(CONFIG_STM32_DAC2_TIMER)
#    warning "A timer number must be specificed in CONFIG_STM32_DAC2_TIMER"
#    undef CONFIG_STM32_DAC2_DMA
#    undef CONFIG_STM32_DAC2_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC2_TIMER_FREQUENCY)
#    warning "A timer frequency must be specificed in CONFIG_STM32_DAC2_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC2_DMA
#    undef CONFIG_STM32_DAC2_TIMER
#  endif
#endif

/* DMA *********************************************************************/
/* DMA channels and interface values differ for the F1 and F4 families */

#undef HAVE_DMA
#if defined(CONFIG_STM32_DAC1_DMA) || defined(CONFIG_STM32_DAC2_DMA)
# if defined(CONFIG_STM32_STM32F10XX)
#   define HAVE_DMA        1
#   define DAC_DMA         2
#   define DAC1_DMA_CHAN   DMACHAN_DAC_CHAN1
#   define DAC2_DMA_CHAN   DMACHAN_DAC_CHAN2
# elif defined(CONFIG_STM32_STM32F40XX)
#   define HAVE_DMA        1
#   define DAC_DMA         1
#   define DAC1_DMA_CHAN   DMAMAP_DAC1
#   define DAC2_DMA_CHAN   DMAMAP_DAC2
#  endif
#endif

/* Timer configuration.  The STM32 supports 8 different trigger for DAC
 * output:
 *
 * TSEL SOURCE                  DEVICES
 * ---- ----------------------- -------------------------------------
 * 000  Timer 6 TRGO event      ALL
 * 001  Timer 3 TRGO event      STM32 F1 Connectivity Line
 *      Timer 8 TRGO event      Other STM32 F1 and all STM32 F4
 * 010  Timer 7 TRGO event      ALL
 * 011  Timer 5 TRGO event      ALL
 * 100  Timer 2 TRGO event      ALL
 * 101  Timer 4 TRGO event      ALL
 * 110  EXTI line9              ALL
 * 111  SWTRIG Software control ALL
 *
 * This driver does not support the EXTI trigger.
 */

#ifdef CONFIG_STM32_DAC1_DMA
#  if CONFIG_STM32_DAC1_TIMER == 6
#    ifndef CONFIG_STM32_TIM6_DAC
#      error "CONFIG_STM32_TIM6_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC1_TIMER_BASE           STM32_TIM6_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1_TIMER == 3 && defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM3_DAC
#      error "CONFIG_STM32_TIM3_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC1_TIMER_BASE           STM32_TIM3_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1_TIMER == 8 && !defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM8_DAC
#      error "CONFIG_STM32_TIM8_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC1_TIMER_BASE           STM32_TIM8_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32_PCLK2_FREQUENCY
#  elif CONFIG_STM32_DAC1_TIMER == 7
#    ifndef CONFIG_STM32_TIM7_DAC
#      error "CONFIG_STM32_TIM7_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE DAC_CR_TSEL_TIM7
#    define DAC1_TIMER_BASE STM32_TIM7_BASE
#  elif CONFIG_STM32_DAC1_TIMER == 5
#    ifndef CONFIG_STM32_TIM5_DAC
#      error "CONFIG_STM32_TIM5_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC1_TIMER_BASE           STM32_TIM5_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1_TIMER == 2
#    ifndef CONFIG_STM32_TIM2_DAC
#      error "CONFIG_STM32_TIM2_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC1_TIMER_BASE           STM32_TIM2_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1_TIMER == 4
#    ifndef CONFIG_STM32_TIM4_DAC
#      error "CONFIG_STM32_TIM4_DAC required for DAC1"
#    endif
#    define DAC1_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC1_TIMER_BASE           STM32_TIM4_BASE
#    define DAC1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32_DAC1_TIMER"
#  endif
#else
#  define DAC1_TSEL_VALUE DAC_CR_TSEL_SW
#endif

#ifdef CONFIG_STM32_DAC2_DMA
#  if CONFIG_STM32_DAC2_TIMER == 6
#    ifndef CONFIG_STM32_TIM6_DAC
#      error "CONFIG_STM32_TIM6_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC2_TIMER_BASE           STM32_TIM6_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2_TIMER == 3 && defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM3_DAC
#      error "CONFIG_STM32_TIM3_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC2_TIMER_BASE           STM32_TIM3_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2_TIMER == 8 && !defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM8_DAC
#      error "CONFIG_STM32_TIM8_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC2_TIMER_BASE           STM32_TIM8_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK2_FREQUENCY
#  elif CONFIG_STM32_DAC2_TIMER == 7
#    ifndef CONFIG_STM32_TIM7_DAC
#      error "CONFIG_STM32_TIM7_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM7
#    define DAC2_TIMER_BASE           STM32_TIM7_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2_TIMER == 5
#    ifndef CONFIG_STM32_TIM5_DAC
#      error "CONFIG_STM32_TIM5_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC2_TIMER_BASE           STM32_TIM5_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2_TIMER == 2
#    ifndef CONFIG_STM32_TIM2_DAC
#      error "CONFIG_STM32_TIM2_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC2_TIMER_BASE           STM32_TIM2_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2_TIMER == 4
#    ifndef CONFIG_STM32_TIM4_DAC
#      error "CONFIG_STM32_TIM4_DAC required for DAC2"
#    endif
#    define DAC2_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC2_TIMER_BASE           STM32_TIM4_BASE
#    define DAC2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32_DAC2_TIMER"
#  endif
#else
#  define DAC2_TSEL_VALUE DAC_CR_TSEL_SW
#endif

/* Calculate timer divider values based upon DACn_TIMER_PCLK_FREQUENCY and
 * CONFIG_STM32_DACn_TIMER_FREQUENCY.
 */
#warning "Missing Logic"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the single STM32 DAC block */

struct stm32_dac_s
{
  uint8_t    init   : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one STM32 DAC channel */

struct stm32_chan_s
{
  uint8_t    inuse  : 1; /* True, the driver is in use and not available */
#ifdef HAVE_DMA
  uint8_t    hasdma : 1; /* True, this channel supports DMA */
#endif
  uint8_t    intf;       /* DAC zero-based interface number (0 or 1) */
#ifdef HAVE_DMA
  uint16_t   dmachan;    /* DMA channel needed by this DAC */
  DMA_HANDLE dma;        /* Allocated DMA channel */
  uint32_t   tsel;       /* CR trigger select value */
  uint32_t   tbase;      /* Timer base address */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* DAC Register access */

#ifdef HAVE_DMA
static uint32_t tim_getreg(struct stm32_chan_s *chan, int offset);
static void tim_putreg(struct stm32_chan_s *chan, int offset, uint32_t value);
#endif

/* Interrupt handler */

#ifdef CONFIG_STM32_STM32F40XX
static int  dac_interrupt(int irq, void *context);
#endif

/* DAC methods */

static void dac_reset(FAR struct dac_dev_s *dev);
static int  dac_setup(FAR struct dac_dev_s *dev);
static void dac_shutdown(FAR struct dac_dev_s *dev);
static void dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);
static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);
static int  dac_interrupt(int irq, void *context);

/* Initialization */

#ifdef HAVE_DMA
static int  dac_timinit(struct stm32_chan_s *chan);
#endif
static int  dac_chaninit(struct stm32_chan_s *chan);
static int  dac_blockinit(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = dac_reset,
  .ao_setup    = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint    = dac_txint,
  .ao_send     = dac_send,
  .ao_ioctl    = dac_ioctl,
};

#ifdef CONFIG_STM32_DAC1
static struct stm32_chan_s g_dac1priv =
{
  .intf       = 0;
#ifdef CONFIG_STM32_DAC1_DMA
  .hasdma     = 1;
  .dmachan    = DAC1_DMA_CHAN,
  .tsel       = DAC1_TSEL_VALUE,
  .tbase      = DAC1_TIMER_BASE
#endif
}

static struct dac_dev_s g_dac1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1priv,
};
#endif

#ifdef CONFIG_STM32_DAC2
static struct stm32_chan_s g_dac2priv =
{
  .intf       = 1;
#ifdef CONFIG_STM32_DAC2_DMA
  .hasdma     = 1;
  .dmachan    = DAC2_DMA_CHAN,
  .tsel       = DAC2_TSEL_VALUE.
  .tbase      = DAC2_TIMER_BASE
#endif
}

static struct dac_dev_s g_dac2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac2priv,
};
#endif

static struct stm32_dac_s g_dacblock; 

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tim_getreg
 *
 * Description:
 *   Read the value of an DMA timer register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static uint32_t tim_getreg(struct stm32_chan_s *chan, int offset)
{
  return getreg32(chan->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Read the value of an DMA timer register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tim_putreg(struct stm32_chan_s *chan, int offset, uint32_t value)
{
  putreg32(value, chan->tbase + offset);
}
#endif

/****************************************************************************
 * Name: dac_interrupt
 *
 * Description:
 *   DAC interrupt handler.  The STM32 F4 family supports a only a DAC
 *   underrun interrupt.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32F40XX
static int dac_interrupt(int irq, void *context)
{
#warning "Missing logic"
  return OK;
}
#endif

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called early to initialize the hardware. This
 *   is called, before dac_setup() and on error conditions.
 *
 *   NOTE:  DAC reset will reset both DAC channels!
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(FAR struct dac_dev_s *dev)
{
  irqstate_t flags;
  uint32_t regval;

  /* Reset only the selected DAC channel; the other DAC channel must remain
   * functional.
   */

  flags   = irqsave();

#warning "Missing logic"

  irqrestore(flags);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts.  Interrupts
 *   are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_setup(FAR struct dac_dev_s *dev)
{
# warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   Disable the DAC.  This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_shutdown(FAR struct dac_dev_s *dev)
{
# warning "Missing logic"
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
# warning "Missing logic"
}

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *   Set the DAC output.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
#ifdef HAVE_DMA
  if (priv->hasdma)
    {
      /* Configure the DMA stream/channel.
       *
       * - Channel number
       * - Peripheral address
       * - Direction: Memory to peripheral
       * - Disable peripheral address increment
       * - Enable memory address increment
       * - Peripheral data size: half word
       * - Mode: circular???
       * - Priority: ?
       * - FIFO mode: disable
       * - FIFO threshold: half full
       * - Memory Burst: single
       * - Peripheral Burst: single
       */
#warning "Missing logic"
 
      /* Enable DMA */
#warning "Missing logic"

      /* Enable DAC Channel */
#warning "Missing logic"

      /* Enable DMA for DAC Channel */
#warning "Missing logic"
    }
  else
    {
      /* Non-DMA transfer */
#warning "Missing logic"
    }
  return -ENOSYS;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: dac_timinit
 *
 * Description:
 *   Initialize the timer that drivers the DAC DMA for this channel using
 *   the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static int dac_timinit(struct stm32_chan_s *chan)
{
  /* Configure the time base: Timer period, prescaler, clock division,
   * counter mode (up).
   */
#warning "Missing Logic"

  /* Selection TRGO selection: update */
#warning "Missing Logic"
  
  /* Enable the counter */
#warning "Missing Logic"
}
#endif

/****************************************************************************
 * Name: dac_chaninit
 *
 * Description:
 *   Initialize the DAC channel.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_chaninit(struct stm32_chan_s *chan)
{
  /* Is the selected channel already in-use? */

  if (chan->inuse)
    {
      /* Yes.. then return EBUSY */

      return -EBUSY;
    }

  /* Configure the DAC output pin:
   *
   * DAC -" Once the DAC channelx is enabled, the corresponding GPIO pin
   * (PA4 or PA5) is automatically connected to the analog converter output
   * (DAC_OUTx). In order to avoid parasitic consumption, the PA4 or PA5 pin
   * should first be configured to analog (AIN)".
   */

  stm32_configgpio(chan->intf ? GPIO_DAC2_OUT : GPIO_DAC1_OUT);

  /* DAC channel configuration:
   *
   * - Set the trigger selection based upon the configuration.
   * - Set wave generation == None.
   * - Enable the output buffer.
   */
#warning "Missing logic"
  
  /* Determine if DMA is supported by this channel */

#ifdef HAVE_DMA
  if (priv->hasdma)
    {
      /* Yes.. allocate a DMA channel */

      priv->dma = stm32_dmachannel(priv->dmachan);
      if (!priv->dma)
        {
          adbg("Failed to allocate a DMA channel\n");
          return -EBUSY;
        }

      /* Configure the timer that supports the DMA operation */

      ret = dac_timinit(chan);
      if (ret < 0)
        {
          adbg("Failed to initialize the DMA timer: %d\n", ret);
          return ret;
        }
    }
#endif

  /* Mark the DAC channel "in-use" */

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: dac_blockinit
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_blockinit(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Has the DMA block already been initialized? */

  if (g_dacblock.init)
    {
      /* Yes.. then return success  We only have to do this once */

      return OK;
    }

  /* Put the entire DAC block in reset state */

  flags   = irqsave();
  regval  = getreg32(STM32_RCC_APB1RSTR);
  regval |= RCC_APB1RSTR_DACRST
  putreg32(regval, STM32_RCC_APB1RSTR);

  /* Take the DAC out of reset state */

  regval &= ~RCC_APB1RSTR_DACRST
  putreg32(regval, STM32_RCC_APB1RSTR);
  irqrestore(flags);

  /* Mark the DAC block as initialized */

  g_dacblock.init = 1;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the DAC block has enabled,
 *   2. Board-specific logic has already configured 
 *
 ****************************************************************************/

FAR struct dac_dev_s *stm32_dacinitialize(int intf)
{
  FAR struct dac_dev_s    *dev;
  FAR struct stm32_chan_s *chan;
  int ret;
  
#ifdef CONFIG_STM32_DAC1
  if (intf == 1)
    {
      avdbg("DAC1 Selected\n");
      dev = &g_dacdev1;
    }
  else
#endif
#ifdef CONFIG_STM32_DAC2
  if (intf == 2)
    {
      avdbg("DAC2 Selected\n");
      dev = &g_dac2dev;
    }
  else
#endif
    {
      adbg("No such DAC interface: %d\n", intf);
      return NULL;
    }

  /* Make sure that the DAC block has been initialized */

  ret = dac_blockinit();
  if (ret < 0)
    {
      adbg("Failed to initialize the DAC block: %d\n", ret);
      return ret;
    }

  /* Configure the selected DAC channel */

  chan = dev->ad_priv;
  ret  = dac_chaninit(chan);
  if (ret < 0)
    {
      adbg("Failed to initialize DAC channel %d: %d\n", intf, ret);
      return ret;
    }

  return dev;
}

#endif /* CONFIG_STM32_DAC1 || CONFIG_STM32_DAC2 */
#endif /* CONFIG_DAC */
