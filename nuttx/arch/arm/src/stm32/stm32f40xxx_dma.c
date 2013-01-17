/****************************************************************************
 * arch/arm/src/stm32/stm32f40xxx_dma.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"
#include "chip.h"
#include "stm32_dma.h"
#include "stm32_internal.h"

/* This file supports only the STM32 F4 family (an probably the F2 family
 * as well?)
 */

#if defined(CONFIG_STM32_STM32F40XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA1_NSTREAMS    8
#if STM32_NDMA > 1
#  define DMA2_NSTREAMS  8
#  define DMA_NSTREAMS   (DMA1_NSTREAMS+DMA2_NSTREAMS)
#else
#  define DMA_NSTREAMS   DMA1_NSTREAMS
#endif

#ifndef CONFIG_DMA_PRI
#  define CONFIG_DMA_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

/* Convert the DMA stream base address to the DMA register block address */
 
#define DMA_BASE(ch)     (ch & 0xfffffc00)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes one DMA channel */

struct stm32_dma_s
{
  uint8_t        stream;   /* DMA stream number (0-7) */
  uint8_t        irq;      /* DMA stream IRQ number */
  uint8_t        shift;    /* ISR/IFCR bit shift value */
  uint8_t        channel;  /* DMA channel number (0-7) */
  sem_t          sem;      /* Used to wait for DMA channel to become available */
  uint32_t       base;     /* DMA register channel base address */
  dma_callback_t callback; /* Callback invoked when the DMA completes */
  void          *arg;      /* Argument passed to callback function */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

static struct stm32_dma_s g_dma[DMA_NSTREAMS] =
{
  {
    .stream   = 0,
    .irq      = STM32_IRQ_DMA1S0,
    .shift    = DMA_INT_STREAM0_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(0),
  },
  {
    .stream   = 1,
    .irq      = STM32_IRQ_DMA1S1,
    .shift    = DMA_INT_STREAM1_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(1),
  },
  {
    .stream   = 2,
    .irq      = STM32_IRQ_DMA1S2,
    .shift    = DMA_INT_STREAM2_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(2),
  },
  {
    .stream   = 3,
    .irq      = STM32_IRQ_DMA1S3,
    .shift    = DMA_INT_STREAM3_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(4),
  },
  {
    .stream   = 4,
    .irq      = STM32_IRQ_DMA1S4,
    .shift    = DMA_INT_STREAM4_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(4),
  },
  {
    .stream   = 5,
    .irq      = STM32_IRQ_DMA1S5,
    .shift    = DMA_INT_STREAM5_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(5),
  },
  {
    .stream   = 6,
    .irq      = STM32_IRQ_DMA1S6,
    .shift    = DMA_INT_STREAM6_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(6),
  },
  {
    .stream   = 7,
    .irq      = STM32_IRQ_DMA1S7,
    .shift    = DMA_INT_STREAM7_SHIFT,
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(7),
  },
#if STM32_NDMA > 1
  {
    .stream   = 0,
    .irq      = STM32_IRQ_DMA2S0,
    .shift    = DMA_INT_STREAM0_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(0),
  },
  {
    .stream   = 1,
    .irq      = STM32_IRQ_DMA2S1,
    .shift    = DMA_INT_STREAM1_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(1),
  },
  {
    .stream   = 2,
    .irq      = STM32_IRQ_DMA2S2,
    .shift    = DMA_INT_STREAM2_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(2),
  },
  {
    .stream   = 3,
    .irq      = STM32_IRQ_DMA2S3,
    .shift    = DMA_INT_STREAM3_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(3),
  },
  {
    .stream   = 4,
    .irq      = STM32_IRQ_DMA2S4,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(4),
  },
  {
    .stream   = 5,
    .irq      = STM32_IRQ_DMA2S5,
    .shift    = DMA_INT_STREAM5_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(5),
  },
  {
    .stream   = 6,
    .irq      = STM32_IRQ_DMA2S6,
    .shift    = DMA_INT_STREAM6_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(6),
  },
  {
    .stream   = 7,
    .irq      = STM32_IRQ_DMA2S7,
    .shift    = DMA_INT_STREAM7_SHIFT,
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(7),
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/* Get non-channel register from DMA1 or DMA2 */

static inline uint32_t dmabase_getreg(struct stm32_dma_s *dmast, uint32_t offset)
{
  return getreg32(DMA_BASE(dmast->base) + offset);
}

/* Write to non-channel register in DMA1 or DMA2 */

static inline void dmabase_putreg(struct stm32_dma_s *dmast, uint32_t offset, uint32_t value)
{
  putreg32(value, DMA_BASE(dmast->base) + offset);
}

/* Get channel register from DMA1 or DMA2 */

static inline uint32_t dmast_getreg(struct stm32_dma_s *dmast, uint32_t offset)
{
  return getreg32(dmast->base + offset);
}

/* Write to channel register in DMA1 or DMA2 */

static inline void dmast_putreg(struct stm32_dma_s *dmast, uint32_t offset, uint32_t value)
{
  putreg32(value, dmast->base + offset);
}

/************************************************************************************
 * Name: stm32_dmatake() and stm32_dmagive()
 *
 * Description:
 *   Used to get exclusive access to a DMA channel.
 *
 ************************************************************************************/

static void stm32_dmatake(FAR struct stm32_dma_s *dmast)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&dmast->sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was awakened
       * by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void stm32_dmagive(FAR struct stm32_dma_s *dmast)
{
  (void)sem_post(&dmast->sem);
}

/************************************************************************************
 * Name: stm32_dmastream
 *
 * Description:
 *   Get the g_dma table entry associated with a DMA controller and a stream number
 *
 ************************************************************************************/

static inline FAR struct stm32_dma_s *stm32_dmastream(unsigned int stream,
                                                      unsigned int controller)
{
  int index;

  DEBUGASSERT(stream < DMA_NSTREAMS && controller < STM32_NDMA);

  /* Convert the controller + stream based on the fact that there are 8 streams
   * per controller.
   */

#if STM32_NDMA > 1
  index = controller << 3 | stream;
#else
  index = stream;
#endif

  /* Then return the stream structure associated with the stream index */

  return &g_dma[index];
}

/************************************************************************************
 * Name: stm32_dmamap
 *
 * Description:
 *   Get the g_dma table entry associated with a bit-encoded DMA selection
 *
 ************************************************************************************/

static inline FAR struct stm32_dma_s *stm32_dmamap(unsigned long dmamap)
{
  /* Extract the DMA controller number from the bit encoded value */

  unsigned int controller = STM32_DMA_CONTROLLER(dmamap);

  /* Extact the stream number from the bit encoded value */

  unsigned int stream = STM32_DMA_STREAM(dmamap);

  /* Return the table entry associated with the controller + stream */

  return stm32_dmastream(stream, controller);
}

/************************************************************************************
 * Name: stm32_dmastreamdisable
 *
 * Description:
 *  Disable the DMA stream
 *
 ************************************************************************************/

static void stm32_dmastreamdisable(struct stm32_dma_s *dmast)
{
  uint32_t regoffset;
  uint32_t regval;

  /* Disable all interrupts at the DMA controller */

  regval = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regval &= ~DMA_SCR_ALLINTS;

  /* Disable the DMA stream */

  regval &= ~DMA_SCR_EN;
  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, regval);

  /* Clear pending stream interrupts by setting bits in the upper or lower IFCR
   * register
   */

  if (dmast->stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }
  
  dmabase_putreg(dmast, regoffset, (DMA_STREAM_MASK << dmast->shift));
}

/************************************************************************************
 * Name: stm32_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ************************************************************************************/

static int stm32_dmainterrupt(int irq, void *context)
{
  struct stm32_dma_s *dmast;
  uint32_t status;
  uint32_t regoffset = 0;
  unsigned int stream = 0;
  unsigned int controller = 0;

  /* Get the stream and the controller that generated the interrupt */

  if (irq >= STM32_IRQ_DMA1S0 && irq <= STM32_IRQ_DMA1S6)
    {
      stream     = irq - STM32_IRQ_DMA1S0;
      controller = DMA1;
    }
  else if (irq == STM32_IRQ_DMA1S7)
    {
      stream     = 7;
      controller = DMA1;
    }
  else
#if STM32_NDMA > 1
  if (irq >= STM32_IRQ_DMA2S0 && irq <= STM32_IRQ_DMA2S4)
    {
      stream     = irq - STM32_IRQ_DMA2S0;
      controller = DMA2;
    }
  else if (irq >= STM32_IRQ_DMA2S5 && irq <= STM32_IRQ_DMA2S7)
    {
      stream     = irq - STM32_IRQ_DMA2S5 + 5;
      controller = DMA2;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }

  /* Get the stream structure from the stream and controller numbers */

  dmast = stm32_dmastream(stream, controller);

  /* Select the interrupt status register (either the LISR or HISR)
   * based on the stream number that caused the interrupt.
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LISR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HISR_OFFSET;
    }

  /* Get the interrupt status for this stream */

  status = (dmabase_getreg(dmast, regoffset) >> dmast->shift) & DMA_STREAM_MASK;

  /* Clear fetched stream interrupts by setting bits in the upper or lower IFCR
   * register
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmast, regoffset, (status << dmast->shift));

  /* Invoke the callback */

  if (dmast->callback)
    {
      dmast->callback(dmast, status, dmast->arg);
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  struct stm32_dma_s *dmast;
  int stream;

  /* Initialize each DMA stream */

  for (stream = 0; stream < DMA_NSTREAMS; stream++)
    {
      dmast = &g_dma[stream];
      sem_init(&dmast->sem, 0, 1);

      /* Attach DMA interrupt vectors */

      (void)irq_attach(dmast->irq, stm32_dmainterrupt);

      /* Disable the DMA stream */

      stm32_dmastreamdisable(dmast);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmast->irq);
 
      /* Set the interrrupt priority */

      up_prioritize_irq(dmast->irq, CONFIG_DMA_PRI);
    }
}

/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   DMA channels are shared on the STM32:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32_dma.h.
 *
 *   If the DMA channel is not available, then stm32_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Input parameter:
 *   dmamap - Identifies the stream/channel resource. For the STM32 F4, this
 *     is a bit-encoded  value as provided by the the DMAMAP_* definitions
 *     in chip/stm32f40xxx_dma.h
 *
 * Returned Value:
 *   Provided that 'dmamap' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'dmamap' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(unsigned int dmamap)
{
  FAR struct stm32_dma_s *dmast;

  /* Get the stream index from the bit-encoded channel value */

  dmast = stm32_dmamap(dmamap);
  DEBUGASSERT(dmast != NULL);

  /* Get exclusive access to the DMA channel -- OR wait until the channel
   * is available if it is currently being used by another driver
   */

  stm32_dmatake(dmast);

  /* The caller now has exclusive use of the DMA channel.  Assign the
   * channel to the stream and return an opaque reference to the stream
   * structure.
   */

  dmast->channel = STM32_DMA_CHANNEL(dmamap);
  return (DMA_HANDLE)dmast;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to stm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void stm32_dmafree(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Release the channel */

  stm32_dmagive(dmast);
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t scr)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t regoffset;
  uint32_t regval;

  dmadbg("paddr: %08x maddr: %08x ntransfers: %d scr: %08x\n",
         paddr, maddr, ntransfers, scr);

  /* "If the stream is enabled, disable it by resetting the EN bit in the
   * DMA_SxCR register, then read this bit in order to confirm that there is no
   * ongoing stream operation. Writing this bit to 0 is not immediately
   * effective since it is actually written to 0 once all the current transfers
   * have finished. When the EN bit is read as 0, this means that the stream is
   * ready to be configured. It is therefore necessary to wait for the EN bit
   * to be cleared before starting any stream configuration. ..."
   */

  while ((dmast_getreg(dmast, STM32_DMA_SCR_OFFSET) & DMA_SCR_EN) != 0);

  /* "... All the stream dedicated bits set in the status register (DMA_LISR
   * and DMA_HISR) from the previous data block DMA transfer should be cleared
   * before the stream can be re-enabled."
   *
   * Clear pending stream interrupts by setting bits in the upper or lower IFCR
   * register
   */

  if (dmast->stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }
  
  dmabase_putreg(dmast, regoffset, (DMA_STREAM_MASK << dmast->shift));

  /* "Set the peripheral register address in the DMA_SPARx register. The data
   *  will be moved from/to this address to/from the memory after the
   *  peripheral event.
   */

  dmast_putreg(dmast, STM32_DMA_SPAR_OFFSET, paddr);

  /* "Set the memory address in the DMA_SM0ARx ... register. The data will be
   *  written to or read from this memory after the peripheral event."
   *
   * Note that in double-buffered mode it is explicitly assumed that the second
   * buffer immediately follows the first.
   */

  dmast_putreg(dmast, STM32_DMA_SM0AR_OFFSET, maddr);
  if (scr & DMA_SCR_DBM)
    {
      dmast_putreg(dmast, STM32_DMA_SM1AR_OFFSET, maddr + ntransfers);
    }

  /* "Configure the total number of data items to be transferred in the
   *  DMA_SNDTRx register.  After each peripheral event, this value will be
   *  decremented."
   *
   * "When the peripheral flow controller is used for a given stream, the value
   *  written into the DMA_SxNDTR has no effect on the DMA transfer. Actually,
   *  whatever the value written, it will be forced by hardware to 0xFFFF as soon
   *  as the stream is enabled..."
   */

  dmast_putreg(dmast, STM32_DMA_SNDTR_OFFSET, ntransfers);

  /* "Select the DMA channel (request) using CHSEL[2:0] in the DMA_SxCR register."
   *
   * "Configure the stream priority using the PL[1:0] bits in the DMA_SCRx"
   *  register."
   */

  regval  = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regval &= ~(DMA_SCR_PL_MASK|DMA_SCR_CHSEL_MASK);
  regval |= scr & DMA_SCR_PL_MASK;
  regval |= (uint32_t)dmast->channel << DMA_SCR_CHSEL_SHIFT;
  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, regval);

  /* "Configure the FIFO usage (enable or disable, threshold in transmission and
   *  reception)"
   *
   * "Caution is required when choosing the FIFO threshold (bits FTH[1:0] of the
   *  DMA_SxFCR register) and the size of the memory burst (MBURST[1:0] of the
   *  DMA_SxCR register): The content pointed by the FIFO threshold must exactly
   *  match to an integer number of memory burst transfers. If this is not in the
   *  case, a FIFO error (flag FEIFx of the DMA_HISR or DMA_LISR register) will be
   *  generated when the stream is enabled, then the stream will be automatically
   *  disabled."
   *
   * The FIFO is disabled in circular mode when transferring data from a 
   * peripheral to memory, as in this case it is usually desirable to know that
   * every byte from the peripheral is transferred immediately to memory.  It is
   * not practical to flush the DMA FIFO, as this requires disabling the channel
   * which triggers the transfer-complete interrupt.
   *
   * NOTE: The FEIFx error interrupt is not enabled because the FEIFx seems to
   * be reported spuriously causing good transfers to be marked as failures.
   */

  regval  = dmast_getreg(dmast, STM32_DMA_SFCR_OFFSET);
  regval &= ~(DMA_SFCR_FTH_MASK | DMA_SFCR_FS_MASK | DMA_SFCR_FEIE);
  if (!((scr & (DMA_SCR_CIRC | DMA_SCR_DIR_MASK)) == (DMA_SCR_CIRC | DMA_SCR_DIR_P2M)))
    {
      regval |= (DMA_SFCR_FTH_FULL | DMA_SFCR_DMDIS);
    }
  dmast_putreg(dmast, STM32_DMA_SFCR_OFFSET, regval);

  /* "Configure data transfer direction, circular mode, peripheral & memory
   *  incremented mode, peripheral & memory data size, and interrupt after
   *  half and/or full transfer in the DMA_CCRx register."
   *
   * Note: The CT bit is always reset.
   */

  regval  = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regval &= ~(DMA_SCR_PFCTRL|DMA_SCR_DIR_MASK|DMA_SCR_PINC|DMA_SCR_MINC|
              DMA_SCR_PSIZE_MASK|DMA_SCR_MSIZE_MASK|DMA_SCR_PINCOS|
              DMA_SCR_CIRC|DMA_SCR_DBM|DMA_SCR_CT|
              DMA_SCR_PBURST_MASK|DMA_SCR_MBURST_MASK);
  scr    &=  (DMA_SCR_PFCTRL|DMA_SCR_DIR_MASK|DMA_SCR_PINC|DMA_SCR_MINC|
              DMA_SCR_PSIZE_MASK|DMA_SCR_MSIZE_MASK|DMA_SCR_PINCOS|
              DMA_SCR_DBM|DMA_SCR_CIRC|
              DMA_SCR_PBURST_MASK|DMA_SCR_MBURST_MASK);
  regval |= scr;
  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg, bool half)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t scr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info.  This will be invoked whent the DMA commpletes */

  dmast->callback = callback;
  dmast->arg      = arg;

  /* Activate the stream by setting the ENABLE bit in the DMA_SCRx register.
   * As soon as the stream is enabled, it can serve any DMA request from the
   * peripheral connected on the stream.
   */

  scr  = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  scr |= DMA_SCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * and double-buffered modes, always interrupt on buffer wrap, and optionally
   * interrupt at the halfway point.
   */

  if ((scr & (DMA_SCR_DBM|DMA_SCR_CIRC)) == 0)
    {
      /* Once half of the bytes are transferred, the half-transfer flag (HTIF) is
       * set and an interrupt is generated if the Half-Transfer Interrupt Enable
       * bit (HTIE) is set. At the end of the transfer, the Transfer Complete Flag
       * (TCIF) is set and an interrupt is generated if the Transfer Complete
       * Interrupt Enable bit (TCIE) is set.
       */

      scr |= (half ? (DMA_SCR_HTIE|DMA_SCR_TEIE) : (DMA_SCR_TCIE|DMA_SCR_TEIE));      
    }
  else
    {
      /* In non-stop modes, when the transfer completes it immediately resets
       * and starts again.  The transfer-complete interrupt is thus always
       * enabled, and the half-complete interrupt can be used in circular 
       * mode to determine when the buffer is half-full, or in double-buffered
       * mode to determine when one of the two buffers is full.
       */

      scr |= (half ? DMA_SCR_HTIE : 0) | DMA_SCR_TCIE | DMA_SCR_TEIE;
    }

  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, scr);
}

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  stm32_dmastreamdisable(dmast);
}

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Read the DMA bytes-remaining register.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t residual;

  /* Fetch the count of bytes remaining to be transferred. 
   *
   * If the FIFO is enabled, this count may be inaccurate.  ST don't
   * appear to document whether this counts the peripheral or the memory
   * side of the channel, and they don't make the memory pointer
   * available either.
   *
   * For reception in circular mode the FIFO is disabled in order that
   * this value can be useful.
   */

  residual = dmast_getreg(dmast, STM32_DMA_SNDTR_OFFSET);

  return (size_t)residual;
}

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  irqstate_t flags;

  flags       = irqsave();
  regs->lisr  = dmabase_getreg(dmast, STM32_DMA_LISR_OFFSET);
  regs->hisr  = dmabase_getreg(dmast, STM32_DMA_HISR_OFFSET);
  regs->scr   = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regs->sndtr = dmast_getreg(dmast, STM32_DMA_SNDTR_OFFSET);
  regs->spar  = dmast_getreg(dmast, STM32_DMA_SPAR_OFFSET);
  regs->sm0ar = dmast_getreg(dmast, STM32_DMA_SM0AR_OFFSET);
  regs->sm1ar = dmast_getreg(dmast, STM32_DMA_SM1AR_OFFSET);
  regs->sfcr  = dmast_getreg(dmast, STM32_DMA_SFCR_OFFSET);
  irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void stm32_dmadump(DMA_HANDLE handle, const struct stm32_dmaregs_s *regs,
                   const char *msg)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t dmabase = DMA_BASE(dmast->base);

  dmadbg("DMA Registers: %s\n", msg);
  dmadbg("   LISR[%08x]: %08x\n", dmabase + STM32_DMA_LISR_OFFSET, regs->lisr);
  dmadbg("   HISR[%08x]: %08x\n", dmabase + STM32_DMA_HISR_OFFSET, regs->hisr);
  dmadbg("    SCR[%08x]: %08x\n", dmast->base + STM32_DMA_SCR_OFFSET, regs->scr);
  dmadbg("  SNDTR[%08x]: %08x\n", dmast->base + STM32_DMA_SNDTR_OFFSET, regs->sndtr);
  dmadbg("   SPAR[%08x]: %08x\n", dmast->base + STM32_DMA_SPAR_OFFSET, regs->spar);
  dmadbg("  SM0AR[%08x]: %08x\n", dmast->base + STM32_DMA_SM0AR_OFFSET, regs->sm0ar);
  dmadbg("  SM1AR[%08x]: %08x\n", dmast->base + STM32_DMA_SM1AR_OFFSET, regs->sm1ar);
  dmadbg("   SFCR[%08x]: %08x\n", dmast->base + STM32_DMA_SFCR_OFFSET, regs->sfcr);
}
#endif

#endif /* CONFIG_STM32_STM32F40XX */
