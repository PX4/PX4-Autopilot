/****************************************************************************
 * arch/arm/src/sam3u-ek/sam3u_dmac.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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

#include "sam3u_internal.h"
#include "sam3u_pmc.h"
#include "sam3u_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Condition out the whole file unless DMA is selected in the configuration */

#ifdef CONFIG_SAM3U_DMA

/* If AT90SAM3U support is enabled, then OS DMA support should also be enabled */

#ifndef CONFIG_ARCH_DMA
#  warning "ATSAM3U DMA enabled but CONFIG_ARCH_DMA disabled"
#endif

/* Check the number of link list descriptors to allocate */

#ifndef CONFIG_SAM3U_NLLDESC
#  define CONFIG_SAM3U_NLLDESC CONFIG_SAM3U_NDMACHAN
#endif

#if CONFIG_SAM3U_NLLDESC < CONFIG_SAM3U_NDMACHAN
#  warning "At least CONFIG_SAM3U_NDMACHAN descriptors must be allocated"

#  undef CONFIG_SAM3U_NLLDESC
#  define CONFIG_SAM3U_NLLDESC CONFIG_SAM3U_NDMACHAN
#endif

/* Register values **********************************************************/

#define DMACHAN_CTRLB_BOTHDSCR \
  (DMACHAN_CTRLB_SRCDSCR | DMACHAN_CTRLB_DSTDSCR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes one DMA channel */

struct sam3u_dma_s
{
  uint8_t                chan;       /* DMA channel number (0-6) */
  bool                   inuse;      /* TRUE: The DMA channel is in use */
  uint32_t               flags;      /* DMA channel flags */
  uint32_t               base;       /* DMA register channel base address */
  uint32_t               cfg;        /* Pre-calculated CFG register for transfer */
  dma_callback_t         callback;   /* Callback invoked when the DMA completes */
  void                  *arg;        /* Argument passed to callback function */
  struct dma_linklist_s *llhead;     /* DMA link list head */
  struct dma_linklist_s *lltail;     /* DMA link list head */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These semaphores protect the DMA channel and descriptor tables */

static sem_t g_chsem;
static sem_t g_dsem;

/* CTRLA field lookups */

static const uint32_t g_srcwidth[3] =
{
  DMACHAN_CTRLA_SRCWIDTH_BYTE,
  DMACHAN_CTRLA_SRCWIDTH_HWORD,
  DMACHAN_CTRLA_SRCWIDTH_WORD
};

static const uint32_t g_destwidth[3] =
{
  DMACHAN_CTRLA_DSTWIDTH_BYTE,
  DMACHAN_CTRLA_DSTWIDTH_HWORD,
  DMACHAN_CTRLA_DSTWIDTH_WORD
};

static const uint32_t g_fifocfg[3] =
{
  DMACHAN_CFG_FIFOCFG_LARGEST,
  DMACHAN_CFG_FIFOCFG_HALF,
  DMACHAN_CFG_FIFOCFG_SINGLE
};

/* This array describes the available link list descriptors */

static struct dma_linklist_s g_linklist[CONFIG_SAM3U_NLLDESC];

/* This array describes the state of each DMA */

static struct sam3u_dma_s g_dma[CONFIG_SAM3U_NDMACHAN] =
{
#ifdef CONFIG_ARCH_CHIP_AT91SAM3U4E
  /* the AT91SAM3U4E has four DMA channels.  The FIFOs for channels 0-2 are
   * 8 bytes in size; channel 3 is 32 bytes.
   */

#if CONFIG_SAM3U_NDMACHAN != 4
#  error "Logic here assumes CONFIG_SAM3U_NDMACHAN is 4"
#endif

  {
    .chan     = 0,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM3U_DMACHAN0_BASE,
  },
  {
    .chan     = 1,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM3U_DMACHAN1_BASE,
  },
  {
    .chan     = 2,
    .flags    = DMACH_FLAG_FIFO_8BYTES,
    .base     = SAM3U_DMACHAN2_BASE,
  },
  {
    .chan     = 3,
    .flags    = (DMACH_FLAG_FIFO_32BYTES | DMACH_FLAG_FLOWCONTROL),
    .base     = SAM3U_DMACHAN3_BASE,
  }
#else
#  error "Nothing is known about the DMA channels for this device"
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_takechsem() and sam3u_givechsem()
 *
 * Description:
 *   Used to get exclusive access to the DMA channel table
 *
 ****************************************************************************/

static void sam3u_takechsem(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&g_chsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam3u_givechsem(void)
{
  (void)sem_post(&g_chsem);
}

/****************************************************************************
 * Name: sam3u_takedsem() and sam3u_givedsem()
 *
 * Description:
 *   Used to wait for availability of descriptors in the descriptor table.
 *
 ****************************************************************************/

static void sam3u_takedsem(void)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&g_dsem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

static inline void sam3u_givedsem(void)
{
  (void)sem_post(&g_dsem);
}

/****************************************************************************
 * Name: sam3u_fifosize
 *
 * Description:
 *  Decode the FIFO size from the flags
 *
 ****************************************************************************/

static unsigned int sam3u_fifosize(uint8_t dmach_flags)
{
  dmach_flags &= DMACH_FLAG_FIFOSIZE_MASK;
  if (dmach_flags == DMACH_FLAG_FIFO_8BYTES)
    {
      return 8;
    }
  else /* if (dmach_flags == DMACH_FLAG_FIFO_32BYTES) */
    {
      return 32;
    }
}

/****************************************************************************
 * Name: sam3u_flowcontrol
 *
 * Description:
 *  Decode the FIFO flow control from the flags
 *
 ****************************************************************************/

static inline bool sam3u_flowcontrol(uint8_t dmach_flags)
{
  return ((dmach_flags & DMACH_FLAG_FLOWCONTROL) != 0);
}

/****************************************************************************
 * Name: sam3u_fifocfg
 *
 * Description:
 *  Decode the FIFO config from the flags
 *
 ****************************************************************************/

static inline uint32_t sam3u_fifocfg(struct sam3u_dma_s *dmach)
{
  unsigned int ndx = (dmach->flags & DMACH_FLAG_FIFOCFG_MASK) >> DMACH_FLAG_FIFOCFG_SHIFT;
  DEBUGASSERT(ndx < 3);
  return g_fifocfg[ndx];
}

/****************************************************************************
 * Name: sam3u_txcfg
 *
 * Description:
 *  Decode the the flags to get the correct CFG register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam3u_txcfg(struct sam3u_dma_s *dmach)
{
  uint32_t regval;

  /* Set transfer (memory to peripheral) DMA channel configuration register */

  regval   = (((dmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT) << DMACHAN_CFG_SRCPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? DMACHAN_CFG_SRCH2SEL : 0;
  regval  |= (((dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT) << DMACHAN_CFG_DSTPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? DMACHAN_CFG_DSTH2SEL : 0;
  regval  |= sam3u_fifocfg(dmach);
  return regval;
}

/****************************************************************************
 * Name: sam3u_rxcfg
 *
 * Description:
 *  Decode the the flags to get the correct CFG register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam3u_rxcfg(struct sam3u_dma_s *dmach)
{
  uint32_t regval;

  /* Set received (peripheral to memory) DMA channel config */
                         
  regval   = (((dmach->flags & DMACH_FLAG_PERIPHPID_MASK) >> DMACH_FLAG_PERIPHPID_SHIFT) << DMACHAN_CFG_SRCPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_PERIPHH2SEL) != 0 ? DMACHAN_CFG_SRCH2SEL : 0;
  regval  |= (((dmach->flags & DMACH_FLAG_MEMPID_MASK) >> DMACH_FLAG_MEMPID_SHIFT) << DMACHAN_CFG_DSTPER_SHIFT);
  regval  |=   (dmach->flags & DMACH_FLAG_MEMH2SEL) != 0 ? DMACHAN_CFG_DSTH2SEL : 0;
  regval  |= sam3u_fifocfg(dmach);
  return regval;
}

/****************************************************************************
 * Name: sam3u_txctrlabits
 *
 * Description:
 *  Decode the the flags to get the correct CTRLA register bit settings for
 *  a transmit (memory to peripheral) transfer.  These are only the "fixed"
 *  CTRLA values and  need to be updated with the actual transfer size before
 *  being written to CTRLA sam3u_txctrla).
 *
 ****************************************************************************/

static inline uint32_t
sam3u_txctrlabits(struct sam3u_dma_s *dmach)
{
  uint32_t regval;
  unsigned int ndx;

  DEBUGASSERT(dmach);

  /* Since this is a transmit, the source is described by the memory selections.
   * Set the source width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval = g_srcwidth[ndx];

  /* Set the source chuck size (memory chunk size) */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_SCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_SCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_SCSIZE_1;
    }
#endif

  /* Since this is a transmit, the destination is described by the peripheral selections.
   * Set the destination width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_destwidth[ndx];

  /* Set the destination chuck size (peripheral chunk size) */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) == DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_DCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_DCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_DCSIZE_1;
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam3u_txctrla
 *
 * Description:
 *  Or in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam3u_txctrla(struct sam3u_dma_s *dmach,
                                     uint32_t dmasize, uint32_t txctrlabits)
{
  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  /* Adjust the the source transfer size for the source chunk size (memory
   * chunk size)
   */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      dmasize >>= 2;
    }
 
  DEBUGASSERT(dmasize <= DMACHAN_CTRLA_BTSIZE_MAX);
  return (txctrlabits & ~DMACHAN_CTRLA_BTSIZE_MASK) | (dmasize << DMACHAN_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam3u_rxctrlabits
 *
 * Description:
 *  Decode the the flags to get the correct CTRLA register bit settings for
 *  a read (peripheral to memory) transfer. These are only the "fixed" CTRLA
 *  values and need to be updated with the actual transfer size before being
 *  written to CTRLA sam3u_rxctrla).
 *
 ****************************************************************************/

static inline uint32_t sam3u_rxctrlabits(struct sam3u_dma_s *dmach)
{
  uint32_t     regval;
  unsigned int ndx;

  DEBUGASSERT(dmach);

  /* Since this is a receive, the source is described by the peripheral
   * selections. Set the source width (peripheral width).
   */

  ndx = (dmach->flags & DMACH_FLAG_PERIPHWIDTH_MASK) >> DMACH_FLAG_PERIPHWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval = g_srcwidth[ndx];

  /* Set the source chuck size (peripheral chunk size) */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) == DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_SCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_SCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_SCSIZE_1;
    }
#endif

  /* Since this is a receive, the destination is described by the memory selections.
   * Set the destination width (memory width).
   */

  ndx = (dmach->flags & DMACH_FLAG_MEMWIDTH_MASK) >> DMACH_FLAG_MEMWIDTH_SHIFT;
  DEBUGASSERT(ndx < 3);
  regval |= g_destwidth[ndx];

  /* Set the destination chuck size (memory chunk size) */

  if ((dmach->flags & DMACH_FLAG_MEMCHUNKSIZE) == DMACH_FLAG_MEMCHUNKSIZE_4)
    {
      regval |= DMACHAN_CTRLA_DCSIZE_4;
    }
#if 0 /* DMACHAN_CTRLA_DCSIZE_1 is zero */
  else
    {
      regval |= DMACHAN_CTRLA_DCSIZE_1;
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: sam3u_rxctrla
 *
 * Description:
 *  'OR' in the variable CTRLA bits
 *
 ****************************************************************************/

static inline uint32_t sam3u_rxctrla(struct sam3u_dma_s *dmach,
                                     uint32_t dmasize, uint32_t txctrlabits)
{
  /* Set the buffer transfer size field.  This is the number of transfers to
   * be performed, that is, the number of source width transfers to perform.
   */

  /* Adjust the the source transfer size for the source chunk size (peripheral
   * chunk size)
   */

  if ((dmach->flags & DMACH_FLAG_PERIPHCHUNKSIZE) == DMACH_FLAG_PERIPHCHUNKSIZE_4)
    {
      dmasize >>= 2;
    }
 
  DEBUGASSERT(dmasize <= DMACHAN_CTRLA_BTSIZE_MAX);
  return (txctrlabits & ~DMACHAN_CTRLA_BTSIZE_MASK) | (dmasize << DMACHAN_CTRLA_BTSIZE_SHIFT);
}

/****************************************************************************
 * Name: sam3u_txctrlb
 *
 * Description:
 *  Decode the the flags to get the correct CTRLB register bit settings for
 *  a transmit (memory to peripheral) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam3u_txctrlb(struct sam3u_dma_s *dmach)
{
  uint32_t regval;
 
  /* Assume that we will not be using the link list and disable the source
   * and destination descriptors.  The default will be single transfer mode.
   */

  regval = DMACHAN_CTRLB_BOTHDSCR;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from TX is memory to peripheral, but that is really be
   * determined by bits in the DMA flags.
   */

  /* Is the memory source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
    {
      /* Yes.. is the peripheral destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMACHAN_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the source is memory.  Is the peripheral destination a
       * peripheral
       */

      if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= DMACHAN_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_M2M;
        }
    }

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_SRCINCR_FIXED;
    }

  /* Select destination address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_DSTINCR_FIXED;
    }
  return regval;
}

/****************************************************************************
 * Name: sam3u_rxctrlb
 *
 * Description:
 *  Decode the the flags to get the correct CTRLB register bit settings for
 *  a receive (peripheral to memory) transfer.
 *
 ****************************************************************************/

static inline uint32_t sam3u_rxctrlb(struct sam3u_dma_s *dmach)
{
  uint32_t regval;
 
  /* Assume that we will not be using the link list and disable the source and
   * destination descriptors.  The default will be single transfer mode.
   */

  regval = DMACHAN_CTRLB_BOTHDSCR;

  /* Select flow control (even if the channel doesn't support it).  The
   * naming convention from RX is peripheral to memory, but that is really be
   * determined by bits in the DMA flags.
   */

  /* Is the peripheral source really a peripheral? */

  if ((dmach->flags & DMACH_FLAG_PERIPHISPERIPH) != 0)
    {
      /* Yes.. is the memory destination also a peripheral? */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use peripheral-to-peripheral flow control */

          regval |= DMACHAN_CTRLB_FC_P2P;
        }
      else
        {
          /* No.. Use peripheral-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_P2M;
        }
    }
  else
    {
      /* No, the peripheral source is memory.  Is the memory destination
       * a peripheral
       */

      if ((dmach->flags & DMACH_FLAG_MEMISPERIPH) != 0)
        {
          /* Yes.. Use memory-to-peripheral flow control */

          regval |= DMACHAN_CTRLB_FC_M2P;
        }
      else
        {
          /* No.. Use memory-to-memory flow control */

          regval |= DMACHAN_CTRLB_FC_M2M;
        }
    }

  /* Select source address incrementing */

  if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_SRCINCR_FIXED;
    }

  /* Select address incrementing */

  if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) == 0)
    {
      regval |= DMACHAN_CTRLB_DSTINCR_FIXED;
    }
  return regval;
}

/****************************************************************************
 * Name: sam3u_allocdesc
 *
 * Description:
 *  Allocate and add one descriptor to the DMA channel's link list.
 *
 *  NOTE: link list entries are freed by the DMA interrupt handler.  However,
 *  since the setting/clearing of the 'in use' indication is atomic, no
 *  special actions need be performed.  It would be a good thing to add logic
 *  to handle the case where all of the entries are exhausted and we could
 *  wait for some to be freed by the interrupt handler.
 *
 ****************************************************************************/

static struct dma_linklist_s *
sam3u_allocdesc(struct sam3u_dma_s *dmach, struct dma_linklist_s *prev,
                uint32_t src, uint32_t dest, uint32_t ctrla, uint32_t ctrlb)
{
  struct dma_linklist_s *desc = NULL;
  int i;

  /* Sanity check -- src == 0 is the indication that the link is unused.
   * Obviously setting it to zero would break that usage.
   */

#ifdef CONFIG_DEBUG
  if (src != 0)
#endif
    {
      /* Table a descriptor table semaphore count.  When we get one, then there
       * is at least one free descriptor in the table and it is ours.
       */

      sam3u_takedsem();

      /* Examine each link list entry to find an available one -- i.e., one
       * with src == 0.  That src field is set to zero by the DMA transfer
       * complete interrupt handler.  The following should be safe because
       * that is an atomic operation.
       */

      sam3u_takechsem();
      for (i = 0; i < CONFIG_SAM3U_NLLDESC; i++)
        {
          if (g_linklist[i].src == 0)
            {
              /* We have it.  Initialize the new link list entry */

              desc        = &g_linklist[i];
              desc->src   = src;    /* Source address */
              desc->dest  = dest;   /* Destination address */
              desc->ctrla = ctrla;  /* Control A value */
              desc->ctrlb = ctrlb;  /* Control B value */
              desc->next  = 0;      /* Next descriptor address */

              /* And then hook it at the tail of the link list */

              if (!prev)
                {
                  /* There is no previous link.  This is the new head of
                   * the list
                   */

                  DEBUGASSERT(dmach->llhead == NULL && dmach->lltail == NULL);
                  dmach->llhead = desc;
                }
              else
                {
                  DEBUGASSERT(dmach->llhead != NULL && dmach->lltail == prev);

                  /* When the second link is added to the list, that is the
                   * cue that we are going to do the link list transfer.
                   *
                   * Enable the source and destination descriptor in the link
                   * list entry just before this one.  We assume that both
                   * source and destination buffers are non-continuous, but
                   * this should work even if that is not the case.
                   */

                  prev->ctrlb &= ~DMACHAN_CTRLB_BOTHDSCR;
 
                  /* Link the previous tail to the new tail */

                  prev->next = (uint32_t)desc;
                }

              /* In any event, this is the new tail of the list.  The source
               * and destination descriptors must be disabled for the last entry
               * in the link list. */

              desc->ctrlb  |= DMACHAN_CTRLB_BOTHDSCR;
              dmach->lltail = desc;
              break;
            }
        }

      /* Because we hold a count from the counting semaphore, the above
       * search loop should always be successful.
       */

      sam3u_givechsem();
      DEBUGASSERT(desc != NULL);
    }
  return desc;
}

/****************************************************************************
 * Name: sam3u_freelinklist
 *
 * Description:
 *  Free all descriptors in the DMA channel's link list.
 *
 *  NOTE: Called from the DMA interrupt handler.
 *
 ****************************************************************************/

static void sam3u_freelinklist(struct sam3u_dma_s *dmach)
{
  struct dma_linklist_s *desc;
  struct dma_linklist_s *next;

  /* Get the head of the link list and detach the link list from the DMA
   * channel
   */

  desc             = dmach->llhead;
  dmach->llhead    = NULL;
  dmach->lltail    = NULL;

  /* Reset each descriptor in the link list (thereby freeing them) */

  while (desc != NULL)
    {
      next = (struct dma_linklist_s *)desc->next;
      DEBUGASSERT(desc->src != 0);
      memset(desc, 0, sizeof(struct dma_linklist_s));
      sam3u_givedsem();
      desc = next;
    }
}

/****************************************************************************
 * Name: sam3u_txbuffer
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam3u_txbuffer(struct sam3u_dma_s *dmach, uint32_t paddr,
                          uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (dmach->llhead)
    {
      regval = dmach->llhead->ctrla;
      ctrlb  = dmach->llhead->ctrlb;
    }
  else
    {
      regval = sam3u_txctrlabits(dmach); 
      ctrlb  = sam3u_txctrlb(dmach);
    }
  ctrla  = sam3u_txctrla(dmach, regval, nbytes);   

  /* Add the new link list entry */

  if (!sam3u_allocdesc(dmach, dmach->lltail, maddr, paddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the transmit CFG register setting (it won't be used until
   * the DMA is started).
   */

  dmach->cfg = sam3u_txcfg(dmach);
  return OK;
}

/****************************************************************************
 * Name: sam3u_rxbuffer
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.
 *
 ****************************************************************************/

static int sam3u_rxbuffer(struct sam3u_dma_s *dmach, uint32_t paddr,
                          uint32_t maddr, size_t nbytes)
{
  uint32_t regval;
  uint32_t ctrla;
  uint32_t ctrlb;

  /* If we are appending a buffer to a linklist, then re-use the CTRLA/B
   * values.  Otherwise, create them from the properties of the transfer.
   */

  if (dmach->llhead)
    {
      regval = dmach->llhead->ctrla;
      ctrlb  = dmach->llhead->ctrlb;
    }
  else
    {
      regval = sam3u_rxctrlabits(dmach); 
      ctrlb  = sam3u_rxctrlb(dmach);
    }
   ctrla  = sam3u_rxctrla(dmach, regval, nbytes);   

  /* Add the new link list entry */

  if (!sam3u_allocdesc(dmach, dmach->lltail, paddr, maddr, ctrla, ctrlb))
    {
      return -ENOMEM;
    }

  /* Pre-calculate the receive CFG register setting (it won't be used until
   * the DMA is started).
   */

  dmach->cfg = sam3u_rxcfg(dmach);
  return OK;
}

/****************************************************************************
 * Name: sam3u_single
 *
 * Description:
 *   Start a single buffer DMA.
 *
 ****************************************************************************/

static inline int sam3u_single(struct sam3u_dma_s *dmach)
{
  struct dma_linklist_s *llhead = dmach->llhead;

  /* Clear any pending interrupts from any previous DMAC transfer by reading
   * the interrupt status register.
   */

  (void)getreg32(SAM3U_DMAC_EBCISR);

  /* Write the starting source address in the SADDR register */

  DEBUGASSERT(llhead != NULL && llhead->src != 0);
  putreg32(llhead->src, dmach->base + SAM3U_DMACHAN_SADDR_OFFSET);

  /* Write the starting destination address in the DADDR register */

  putreg32(llhead->dest, dmach->base + SAM3U_DMACHAN_DADDR_OFFSET);

  /* Set up the CTRLA register */

  putreg32(llhead->ctrla, dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  putreg32(llhead->ctrlb, dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET);

  /* Both the DST and SRC DSCR bits should be '1' in CTRLB */

  DEBUGASSERT((llhead->ctrlb & DMACHAN_CTRLB_BOTHDSCR) == DMACHAN_CTRLB_BOTHDSCR);

  /* Set up the CFG register */

  putreg32(dmach->cfg, dmach->base + SAM3U_DMACHAN_CFG_OFFSET);

  /* Enable the channel by writing a ‘1’ to the CHER enable bit */

  putreg32(DMAC_CHER_ENA(dmach->chan), SAM3U_DMAC_CHER);

  /* The DMA has been started. Once the transfer completes, hardware sets the
   * interrupts and disables the channel.  We will received buffer complete and
   * transfer complete interrupts.
   *
   * Enable error, buffer complete and transfer complete interrupts.
   * (Since there is only a single buffer, we don't need the buffer complete
   * interrupt).
   */

  putreg32(DMAC_EBC_CBTCINTS(dmach->chan), SAM3U_DMAC_EBCIER);
  return OK;
}

/****************************************************************************
 * Name: sam3u_multiple
 *
 * Description:
 *   Start a multiple buffer DMA.
 *
 ****************************************************************************/

static inline int sam3u_multiple(struct sam3u_dma_s *dmach)
{
  struct dma_linklist_s *llhead = dmach->llhead;

  DEBUGASSERT(llhead != NULL && llhead->src != 0);

  /* Check the first and last CTRLB values */

  DEBUGASSERT((llhead->ctrlb & DMACHAN_CTRLB_BOTHDSCR) == 0);
  DEBUGASSERT((dmach->lltail->ctrlb & DMACHAN_CTRLB_BOTHDSCR) == DMACHAN_CTRLB_BOTHDSCR);

  /* Clear any pending interrupts from any previous DMAC transfer by reading the
   * status register
   */

  (void)getreg32(SAM3U_DMAC_EBCISR);

  /* Set up the initial CTRLB register (to enable descriptors) */

  putreg32(llhead->ctrlb, dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET);

  /* Set up the CTRLB register */

  putreg32(llhead->ctrlb, dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET);

  /* Write the channel configuration information into the CFG register */

  putreg32(dmach->cfg, dmach->base + SAM3U_DMACHAN_CFG_OFFSET);

  /* Program the DSCR register with the pointer to the firstlink list entry. */

  putreg32((uint32_t)llhead, dmach->base + SAM3U_DMACHAN_DSCR_OFFSET);

  /* Finally, enable the channel by writing a ‘1’ to the CHER enable */

  putreg32(DMAC_CHER_ENA(dmach->chan), SAM3U_DMAC_CHER);

  /* As each buffer of data is transferred, the CTRLA register is written back
   * into the link list entry.  The CTRLA contains updated BTSIZE and DONE bits.
   * Additionally, the CTRLA DONE bit is asserted when the buffer transfer has completed.
   *
   * The DMAC transfer continues until the CTRLB register disables the descriptor
   * (DSCR bits) registers at the final buffer tranfer.
   *
   * Enable error, buffer complete and transfer complete interrupts.  We
   * don't really need the buffer complete interrupts, but we will take them
   * just to handle stall conditions.
   */

  putreg32(DMAC_EBC_CHANINTS(dmach->chan), SAM3U_DMAC_EBCIER);
  return OK;
}

/****************************************************************************
 * Name: sam3u_dmasterminate
 *
 * Description:
 *   Terminate the DMA transfer and disable the DMA channel
 *
 ****************************************************************************/

static void sam3u_dmaterminate(struct sam3u_dma_s *dmach, int result)
{
  /* Disable all channel interrupts */

  putreg32(DMAC_EBC_CHANINTS(dmach->chan), SAM3U_DMAC_EBCIDR);

  /* Disable the channel by writing one to the write-only channel disable register */
 
  putreg32(DMAC_CHDR_DIS(dmach->chan), SAM3U_DMAC_CHDR);
  
  /* Free the linklist */
 
  sam3u_freelinklist(dmach);

  /* Perform the DMA complete callback */

  if (dmach->callback)
    {
      dmach->callback((DMA_HANDLE)dmach, dmach->arg, result);
    }

  dmach->callback = NULL;
  dmach->arg      = NULL;
}

/****************************************************************************
 * Name: sam3u_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int sam3u_dmainterrupt(int irq, void *context)
{
  struct sam3u_dma_s *dmach;
  unsigned int chndx;
  uint32_t regval;

  /* Get the DMAC status register value.  Ignore all masked interrupt
   * status bits.
   */

  regval = getreg32(SAM3U_DMAC_EBCISR) & getreg32(SAM3U_DMAC_EBCIMR);

  /* Check if the any transfer has completed */

  if (regval & DMAC_EBC_BTC_MASK)
    {
      /* Yes.. Check each bit  to see which channel has interrupted */

      for (chndx = 0; chndx < CONFIG_SAM3U_NDMACHAN; chndx++)
        {
          /* Are any interrupts pending for this channel? */

          if ((regval & DMAC_EBC_CHANINTS(chndx)) != 0)
            {
              dmach = &g_dma[chndx];

              /* Yes.. Did an error occur? */

              if ((regval & DMAC_EBC_ERR(chndx)) != 0)
                {
                   /* Yes... Terminate the transfer with an error? */

                  sam3u_dmaterminate(dmach, -EIO);
                }

              /* Is the transfer complete? */

              else if ((regval & DMAC_EBC_CBTC(chndx)) != 0)
               {
                  /* Yes.. Terminate the transfer with success */

                  sam3u_dmaterminate(dmach, OK);
                }

              /* Otherwise, this must be a Bufffer Transfer Complete (BTC)
               * interrupt as part of a multiple buffer transfer.
               */

              else /* f ((regval & DMAC_EBC_BTC(chndx)) != 0) */
                {
                  /* Write the KEEPON field to clear the STALL states */

                  putreg32(DMAC_CHER_KEEP(dmach->chan), SAM3U_DMAC_CHER);
                }
            }
        }
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_dmainitialize
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
  /* Enable peripheral clock */

  putreg32((1 << SAM3U_PID_DMAC), SAM3U_PMC_PCER);

  /* Disable all DMA interrupts */

  putreg32(DMAC_EBC_ALLINTS, SAM3U_DMAC_EBCIDR);

  /* Disable all DMA channels */

  putreg32(DMAC_CHDR_DIS_ALL, SAM3U_DMAC_CHDR);

  /* Attach DMA interrupt vector */

  (void)irq_attach(SAM3U_IRQ_DMAC, sam3u_dmainterrupt);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(SAM3U_IRQ_DMAC);

  /* Enable the DMA controller */

  putreg32(DMAC_EN_ENABLE, SAM3U_DMAC_EN);

  /* Initialize semaphores */

  sem_init(&g_chsem, 0, 1);
  sem_init(&g_dsem, 0, CONFIG_SAM3U_NDMACHAN);
}

/****************************************************************************
 * Name: sam3u_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel with
 *   the required FIFO size and flow control capabilities (determined by
 *   dma_flags) then  gives the caller exclusive access to the DMA channel.
 *
 *   The naming convention in all of the DMA interfaces is that one side is
 *   the 'peripheral' and the other is 'memory'.  Howerver, the interface
 *   could still be used if, for example, both sides were memory although
 *   the naming would be awkward.
 *
 * Returned Value:
 *   If a DMA channel if the required FIFO size is available, this function
 *   returns a non-NULL, void* DMA channel handle.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

DMA_HANDLE sam3u_dmachannel(uint32_t dmach_flags)
{
  struct sam3u_dma_s *dmach;
  unsigned int chndx;

  /* Get the search parameters */

  bool flowcontrol = sam3u_flowcontrol(dmach_flags);
  unsigned int fifosize = sam3u_fifosize(dmach_flags);

  /* Search for an available DMA channel with at least the requested FIFO
   * size.
   */

  dmach = NULL;
  sam3u_takechsem();
  for (chndx = 0; chndx < CONFIG_SAM3U_NDMACHAN; chndx++)
    {
      struct sam3u_dma_s *candidate = &g_dma[chndx];
      if (!candidate->inuse &&
          (sam3u_fifosize(candidate->flags) >= fifosize) &&
          (!flowcontrol || sam3u_flowcontrol(dmach_flags)))
        {
          dmach         = candidate;
          dmach->inuse  = true;

          /* Read the status register to clear any pending interrupts on the
           * channel
           */

          (void)getreg32(SAM3U_DMAC_EBCISR);

          /* Disable the channel by writing one to the write-only channel
           * disable register
           */
 
          putreg32(DMAC_CHDR_DIS(chndx), SAM3U_DMAC_CHDR);

          /* See the DMA channel flags, retaining the fifo size and flow
           * control settings which are inherent properties of the FIFO
           * and cannot be changed.
           */

          dmach->flags &= (DMACH_FLAG_FLOWCONTROL | DMACH_FLAG_FIFOSIZE_MASK);
          dmach->flags |= (dmach_flags & ~((DMACH_FLAG_FLOWCONTROL | DMACH_FLAG_FIFOSIZE_MASK)));
          break;
        }
    }
  sam3u_givechsem();
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: sam3u_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until sam3u_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam3u_dmafree(DMA_HANDLE handle)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  DEBUGASSERT((dmach != NULL) && (dmach->inuse));
  dmach->flags &= (DMACH_FLAG_FLOWCONTROL | DMACH_FLAG_FIFOSIZE_MASK);
  dmach->inuse  = false;                   /* No longer in use */
}

/****************************************************************************
 * Name: sam3u_dmatxsetup
 *
 * Description:
 *   Configure DMA for transmit of one buffer (memory to peripheral).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam3u_dmatxsetup() and sam3u_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam3u_dmatxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  int ret = OK;

  DEBUGASSERT(dmach && dmach->llhead != NULL && dmach->lltail != 0);

  /* If this is a large transfer, break it up into smaller buffers */

  while (nbytes > DMACHAN_CTRLA_BTSIZE_MAX)
    {
      /* Set up the maximum size transfer */

      ret = sam3u_txbuffer(dmach, paddr, maddr, DMACHAN_CTRLA_BTSIZE_MAX);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */

          nbytes -= DMACHAN_CTRLA_BTSIZE_MAX;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do do).
           */
 
          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += DMACHAN_CTRLA_BTSIZE_MAX;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += DMACHAN_CTRLA_BTSIZE_MAX;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && nbytes > 0)
    {
      ret = sam3u_txbuffer(dmach, paddr, maddr, nbytes);
    }
  return ret;
}

/****************************************************************************
 * Name: sam3u_dmarxsetup
 *
 * Description:
 *   Configure DMA for receipt of one buffer (peripheral to memory).  This
 *   function may be called multiple times to handle large and/or dis-
 *   continuous transfers.  Calls to sam3u_dmatxsetup() and sam3u_dmarxsetup()
 *   must not be intermixed on the same transfer, however.
 *
 ****************************************************************************/

int sam3u_dmarxsetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr, size_t nbytes)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  int ret = OK;

  DEBUGASSERT(dmach && dmach->llhead != NULL && dmach->lltail != 0);

  /* If this is a large transfer, break it up into smaller buffers */

  while (nbytes > DMACHAN_CTRLA_BTSIZE_MAX)
    {
      /* Set up the maximum size transfer */

      ret = sam3u_rxbuffer(dmach, paddr, maddr, DMACHAN_CTRLA_BTSIZE_MAX);
      if (ret == OK);
        {
          /* Decrement the number of bytes left to transfer */
 
          nbytes -= DMACHAN_CTRLA_BTSIZE_MAX;

          /* Increment the memory & peripheral address (if it is appropriate to
           * do do).
           */
 
          if ((dmach->flags & DMACH_FLAG_PERIPHINCREMENT) != 0)
            {
              paddr += DMACHAN_CTRLA_BTSIZE_MAX;
            }

          if ((dmach->flags & DMACH_FLAG_MEMINCREMENT) != 0)
            {
              maddr += DMACHAN_CTRLA_BTSIZE_MAX;
            }
        }
    }

  /* Then set up the final buffer transfer */

  if (ret == OK && nbytes > 0)
    {
      ret = sam3u_rxbuffer(dmach, paddr, maddr, nbytes);
    }
  return ret;
}

/****************************************************************************
 * Name: sam3u_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int sam3u_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  int ret = -EINVAL;

  /* Verify that the DMA has been setup (i.e., at least one entry in the
   * link list).
   */

  DEBUGASSERT(dmach != NULL);
  if (dmach->llhead)
    {
      /* Save the callback info.  This will be invoked whent the DMA commpletes */

      dmach->callback = callback;
      dmach->arg      = arg;

      /* Is this a single block transfer?  Or a multiple block tranfer? */

      if (dmach->llhead == dmach->lltail)
        {
          ret = sam3u_single(dmach);
        }
      else
        {
          ret = sam3u_multiple(dmach);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: sam3u_dmastop
 *
 * Description:
 *   Cancel the DMA.  After sam3u_dmastop() is called, the DMA channel is
 *   reset and sam3u_dmasetup() must be called before sam3u_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void sam3u_dmastop(DMA_HANDLE handle)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  irqstate_t flags;
 
  DEBUGASSERT(dmach != NULL);
  flags = irqsave();
  sam3u_dmaterminate(dmach, -EINTR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam3u_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam3u_dmasample(DMA_HANDLE handle, struct sam3u_dmaregs_s *regs)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;
  irqstate_t flags;

  /* Sample global registers.  NOTE: reading EBCISR clears interrupts, but
   * that should be okay IF interrupts are enabled when this function is
   * called.  But there is a race condition where this instrumentation could
   * cause lost interrupts.
   */

  flags        = irqsave();
  regs->gcfg   = getreg32(SAM3U_DMAC_GCFG);
  regs->en     = getreg32(SAM3U_DMAC_EN);
  regs->sreq   = getreg32(SAM3U_DMAC_SREQ);
  regs->creq   = getreg32(SAM3U_DMAC_CREQ);
  regs->last   = getreg32(SAM3U_DMAC_LAST);
  regs->ebcimr = getreg32(SAM3U_DMAC_EBCIMR);
  regs->ebcisr = getreg32(SAM3U_DMAC_EBCISR);
  regs->chsr   = getreg32(SAM3U_DMAC_CHSR);

  /* Sample channel registers */

  regs->saddr  = getreg32(dmach->base + SAM3U_DMACHAN_SADDR_OFFSET);
  regs->daddr  = getreg32(dmach->base + SAM3U_DMACHAN_DADDR_OFFSET);
  regs->dscr   = getreg32(dmach->base + SAM3U_DMACHAN_DSCR_OFFSET);
  regs->ctrla  = getreg32(dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET);
  regs->ctrlb  = getreg32(dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET);
  regs->cfg    = getreg32(dmach->base + SAM3U_DMACHAN_CFG_OFFSET);
  irqrestore(flags);
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: sam3u_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by sam3u_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void sam3u_dmadump(DMA_HANDLE handle, const struct sam3u_dmaregs_s *regs,
                   const char *msg)
{
  struct sam3u_dma_s *dmach = (struct sam3u_dma_s *)handle;

  dmadbg("%s\n", msg);
  dmadbg("  DMA Global Registers:\n");
  dmadbg("      GCFG[%08x]: %08x\n", SAM3U_DMAC_GCFG, regs->gcfg);
  dmadbg("        EN[%08x]: %08x\n", SAM3U_DMAC_EN, regs->en);
  dmadbg("      SREQ[%08x]: %08x\n", SAM3U_DMAC_SREQ, regs->sreq);
  dmadbg("      CREQ[%08x]: %08x\n", SAM3U_DMAC_CREQ, regs->creq);
  dmadbg("      LAST[%08x]: %08x\n", SAM3U_DMAC_LAST, regs->last);
  dmadbg("    EBCIMR[%08x]: %08x\n", SAM3U_DMAC_EBCIMR, regs->ebcimr);
  dmadbg("    EBCISR[%08x]: %08x\n", SAM3U_DMAC_EBCISR, regs->ebcisr);
  dmadbg("      CHSR[%08x]: %08x\n", SAM3U_DMAC_CHSR, regs->chsr);
  dmadbg("  DMA Channel Registers:\n");
  dmadbg("     SADDR[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_SADDR_OFFSET, regs->saddr);
  dmadbg("     DADDR[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_DADDR_OFFSET, regs->daddr);
  dmadbg("      DSCR[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_DSCR_OFFSET, regs->dscr);
  dmadbg("     CTRLA[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_CTRLA_OFFSET, regs->ctrla);
  dmadbg("     CTRLB[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_CTRLB_OFFSET, regs->ctrlb);
  dmadbg("       CFG[%08x]: %08x\n", dmach->base + SAM3U_DMACHAN_CFG_OFFSET, regs->cfg);
}
#endif /* CONFIG_DEBUG_DMA */
#endif /* CONFIG_SAM3U_DMA */
