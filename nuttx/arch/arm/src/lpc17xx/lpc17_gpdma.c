/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_gpdma.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "chip/lpc17_syscon.h"
#include "lpc17_gpdma.h"

#ifdef CONFIG_LPC17_GPDMA

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef DMA_DEBUG     /* Define to enable debug */
#undef DMA_VERBOSE   /* Define to enable verbose debug */

#ifdef DMA_DEBUG
#  define dmadbg  lldbg
#  ifdef DMA_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef DMA_VERBOSE
#  define dmadbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_dmainitialize
 *
 * Description:
 *   Initialize the GPDMA subsystem.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_dmainitilaize(void)
{
}

/****************************************************************************
 * Name: lpc17_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function sets aside a DMA channel and
 *   gives the caller exclusive access to the DMA channel.
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 ****************************************************************************/

DMA_HANDLE lpc17_dmachannel(void)
{
  return NULL;
}

/****************************************************************************
 * Name: lpc17_dmafree
 *
 * Description:
 *   Release a DMA channel.  NOTE:  The 'handle' used in this argument must
 *   NEVER be used again until lpc17_dmachannel() is called again to re-gain
 *   a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpc17_dmafree(DMA_HANDLE handle)
{
}

/****************************************************************************
 * Name: lpc17_dmasetup
 *
 * Description:
 *   Configure DMA for one transfer.
 *
 ****************************************************************************/

int lpc17_dmarxsetup(DMA_HANDLE handle, uint32_t control, uint32_t config,
                     uint32_t srcaddr, uint32_t destaddr, size_t nbytes)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc17_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 ****************************************************************************/

int lpc17_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc17_dmastop
 *
 * Description:
 *   Cancel the DMA.  After lpc17_dmastop() is called, the DMA channel is
 *   reset and lpc17_dmasetup() must be called before lpc17_dmastart() can be
 *   called again
 *
 ****************************************************************************/

void lpc17_dmastop(DMA_HANDLE handle)
{
}

/****************************************************************************
 * Name: lpc17_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc17_dmasample(DMA_HANDLE handle, struct lpc17_dmaregs_s *regs)
{
}
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Name: lpc17_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void lpc17_dmadump(DMA_HANDLE handle, const struct lpc17_dmaregs_s *regs, const char *msg)
{
}
#endif /* CONFIG_DEBUG_DMA */

#endif /* CONFIG_LPC17_GPDMA */
