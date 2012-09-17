/****************************************************************************
 * arch/arm/src/sam3u/sam3u_sdio.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <wdog.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/mmcsd.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "sam3u_internal.h"
#include "sam3u_dmac.h"
#include "sam3u_pmc.h"
#include "sam3u_hsmci.h"

#if CONFIG_SAM3U_HSMCI

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAM3U_DMA
#  warning "HSMCI driver requires CONFIG_SAM3U_DMA"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "This driver requires CONFIG_SDIO_BLOCKSETUP"
#endif

#ifndef CONFIG_HSMCI_PRI
#  define CONFIG_HSMCI_PRI        NVIC_SYSH_PRIORITY_DEFAULT
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_VERBOSE)
#  undef CONFIG_HSMCI_CMDDEBUG
#  undef CONFIG_HSMCI_XFRDEBUG
#endif

#ifdef CONFIG_SAM3U_HSMCI_RDPROOF
#  ifdef CONFIG_SAM3U_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS (HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF)
#  else
#    define HSMCU_PROOF_BITS HSMCI_MR_RDPROOF
#  endif
#else
#  ifdef CONFIG_SAM3U_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS HSMCI_MR_WRPROOF
#  else
#    define HSMCU_PROOF_BITS (0)
#  endif
#endif

/* Timing */

#define HSMCI_CMDTIMEOUT         (100000)
#define HSMCI_LONGTIMEOUT        (0x7fffffff)

/* Big DTIMER setting */

#define HSMCI_DTIMER_DATATIMEOUT (0x000fffff)

/* DMA configuration flags */

#define DMA_FLAGS \
  (DMACH_FLAG_FIFO_8BYTES | DMACH_FLAG_FIFOCFG_LARGEST | \
  (DMACHAN_PID_MCI0 << DMACH_FLAG_PERIPHPID_SHIFT) | \
   DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |  \
   DMACH_FLAG_PERIPHWIDTH_32BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   DMACH_FLAG_MEMWIDTH_32BITS | DMACH_FLAG_MEMINCREMENT | DMACH_FLAG_MEMCHUNKSIZE_4)

/* Status errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_STATUS_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
    HSMCI_INT_DTOE  | HSMCI_INT_DCRCE | HSMCI_INT_RTOE    | HSMCI_INT_RENDE | \
    HSMCI_INT_RCRCE | HSMCI_INT_RDIRE | HSMCI_INT_RINDE )  

/* Response errors:
 *
 *   HSMCI_INT_CSTOE         Completion signal time-out error (see HSMCI_CSTOR)
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_RESPONSE_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RCRCE | \
    HSMCI_INT_RDIRE | HSMCI_INT_RINDE ) 
#define HSMCI_RESPONSE_NOCRC_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RDIRE | \
    HSMCI_INT_RINDE ) 
#define HSMCI_RESPONSE_TIMEOUT_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_RTOE  )

/* Data transfer errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 */

#define HSMCI_DATA_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
    HSMCI_INT_DTOE  | HSMCI_INT_DCRCE )

#define HSMCI_DATA_TIMEOUT_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_DTOE )

#define HSMCI_DATA_DMARECV_ERRORS \
  ( HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
    HSMCI_INT_DCRCE )

#define HSMCI_DATA_DMASEND_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE    | HSMCI_INT_DCRCE )

/* Data transfer status and interrupt mask bits.
 *
 * The XFRDONE flag in the HSMCI_SR indicates exactly when the read or
 * write sequence is finished.
 *
 *   0: A transfer is in progress.
 *   1: Command register is ready to operate and the data bus is in the idle state.
 *
 * DMADONE: DMA Transfer done
 *
 *   0: DMA buffer transfer has not completed since the last read of HSMCI_SR register.
 *   1: DMA buffer transfer has completed.
 */

#define HSMCI_DMARECV_INTS \
  ( HSMCI_DATA_DMARECV_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */ )
#define HSMCI_DMASEND_INTS \
  ( HSMCI_DATA_DMASEND_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */ )

/* Event waiting interrupt mask bits.
 *
 * CMDRDY (Command Ready):
 *
 *   0: A command is in progress
 *   1: The last command has been sent.  The CMDRDY flag is released 8 bits
 *     after the end of the card response. Cleared when writing in the HSMCI_CMDR
 */

#define HSMCI_CMDRESP_INTS \
  ( HSMCI_RESPONSE_ERRORS | HSMCI_INT_CMDRDY )
#define HSMCI_CMDRESP_NOCRC_INTS \
  ( HSMCI_RESPONSE_NOCRC_ERRORS | HSMCI_INT_CMDRDY )

/* Register logging support */

#ifdef CONFIG_HSMCI_XFRDEBUG
#  ifdef CONFIG_DEBUG_DMA
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_BEFORE_ENABLE 1
#    define SAMPLENDX_AFTER_SETUP   2
#    define SAMPLENDX_END_TRANSFER  3
#    define SAMPLENDX_DMA_CALLBACK  4
#    define DEBUG_NDMASAMPLES       5
#  else
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_AFTER_SETUP   1
#    define SAMPLENDX_END_TRANSFER  2
#    define DEBUG_NDMASAMPLES       3
#  endif
#endif

#ifdef CONFIG_HSMCI_CMDDEBUG
#  define SAMPLENDX_AFTER_CMDR      0
#  define SAMPLENDX_AT_WAKEUP       1
#  define DEBUG_NCMDSAMPLES         2
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the SAM3U HSMCI interface */

struct sam3u_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SDIO interface */
  
  /* SAM3U-specific extensions */
  /* Event support */

  sem_t              waitsem;    /* Implements event waiting */
  sdio_eventset_t    waitevents; /* Set of events to be waited for */
  uint32_t           waitmask;   /* Interrupt enables for event waiting */
  uint32_t           cmdrmask;   /* Interrupt enables for this particular cmd/response */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  WDOG_ID            waitwdog;   /* Watchdog that handles event timeouts */

  /* Callback support */

  uint8_t            cdstatus;   /* Card status */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  worker_t           callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */
  struct work_s      cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t           xfrmask;    /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  bool               widebus;    /* Required for DMA support */
  DMA_HANDLE         dma;        /* Handle for DMA channel */
};

/* Register logging support */

#if defined(CONFIG_HSMCI_XFRDEBUG) || defined(CONFIG_HSMCI_CMDDEBUG)
struct sam3u_hsmciregs_s
{
  uint32_t mr;    /* Mode Register */
  uint32_t dtor;  /* Data Timeout Register */
  uint32_t sdcr;  /* SD/SDIO Card Register */
  uint32_t argr;  /* Argument Register */
  uint32_t blkr;  /* Block Register */
  uint32_t cstor; /* Completion Signal Timeout Register */
  uint32_t rsp0;  /* Response Register 0 */
  uint32_t rsp1;  /* Response Register 1 */
  uint32_t rsp2;  /* Response Register 2 */
  uint32_t rsp3;  /* Response Register 3 */
  uint32_t sr;    /* Status Register */
  uint32_t imr;   /* Interrupt Mask Register */
  uint32_t dma;   /* DMA Configuration Register */
  uint32_t cfg;   /* Configuration Register */
  uint32_t wpmr;  /* Write Protection Mode Register */
  uint32_t wpsr;  /* Write Protection Status Register */
};
#endif

#ifdef CONFIG_HSMCI_XFRDEBUG
struct sam3u_xfrregs_s
{
  struct sam3u_hsmciregs_s hsmci;
#ifdef CONFIG_DEBUG_DMA
  struct sam3u_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam3u_takesem(struct sam3u_dev_s *priv);
#define     sam3u_givesem(priv) (sem_post(&priv->waitsem))
static void sam3u_enablewaitints(struct sam3u_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents);
static void sam3u_disablewaitints(struct sam3u_dev_s *priv, sdio_eventset_t wkupevents);
static void sam3u_enablexfrints(struct sam3u_dev_s *priv, uint32_t xfrmask);
static void sam3u_disablexfrints(struct sam3u_dev_s *priv);
static inline void sam3u_disable(void);
static inline void sam3u_enable(void);

/* Register Sampling ********************************************************/

#if defined(CONFIG_HSMCI_XFRDEBUG) || defined(CONFIG_HSMCI_CMDDEBUG)
static void sam3u_hsmcisample(struct sam3u_hsmciregs_s *regs);
static void sam3u_hsmcidump(struct sam3u_hsmciregs_s *regs, const char *msg);
#endif

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_xfrsampleinit(void);
static void sam3u_xfrsample(struct sam3u_dev_s *priv, int index);
static void sam3u_xfrdumpone(struct sam3u_dev_s *priv,
              struct sam3u_xfrregs_s *regs, const char *msg);
static void sam3u_xfrdump(struct sam3u_dev_s *priv);
#else
#  define   sam3u_xfrsampleinit()
#  define   sam3u_xfrsample(priv,index)
#  define   sam3u_xfrdump(priv)
#endif

#ifdef CONFIG_HSMCI_CMDDEBUG
static void sam3u_cmdsampleinit(void);
static inline void sam3u_cmdsample1(int index3);
static inline void sam3u_cmdsample2(int index, uint32_t sr);
static void sam3u_cmddump(void);
#else
#  define   sam3u_cmdsampleinit()
#  define   sam3u_cmdsample1(index)
#  define   sam3u_cmdsample2(index,sr)
#  define   sam3u_cmddump()
#endif

/* DMA Helpers **************************************************************/

static void sam3u_dmacallback(DMA_HANDLE handle, void *arg, int result);

/* Data Transfer Helpers ****************************************************/

static void sam3u_eventtimeout(int argc, uint32_t arg);
static void sam3u_endwait(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent);
static void sam3u_endtransfer(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent);
static void sam3u_notransfer(struct sam3u_dev_s *priv);

/* Interrupt Handling *******************************************************/

static int  sam3u_interrupt(int irq, void *context);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void sam3u_reset(FAR struct sdio_dev_s *dev);
static uint8_t sam3u_status(FAR struct sdio_dev_s *dev);
static void sam3u_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void sam3u_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  sam3u_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  sam3u_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
static void sam3u_blocksetup(FAR struct sdio_dev_s *dev, unsigned int blocklen,
              unsigned int nblocks);
static int  sam3u_cancel(FAR struct sdio_dev_s *dev);
static int  sam3u_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  sam3u_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  sam3u_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  sam3u_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void sam3u_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static sdio_eventset_t
            sam3u_eventwait(FAR struct sdio_dev_s *dev, uint32_t timeout);
static void sam3u_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  sam3u_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static bool sam3u_dmasupported(FAR struct sdio_dev_s *dev);
#endif
static int  sam3u_dmarecvsetup(FAR struct sdio_dev_s *dev,
              FAR uint8_t *buffer, size_t buflen);
static int  sam3u_dmasendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);

/* Initialization/uninitialization/reset ************************************/

static void sam3u_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sam3u_dev_s g_sdiodev =
{
  .dev =
  {
    .reset            = sam3u_reset,
    .status           = sam3u_status,
    .widebus          = sam3u_widebus,
    .clock            = sam3u_clock,
    .attach           = sam3u_attach,
    .sendcmd          = sam3u_sendcmd,
    .blocksetup       = sam3u_blocksetup,
    .recvsetup        = sam3u_dmarecvsetup,
    .sendsetup        = sam3u_dmasendsetup,
    .cancel           = sam3u_cancel,
    .waitresponse     = sam3u_waitresponse,
    .recvR1           = sam3u_recvshort,
    .recvR2           = sam3u_recvlong,
    .recvR3           = sam3u_recvshort,
    .recvR4           = sam3u_recvnotimpl,
    .recvR5           = sam3u_recvnotimpl,
    .recvR6           = sam3u_recvshort,
    .recvR7           = sam3u_recvshort,
    .waitenable       = sam3u_waitenable,
    .eventwait        = sam3u_eventwait,
    .callbackenable   = sam3u_callbackenable,
    .registercallback = sam3u_registercallback,
#ifdef CONFIG_SDIO_DMA
    .dmasupported     = sam3u_dmasupported,
    .dmarecvsetup     = sam3u_dmarecvsetup,
    .dmasendsetup     = sam3u_dmasendsetup,
#endif
  },
};

/* Register logging support */

#ifdef CONFIG_HSMCI_XFRDEBUG
static struct sam3u_xfrregs_s   g_xfrsamples[DEBUG_NDMASAMPLES];
#endif
#ifdef CONFIG_HSMCI_CMDDEBUG
static struct sam3u_hsmciregs_s g_cmdsamples[DEBUG_NCMDSAMPLES];
#endif
#if defined(CONFIG_HSMCI_XFRDEBUG) && defined(CONFIG_HSMCI_CMDDEBUG)
static bool                     g_xfrinitialized;
static bool                     g_cmdinitialized;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam3u_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_takesem(struct sam3u_dev_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: sam3u_enablewaitints
 *
 * Description:
 *   Enable HSMCI interrupts needed to suport the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the HSMCI device state structure
 *   waitmask   - The set of bits in the HSMCI MASK register to set
 *   waitevents - Waited for events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_enablewaitints(struct sam3u_dev_s *priv, uint32_t waitmask,
                                 sdio_eventset_t waitevents)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = irqsave();
  priv->waitevents = waitevents;
  priv->wkupevent  = 0;
  priv->waitmask   = waitmask;
  putreg32(priv->xfrmask | priv->waitmask, SAM3U_HSMCI_IER);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_disablewaitints
 *
 * Description:
 *   Disable HSMCI interrupts and save wakeup event.  Called
 *
 * Input Parameters:
 *   priv       - A reference to the HSMCI device state structure
 *   wkupevent  - Wake-up event(s)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_disablewaitints(struct sam3u_dev_s *priv,
                                  sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = irqsave();
  priv->waitevents = 0;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = 0;
  putreg32(~priv->xfrmask, SAM3U_HSMCI_IDR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_enablexfrints
 *
 * Description:
 *   Enable HSMCI interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_enablexfrints(struct sam3u_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags = irqsave();
  priv->xfrmask = xfrmask;
  putreg32(priv->xfrmask | priv->waitmask, SAM3U_HSMCI_IER);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_disablexfrints
 *
 * Description:
 *   Disable HSMCI interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_disablexfrints(struct sam3u_dev_s *priv)
{
  irqstate_t flags = irqsave();
  priv->xfrmask = 0;
  putreg32(~priv->waitmask, SAM3U_HSMCI_IDR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_disable
 *
 * Description:
 *   Disable the HSMCI
 *
 ****************************************************************************/

static inline void sam3u_disable(void) 
{
  /* Disable the MCI peripheral clock */

  putreg32((1 << SAM3U_PID_HSMCI), SAM3U_PMC_PCDR);
  
  /* Disable the MCI */

  putreg32(HSMCI_CR_MCIDIS, SAM3U_HSMCI_CR);

  /* Disable all the interrupts */

  putreg32(0xffffffff, SAM3U_HSMCI_IDR);
}

/****************************************************************************
 * Name: sam3u_enable
 *
 * Description:
 *   Enable the HSMCI
 *
 ****************************************************************************/

static inline void sam3u_enable(void)
{
  /* Enable the MCI peripheral clock */

  putreg32((1 << SAM3U_PID_HSMCI), SAM3U_PMC_PCER);

  /* Enable the MCI and the Power Saving */

  putreg32(HSMCI_CR_MCIEN, SAM3U_HSMCI_CR);
}

/****************************************************************************
 * Register Sampling
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_hsmcisample
 *
 * Description:
 *   Sample HSMCI registers
 *
 ****************************************************************************/

#if defined(CONFIG_HSMCI_XFRDEBUG) || defined(CONFIG_HSMCI_CMDDEBUG)
static void sam3u_hsmcisample(struct sam3u_hsmciregs_s *regs)
{
  regs->mr    = getreg32(SAM3U_HSMCI_MR);
  regs->dtor  = getreg32(SAM3U_HSMCI_DTOR);
  regs->sdcr  = getreg32(SAM3U_HSMCI_SDCR);
  regs->argr  = getreg32(SAM3U_HSMCI_ARGR);
  regs->blkr  = getreg32(SAM3U_HSMCI_BLKR);
  regs->cstor = getreg32(SAM3U_HSMCI_CSTOR);
  regs->rsp0  = getreg32(SAM3U_HSMCI_RSPR0);
  regs->rsp1  = getreg32(SAM3U_HSMCI_RSPR1);
  regs->rsp2  = getreg32(SAM3U_HSMCI_RSPR2);
  regs->rsp3  = getreg32(SAM3U_HSMCI_RSPR3);
  regs->sr    = getreg32(SAM3U_HSMCI_SR);
  regs->imr   = getreg32(SAM3U_HSMCI_IMR);
  regs->dma   = getreg32(SAM3U_HSMCI_DMA);
  regs->cfg   = getreg32(SAM3U_HSMCI_CFG);
  regs->wpmr  = getreg32(SAM3U_HSMCI_WPMR);
  regs->wpsr  = getreg32(SAM3U_HSMCI_WPSR);
}
#endif

/****************************************************************************
 * Name: sam3u_hsmcidump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#if defined(CONFIG_HSMCI_XFRDEBUG) || defined(CONFIG_HSMCI_CMDDEBUG)
static void sam3u_hsmcidump(struct sam3u_hsmciregs_s *regs, const char *msg)
{
  fdbg("HSMCI Registers: %s\n", msg);
  fdbg("     MR[%08x]: %08x\n", SAM3U_HSMCI_MR,    regs->mr);
  fdbg("   DTOR[%08x]: %08x\n", SAM3U_HSMCI_DTOR,  regs->dtor);
  fdbg("   SDCR[%08x]: %08x\n", SAM3U_HSMCI_SDCR,  regs->sdcr);
  fdbg("   ARGR[%08x]: %08x\n", SAM3U_HSMCI_ARGR,  regs->argr);
  fdbg("   BLKR[%08x]: %08x\n", SAM3U_HSMCI_BLKR,  regs->blkr);
  fdbg("  CSTOR[%08x]: %08x\n", SAM3U_HSMCI_CSTOR, regs->cstor);
  fdbg("  RSPR0[%08x]: %08x\n", SAM3U_HSMCI_RSPR0, regs->rsp0);
  fdbg("  RSPR1[%08x]: %08x\n", SAM3U_HSMCI_RSPR1, regs->rsp1);
  fdbg("  RSPR2[%08x]: %08x\n", SAM3U_HSMCI_RSPR2, regs->rsp2);
  fdbg("  RSPR3[%08x]: %08x\n", SAM3U_HSMCI_RSPR3, regs->rsp3);
  fdbg("     SR[%08x]: %08x\n", SAM3U_HSMCI_SR,    regs->sr);
  fdbg("    IMR[%08x]: %08x\n", SAM3U_HSMCI_IMR,   regs->imr);
  fdbg("    DMA[%08x]: %08x\n", SAM3U_HSMCI_DMA,   regs->dma);
  fdbg("    CFG[%08x]: %08x\n", SAM3U_HSMCI_CFG,   regs->cfg);
  fdbg("   WPMR[%08x]: %08x\n", SAM3U_HSMCI_WPMR,  regs->wpmr);
  fdbg("   WPSR[%08x]: %08x\n", SAM3U_HSMCI_WPSR,  regs->wpsr);
}
#endif

/****************************************************************************
 * Name: sam3u_xfrsample
 *
 * Description:
 *   Sample HSMCI/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_xfrsample(struct sam3u_dev_s *priv, int index)
{
  struct sam3u_xfrregs_s *regs = &g_xfrsamples[index];
#ifdef CONFIG_DEBUG_DMA
  sam3u_dmasample(priv->dma, &regs->dma);
#endif
  sam3u_hsmcisample(&regs->hsmci);
}
#endif

/****************************************************************************
 * Name: sam3u_xfrsampleinit
 *
 * Description:
 *   Setup prior to collecting transfer samples
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_xfrsampleinit(void)
{
  memset(g_xfrsamples, 0xff, DEBUG_NDMASAMPLES * sizeof(struct sam3u_xfrregs_s));
#ifdef CONFIG_HSMCI_CMDDEBUG
  g_xfrinitialized = true;
#endif
}
#endif

/****************************************************************************
 * Name: sam3u_xfrdumpone
 *
 * Description:
 *   Dump one transfer register sample
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_xfrdumpone(struct sam3u_dev_s *priv,
                             struct sam3u_xfrregs_s *regs, const char *msg)
{
#ifdef CONFIG_DEBUG_DMA
  sam3u_dmadump(priv->dma, &regs->dma, msg);
#endif
  sam3u_hsmcidump(&regs->hsmci, msg);
}
#endif

/****************************************************************************
 * Name: sam3u_xfrdump
 *
 * Description:
 *   Dump all transfer-related, sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void  sam3u_xfrdump(struct sam3u_dev_s *priv)
{
#ifdef CONFIG_HSMCI_CMDDEBUG
  if (g_xfrinitialized)
#endif
    {
      sam3u_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_BEFORE_SETUP], "Before setup");
#ifdef CONFIG_DEBUG_DMA
      sam3u_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_BEFORE_ENABLE], "Before DMA enable");
#endif
      sam3u_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_AFTER_SETUP], "After setup");
      sam3u_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_END_TRANSFER], "End of transfer");
#ifdef CONFIG_DEBUG_DMA
      sam3u_xfrdumpone(priv, &g_xfrsamples[SAMPLENDX_DMA_CALLBACK], "DMA Callback");
#endif
#ifdef CONFIG_HSMCI_CMDDEBUG
      g_xfrinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * Name: sam3u_cmdsampleinit
 *
 * Description:
 *   Setup prior to collecting command/response samples
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_CMDDEBUG
static void sam3u_cmdsampleinit(void)
{
  memset(g_cmdsamples, 0xff, DEBUG_NCMDSAMPLES * sizeof(struct sam3u_hsmciregs_s));
#ifdef CONFIG_HSMCI_XFRDEBUG
  g_cmdinitialized = true;
#endif
}
#endif

/****************************************************************************
 * Name: sam3u_cmdsample1 & 2
 *
 * Description:
 *   Sample command/response registers
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_CMDDEBUG
static inline void sam3u_cmdsample1(int index)
{
  sam3u_hsmcisample(&g_cmdsamples[index]);
}

static inline void sam3u_cmdsample2(int index, uint32_t sr)
{
  sam3u_hsmcisample(&g_cmdsamples[index]);
  g_cmdsamples[index].sr = sr;
}
#endif

/****************************************************************************
 * Name: sam3u_cmddump
 *
 * Description:
 *   Dump all comand/response register data
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_CMDDEBUG
static void sam3u_cmddump(void)
{
#ifdef CONFIG_HSMCI_XFRDEBUG
  if (g_cmdinitialized)
#endif
    {
      sam3u_hsmcidump(&g_cmdsamples[SAMPLENDX_AFTER_CMDR], "After command setup");
      sam3u_hsmcidump(&g_cmdsamples[SAMPLENDX_AT_WAKEUP],  "After wakeup");
#ifdef CONFIG_HSMCI_XFRDEBUG
      g_cmdinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_dmacallback
 *
 * Description:
 *   Called when HSMCI DMA completes
 *
 ****************************************************************************/

static void sam3u_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  /* We don't really do anything at the completion of DMA.  The termination
   * of the transfer is driven by the HSMCI interrupts.
   */

  sam3u_xfrsample((struct sam3u_dev_s*)arg, SAMPLENDX_DMA_CALLBACK);
}

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired without
 *   any other waited-for event occurring.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam3u_eventtimeout(int argc, uint32_t arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)arg;

  DEBUGASSERT(argc == 1 && priv != NULL);
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam3u_endwait(priv, SDIOWAIT_TIMEOUT);
      flldbg("Timeout\n");
    }
}

/****************************************************************************
 * Name: sam3u_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam3u_endwait(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* Disable event-related interrupts and save wakeup event */

  sam3u_disablewaitints(priv, wkupevent);

  /* Wake up the waiting thread */

  sam3u_givesem(priv);
}

/****************************************************************************
 * Name: sam3u_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the HSMCI interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam3u_endtransfer(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  sam3u_disablexfrints(priv);

  /* No data transfer */

  sam3u_notransfer(priv);

  /* DMA debug instrumentation */

  sam3u_xfrsample(priv, SAMPLENDX_END_TRANSFER);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition.
   */

  sam3u_dmastop(priv->dma);

  /* Disable the DMA handshaking */

  putreg32(0, SAM3U_HSMCI_DMA);
  
  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam3u_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: sam3u_notransfer
 *
 * Description:
 *   Setup for no transfer.  This is the default setup that is overriddden
 *   by sam3u_dmarecvsetup or sam3u_dmasendsetup
 *
 * Input Parameters:
 *   priv   - An instance of the HSMCI device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_notransfer(struct sam3u_dev_s *priv)
{
  uint32_t regval = getreg32(SAM3U_HSMCI_MR);
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF | HSMCI_MR_BLKLEN_MASK);
  putreg32(regval, SAM3U_HSMCI_MR);
}

/****************************************************************************
 * Interrrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_interrupt
 *
 * Description:
 *   HSMCI interrupt handler
 *
 * Input Parameters:
 *   irq - IRQ number of the interrupts
 *   context - Saved machine context at the time of the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam3u_interrupt(int irq, void *context)
{
  struct sam3u_dev_s *priv = &g_sdiodev;
  uint32_t sr;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts. */

  for (;;)
    {
      /* Check the HSMCI status register.  Mask out all bits that don't
       * correspond to enabled interrupts.  (This depends on the fact that
       * bits are ordered the same in both the SR and IMR registers).  If
       * there are non-zero bits remaining, then we have work to do here.
       */

      sr      = getreg32(SAM3U_HSMCI_SR);
      enabled = sr & getreg32(SAM3U_HSMCI_IMR);
      if (enabled == 0)
        {
          break;
        }

      /* Handle in progress, interrupt driven data transfers ****************/
      /* Do any of these interrupts signal the end a data transfer? */

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
          /* Yes.. the transfer is complete.  Did it complete with an error? */

          if ((pending & HSMCI_DATA_ERRORS) != 0)
            {
              /* Yes.. Was it some kind of timeout error? */

              flldbg("ERROR: enabled: %08x pending: %08x\n", enabled, pending);
              if ((pending & HSMCI_DATA_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. Terminate with a timeout. */

                  sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_TIMEOUT);          
                }
              else
                {
                  /* No..  Terminate with an I/O error. */

                  sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
                }
            }
          else
            {
              /* No.. Then the transfer must have completed successfully */

              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      /* Handle wait events *************************************************/
      /* Do any of these interrupts signal wakeup event? */

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          sdio_eventset_t wkupevent = 0;

          /* Is this a Command-Response sequence completion event? */

          if ((pending & priv->cmdrmask) != 0)
            {
              sam3u_cmdsample2(SAMPLENDX_AT_WAKEUP, sr);

              /* Yes.. Did the Command-Response sequence end with an error? */

              if ((pending & HSMCI_RESPONSE_ERRORS) != 0)
                {
                  /* Yes.. Was the error some kind of timeout? */

                  fllvdbg("ERROR:events: %08x SR: %08x\n",
                          priv->cmdrmask, enabled);

                  if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                    {
                      /* Yes.. signal a timeout error */

                      wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_TIMEOUT;
                    }
                  else
                    {
                      /* No.. signal some generic I/O error */

                      wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_ERROR;
                    }
                }
              else
               {
                  /* The Command-Response sequence ended with no error */

                      wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE;
                }
 
             /* Yes.. Is there a thread waiting for this event set? */

             wkupevent &= priv->waitevents;
              if (wkupevent != 0)
                {
                  /* Yes.. wake the thread up */

                  sam3u_endwait(priv, wkupevent);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/
/****************************************************************************
 * Name: sam3u_reset
 *
 * Description:
 *   Reset the HSMCI controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct sam3u_dev_s *priv = (FAR struct sam3u_dev_s *)dev;
  irqstate_t flags;

  /* Enable the MCI clock */

  flags = irqsave();
  putreg32((1 << SAM3U_PID_HSMCI), SAM3U_PMC_PCER);
  fdbg("PCSR: %08x\n", getreg32(SAM3U_PMC_PCSR));
  
  /* Reset the MCI */

  putreg32(HSMCI_CR_SWRST, SAM3U_HSMCI_CR);
  
  /* Disable the MCI */

  putreg32(HSMCI_CR_MCIDIS | HSMCI_CR_PWSDIS, SAM3U_HSMCI_CR);
  
  /* Disable all the interrupts */

  putreg32(0xffffffff, SAM3U_HSMCI_IDR);
  
  /* Set the Data Timeout Register */

  putreg32(HSMCI_DTOR_DTOCYC_MAX | HSMCI_DTOR_DTOMUL_MAX, SAM3U_HSMCI_DTOR);
  
  /* Set the Mode Register for ID mode frequency (probably 400KHz) */

  sam3u_clock(dev, CLOCK_IDMODE);

  /* Set the SDCard Register */

  putreg32(HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_4BIT, SAM3U_HSMCI_SDCR);

  /* Enable the MCI controller */

  putreg32(HSMCI_CR_MCIEN, SAM3U_HSMCI_CR);

  /* Disable the DMA interface */

  putreg32(0, SAM3U_HSMCI_DMA);
  
  /* Configure MCI */

  putreg32(HSMCI_CFG_FIFOMODE, SAM3U_HSMCI_CFG);

  /* No data transfer */

  sam3u_notransfer(priv);

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
  wd_cancel(priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see sam3u_status_* defines)
 *
 ****************************************************************************/

static uint8_t sam3u_status(FAR struct sdio_dev_s *dev)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: sam3u_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint32_t regval;

  /* Set 1-bit or 4-bit bus by configuring the SDCBUS field of the SDCR register */

  regval  = getreg32(SAM3U_HSMCI_SDCR);
  regval &= ~HSMCI_SDCR_SDCBUS_MASK;
  regval |= wide ? HSMCI_SDCR_SDCBUS_4BIT : HSMCI_SDCR_SDCBUS_1BIT;
  putreg32(regval, SAM3U_HSMCI_SDCR);

  /* Remember the setting */

  priv->widebus = wide;
}

/****************************************************************************
 * Name: sam3u_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t regval;
  bool enable = true;

  /* Fetch the current mode register and mask out the clkdiv (and pwsdiv) */

  regval = getreg32(SAM3U_HSMCI_MR);
  regval &= ~(HSMCI_MR_CLKDIV_MASK | HSMCI_MR_PWSDIV_MASK);

 /* These clock devisor values that must be defined in the board-specific
  * board.h header file: HSMCI_INIT_CLKDIV, HSMCI_MMCXFR_CLKDIV,
  * HSMCI_SDXFR_CLKDIV, and HSMCI_SDWIDEXFR_CLKDIV.
  */
  
  switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED:     /* Clock is disabled */
      regval |= HSMCI_INIT_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      enable = false;
      return;

    case CLOCK_IDMODE:            /* Initial ID mode clocking (<400KHz) */
      regval |= HSMCI_INIT_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_MMC_TRANSFER:      /* MMC normal operation clocking */
      regval |= HSMCI_MMCXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_SD_TRANSFER_1BIT:  /* SD normal operation clocking (narrow 1-bit mode) */
      regval |= HSMCI_SDXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_SD_TRANSFER_4BIT:  /* SD normal operation clocking (wide 4-bit mode) */
      regval |= HSMCI_SDWIDEXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;
    };

  /* Set the new clock  diver and make sure that the clock is enabled or
   * disabled, whichever the case.
   */

  putreg32(regval, SAM3U_HSMCI_MR);
  if (enable)
    {
      sam3u_enable();
    }
  else
    {
      sam3u_disable();
    }
}

/****************************************************************************
 * Name: sam3u_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int sam3u_attach(FAR struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the HSMCI interrupt handler */

  ret = irq_attach(SAM3U_IRQ_HSMCI, sam3u_interrupt);
  if (ret == OK)
    {

      /* Disable all interrupts at the HSMCI controller and clear (most) static
       * interrupt flags by reading the status register.
       */

      putreg32(0xffffffff, SAM3U_HSMCI_IDR);
      (void)getreg32(SAM3U_HSMCI_SR);

      /* Enable HSMCI interrupts at the NVIC.  They can now be enabled at
       * the HSMCI controller as needed.
       */

      up_enable_irq(SAM3U_IRQ_HSMCI);

      /* Set the interrrupt priority */

      up_prioritize_irq(SAM3U_IRQ_HSMCI, CONFIG_HSMCI_PRI);
    }

  return ret;
}

/****************************************************************************
 * Name: sam3u_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam3u_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  uint32_t regval;
  uint32_t cmdidx;

  sam3u_cmdsampleinit();

    /* Set the HSMCI Argument value */

  putreg32(arg, SAM3U_HSMCI_ARGR);

  /* Construct the command valid, starting with the command index */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval  = cmdidx << HSMCI_CMDR_CMDNB_SHIFT;

  /* 'OR' in response related bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    /* No response */

    case MMCSD_NO_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_NONE;
      
      break;

    /* 48-bit response with CRC */

    case MMCSD_R1_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT);
      break;

    case MMCSD_R1B_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_R1B | HSMCI_CMDR_MAXLAT);
      break;

    /* 48-bit response without CRC */
 
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_NOCRC_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT);
      break;

    /* 136-bit response with CRC */
      
    case MMCSD_R2_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_136BIT | HSMCI_CMDR_MAXLAT);
      break;
    }

  /* 'OR' in data transer related bits */

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
#if 0 /* No MMC support */
    case MMCSD_RDSTREAM: /* MMC Read stream */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM | HSMCI_CMDR_TRDIR_READ);
      break;
 
    case MMCSD_WRSTREAM: /* MMC Write stream */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM | HSMCI_CMDR_TRDIR_WRITE);
      break;
#endif

    case MMCSD_RDDATAXFR: /* Read block transfer */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_READ);
      regval |= (cmd & MMCSD_MULTIBLOCK) ? HSMCI_CMDR_TRTYP_MULTI : HSMCI_CMDR_TRTYP_SINGLE;
      break;
 
    case MMCSD_WRDATAXFR: /* Write block transfer */   
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_WRITE);
      regval |= (cmd & MMCSD_MULTIBLOCK) ? HSMCI_CMDR_TRTYP_MULTI : HSMCI_CMDR_TRTYP_SINGLE;
      break;
 
    case MMCSD_NODATAXFR:
    default:
      if ((cmd & MMCSD_STOPXFR) != 0)
        {
          regval |= HSMCI_CMDR_TRCMD_STOP;
        }
      break;
    }

  /* 'OR' in Open Drain option */

#if 0 /* No MMC support */
  if ((cmd & MMCSD_OPENDRAIN) != 0)
    {
      regval |= HSMCI_CMDR_OPDCMD;
    }
#endif

  /* Write the fully decorated command to CMDR */

  fvdbg("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);
  putreg32(regval, SAM3U_HSMCI_CMDR);
  sam3u_cmdsample1(SAMPLENDX_AFTER_CMDR);
  return OK;
}

/****************************************************************************
 * Name: sam3u_blocksetup
 *
 * Description:
 *   Some hardward needs to be informed of the selected blocksize.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   blocklen - The selected block size.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_blocksetup(FAR struct sdio_dev_s *dev, unsigned int blocklen,
                             unsigned int nblocks)
{
  uint32_t regval;

  DEBUGASSERT(dev != NULL && nblocks > 0 && nblocks < 65535 && blocklen < 65535);

  /* Set the block size */

  regval = getreg32(SAM3U_HSMCI_MR);
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF | HSMCI_MR_BLKLEN_MASK);
  regval |= HSMCU_PROOF_BITS;
  regval |= (blocklen << HSMCI_MR_BLKLEN_SHIFT);
  putreg32(regval, SAM3U_HSMCI_MR);

  /* Set the block count */

  regval  = getreg32(SAM3U_HSMCI_BLKR);
  regval &= ~HSMCI_BLKR_BCNT_MASK;
  regval |= (nblocks << HSMCI_BLKR_BCNT_SHIFT);
  putreg32(regval, SAM3U_HSMCI_BLKR);
}

/****************************************************************************
 * Name: sam3u_cancel
 *
 * Description:
 *   Cancel the data transfer setup of HSMCI_RECVSETUP, HSMCI_SENDSETUP,
 *   HSMCI_DMARECVSETUP or HSMCI_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sam3u_cancel(FAR struct sdio_dev_s *dev)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;

  /* Disable all transfer- and event- related interrupts */

  sam3u_disablexfrints(priv);
  sam3u_disablewaitints(priv, 0);

  /* No data transfer */

  sam3u_notransfer(priv);

  /* Clearing (most) pending interrupt status by reading the status register */
 
  (void)getreg32(SAM3U_HSMCI_SR);

  /* Cancel any watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition.
   */

  sam3u_dmastop(priv->dma);

  /* Disable the DMA handshaking */

  putreg32(0, SAM3U_HSMCI_DMA);
  
  return OK;
}

/****************************************************************************
 * Name: sam3u_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sam3u_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  uint32_t sr;
  int32_t  timeout;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = HSMCI_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_NO_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = HSMCI_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  for (;;)
    {
      /* Did a Command-Response sequence termination evernt occur? */

      sr = getreg32(SAM3U_HSMCI_SR);
      if ((sr & priv->cmdrmask) != 0)
        {
          sam3u_cmdsample2(SAMPLENDX_AT_WAKEUP, sr);
          sam3u_cmddump();

          /* Yes.. Did the Command-Response sequence end with an error? */

          if ((sr & HSMCI_RESPONSE_ERRORS) != 0)
            {
              /* Yes.. Was the error some kind of timeout? */

              fdbg("ERROR: cmd: %08x events: %08x SR: %08x\n",
                   cmd, priv->cmdrmask, sr);

              if ((sr & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. return a timeout error */

                  priv->wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_TIMEOUT;
                  return -ETIMEDOUT;
                }
              else
                {
                  /* No.. return some generic I/O error */

                  priv->wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_ERROR;
                  return -EIO;
                }
            }
          else
            {
              /* The Command-Response sequence ended with no error */

              priv->wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE;
              return OK;
            }
       }
      else if (--timeout <= 0)
        {
          fdbg("ERROR: Timeout cmd: %08x events: %08x SR: %08x\n",
               cmd, priv->cmdrmask, sr);

          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return -ETIMEDOUT;
        }
    }
}

/****************************************************************************
 * Name: sam3u_recvRx
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *   Rx  - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a failure to obtain the requested reponse (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int sam3u_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  int ret = OK;

  /* These responses could have CRC errors:
   *
   * R1  Command response (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit card status
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * R1b Identical to R1 with the additional busy signaling via the data
   *     line.
   *
   * R6  Published RCA Response (48-bit, SD card only)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit Argument Field, consisting of:
   *                               [31:16] New published RCA of card
   *                               [15:0]  Card status bits {23,22,19,12:0}
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * But there is no parity on the R3 response and parity errors should
   * be ignored.
   *
   * R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

#ifdef CONFIG_DEBUG
  if (!rshort)
    {
      fdbg("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif

  /* Check for timeout errors */

  if ((priv->wkupevent & SDIOWAIT_TIMEOUT) != 0)
    {
      ret = -EINVAL;
    }

  /* Check for other errors */

  else if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
    {
      ret = -EIO;
    }

  /* Return the R1/R6 response */

  else if (rshort)
    {
      *rshort = getreg32(SAM3U_HSMCI_RSPR0);
    }

  priv->wkupevent = 0;
  return ret;
}

static int sam3u_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t rlong[4])
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  int ret = OK;

 /* R2  CID, CSD register (136-bit)
  *     135       0               Start bit
  *     134       0               Transmission bit (0=from card)
  *     133:128   bit5   - bit0   Reserved
  *     127:1     bit127 - bit1   127-bit CID or CSD register
  *                               (including internal CRC)
  *     0         1               End bit
  */

#ifdef CONFIG_DEBUG
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    
  /* Check for timeout errors */

  if ((priv->wkupevent & SDIOWAIT_TIMEOUT) != 0)
    {
      ret = -EINVAL;
    }

  /* Check for other errors */

  else if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
    {
      ret = -EIO;
    }

  /* Return the long response */

  else if (rlong)
    {
      rlong[0] = getreg32(SAM3U_HSMCI_RSPR0);
      rlong[1] = getreg32(SAM3U_HSMCI_RSPR1);
      rlong[2] = getreg32(SAM3U_HSMCI_RSPR2);
      rlong[3] = getreg32(SAM3U_HSMCI_RSPR3);
    }

  priv->wkupevent = 0;
  return ret;
}

/* MMC responses not supported */

static int sam3u_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rnotimpl)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  priv->wkupevent = 0;
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam3u_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the HSMCI_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling sam3u_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) HSMCI_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) HSMCI_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_waitenable(FAR struct sdio_dev_s *dev,
                             sdio_eventset_t eventset)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  uint32_t waitmask;
 
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  sam3u_disablewaitints(priv, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & (SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitmask |= priv->cmdrmask;
    }

  /* Enable event-related interrupts */

  (void)getreg32(SAM3U_HSMCI_SR);
  sam3u_enablewaitints(priv, waitmask, eventset);
}

/****************************************************************************
 * Name: sam3u_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by HSMCI_WAITEVENTS are disabled when sam3u_eventwait
 *   returns.  HSMCI_WAITEVENTS must be called again before sam3u_eventwait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t sam3u_eventwait(FAR struct sdio_dev_s *dev,
                                       uint32_t timeout)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  DEBUGASSERT((priv->waitevents != 0 && priv->wkupevent == 0) ||
              (priv->waitevents == 0 && priv->wkupevent != 0));

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;

      /* Yes.. Handle a cornercase */

      if (!timeout)
        {
           return SDIOWAIT_TIMEOUT;
        }

      /* Start the watchdog timer */

      delay = (timeout + (MSEC_PER_TICK-1)) / MSEC_PER_TICK;
      ret   = wd_start(priv->waitwdog, delay, (wdentry_t)sam3u_eventtimeout,
                       1, (uint32_t)priv);
      if (ret != OK)
        {
           fdbg("ERROR: wd_start failed: %d\n", ret);
         }
    }

  /* Loop until the event (or the timeout occurs). Race conditions are avoided
   * by calling sam3u_waitenable prior to triggering the logic that will cause
   * the wait to terminate.  Under certain race conditions, the waited-for
   * may have already occurred before this function was called!
   */

  for (;;)
    {
      /* Wait for an event in event set to occur.  If this the event has already
       * occurred, then the semaphore will already have been incremented and
       * there will be no wait.
       */

      sam3u_takesem(priv);
      wkupevent = priv->wkupevent;
 
      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero value.
       * When wkupevent becomes non-zero, further interrupts will have already
       * been disabled.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  sam3u_cmddump();
  sam3u_xfrdump(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: sam3u_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in sam3u_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methos.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_callbackenable(FAR struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;

  fvdbg("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  sam3u_callback(priv);
}

/****************************************************************************
 * Name: sam3u_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to HSMCI_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int sam3u_registercallback(FAR struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;

  /* Disable callbacks and register this callback and is argument */

  fvdbg("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: sam3u_dmasupported
 *
 * Description:
 *   Return true if the hardware can support DMA
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   true if DMA is supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static bool sam3u_dmasupported(FAR struct sdio_dev_s *dev)
{
  return true;
}
#endif

/****************************************************************************
 * Name: sam3u_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam3u_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                              size_t buflen)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Setup register sampling */

  sam3u_xfrsampleinit();
  sam3u_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Configure the RX DMA */

  sam3u_enablexfrints(priv, HSMCI_DMARECV_INTS);
  sam3u_dmarxsetup(priv->dma, SAM3U_HSMCI_FIFO, (uint32_t)buffer, buflen);

  /* Enable DMA handshaking */

  putreg32(HSMCI_DMA_DMAEN, SAM3U_HSMCI_DMA);
  sam3u_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  sam3u_dmastart(priv->dma, sam3u_dmacallback, priv);
  sam3u_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: sam3u_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam3u_dmasendsetup(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Setup register sampling */

  sam3u_xfrsampleinit();
  sam3u_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Configure the TX DMA */

  sam3u_dmatxsetup(priv->dma, SAM3U_HSMCI_FIFO, (uint32_t)buffer, buflen);

  /* Enable DMA handshaking */

  putreg32(HSMCI_DMA_DMAEN, SAM3U_HSMCI_DMA);
  sam3u_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  sam3u_dmastart(priv->dma, sam3u_dmacallback, priv);
  sam3u_xfrsample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrrupts */

  sam3u_enablexfrints(priv, HSMCI_DMASEND_INTS);
  return OK;
}

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: sam3u_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void sam3u_callback(void *arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  fvdbg("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
        priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
           {
             /* No... return without performing the callback */

              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* Callbacks cannot be performed in the context of an interrupt handler.
       * If we are in an interrupt handler, then queue the callback to be
       * performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

           fvdbg("Queuing callback to %p(%p)\n", priv->callback, priv->cbarg);
          (void)work_queue(HPWORK, &priv->cbwork, (worker_t)priv->callback, priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          fvdbg("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SD for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  /* There is only one slot */

  struct sam3u_dev_s *priv = &g_sdiodev;

  fdbg("slotno: %d\n", slotno);

  /* Initialize the HSMCI slot structure */

  sem_init(&priv->waitsem, 0, 0);
  priv->waitwdog = wd_create();
  DEBUGASSERT(priv->waitwdog);

  /* Allocate a DMA channel.  A FIFO size of 8 is sufficient. */

  priv->dma = sam3u_dmachannel(DMA_FLAGS);
  DEBUGASSERT(priv->dma);

  /* Configure GPIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip is capable of
   * 8-bit wide bus operation but D4-D7 are not configured, (2) any card detection
   * GPIOs must be set up in board-specific logic.
   */

  sam3u_configgpio(GPIO_MCI_DAT0);   /* Data 0 of Slot A */
  sam3u_configgpio(GPIO_MCI_DAT1);   /* Data 1 of Slot A */
  sam3u_configgpio(GPIO_MCI_DAT2);   /* Data 2 of Slot A */
  sam3u_configgpio(GPIO_MCI_DAT3);   /* Data 3 of Slot A */
  sam3u_configgpio(GPIO_MCI_CK);     /* SD clock */
  sam3u_configgpio(GPIO_MCI_DA);     /* Command/Response */

#ifdef CONFIG_DEBUG_FS
  sam3u_dumpgpio(GPIO_PORT_PIOA, "Pins: 3-8");
  sam3u_dumpgpio(GPIO_PORT_PIOB, "Pins: 28-31");
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  sam3u_reset(&priv->dev);
  return &g_sdiodev.dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- posssible from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a 
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint8_t cdstatus;
  irqstate_t flags;

  /* Update card status */

  flags = irqsave();
  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }
  fvdbg("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      sam3u_callback(priv);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  irqstate_t flags;

  /* Update card status */

  flags = irqsave();
  if (wrprotect)
    {
      priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
    }
  fvdbg("cdstatus: %02x\n", priv->cdstatus);
  irqrestore(flags);
}
#endif /* CONFIG_SAM3U_HSMCI */
