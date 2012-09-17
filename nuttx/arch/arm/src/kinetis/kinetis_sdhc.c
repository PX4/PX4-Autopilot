/****************************************************************************
 * arch/arm/src/kinetis/kinetis_sdhc.c
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

#include "kinetis_internal.h"
#include "kinetis_pinmux.h"
#include "kinetis_sim.h"
#include "kinetis_sdhc.h"

#if CONFIG_KINETIS_SDHC

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SDIO_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_KINETIS_SDHC_PRIO
#  define CONFIG_KINETIS_SDHC_PRIO NVIC_SYSH_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_KINETIS_SDHC_DMAPRIO
#  define CONFIG_KINETIS_SDHC_DMAPRIO    DMA_CCR_PRIMED
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_VERBOSE)
#  undef CONFIG_SDIO_XFRDEBUG
#endif

/* SDCLK frequencies corresponding to various modes of operation.  These
 * values may be provided in either the NuttX configuration file or in
 * the board.h file
 *
 * NOTE:  These settings are not currently used.  Since there are only four
 * frequencies, it makes more sense to just "can" the fixed frequency prescaler
 * and divider values.
 */

#if CONFIG_KINETIS_SDHC_ABSFREQ
#  ifndef CONFIG_KINETIS_IDMODE_FREQ
#    define CONFIG_KINETIS_IDMODE_FREQ 400000    /* 400 KHz, ID mode */
#  endif
#  ifndef CONFIG_KINETIS_MMCXFR_FREQ
#    define CONFIG_KINETIS_MMCXFR_FREQ 20000000  /* 20MHz MMC, normal clocking */
#  endif
#  ifndef CONFIG_KINETIS_SD1BIT_FREQ
#    define CONFIG_KINETIS_SD1BIT_FREQ 20000000  /* 20MHz SD 1-bit, normal clocking */
#  endif
#  ifndef CONFIG_KINETIS_SD4BIT_FREQ
#    define CONFIG_KINETIS_SD4BIT_FREQ 25000000  /* 25MHz SD 4-bit, normal clocking */
#  endif
#endif

/* Timing */

#define SDHC_CMDTIMEOUT         (100000)
#define SDHC_LONGTIMEOUT        (0x7fffffff)

/* Big DVS setting.  Range is 0=SDCLK*213 through 14=SDCLK*227 */

#define SDHC_DVS_MAXTIMEOUT     (14)
#define SDHC_DVS_DATATIMEOUT    (14)

/* Maximum watermark value */

#define SDHC_MAX_WATERMARK      128

/* Data transfer / Event waiting interrupt mask bits */

#define SDHC_RESPERR_INTS  (SDHC_INT_CCE|SDHC_INT_CTOE|SDHC_INT_CEBE|SDHC_INT_CIE)
#define SDHC_RESPDONE_INTS (SDHC_RESPERR_INTS|SDHC_INT_CC)

#define SCHC_XFRERR_INTS   (SDHC_INT_DCE|SDHC_INT_DTOE|SDHC_INT_DEBE)
#define SDHC_RCVDONE_INTS  (SCHC_XFRERR_INTS|SDHC_INT_BRR|SDHC_INT_TC)
#define SDHC_SNDDONE_INTS  (SCHC_XFRERR_INTS|SDHC_INT_BWR|SDHC_INT_TC)
#define SDHC_XFRDONE_INTS  (SCHC_XFRERR_INTS|SDHC_INT_BRR|SDHC_INT_BWR|SDHC_INT_TC)

#define SCHC_DMAERR_INTS   (SDHC_INT_DCE|SDHC_INT_DTOE|SDHC_INT_DEBE|SDHC_INT_DMAE)
#define SDHC_DMADONE_INTS  (SCHC_DMAERR_INTS|SDHC_INT_DINT)

#define SDHC_WAITALL_INTS  (SDHC_RESPDONE_INTS|SDHC_XFRDONE_INTS|SDHC_DMADONE_INTS)

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
#  define SAMPLENDX_BEFORE_SETUP  0
#  define SAMPLENDX_AFTER_SETUP   1
#  define SAMPLENDX_END_TRANSFER  2
#  define DEBUG_NSAMPLES          3
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the Kinetis SDIO interface */

struct kinetis_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SDIO interface */
  
  /* Kinetis-specific extensions */
  /* Event support */

  sem_t              waitsem;    /* Implements event waiting */
  sdio_eventset_t    waitevents; /* Set of events to be waited for */
  uint32_t           waitints;   /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  WDOG_ID            waitwdog;   /* Watchdog that handles event timeouts */

  /* Callback support */

  uint8_t            cdstatus;   /* Card status */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  worker_t           callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */
  struct work_s      cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;     /* Address of current R/W buffer */
  size_t             remaining;  /* Number of bytes remaining in the transfer */
  uint32_t           xfrints;    /* Interrupt enables for data transfer */

  /* DMA data transfer support */

#ifdef CONFIG_SDIO_DMA
  volatile uint8_t   xfrflags;   /* Used to synchronize SDIO and DMA completion events */
#endif
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct kinetis_sdhcregs_s
{
  /* All read-able SDHC registers */

  uint32_t dsaddr;    /* DMA System Address Register */
  uint32_t blkattr;   /* Block Attributes Register */
  uint32_t cmdarg;    /* Command Argument Register */
  uint32_t xferty;    /* Transfer Type Register */
  uint32_t cmdrsp0;   /* Command Response 0 */
  uint32_t cmdrsp1;   /* Command Response 1 */
  uint32_t cmdrsp2;   /* Command Response 2 */
  uint32_t cmdrsp3;   /* Command Response 3 */
  uint32_t prsstat;   /* Present State Register */
  uint32_t proctl;    /* Protocol Control Register */
  uint32_t sysctl;    /* System Control Register */
  uint32_t irqstat;   /* Interrupt Status Register */
  uint32_t irqstaten; /* Interrupt Status Enable Register */
  uint32_t irqsigen;  /* Interrupt Signal Enable Register */
  uint32_t ac12err;   /* Auto CMD12 Error Status Register */
  uint32_t htcapblt;  /* Host Controller Capabilities */
  uint32_t wml;       /* Watermark Level Register */
  uint32_t admaes;    /* ADMA Error Status Register */
  uint32_t adsaddr;   /* ADMA System Address Register */
  uint32_t vendor;    /* Vendor Specific Register */
  uint32_t mmcboot;   /* MMC Boot Register */
  uint32_t hostver;   /* Host Controller Version */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void kinetis_takesem(struct kinetis_dev_s *priv);
#define     kinetis_givesem(priv) (sem_post(&priv->waitsem))
static void kinetis_configwaitints(struct kinetis_dev_s *priv, uint32_t waitints,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void kinetis_configxfrints(struct kinetis_dev_s *priv, uint32_t xfrints);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void kinetis_sampleinit(void);
static void kinetis_sdhcsample(struct kinetis_sdhcregs_s *regs);
static void kinetis_sample(struct kinetis_dev_s *priv, int index);
static void kinetis_dumpsample(struct kinetis_dev_s *priv,
              struct kinetis_sdhcregs_s *regs, const char *msg);
static void kinetis_dumpsamples(struct kinetis_dev_s *priv);
static void kinetis_showregs(struct kinetis_dev_s *priv, const char *msg);
#else
#  define   kinetis_sampleinit()
#  define   kinetis_sample(priv,index)
#  define   kinetis_dumpsamples(priv)
#  define   kinetis_showregs(priv,msg)
#endif

/* Data Transfer Helpers ****************************************************/

static void kinetis_dataconfig(struct kinetis_dev_s *priv, bool bwrite,
                               unsigned int blocksize, unsigned int nblocks,
                               unsigned int timeout);
static void kinetis_datadisable(void);
#ifndef CONFIG_SDIO_DMA
static void kinetis_transmit(struct kinetis_dev_s *priv);
static void kinetis_receive(struct kinetis_dev_s *priv);
#endif
static void kinetis_eventtimeout(int argc, uint32_t arg);
static void kinetis_endwait(struct kinetis_dev_s *priv, sdio_eventset_t wkupevent);
static void kinetis_endtransfer(struct kinetis_dev_s *priv, sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  kinetis_interrupt(int irq, void *context);

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int kinetis_lock(FAR struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void kinetis_reset(FAR struct sdio_dev_s *dev);
static uint8_t kinetis_status(FAR struct sdio_dev_s *dev);
static void kinetis_widebus(FAR struct sdio_dev_s *dev, bool enable);
#if CONFIG_KINETIS_SDHC_ABSFREQ
static void kinetis_frequency(FAR struct sdio_dev_s *dev, uint32_t frequency);
#endif
static void kinetis_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  kinetis_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  kinetis_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
#ifndef CONFIG_SDIO_DMA
static int  kinetis_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
              size_t nbytes);
static int  kinetis_sendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, uint32_t nbytes);
#endif
static int  kinetis_cancel(FAR struct sdio_dev_s *dev);

static int  kinetis_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  kinetis_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  kinetis_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  kinetis_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  kinetis_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void kinetis_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static sdio_eventset_t
            kinetis_eventwait(FAR struct sdio_dev_s *dev, uint32_t timeout);
static void kinetis_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  kinetis_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static bool kinetis_dmasupported(FAR struct sdio_dev_s *dev);
static int  kinetis_dmarecvsetup(FAR struct sdio_dev_s *dev,
              FAR uint8_t *buffer, size_t buflen);
static int  kinetis_dmasendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void kinetis_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct kinetis_dev_s g_sdhcdev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = kinetis_lock,
#endif
    .reset            = kinetis_reset,
    .status           = kinetis_status,
    .widebus          = kinetis_widebus,
    .clock            = kinetis_clock,
    .attach           = kinetis_attach,
    .sendcmd          = kinetis_sendcmd,
#ifndef CONFIG_SDIO_DMA
    .recvsetup        = kinetis_recvsetup,
    .sendsetup        = kinetis_sendsetup,
#else
    .recvsetup        = kinetis_dmarecvsetup,
    .sendsetup        = kinetis_dmasendsetup,
#endif
    .cancel           = kinetis_cancel,
    .waitresponse     = kinetis_waitresponse,
    .recvR1           = kinetis_recvshortcrc,
    .recvR2           = kinetis_recvlong,
    .recvR3           = kinetis_recvshort,
    .recvR4           = kinetis_recvnotimpl,
    .recvR5           = kinetis_recvnotimpl,
    .recvR6           = kinetis_recvshortcrc,
    .recvR7           = kinetis_recvshort,
    .waitenable       = kinetis_waitenable,
    .eventwait        = kinetis_eventwait,
    .callbackenable   = kinetis_callbackenable,
    .registercallback = kinetis_registercallback,
#ifdef CONFIG_SDIO_DMA
    .dmasupported     = kinetis_dmasupported,
    .dmarecvsetup     = kinetis_dmarecvsetup,
    .dmasendsetup     = kinetis_dmasendsetup,
#endif
  },
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
static struct kinetis_sdhcregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: kinetis_takesem
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

static void kinetis_takesem(struct kinetis_dev_s *priv)
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
 * Name: kinetis_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to suport the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitints   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void kinetis_configwaitints(struct kinetis_dev_s *priv, uint32_t waitints,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = irqsave();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitints   = waitints;
#ifdef CONFIG_SDIO_DMA
  priv->xfrflags   = 0;
#endif
  putreg32(priv->xfrints | priv->waitints | SDHC_INT_CINT,
           KINETIS_SDHC_IRQSIGEN);
  irqrestore(flags);
}

/****************************************************************************
 * Name: kinetis_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrints - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void kinetis_configxfrints(struct kinetis_dev_s *priv, uint32_t xfrints)
{
  irqstate_t flags;
  flags = irqsave();
  priv->xfrints = xfrints;
  putreg32(priv->xfrints | priv->waitints | SDHC_INT_CINT,
           KINETIS_SDHC_IRQSIGEN);
  irqrestore(flags);
}

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void kinetis_sampleinit(void)
{
  memset(g_sampleregs, 0xff, DEBUG_NSAMPLES * sizeof(struct kinetis_sdhcregs_s));
}
#endif

/****************************************************************************
 * Name: kinetis_sdhcsample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void kinetis_sdhcsample(struct kinetis_sdhcregs_s *regs)
{
  regs->dsaddr    = getreg32(KINETIS_SDHC_DSADDR);    /* DMA System Address Register */
  regs->blkattr   = getreg32(KINETIS_SDHC_BLKATTR);   /* Block Attributes Register */
  regs->cmdarg    = getreg32(KINETIS_SDHC_CMDARG);    /* Command Argument Register */
  regs->xferty    = getreg32(KINETIS_SDHC_XFERTYP);   /* Transfer Type Register */
  regs->cmdrsp0   = getreg32(KINETIS_SDHC_CMDRSP0);   /* Command Response 0 */
  regs->cmdrsp1   = getreg32(KINETIS_SDHC_CMDRSP1);   /* Command Response 1 */
  regs->cmdrsp2   = getreg32(KINETIS_SDHC_CMDRSP2);   /* Command Response 2 */
  regs->cmdrsp3   = getreg32(KINETIS_SDHC_CMDRSP3);   /* Command Response 3 */
  regs->prsstat   = getreg32(KINETIS_SDHC_PRSSTAT);   /* Present State Register */
  regs->proctl    = getreg32(KINETIS_SDHC_PROCTL);    /* Protocol Control Register */
  regs->sysctl    = getreg32(KINETIS_SDHC_SYSCTL);    /* System Control Register */
  regs->irqstat   = getreg32(KINETIS_SDHC_IRQSTAT);   /* Interrupt Status Register */
  regs->irqstaten = getreg32(KINETIS_SDHC_IRQSTATEN); /* Interrupt Status Enable Register */
  regs->irqsigen  = getreg32(KINETIS_SDHC_IRQSIGEN);  /* Interrupt Signal Enable Register */
  regs->ac12err   = getreg32(KINETIS_SDHC_AC12ERR);   /* Auto CMD12 Error Status Register */
  regs->htcapblt  = getreg32(KINETIS_SDHC_HTCAPBLT);  /* Host Controller Capabilities */
  regs->wml       = getreg32(KINETIS_SDHC_WML);       /* Watermark Level Register */
  regs->admaes    = getreg32(KINETIS_SDHC_ADMAES);    /* ADMA Error Status Register */
  regs->adsaddr   = getreg32(KINETIS_SDHC_ADSADDR);   /* ADMA System Address Register */
  regs->vendor    = getreg32(KINETIS_SDHC_VENDOR);    /* Vendor Specific Register */
  regs->mmcboot   = getreg32(KINETIS_SDHC_MMCBOOT);   /* MMC Boot Register */
  regs->hostver   = getreg32(KINETIS_SDHC_HOSTVER);   /* Host Controller Version */
}
#endif

/****************************************************************************
 * Name: kinetis_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void kinetis_sample(struct kinetis_dev_s *priv, int index)
{
  kinetis_sdhcsample(&g_sampleregs[index]);
}
#endif

/****************************************************************************
 * Name: kinetis_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void kinetis_dumpsample(struct kinetis_dev_s *priv,
                               struct kinetis_sdhcregs_s *regs, const char *msg)
{
  fdbg("SDHC Registers: %s\n", msg);
  fdbg("   DSADDR[%08x]: %08x\n", KINETIS_SDHC_DSADDR,    regs->dsaddr);
  fdbg("  BLKATTR[%08x]: %08x\n", KINETIS_SDHC_BLKATTR,   regs->blkattr);
  fdbg("   CMDARG[%08x]: %08x\n", KINETIS_SDHC_CMDARG,    regs->cmdarg);
  fdbg("   XFERTY[%08x]: %08x\n", KINETIS_SDHC_XFERTYP,   regs->xferty);
  fdbg("  CMDRSP0[%08x]: %08x\n", KINETIS_SDHC_CMDRSP0,   regs->cmdrsp0);
  fdbg("  CMDRSP1[%08x]: %08x\n", KINETIS_SDHC_CMDRSP1,   regs->cmdrsp1);
  fdbg("  CMDRSP2[%08x]: %08x\n", KINETIS_SDHC_CMDRSP2,   regs->cmdrsp2);
  fdbg("  CMDRSP3[%08x]: %08x\n", KINETIS_SDHC_CMDRSP3,   regs->cmdrsp3);
  fdbg("  PRSSTAT[%08x]: %08x\n", KINETIS_SDHC_PRSSTAT,   regs->prsstat);
  fdbg("   PROCTL[%08x]: %08x\n", KINETIS_SDHC_PROCTL,    regs->proctl);
  fdbg("   SYSCTL[%08x]: %08x\n", KINETIS_SDHC_SYSCTL,    regs->sysctl);
  fdbg("  IRQSTAT[%08x]: %08x\n", KINETIS_SDHC_IRQSTAT,   regs->irqstat);
  fdbg("IRQSTATEN[%08x]: %08x\n", KINETIS_SDHC_IRQSTATEN, regs->irqstaten);
  fdbg(" IRQSIGEN[%08x]: %08x\n", KINETIS_SDHC_IRQSIGEN,  regs->irqsigen);
  fdbg("  AC12ERR[%08x]: %08x\n", KINETIS_SDHC_AC12ERR,   regs->ac12err);
  fdbg(" HTCAPBLT[%08x]: %08x\n", KINETIS_SDHC_HTCAPBLT,  regs->htcapblt);
  fdbg("      WML[%08x]: %08x\n", KINETIS_SDHC_WML,       regs->wml);
  fdbg("   ADMAES[%08x]: %08x\n", KINETIS_SDHC_ADMAES,    regs->admaes);
  fdbg("  ADSADDR[%08x]: %08x\n", KINETIS_SDHC_ADSADDR,   regs->adsaddr);
  fdbg("   VENDOR[%08x]: %08x\n", KINETIS_SDHC_VENDOR,    regs->vendor);
  fdbg("  MMCBOOT[%08x]: %08x\n", KINETIS_SDHC_MMCBOOT,   regs->mmcboot);
  fdbg("  HOSTVER[%08x]: %08x\n", KINETIS_SDHC_HOSTVER,   regs->hostver);
}
#endif

/****************************************************************************
 * Name: kinetis_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void  kinetis_dumpsamples(struct kinetis_dev_s *priv)
{
  kinetis_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP], "Before setup");
  kinetis_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP], "After setup");
  kinetis_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER], "End of transfer");
}
#endif

/****************************************************************************
 * Name: kinetis_showregs
 *
 * Description:
 *   Dump the current state of all registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void kinetis_showregs(struct kinetis_dev_s *priv, const char *msg)
{
  struct kinetis_sdhcregs_s regs;

  kinetis_sdhcsample(&regs);
  kinetis_dumpsample(priv, &regs, msg);
}
#endif

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static void kinetis_dataconfig(struct kinetis_dev_s *priv, bool bwrite,
                               unsigned int blocksize, unsigned int nblocks,
                               unsigned int timeout)
{
  unsigned int watermark;
  uint32_t regval = 0;

  /* Set the data timeout value in the SDHC_SYSCTL field to the selected value */

  regval  = getreg32(KINETIS_SDHC_SYSCTL);
  regval &= ~SDHC_SYSCTL_DVS_MASK;
  regval |= timeout << SDHC_SYSCTL_DVS_SHIFT;
  putreg32(regval, KINETIS_SDHC_SYSCTL);

  /* Set the block size and count in the SDHC_BLKATTR register.  The block
   * size is only valid for multiple block transfers.
   */

  regval = blocksize << SDHC_BLKATTR_SIZE_SHIFT |
           nblocks   << SDHC_BLKATTR_CNT_SHIFT;
  putreg32(regval, KINETIS_SDHC_BLKATTR);

  /* Set the watermark level */

#ifdef CONFIG_SDIO_DMA
  /* Set the Read Watermark Level to the blocksize to be read
   * (limited to half of the maximum watermark value).  BRR will be
   * set when the number of queued words is greater than or equal
   * to this value.
   */

  watermark = (blocksize + 3) >> 2;
  if (watermark > (SDHC_MAX_WATERMARK / 2))
    {
      watermark = (SDHC_MAX_WATERMARK / 2);
    }

  /* When the watermark level requirement is met in data transfer, and the
   * internal DMA is enabled, the data buffer block sends a DMA request to
   * the crossbar switch interface.
   */

  if (bwrite)
    {
      /* The SDHC will not start data transmission until the number of
       * words set in the WML register can be held in the buffer. If the
       * buffer is empty and the host system does not write data in time,
       * the SDHC will stop the SD_CLK to avoid the data buffer under-run
       * situation.
       */

      putreg32(watermark << SDHC_WML_WR_SHIFT, KINETIS_SDHC_WML);
    }
  else
    {
      /* The SDHC will not start data transmission until the number of
       * words set in the WML register are in the buffer. If the buffer
       * is full and the Host System does not read data in time, the
       * SDHC will stop the SDHC_DCLK to avoid the data buffer over-run
       * situation.
       */

      putreg32(watermark << SDHC_WML_RD_SHIFT, KINETIS_SDHC_WML);
    }
#else
  if (bwrite)
    {
      /* Write Watermark Level = 0:  BWR will be set when the number of
       * queued words is less than or equal to 0.
       */

      putreg32(0, KINETIS_SDHC_WML);
    }
  else
    {
      /* Set the Read Watermark Level to the blocksize to be read
       * (limited to half of the maximum watermark value).  BRR will be
       * set when the number of queued words is greater than or equal
       * to this value.
       */

      watermark = (blocksize + 3) >> 2;
      if (watermark > (SDHC_MAX_WATERMARK / 2))
        {
          watermark = (SDHC_MAX_WATERMARK / 2);
        }
      putreg32(watermark << SDHC_WML_RD_SHIFT, KINETIS_SDHC_WML);        
    }
#endif
}

/****************************************************************************
 * Name: kinetis_datadisable
 *
 * Description:
 *   Disable the the SDIO data path setup by kinetis_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void kinetis_datadisable(void)
{
  uint32_t regval;

  /* Set the data timeout value in the SDHC_SYSCTL field to the maximum value */

  regval  = getreg32(KINETIS_SDHC_SYSCTL);
  regval &= ~SDHC_SYSCTL_DVS_MASK;
  regval |= SDHC_DVS_MAXTIMEOUT << SDHC_SYSCTL_DVS_SHIFT;
  putreg32(regval, KINETIS_SDHC_SYSCTL);

  /* Set the block size to zero (no transfer) */

  putreg32(0, KINETIS_SDHC_BLKATTR);
}

/****************************************************************************
 * Name: kinetis_transmit
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SDIO_DMA
static void kinetis_transmit(struct kinetis_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent, waiting for buffer write
   * ready (BWR)
   */

  fllvdbg("Entry: remaining: %d IRQSTAT: %08x\n",
          priv->remaining, getreg32(KINETIS_SDHC_IRQSTAT));

  while (priv->remaining > 0 &&
         (getreg32(KINETIS_SDHC_IRQSTAT) & SDHC_INT_BWR) != 0)
    {
      /* Clear BWR.  If there is more data in the buffer, writing to the
       * buffer should reset BRR.
       */

      putreg32(SDHC_INT_BWR, KINETIS_SDHC_IRQSTAT);

      /* Is there a full word remaining in the user buffer? */

      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Yes, transfer the word to the TX FIFO */

          data.w           = *priv->buffer++;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          /* No.. transfer just the bytes remaining in the user buffer,
           * padding with zero as necessary to extend to a full word.
           */

          uint8_t *ptr = (uint8_t *)priv->remaining;
          int i;

          data.w = 0;
          for (i = 0; i < priv->remaining; i++)
            {
               data.b[i] = *ptr++;
            }
 
          /* Now the transfer is finished */

          priv->remaining = 0;
        }

       /* Put the word in the FIFO */

       putreg32(data.w, KINETIS_SDHC_DATPORT);
    }

  fllvdbg("Exit: remaining: %d IRQSTAT: %08x\n",
          priv->remaining, getreg32(KINETIS_SDHC_IRQSTAT));

}
#endif

/****************************************************************************
 * Name: kinetis_receive
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SDIO_DMA
static void kinetis_receive(struct kinetis_dev_s *priv)
{
  unsigned int watermark;
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Set the Read Watermark Level to 1:  BRR will be set when the number of
   * queued words is greater than or equal to 1.
   */

  putreg32(1 << SDHC_WML_RD_SHIFT, KINETIS_SDHC_WML);        

  /* Loop while there is space to store the data, waiting for buffer read
   * ready (BRR)
   */

  fllvdbg("Entry: remaining: %d IRQSTAT: %08x\n",
          priv->remaining, getreg32(KINETIS_SDHC_IRQSTAT));

  while (priv->remaining > 0 &&
         (getreg32(KINETIS_SDHC_IRQSTAT) & SDHC_INT_BRR) != 0)
    {
      /* Clear BRR.  If there is more data in the buffer, reading from the
       * buffer should reset BRR.
       */

      putreg32(SDHC_INT_BRR, KINETIS_SDHC_IRQSTAT);

      /* Read the next word from the RX buffer */

      data.w = getreg32(KINETIS_SDHC_DATPORT);
      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Transfer the whole word to the user buffer */

          *priv->buffer++  = data.w;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          /* Transfer any trailing fractional word */

          uint8_t *ptr = (uint8_t*)priv->buffer;
          int i;

          for (i = 0; i < priv->remaining; i++)
            {
               *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }

  /* Set the Read Watermark Level either the number of remaining words to be
   * read (limited to half of the maximum watermark value)
   */

  watermark = ((priv->remaining + 3) >> 2);
  if (watermark > (SDHC_MAX_WATERMARK / 2))
    {
      watermark = (SDHC_MAX_WATERMARK / 2);
    }
  putreg32(watermark << SDHC_WML_RD_SHIFT, KINETIS_SDHC_WML);        

  fllvdbg("Exit: remaining: %d IRQSTAT: %08x WML: %08x\n",
          priv->remaining, getreg32(KINETIS_SDHC_IRQSTAT),
          getreg32(KINETIS_SDHC_WML));

}
#endif

/****************************************************************************
 * Name: kinetis_eventtimeout
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

static void kinetis_eventtimeout(int argc, uint32_t arg)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)arg;

  DEBUGASSERT(argc == 1 && priv != NULL);
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. Sample registers at the time of the timeout */

      kinetis_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Wake up any waiting threads */

      kinetis_endwait(priv, SDIOWAIT_TIMEOUT);
      flldbg("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: kinetis_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void kinetis_endwait(struct kinetis_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* Disable event-related interrupts */

  kinetis_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  kinetis_givesem(priv);
}

/****************************************************************************
 * Name: kinetis_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the SDIO device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void kinetis_endtransfer(struct kinetis_dev_s *priv, sdio_eventset_t wkupevent)
{
#ifdef CONFIG_SDIO_DMA
  uint32_t regval;
#endif

  /* Disable all transfer related interrupts */

  kinetis_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */
 
  putreg32(SDHC_XFRDONE_INTS, KINETIS_SDHC_IRQSTAT);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_SDIO_DMA
  /* Stop the DMA by resetting the data path*/

  regval = getreg32(KINETIS_SDHC_SYSCTL);
  regval |= SDHC_SYSCTL_RSTD;
  putreg32(regval, KINETIS_SDHC_SYSCTL);
#endif

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Debug instrumentation */

  kinetis_sample(priv, SAMPLENDX_END_TRANSFER);

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      kinetis_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Interrrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int kinetis_interrupt(int irq, void *context)
{
  struct kinetis_dev_s *priv = &g_sdhcdev;
  uint32_t enabled;
  uint32_t pending;
  uint32_t regval;

  /* Check the SDHC IRQSTAT register.  Mask out all bits that don't
   * correspond to enabled interrupts.  (This depends on the fact that bits
   * are ordered the same in both the IRQSTAT and IRQSIGEN registers).  If
   * there are non-zero bits remaining, then we have work to do here.
   */

  regval  = getreg32(KINETIS_SDHC_IRQSIGEN);
  enabled = getreg32(KINETIS_SDHC_IRQSTAT) & regval;
  fllvdbg("IRQSTAT: %08x IRQSIGEN %08x enabled: %08x\n", 
          getreg32(KINETIS_SDHC_IRQSTAT), regval, enabled);
  
  /* Disable card interrupts to clear the card interrupt to the host system. */

  regval &= ~SDHC_INT_CINT;
  putreg32(regval, KINETIS_SDHC_IRQSIGEN);

  /* Clear all pending interrupts */

  putreg32(enabled, KINETIS_SDHC_IRQSTAT);

  /* Handle in progress, interrupt driven data transfers ********************/

  pending  = enabled & priv->xfrints;
  if (pending != 0)
    {
#ifndef CONFIG_SDIO_DMA
      /* Is the RX buffer read ready?  Is so then we must be processing a
       * non-DMA receive transaction.
       */

      if ((pending & SDHC_INT_BRR) != 0)
        {
          /* Receive data from the RX buffer */

          kinetis_receive(priv);
        }

        /* Otherwise, Is the TX buffer write ready? If so we must
         * be processing a non-DMA send transaction.  NOTE:  We can't be
         * processing both!
         */

      else if ((pending & SDHC_INT_BWR) != 0)
        {
          /* Send data via the TX FIFO */

          kinetis_transmit(priv);
        }
#endif

      /* Handle transfer complete events */

      if ((pending & SDHC_INT_TC) != 0)
        {
          /* Terminate the transfer */

          kinetis_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
        }

      /* Handle data block send/receive CRC failure */

      else if ((pending & SDHC_INT_DCE) != 0)
        {
          /* Terminate the transfer with an error */

          flldbg("ERROR: Data block CRC failure, remaining: %d\n", priv->remaining);
          kinetis_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
        }

      /* Handle data timeout error */

      else if ((pending & SDHC_INT_DTOE) != 0)
        {
          /* Terminate the transfer with an error */

          flldbg("ERROR: Data timeout, remaining: %d\n", priv->remaining);
          kinetis_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_TIMEOUT);
        }
    }

  /* Handle wait events *****************************************************/

  pending  = enabled & priv->waitints;
  if (pending != 0)
    {
      /* Is this a response completion event? */

      if ((pending & SDHC_RESPDONE_INTS) != 0)
        {
          /* Yes.. Is their a thread waiting for response done? */

          if ((priv->waitevents & (SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE)) != 0)
            {
              /* Yes.. mask further interrupts and wake the thread up */

              regval = getreg32(KINETIS_SDHC_IRQSIGEN);
              regval &= ~SDHC_RESPDONE_INTS;
              putreg32(regval, KINETIS_SDHC_IRQSIGEN);

              kinetis_endwait(priv, SDIOWAIT_RESPONSEDONE);
            }
        }
    }

  /* Re-enable card interrupts */

  regval  = getreg32(KINETIS_SDHC_IRQSIGEN);
  regval |= SDHC_INT_CINT;
  putreg32(regval, KINETIS_SDHC_IRQSIGEN);

  return OK;
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int kinetis_lock(FAR struct sdio_dev_s *dev, bool lock)
{  
  /* Single SDIO instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  kinetis_muxbus_sdio_lock(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: kinetis_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void kinetis_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct kinetis_dev_s *priv = (FAR struct kinetis_dev_s *)dev;
  uint32_t regval;

  /* Disable all interrupts so that nothing interferes with the following. */

  putreg32(0, KINETIS_SDHC_IRQSIGEN);

  /* Reset the SDHC block, putting registers in their default, reset state.
   * Initiate the reset by setting the RSTA bit in the SYSCTL register.
   */

  regval  = getreg32(KINETIS_SDHC_SYSCTL);
  regval |= SDHC_SYSCTL_RSTA;
  putreg32(regval, KINETIS_SDHC_SYSCTL);

  /* The SDHC will reset the RSTA bit to 0 when the capabilities
   * registers are valid and the host driver can read them.
   */

  while ((getreg32(KINETIS_SDHC_SYSCTL) & SDHC_SYSCTL_RSTA) != 0);

  /* Make sure that all clocking is disabled */

  kinetis_clock(dev, CLOCK_SDIO_DISABLED);

  /* Enable all status bits (these could not all be potential sources of
   * interrupts.
   */

  putreg32(SDHC_INT_ALL, KINETIS_SDHC_IRQSTATEN);

  fvdbg("SYSCTL: %08x PRSSTAT: %08x IRQSTATEN: %08x\n",
        getreg32(KINETIS_SDHC_SYSCTL), getreg32(KINETIS_SDHC_PRSSTAT),
        getreg32(KINETIS_SDHC_IRQSTATEN));

  /* The next phase of the hardware reset would be to set the SYSCTRL INITA
   * bit to send 80 clock ticks for card to power up and then reset the card
   * with CMD0.  This is done elsewhere.
   */

  /* Reset state data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitints   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_SDIO_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SDIO and DMA completion events */
#endif

  wd_cancel(priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrints    = 0;      /* Interrupt enables for data transfer */
}

/****************************************************************************
 * Name: kinetis_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see kinetis_status_* defines)
 *
 ****************************************************************************/

static uint8_t kinetis_status(FAR struct sdio_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: kinetis_widebus
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

static void kinetis_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
  uint32_t regval;

  /* Set the Data Transfer Width (DTW) field in the PROCTL register */

  regval = getreg32(KINETIS_SDHC_PROCTL);
  regval &= ~SDHC_PROCTL_DTW_MASK;
  if (wide)
    {
      regval |= SDHC_PROCTL_DTW_4BIT;
    }
  else
    {
      regval |= SDHC_PROCTL_DTW_1BIT;
    }
  putreg32(regval, KINETIS_SDHC_PROCTL);
}

/****************************************************************************
 * Name: kinetis_frequency
 *
 * Description:
 *   Set the SD clock frequency
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface
 *   frequency - The frequency to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_KINETIS_SDHC_ABSFREQ
static void kinetis_frequency(FAR struct sdio_dev_s *dev, uint32_t frequency)
{
  uint32_t sdclkfs;
  uint32_t prescaled;
  uint32_t regval;
  unsigned int prescaler;
  unsigned int divisor;

  /* The SDCLK frequency is determined by (1) the frequency of the base clock
   * that was selected as the input clock, and (2) by a prescaler and a
   * divisor that are selected here:
   *
   * SDCLK  frequency = (base clock) / (prescaler * divisor)
   *
   * The prescaler is avalable only for the values:  2, 4, 8, 16, 32, 64, 128,
   * and 256.  Pick the smallest value of SDCLKFS that would result in an
   * in-range frequency.
   *
   * For example, if the base clock frequency is 96 MHz, and the target
   * frequency is 25 MHz, the following logic will select prescaler.
   * 
   *   96MHz / 2  <= 25MHz <= 96MHz / 2 /16       -- YES, prescaler == 2
   *
   * If the target frequency is 400 kHz, the following logic will select
   * prescaler:
   * 
   *   96MHz / 2  <= 400KHz <= 96MHz / 2 / 16     -- NO
   *   96MHz / 4  <= 400KHz <= 96MHz / 4 / 16     -- NO
   *   96MHz / 8  <= 400KHz <= 96MHz / 8 / 16     -- NO
   *   96MHz / 16 <= 400KHz <= 96MHz / 16 / 16    -- YES, prescaler == 16
   */

  if (/*frequency >= (BOARD_CORECLK_FREQ / 2) && */
        frequency <= (BOARD_CORECLK_FREQ / 2 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV2;
      prescaler = 2;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 4) &&
           frequency <= (BOARD_CORECLK_FREQ / 4 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV4;
      prescaler = 4;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 8) &&
           frequency <= (BOARD_CORECLK_FREQ / 8 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV8;
      prescaler = 8;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 16) &&
           frequency <= (BOARD_CORECLK_FREQ / 16 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV16;
      prescaler = 16;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 32) &&
           frequency <= (BOARD_CORECLK_FREQ / 32 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV32;
      prescaler = 32;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 64) &&
           frequency <= (BOARD_CORECLK_FREQ / 64 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV64;
      prescaler = 64;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 128) &&
           frequency <= (BOARD_CORECLK_FREQ / 128 / 16))
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV128;
      prescaler = 128;
    }
  else /* if (frequency >= (BOARD_CORECLK_FREQ / 256) &&
              frequency <= (BOARD_CORECLK_FREQ / 256 / 16)) */
    {
      sdclkfs   = SDHC_SYSCTL_SDCLKFS_DIV256;
      prescaler = 256;
    }

  /* The optimal divider can than be calculated.
   *
   * For example, if the base clock frequency is 96 MHz, the target
   * frequency is 25 MHz, and the selected prescaler value is 2, then
   *
   *   prescaled = 96MHz / 2 = 48MHz
   *   divisor   = (48MHz + 12.5HMz/ 25MHz = 2
   *
   * And the resulting frequency will be 24MHz.
   *
   * Or, for example, if the target frequency is 400 kHz and the selected
   * prescaler is 16, the following* logic will select prescaler:
   * 
   *   prescaled = 96MHz / 16 = 6MHz
   *   divisor   = (6MHz + 200KHz) / 400KHz = 15
   *
   * And the restuling frequency will be exactly 400KHz.
   */

  prescaled = frequency / prescaler;
  divisor   = (prescaled + (frequency >> 1)) / frequency;

  /* Set the new divisor information and enable all clocks in the SYSCTRL
   * register.
   *
   * TODO:  Investigate using the automatically gated clocks to reduce power
   *        consumption.
   */

  regval  = getreg32(KINETIS_SDHC_SYSCTL);
  regval &= ~(SDHC_SYSCTL_SDCLKFS_MASK|SDHC_SYSCTL_DVS_MASK);
  regval |= (sdclkfs | SDHC_SYSCTL_DVS_DIV(divisor));
  regval |= (SDHC_SYSCTL_SDCLKEN|SDHC_SYSCTL_PEREN|SDHC_SYSCTL_HCKEN|
             SDHC_SYSCTL_IPGEN);
  putreg32(regval, KINETIS_SDHC_SYSCTL);
  fvdbg("SYSCTRL: %08x\n", getreg32(KINETIS_SDHC_SYSCTL));
}
#endif

/****************************************************************************
 * Name: kinetis_clock
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

#if CONFIG_KINETIS_SDHC_ABSFREQ
static void kinetis_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t frequency;
  uint32_t regval;

  /* The SDCLK must be disabled before its frequency can be changed: "SDCLK
   * frequency can be changed when this bit is 0. Then, the host controller
   * shall maintain the same clock frequency until SDCLK is stopped (stop at
   * SDCLK = 0).
   */

  regval  = getreg32(KINETIS_SDHC_SYSCTL);
  regval &= ~SDHC_SYSCTL_SDCLKEN;
  putreg32(regval, KINETIS_SDHC_SYSCTL);
  fvdbg("SYSCTRL: %08x\n", getreg32(KINETIS_SDHC_SYSCTL));

  switch (rate)
    {
      default:
      case CLOCK_SDIO_DISABLED :     /* Clock is disabled */
        {
          /* Clear the prescaler and divisor settings and other clock
           * enables as well.
           */

          regval &= ~(SDHC_SYSCTL_IPGEN|SDHC_SYSCTL_HCKEN|SDHC_SYSCTL_PEREN|
                      SDHC_SYSCTL_SDCLKFS_MASK|SDHC_SYSCTL_DVS_MASK);
          putreg32(regval, KINETIS_SDHC_SYSCTL);
          fvdbg("SYSCTRL: %08x\n", getreg32(KINETIS_SDHC_SYSCTL));
          return;
        }

      case CLOCK_IDMODE :            /* Initial ID mode clocking (<400KHz) */
        frequency = CONFIG_KINETIS_IDMODE_FREQ;
        break;

      case CLOCK_MMC_TRANSFER :      /* MMC normal operation clocking */
        frequency = CONFIG_KINETIS_MMCXFR_FREQ;
        break;

      case CLOCK_SD_TRANSFER_1BIT :  /* SD normal operation clocking (narrow 1-bit mode) */
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
        frequency = CONFIG_KINETIS_SD1BIT_FREQ;
        break;
#endif

      case CLOCK_SD_TRANSFER_4BIT :  /* SD normal operation clocking (wide 4-bit mode) */
        frequency = CONFIG_KINETIS_SD4BIT_FREQ;
        break;
    }

  /* Then set the selected frequency */

  kinetis_frequency(dev, frequency);
}
#else
static void kinetis_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t regval;

  /* The SDCLK must be disabled before its frequency can be changed: "SDCLK
   * frequency can be changed when this bit is 0. Then, the host controller
   * shall maintain the same clock frequency until SDCLK is stopped (stop at
   * SDCLK = 0).
   */

  regval  = getreg32(KINETIS_SDHC_SYSCTL);
  regval &= ~SDHC_SYSCTL_SDCLKEN;
  putreg32(regval, KINETIS_SDHC_SYSCTL);
  fvdbg("SYSCTRL: %08x\n", getreg32(KINETIS_SDHC_SYSCTL));

  /* Clear the old prescaler and divisor values so that new ones can be ORed
   * in.
   */

  regval &= ~(SDHC_SYSCTL_SDCLKFS_MASK|SDHC_SYSCTL_DVS_MASK);

  /* Select the new prescaler and divisor values based on the requested mode
   * and the settings from the board.h file.
   *
   * TODO:  Investigate using the automatically gated clocks to reduce power
   *        consumption.
   */

  switch (rate)
    {
      default:
      case CLOCK_SDIO_DISABLED :     /* Clock is disabled */
        {
          /* Clear the prescaler and divisor settings and other clock
           * enables as well.
           */

          regval &= ~(SDHC_SYSCTL_IPGEN|SDHC_SYSCTL_HCKEN|SDHC_SYSCTL_PEREN);
          putreg32(regval, KINETIS_SDHC_SYSCTL);
          fvdbg("SYSCTRL: %08x\n", getreg32(KINETIS_SDHC_SYSCTL));
          return;
        }

      case CLOCK_IDMODE :            /* Initial ID mode clocking (<400KHz) */
        regval |= (BOARD_SDHC_IDMODE_PRESCALER|BOARD_SDHC_IDMODE_DIVISOR|
                   SDHC_SYSCTL_SDCLKEN|SDHC_SYSCTL_PEREN|SDHC_SYSCTL_HCKEN|
                   SDHC_SYSCTL_IPGEN);
        break;

      case CLOCK_MMC_TRANSFER :      /* MMC normal operation clocking */
        regval |= (BOARD_SDHC_MMCMODE_PRESCALER|BOARD_SDHC_MMCMODE_DIVISOR|
                   SDHC_SYSCTL_SDCLKEN|SDHC_SYSCTL_PEREN|SDHC_SYSCTL_HCKEN|
                   SDHC_SYSCTL_IPGEN);
        break;

      case CLOCK_SD_TRANSFER_1BIT :  /* SD normal operation clocking (narrow
                                      * 1-bit mode) */
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
        regval |= (BOARD_SDHC_SD1MODE_PRESCALER|BOARD_SDHC_IDMODE_DIVISOR|
                   SDHC_SYSCTL_SDCLKEN|SDHC_SYSCTL_PEREN|SDHC_SYSCTL_HCKEN|
                   SDHC_SYSCTL_IPGEN);
        break;
#endif

      case CLOCK_SD_TRANSFER_4BIT :  /* SD normal operation clocking (wide
                                      * 4-bit mode) */
        regval |= (BOARD_SDHC_SD4MODE_PRESCALER|BOARD_SDHC_SD4MODE_DIVISOR|
                   SDHC_SYSCTL_SDCLKEN|SDHC_SYSCTL_PEREN|SDHC_SYSCTL_HCKEN|
                   SDHC_SYSCTL_IPGEN);
        break;
    }

  putreg32(regval, KINETIS_SDHC_SYSCTL);
  fvdbg("SYSCTRL: %08x\n", getreg32(KINETIS_SDHC_SYSCTL));
}
#endif

/****************************************************************************
 * Name: kinetis_attach
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

static int kinetis_attach(FAR struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(KINETIS_IRQ_SDHC, kinetis_interrupt);
  if (ret == OK)
    {

      /* Disable all interrupts at the SDIO controller and clear all pending
       * interrupts.
       */

      putreg32(0,            KINETIS_SDHC_IRQSIGEN);
      putreg32(SDHC_INT_ALL, KINETIS_SDHC_IRQSTAT);

      /* Set the interrrupt priority */

      up_prioritize_irq(KINETIS_IRQ_SDHC, CONFIG_KINETIS_SDHC_PRIO);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(KINETIS_IRQ_SDHC);
    }

  return ret;
}

/****************************************************************************
 * Name: kinetis_sendcmd
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

static int kinetis_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg)
{
  uint32_t regval;
  uint32_t cmdidx;
  int32_t  timeout;

  /* Initialize the command index */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval = cmdidx << SDHC_XFERTYP_CMDINX_SHIFT;

  /* Does a data transfer accompany the command? */

  if ((cmd & MMCSD_DATAXFR) != 0)
    {
      /* Yes.. Configure the data transfer */

      switch (cmd & MMCSD_DATAXFR_MASK)
        {
          default:
          case MMCSD_NODATAXFR : /* No.. no data transfer */
            break;

          /* The following two cases are probably missing some setup logic */

          case MMCSD_RDSTREAM :  /* Yes.. streaming read data transfer */
            regval |= (SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DTDSEL);
            break;

          case MMCSD_WRSTREAM :  /* Yes.. streaming write data transfer */
            regval |= SDHC_XFERTYP_DPSEL;
            break;

          case MMCSD_RDDATAXFR : /* Yes.. normal read data transfer */
            regval |= (SDHC_XFERTYP_DPSEL | SDHC_XFERTYP_DTDSEL);
            break;

          case MMCSD_WRDATAXFR : /* Yes.. normal write data transfer */
            regval |= SDHC_XFERTYP_DPSEL;
            break;
        }

      /* Is it a multi-block transfer? */

      if ((cmd & MMCSD_MULTIBLOCK) != 0)
        {
          /* Yes.. should the transfer be stopped with ACMD12? */

          if ((cmd & MMCSD_STOPXFR) != 0)
            {
              /* Yes.. Indefinite block transfer */

              regval |= (SDHC_XFERTYP_MSBSEL | SDHC_XFERTYP_AC12EN);
            }
          else
            {
              /* No.. Fixed block transfer */

              regval |= (SDHC_XFERTYP_MSBSEL | SDHC_XFERTYP_BCEN);
            }
        }
    }
    
  /* Configure response type bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:                /* No response */
      regval |= SDHC_XFERTYP_RSPTYP_NONE;
      break;

    case MMCSD_R1B_RESPONSE:              /* Response length 48, check busy & cmdindex*/
      regval |= (SDHC_XFERTYP_RSPTYP_LEN48BSY|SDHC_XFERTYP_CICEN|SDHC_XFERTYP_CCCEN);
      break;
    
    case MMCSD_R1_RESPONSE:              /* Response length 48, check cmdindex */
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      regval |= (SDHC_XFERTYP_RSPTYP_LEN48|SDHC_XFERTYP_CICEN|SDHC_XFERTYP_CCCEN);
      break;

    case MMCSD_R2_RESPONSE:              /* Response length 136, check CRC */
      regval |= (SDHC_XFERTYP_RSPTYP_LEN136|SDHC_XFERTYP_CCCEN);
      break;

    case MMCSD_R3_RESPONSE:              /* Response length 48 */
    case MMCSD_R4_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDHC_XFERTYP_RSPTYP_LEN48;
      break;
    }

  /* Enable DMA */

#ifdef CONFIG_SDIO_DMA
  /* Internal DMA is used */

  regval |= SDHC_XFERTYP_DMAEN;
#endif

  /* Other bits? What about CMDTYP? */

  fvdbg("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);

  /* The Command Inhibit (CIHB) bit is set in the PRSSTAT bit immediately
   * after the transfer type register is written.  This bit is cleared when
   * the command response is received.  If this status bit is 0, it
   * indicates that the CMD line is not in use and the SDHC can issue a
   * SD/MMC Command using the CMD line.
   *
   * CIHB should always be set when this function is called.
   */

  timeout = SDHC_CMDTIMEOUT;
  while ((getreg32(KINETIS_SDHC_PRSSTAT) & SDHC_PRSSTAT_CIHB) != 0)
    {
      if (--timeout <= 0)
        {
          fdbg("ERROR: Timeout cmd: %08x PRSSTAT: %08x\n",
               cmd, getreg32(KINETIS_SDHC_PRSSTAT));

          return -EBUSY;
        }
    }

  /* Set the SDHC Argument value */

  putreg32(arg, KINETIS_SDHC_CMDARG);

  /* Clear interrupt status and write the SDHC CMD */

  putreg32(SDHC_RESPDONE_INTS, KINETIS_SDHC_IRQSTAT);
  putreg32(regval, KINETIS_SDHC_XFERTYP);
  return OK;
}

/****************************************************************************
 * Name: kinetis_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally, SDIO_WAITEVENT
 *   will be called to receive the indication that the transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SDIO_DMA
static int kinetis_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                             size_t nbytes)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  kinetis_datadisable();
  kinetis_sampleinit();
  kinetis_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  kinetis_dataconfig(priv, false, nbytes, 1, SDHC_DVS_DATATIMEOUT);

  /* And enable interrupts */

  kinetis_configxfrints(priv, SDHC_RCVDONE_INTS);
  kinetis_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: kinetis_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SDIO_DMA
static int kinetis_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer,
                           size_t nbytes)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  kinetis_datadisable();
  kinetis_sampleinit();
  kinetis_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  kinetis_dataconfig(priv, true, nbytes, 1, SDHC_DVS_DATATIMEOUT);

  /* Enable TX interrrupts */

  kinetis_configxfrints(priv, SDHC_SNDDONE_INTS);
  kinetis_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: kinetis_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
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

static int kinetis_cancel(FAR struct sdio_dev_s *dev)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s*)dev;
#ifdef CONFIG_SDIO_DMA
  uint32_t regval;
#endif

  /* Disable all transfer- and event- related interrupts */

  kinetis_configxfrints(priv, 0);
  kinetis_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */
 
  putreg32(SDHC_WAITALL_INTS, KINETIS_SDHC_IRQSTAT);

  /* Cancel any watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_SDIO_DMA
  /* Stop the DMA by resetting the data path*/

  regval = getreg32(KINETIS_SDHC_SYSCTL);
  regval |= SDHC_SYSCTL_RSTD;
  putreg32(regval, KINETIS_SDHC_SYSCTL);
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: kinetis_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.  This
 *   function should be called even after sending commands that have no
 *   response (such as CMD0) to make sure that the hardware is ready to
 *   receive the next command.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int kinetis_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  uint32_t errors;
  int32_t  timeout;
  int      ret = OK;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      timeout = SDHC_CMDTIMEOUT;
      errors  = 0;
      return OK;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = SDHC_LONGTIMEOUT;
      errors  = SDHC_RESPERR_INTS;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = SDHC_CMDTIMEOUT;
      errors  = SDHC_RESPERR_INTS;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the Command Complete (CC) indication (or timeout).  The
   * CC bit is set when the end bit of the command response is received
   * (except Auto CMD12). 
   */

  while ((getreg32(KINETIS_SDHC_IRQSTAT) & SDHC_INT_CC) == 0)
    {
      if (--timeout <= 0)
        {
          fdbg("ERROR: Timeout cmd: %08x IRQSTAT: %08x\n",
               cmd, getreg32(KINETIS_SDHC_IRQSTAT));

          return -ETIMEDOUT;
        }
    }

  /* Check for hardware detected errors */

  if ((getreg32(KINETIS_SDHC_IRQSTAT) & errors) != 0)
    {
      fdbg("ERROR: cmd: %08x errors: %08x IRQSTAT: %08x\n",
           cmd, errors, getreg32(KINETIS_SDHC_IRQSTAT));
      ret = -EIO;
    }

  /* Clear the response wait status bits */

  putreg32(SDHC_RESPDONE_INTS, KINETIS_SDHC_IRQSTAT);
  return ret;
}

/****************************************************************************
 * Name: kinetis_recvRx
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   Rx - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested reponse (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int kinetis_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  /* R1  Command response (48-bit)
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
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(KINETIS_SDHC_IRQSTAT);
      if ((regval & SDHC_INT_CTOE) != 0)
        {
          fdbg("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDHC_INT_CCE) != 0)
        {
          fdbg("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the R1/R1b/R6 response.  These responses are returned in
   * CDMRSP0.  NOTE: This is not true for R1b (Auto CMD12 response) which
   * is returned in CMDRSP3.
   */

  *rshort = getreg32(KINETIS_SDHC_CMDRSP0);
  return ret;
}

static int kinetis_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t rlong[4])
{
  uint32_t regval;
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
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(KINETIS_SDHC_IRQSTAT);
      if (regval & SDHC_INT_CTOE)
        {
          fdbg("ERROR: Timeout IRQSTAT: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDHC_INT_CCE)
        {
          fdbg("ERROR: CRC fail IRQSTAT: %08x\n", regval);
          ret = -EIO;
        }
    }
    
  /* Return the long response in CMDRSP3..0*/

  if (rlong)
    {
      rlong[0] = getreg32(KINETIS_SDHC_CMDRSP3);
      rlong[1] = getreg32(KINETIS_SDHC_CMDRSP2);
      rlong[2] = getreg32(KINETIS_SDHC_CMDRSP1);
      rlong[3] = getreg32(KINETIS_SDHC_CMDRSP0);
    }
  return ret;
}

static int kinetis_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

 /* R3  OCR (48-bit)
  *     47        0               Start bit
  *     46        0               Transmission bit (0=from card)
  *     45:40     bit5   - bit0   Reserved
  *     39:8      bit31  - bit0   32-bit OCR register
  *     7:1       bit6   - bit0   Reserved
  *     0         1               End bit
  */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = getreg32(KINETIS_SDHC_IRQSTAT);
      if (regval & SDHC_INT_CTOE)
        {
          fdbg("ERROR: Timeout IRQSTAT: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  /* Return the short response in CMDRSP0 */

  if (rshort)
    {
      *rshort = getreg32(KINETIS_SDHC_CMDRSP0);
    }

  return ret;
}

/* MMC responses not supported */

static int kinetis_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rnotimpl)
{
  /* Just return an error */

  return -ENOSYS;
}

/****************************************************************************
 * Name: kinetis_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling kinetis_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
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

static void kinetis_waitenable(FAR struct sdio_dev_s *dev,
                             sdio_eventset_t eventset)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s*)dev;
  uint32_t waitints;
 
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  kinetis_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitints = 0;
  if ((eventset & (SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitints |= SDHC_RESPDONE_INTS;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitints |= SDHC_XFRDONE_INTS;
    }

  /* Enable event-related interrupts */

  kinetis_configwaitints(priv, waitints, eventset, 0);
}

/****************************************************************************
 * Name: kinetis_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when kinetis_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before kinetis_eventwait
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

static sdio_eventset_t kinetis_eventwait(FAR struct sdio_dev_s *dev,
                                       uint32_t timeout)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s*)dev;
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
      ret   = wd_start(priv->waitwdog, delay, (wdentry_t)kinetis_eventtimeout,
                       1, (uint32_t)priv);
      if (ret != OK)
        {
          fdbg("ERROR: wd_start failed: %d\n", ret);
        }
    }

  /* Loop until the event (or the timeout occurs). Race conditions are avoided
   * by calling kinetis_waitenable prior to triggering the logic that will cause
   * the wait to terminate.  Under certain race conditions, the waited-for
   * may have already occurred before this function was called!
   */

  for (;;)
    {
      /* Wait for an event in event set to occur.  If this the event has already
       * occurred, then the semaphore will already have been incremented and
       * there will be no wait.
       */

      kinetis_takesem(priv);
      wkupevent = priv->wkupevent;
 
      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

  kinetis_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_SDIO_DMA
  priv->xfrflags   = 0;
#endif

  kinetis_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: kinetis_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in kinetis_registercallback.
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

static void kinetis_callbackenable(FAR struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s*)dev;

  fvdbg("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  kinetis_callback(priv);
}

/****************************************************************************
 * Name: kinetis_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE
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

static int kinetis_registercallback(FAR struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s*)dev;

  /* Disable callbacks and register this callback and is argument */

  fvdbg("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: kinetis_dmasupported
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
static bool kinetis_dmasupported(FAR struct sdio_dev_s *dev)
{
  return true;
}
#endif

/****************************************************************************
 * Name: kinetis_dmarecvsetup
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

#ifdef CONFIG_SDIO_DMA
static int kinetis_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                              size_t buflen)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  kinetis_datadisable();

  /* Begin sampling register values */

  kinetis_sampleinit();
  kinetis_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
  priv->remaining = buflen;

  /* Then set up the SDIO data path */

  kinetis_dataconfig(priv, false, buflen, 1, SDHC_DVS_DATATIMEOUT);

  /* Configure the RX DMA */

  kinetis_configxfrints(priv, SDHC_DMADONE_INTS);
  putreg32((uint32_t)buffer, KINETIS_SDHC_DSADDR);
 
  /* Sample the register state */

  kinetis_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: kinetis_dmasendsetup
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

#ifdef CONFIG_SDIO_DMA
static int kinetis_dmasendsetup(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  kinetis_datadisable();

  /* Begin sampling register values */

  kinetis_sampleinit();
  kinetis_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
  priv->remaining = buflen;

  /* Then set up the SDIO data path */

  kinetis_dataconfig(priv, true, buflen, 1, SDHC_DVS_DATATIMEOUT);

  /* Configure the TX DMA */

  putreg32((uint32_t)buffer, KINETIS_SDHC_DSADDR);

  /* Sample the register state */

  kinetis_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrrupts */

  kinetis_configxfrints(priv, SDHC_DMADONE_INTS);
  return OK;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: kinetis_callback
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

static void kinetis_callback(void *arg)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s*)arg;

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
 * Name: sdhc_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdhc_initialize(int slotno)
{
  uint32_t regval;

  /* There is only one slot */

  struct kinetis_dev_s *priv = &g_sdhcdev;
  DEBUGASSERT(slotno == 0);

  /* Initialize the SDHC slot structure data structure */

  sem_init(&priv->waitsem, 0, 0);
  priv->waitwdog = wd_create();
  DEBUGASSERT(priv->waitwdog);

  /* Enable clocking to the SDHC module.  Clocking is still diabled in
   * the SYSCTRL register.
   */

  regval = getreg32(KINETIS_SIM_SCGC3);
  regval |= SIM_SCGC3_SDHC;
  putreg32(regval, KINETIS_SIM_SCGC3);
  fvdbg("SIM_SCGC3: %08x\n", regval);

  /* In addition to the system clock, the SDHC module needs a clock for the
   * base for the external card clock.  There are four possible sources for
   * this clock, selected by the SIM's SOPT2 register:
   *
   * - Core/system clock
   * - MCGPLLCLK/MCGFLLCLK clock
   * - OSCERCLK EXTAL clock
   * - External bypass clock from off-chip (SCHC0_CLKINB)
   */

  regval = getreg32(KINETIS_SIM_SOPT2);
  regval &= ~SIM_SOPT2_SDHCSRC_MASK;
  regval |= SIM_SOPT2_SDHCSRC_CORE;
  putreg32(regval, KINETIS_SIM_SOPT2);
  fvdbg("SIM_SOPT2: %08x\n", regval);

  /* Configure pins for 1 or 4-bit, wide-bus operation (the chip is capable
   * of 8-bit wide bus operation but D4-D7 are not configured).
   * 
   * If bus is multiplexed then there is a custom bus configuration utility
   * in the scope of the board support package.
   */

#ifndef CONFIG_SDIO_MUXBUS
  /* Data width 1, 4 or 8 */

  kinetis_pinconfig(PIN_SDHC0_D0);

  /* Data width 4 or 8 */

#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
  kinetis_pinconfig(PIN_SDHC0_D1);
  kinetis_pinconfig(PIN_SDHC0_D2);
  kinetis_pinconfig(PIN_SDHC0_D3);

  /* Data width 8 (not supported) */

#if 0
  kinetis_pinconfig(PIN_SDHC0_D4);
  kinetis_pinconfig(PIN_SDHC0_D5);
  kinetis_pinconfig(PIN_SDHC0_D6);
  kinetis_pinconfig(PIN_SDHC0_D7);
#endif
#endif

  /* Clocking and CMD pins (all data widths) */

  kinetis_pinconfig(PIN_SDHC0_DCLK);
  kinetis_pinconfig(PIN_SDHC0_CMD);
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  kinetis_reset(&priv->dev);
  kinetis_showregs(priv, "After reset");
  return &g_sdhcdev.dev;
}

/****************************************************************************
 * Name: sdhc_mediachange
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

void sdhc_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;
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
      kinetis_callback(priv);
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

void sdhc_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect)
{
  struct kinetis_dev_s *priv = (struct kinetis_dev_s *)dev;
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
#endif /* CONFIG_KINETIS_SDHC */
