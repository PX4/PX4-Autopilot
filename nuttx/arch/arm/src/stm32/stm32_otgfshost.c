/*******************************************************************************
 * arch/arm/src/stm32/stm32_otgfshost.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#include <arch/irq.h>

#include "chip.h"             /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "stm32_usbhost.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

/* All I/O buffers must lie in AHB SRAM because of the OHCI DMA. It might be
 * okay if no I/O buffers are used *IF* the application can guarantee that all
 * end-user I/O buffers reside in AHB SRAM.
 */

#if STM32_IOBUFFERS < 1
#  warning "No IO buffers allocated"
#endif

/* OHCI Setup ******************************************************************/
/* Frame Interval / Periodic Start */

#define BITS_PER_FRAME          12000
#define FI                     (BITS_PER_FRAME-1)
#define FSMPS                  ((6 * (FI - 210)) / 7)
#define DEFAULT_FMINTERVAL     ((FSMPS << OHCI_FMINT_FSMPS_SHIFT) | FI)
#define DEFAULT_PERSTART       (((9 * BITS_PER_FRAME) / 10) - 1)

/* CLKCTRL enable bits */

#define STM32_CLKCTRL_ENABLES   (USBOTG_CLK_HOSTCLK|USBOTG_CLK_PORTSELCLK|USBOTG_CLK_AHBCLK)

/* Interrupt enable bits */

#ifdef CONFIG_DEBUG_USB
#  define STM32_DEBUG_INTS      (OHCI_INT_SO|OHCI_INT_RD|OHCI_INT_UE|OHCI_INT_OC)
#else
#  define STM32_DEBUG_INTS      0
#endif

#define STM32_NORMAL_INTS       (OHCI_INT_WDH|OHCI_INT_RHSC)
#define STM32_ALL_INTS          (STM32_NORMAL_INTS|STM32_DEBUG_INTS)

/* Dump GPIO registers */

#if defined(CONFIG_STM32_USBHOST_REGDEBUG) && defined(CONFIG_DEBUG_GPIO)
#  define usbhost_dumpgpio() \
   do { \
     stm32_dumpgpio(GPIO_USB_DP, "D+ P0.29; D- P0.30"); \
     stm32_dumpgpio(GPIO_USB_UPLED, "LED P1:18; PPWR P1:19 PWRD P1:22 PVRCR P1:27"); \
   } while (0);
#else
#  define usbhost_dumpgpio()
#endif

/* USB Host Memory *************************************************************/

/* Helper definitions */

#define HCCA        ((struct ohci_hcca_s *)STM32_HCCA_BASE)
#define TDTAIL      ((struct stm32_gtd_s *)STM32_TDTAIL_ADDR)
#define EDCTRL      ((struct stm32_ed_s *)STM32_EDCTRL_ADDR)

/* Periodic intervals 2, 4, 8, 16,and 32 supported */

#define MIN_PERINTERVAL 2
#define MAX_PERINTERVAL 32

/* Descriptors *****************************************************************/

/* TD delay interrupt value */

#define TD_DELAY(n) (uint32_t)((n) << GTD_STATUS_DI_SHIFT)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* This structure retains the state of the USB host controller */

struct stm32_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to structstm32_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* The bound device class driver */

  struct usbhost_class_s *class;

  /* Driver status */

  volatile bool    connected;   /* Connected to device */
  volatile bool    lowspeed;    /* Low speed device attached. */
  volatile bool    rhswait;     /* TRUE: Thread is waiting for Root Hub Status change */
#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t          ininterval;  /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */
  uint8_t          outinterval; /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */
#endif
  sem_t            exclsem;     /* Support mutually exclusive access */
  sem_t            rhssem;      /* Semaphore to wait Writeback Done Head event */
};

/* The OCHI expects the size of an endpoint descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes in
 * stm32_ohciram.h.  This extra 16-bytes is used by the OHCI host driver in
 * order to maintain additional endpoint-specific data.
 */

struct stm32_ed_s
{
  /* Hardware specific fields */

  struct ohci_ed_s hw;

  /* Software specific fields */

  uint8_t          xfrtype;   /* Transfer type.  See SB_EP_ATTR_XFER_* in usb.h */
  uint8_t          interval;  /* Periodic EP polling interval: 2, 4, 6, 16, or 32 */
  volatile uint8_t tdstatus;  /* TD control status bits from last Writeback Done Head event */
  volatile bool    wdhwait;   /* TRUE: Thread is waiting for WDH interrupt */
  sem_t            wdhsem;    /* Semaphore used to wait for Writeback Done Head event */
                              /* Unused bytes follow, depending on the size of sem_t */
};

/* The OCHI expects the size of an transfer descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes in
 * stm32_ohciram.h.  This extra 16-bytes is used by the OHCI host driver in
 * order to maintain additional endpoint-specific data.
 */

struct stm32_gtd_s
{
  /* Hardware specific fields */

  struct ohci_gtd_s hw;

  /* Software specific fields */

  struct stm32_ed_s *ed;      /* Pointer to parent ED */
  uint8_t           pad[12];
};

/* The following is used to manage lists of free EDs, TDs, and TD buffers */

struct stm32_list_s
{
  struct stm32_list_s *flink; /* Link to next buffer in the list */
                              /* Variable length buffer data follows */
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void stm32_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t val, uint32_t addr);
#else
# define stm32_getreg(addr)     getreg32(addr)
# define stm32_putreg(val,addr) putreg32(val,addr)
#endif

/* Semaphores ******************************************************************/

static void stm32_takesem(sem_t *sem);
#define stm32_givesem(s) sem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t stm32_getle16(const uint8_t *val);
static void stm32_putle16(uint8_t *dest, uint16_t val);

/* OHCI memory pool helper functions *******************************************/

static inline void stm32_edfree(struct stm32_ed_s *ed);
static  struct stm32_gtd_s *stm32_tdalloc(void);
static void stm32_tdfree(struct stm32_gtd_s *buffer);
static uint8_t *stm32_tballoc(void);
static void stm32_tbfree(uint8_t *buffer);
#if STM32_IOBUFFERS > 0
static uint8_t *stm32_allocio(void);
static void stm32_freeio(uint8_t *buffer);
#endif

/* ED list helper functions ****************************************************/

static inline int stm32_addbulked(struct stm32_usbhost_s *priv,
                                  struct stm32_ed_s *ed);
static inline int stm32_rembulked(struct stm32_usbhost_s *priv,
                                  struct stm32_ed_s *ed);

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int stm32_getinterval(uint8_t interval);
static void stm32_setinttab(uint32_t value, unsigned int interval, unsigned int offset);
#endif

static inline int stm32_addinted(struct stm32_usbhost_s *priv,
                                 const FAR struct usbhost_epdesc_s *epdesc, 
                                 struct stm32_ed_s *ed);
static inline int stm32_reminted(struct stm32_usbhost_s *priv,
                                 struct stm32_ed_s *ed);

static inline int stm32_addisoced(struct stm32_usbhost_s *priv,
                                  const FAR struct usbhost_epdesc_s *epdesc, 
                                  struct stm32_ed_s *ed);
static inline int stm32_remisoced(struct stm32_usbhost_s *priv,
                                  struct stm32_ed_s *ed);

/* Descriptor helper functions *************************************************/

static int stm32_enqueuetd(struct stm32_usbhost_s *priv,
                           struct stm32_ed_s *ed, uint32_t dirpid,
                           uint32_t toggle, volatile uint8_t *buffer,
                           size_t buflen);
static int stm32_ctrltd(struct stm32_usbhost_s *priv, uint32_t dirpid,
                        uint8_t *buffer, size_t buflen);

/* Interrupt handling **********************************************************/

static int stm32_usbinterrupt(int irq, FAR void *context);

/* USB host controller operations **********************************************/

static int stm32_wait(FAR struct usbhost_driver_s *drvr, bool connected);
static int stm32_enumerate(FAR struct usbhost_driver_s *drvr);
static int stm32_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize);
static int stm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep);
static int stm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int stm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen);
static int stm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int stm32_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t buflen);
static int stm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int stm32_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer);
static int stm32_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer);
static int stm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                          FAR uint8_t *buffer, size_t buflen);
static void stm32_disconnect(FAR struct usbhost_driver_s *drvr);

/* Initialization **************************************************************/

static inline void stm32_ep0init(struct stm32_usbhost_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct stm32_usbhost_s g_usbhost =
{
  .drvr             =
    {
      .wait         = stm32_wait,
      .enumerate    = stm32_enumerate,
      .ep0configure = stm32_ep0configure,
      .epalloc      = stm32_epalloc,
      .epfree       = stm32_epfree,
      .alloc        = stm32_alloc,
      .free         = stm32_free,
      .ioalloc      = stm32_ioalloc,
      .iofree       = stm32_iofree,
      .ctrlin       = stm32_ctrlin,
      .ctrlout      = stm32_ctrlout,
      .transfer     = stm32_transfer,
      .disconnect   = stm32_disconnect,
    },
  .class            = NULL,
};

/* This is a free list of EDs and TD buffers */

static struct stm32_list_s *g_edfree; /* List of unused EDs */
static struct stm32_list_s *g_tdfree; /* List of unused TDs */
static struct stm32_list_s *g_tbfree; /* List of unused transfer buffers */
#if STM32_IOBUFFERS > 0
static struct stm32_list_s *g_iofree; /* List of unused I/O buffers */
#endif

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_printreg
 *
 * Description:
 *   Print the contents of an STM32xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: stm32_checkreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              stm32_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              lldbg("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new regisgter access */

      stm32_printreg(addr, val, iswrite);
    }
}
#endif

/*******************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  stm32_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  stm32_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 *******************************************************************************/

static void stm32_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: stm32_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static inline uint16_t stm32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: stm32_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static void stm32_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/*******************************************************************************
 * Name: stm32_edfree
 *
 * Description:
 *   Return an endpoint descriptor to the free list
 *
 *******************************************************************************/

static inline void stm32_edfree(struct stm32_ed_s *ed)
{
  struct stm32_list_s *entry = (struct stm32_list_s *)ed;

  /* Put the ED back into the free list */

  entry->flink = g_edfree;
  g_edfree     = entry;
}

/*******************************************************************************
 * Name: stm32_tdalloc
 *
 * Description:
 *   Allocate an transfer descriptor from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protected from conconcurrent access to the TD pool by the interrupt
 *     handler
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

static struct stm32_gtd_s *stm32_tdalloc(void)
{
  struct stm32_gtd_s *ret;
  irqstate_t flags;

  /* Disable interrupts momentarily so that stm32_tdfree is not called from the
   * interrupt handler.
   */

  flags = irqsave();
  ret   = (struct stm32_gtd_s *)g_tdfree;
  if (ret)
    {
      g_tdfree = ((struct stm32_list_s*)ret)->flink;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_tdfree
 *
 * Description:
 *   Return an transfer descriptor to the free list
 *
 * Assumptions:
 *   - Only called from the WDH interrupt handler (and during initialization).
 *   - Interrupts are disabled in any case.
 *
 *******************************************************************************/

static void stm32_tdfree(struct stm32_gtd_s *td)
{
  struct stm32_list_s *tdfree = (struct stm32_list_s *)td;

  /* This should not happen but just to be safe, don't free the common, pre-
   * allocated tail TD.
   */

 if (tdfree != NULL && td != TDTAIL)
    {
      tdfree->flink           = g_tdfree;
      g_tdfree                = tdfree;
    }
}

/*******************************************************************************
 * Name: stm32_tballoc
 *
 * Description:
 *   Allocate an request/descriptor transfer buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

static uint8_t *stm32_tballoc(void)
{
  uint8_t *ret = (uint8_t *)g_tbfree;
  if (ret)
    {
      g_tbfree = ((struct stm32_list_s*)ret)->flink;
    }
  return ret;
}

/*******************************************************************************
 * Name: stm32_tbfree
 *
 * Description:
 *   Return an request/descriptor transfer buffer to the free list
 *
 *******************************************************************************/

static void stm32_tbfree(uint8_t *buffer)
{
  struct stm32_list_s *tbfree = (struct stm32_list_s *)buffer;

  if (tbfree)
    {
      tbfree->flink              = g_tbfree;
      g_tbfree                   = tbfree;
    }
}

/*******************************************************************************
 * Name: stm32_allocio
 *
 * Description:
 *   Allocate an IO buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

#if STM32_IOBUFFERS > 0
static uint8_t *stm32_allocio(void)
{
  uint8_t *ret = (uint8_t *)g_iofree;
  if (ret)
    {
      g_iofree = ((struct stm32_list_s*)ret)->flink;
    }
  return ret;
}
#endif

/*******************************************************************************
 * Name: stm32_freeio
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 *******************************************************************************/

#if STM32_IOBUFFERS > 0
static void stm32_freeio(uint8_t *buffer)
{
  struct stm32_list_s *iofree = (struct stm32_list_s *)buffer;
  iofree->flink               = g_iofree;
  g_iofree                    = iofree;
}
#endif

/*******************************************************************************
 * Name: stm32_addbulked
 *
 * Description:
 *   Helper function to add an ED to the bulk list.
 *
 *******************************************************************************/
 
static inline int stm32_addbulked(struct stm32_usbhost_s *priv,
                                  struct stm32_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  uint32_t regval;

  /* Add the new bulk ED to the head of the bulk list */

  ed->hw.nexted = stm32_getreg(STM32_USBHOST_BULKHEADED);
  stm32_putreg((uint32_t)ed, STM32_USBHOST_BULKHEADED);

  /* BulkListEnable. This bit is set to enable the processing of the
   * Bulk list.  Note: once enabled, it remains.  We really should
   * never modify the bulk list while BLE is set.
   */

  regval  = stm32_getreg(STM32_USBHOST_CTRL);
  regval |= OHCI_CTRL_BLE;
  stm32_putreg(regval, STM32_USBHOST_CTRL);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: stm32_rembulked
 *
 * Description:
 *   Helper function remove an ED from the bulk list.
 *
 *******************************************************************************/
 
static inline int stm32_rembulked(struct stm32_usbhost_s *priv,
                                  struct stm32_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  struct stm32_ed_s *curr;
  struct stm32_ed_s *prev;
  uint32_t           regval;

  /* Find the ED in the bulk list.  NOTE: We really should never be mucking
   * with the bulk list while BLE is set.
   */

  for (curr = (struct stm32_ed_s *)stm32_getreg(STM32_USBHOST_BULKHEADED),
       prev = NULL;
       curr && curr != ed;
       prev = curr, curr = (struct stm32_ed_s *)curr->hw.nexted);

  /* Hmmm.. It would be a bug if we do not find the ED in the bulk list. */

  DEBUGASSERT(curr != NULL);

  /* Remove the ED from the bulk list */

  if (curr != NULL)
    {
      /* Is this ED the first on in the bulk list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the bulk list to skip over this ED */

          stm32_putreg(ed->hw.nexted, STM32_USBHOST_BULKHEADED);

          /* If the bulk list is now empty, then disable it */

          regval  = stm32_getreg(STM32_USBHOST_CTRL);
          regval &= ~OHCI_CTRL_BLE;
          stm32_putreg(regval, STM32_USBHOST_CTRL);
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
        }
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: stm32_getinterval
 *
 * Description:
 *   Convert the endpoint polling interval into a HCCA table increment
 *
 *******************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int stm32_getinterval(uint8_t interval)
{
  /* The bInterval field of the endpoint descriptor contains the polling interval
   * for interrupt and isochronous endpoints. For other types of endpoint, this
   * value should be ignored. bInterval is provided in units of 1MS frames.
   */

  if (interval < 3)
    {
      return 2;
    }
  else if (interval < 7)
    {
      return 4;
    }
  else if (interval < 15)
    {
      return 8;
    }
  else if (interval < 31)
    {
      return 16;
    }
  else
    {
      return 32;
    }
}
#endif

/*******************************************************************************
 * Name: stm32_setinttab
 *
 * Description:
 *   Set the interrupt table to the selected value using the provided interval
 *   and offset.
 *
 *******************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static void stm32_setinttab(uint32_t value, unsigned int interval, unsigned int offset)
{
  unsigned int i;
  for (i = offset; i < HCCA_INTTBL_WSIZE; i += interval)
    {
      HCCA->inttbl[i] = value;
    }
}
#endif

/*******************************************************************************
 * Name: stm32_addinted
 *
 * Description:
 *   Helper function to add an ED to the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in general,
 *    the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of all
 *        IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they request.
 *
 *******************************************************************************/
 
static inline int stm32_addinted(struct stm32_usbhost_s *priv,
                                 const FAR struct usbhost_epdesc_s *epdesc, 
                                 struct stm32_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  unsigned int interval;
  unsigned int offset;
  uint32_t head;
  uint32_t regval;

  /* Disable periodic list processing.  Does this take effect immediately?  Or
   * at the next SOF... need to check.
   */

  regval  = stm32_getreg(STM32_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_PLE;
  stm32_putreg(regval, STM32_USBHOST_CTRL);

  /* Get the quanitized interval value associated with this ED and save it
   * in the ED.
   */

  interval     = stm32_getinterval(epdesc->interval);
  ed->interval = interval;
  uvdbg("interval: %d->%d\n", epdesc->interval, interval);

  /* Get the offset associated with the ED direction. IN EDs get the even
   * entries, OUT EDs get the odd entries.
   *
   * Get the new, minimum interval. Add IN/OUT EDs are scheduled together
   * at the minimum interval of all IN/OUT EDs.
   */

  if (epdesc->in)
    {
      offset = 0;
      if (priv->ininterval > interval)
        {
          priv->ininterval = interval;
        }
      else
        {
          interval = priv->ininterval;
        }
    }
  else
    {
      offset = 1;
      if (priv->outinterval > interval)
        {
          priv->outinterval = interval;
        }
      else
        {
          interval = priv->outinterval;
        }
    }
  uvdbg("min interval: %d offset: %d\n", interval, offset);

  /* Get the head of the first of the duplicated entries.  The first offset
   * entry is always guaranteed to contain the common ED list head.
   */

  head = HCCA->inttbl[offset];

  /* Clear all current entries in the interrupt table for this direction */

  stm32_setinttab(0, 2, offset);

  /* Add the new ED before the old head of the periodic ED list and set the
   * new ED as the head ED in all of the appropriate entries of the HCCA
   * interrupt table.
   */

  ed->hw.nexted = head;
  stm32_setinttab((uint32_t)ed, interval, offset);
  uvdbg("head: %08x next: %08x\n", ed, head);

  /* Re-enabled periodic list processing */

  regval  = stm32_getreg(STM32_USBHOST_CTRL);
  regval |= OHCI_CTRL_PLE;
  stm32_putreg(regval, STM32_USBHOST_CTRL);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: stm32_reminted
 *
 * Description:
 *   Helper function to remove an ED from the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in general,
 *    the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of all
 *        IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they request.
 *
 *******************************************************************************/
 
static inline int stm32_reminted(struct stm32_usbhost_s *priv,
                                 struct stm32_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  struct stm32_ed_s *head;
  struct stm32_ed_s *curr;
  struct stm32_ed_s *prev;
  unsigned int       interval;
  unsigned int       offset;
  uint32_t           regval;

  /* Disable periodic list processing.  Does this take effect immediately?  Or
   * at the next SOF... need to check.
   */

  regval  = stm32_getreg(STM32_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_PLE;
  stm32_putreg(regval, STM32_USBHOST_CTRL);

  /* Get the offset associated with the ED direction. IN EDs get the even
   * entries, OUT EDs get the odd entries.
   */

  if ((ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
    {
      offset = 0;
    }
  else
    {
      offset = 1;
    }

  /* Get the head of the first of the duplicated entries.  The first offset
   * entry is always guaranteed to contain the common ED list head.
   */

  head = (struct stm32_ed_s *)HCCA->inttbl[offset];
  uvdbg("ed: %08x head: %08x next: %08x offset: %d\n",
        ed, head, head ? head->hw.nexted : 0, offset);

  /* Find the ED to be removed in the ED list */

  for (curr = head, prev = NULL;
       curr && curr != ed;
       prev = curr, curr = (struct stm32_ed_s *)curr->hw.nexted);

  /* Hmmm.. It would be a bug if we do not find the ED in the bulk list. */

  DEBUGASSERT(curr != NULL);
  if (curr != NULL)
    {
      /* Clear all current entries in the interrupt table for this direction */

      stm32_setinttab(0, 2, offset);

      /* Remove the ED from the list..  Is this ED the first on in the list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the bulk list to skip over this ED */

          head = (struct stm32_ed_s *)ed->hw.nexted;
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
        }
        uvdbg("ed: %08x head: %08x next: %08x\n",
              ed, head, head ? head->hw.nexted : 0);

      /* Calculate the new minimum interval for this list */

      interval = MAX_PERINTERVAL;
      for (curr = head; curr; curr = (struct stm32_ed_s *)curr->hw.nexted)
        {
          if (curr->interval < interval)
            {
              interval = curr->interval;
            }
        }
      uvdbg("min interval: %d offset: %d\n", interval, offset);

      /* Save the new minimum interval */
 
      if ((ed->hw.ctrl && ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
        {
          priv->ininterval  = interval;
        }
      else
        {
          priv->outinterval = interval;
        }

      /* Set the head ED in all of the appropriate entries of the HCCA interrupt
       * table (head might be NULL).
       */

      stm32_setinttab((uint32_t)head, interval, offset);
    }

  /* Re-enabled periodic list processing */

  if (head != NULL)
    {
      regval  = stm32_getreg(STM32_USBHOST_CTRL);
      regval |= OHCI_CTRL_PLE;
      stm32_putreg(regval, STM32_USBHOST_CTRL);
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: stm32_addisoced
 *
 * Description:
 *   Helper functions to add an ED to the periodic table.
 *
 *******************************************************************************/
 
static inline int stm32_addisoced(struct stm32_usbhost_s *priv,
                                  const FAR struct usbhost_epdesc_s *epdesc, 
                                  struct stm32_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
#  warning "Isochronous endpoints not yet supported"
#endif
  return -ENOSYS;

}

/*******************************************************************************
 * Name: stm32_remisoced
 *
 * Description:
 *   Helper functions to remove an ED from the periodic table.
 *
 *******************************************************************************/
 
static inline int stm32_remisoced(struct stm32_usbhost_s *priv,
                                  struct stm32_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
#  warning "Isochronous endpoints not yet supported"
#endif
  return -ENOSYS;
}

/*******************************************************************************
 * Name: stm32_enqueuetd
 *
 * Description:
 *   Enqueue a transfer descriptor.  Notice that this function only supports
 *   queue on TD per ED.
 *
 *******************************************************************************/

static int stm32_enqueuetd(struct stm32_usbhost_s *priv,
                           struct stm32_ed_s *ed, uint32_t dirpid,
                           uint32_t toggle, volatile uint8_t *buffer, size_t buflen)
{
  struct stm32_gtd_s *td;
  int ret = -ENOMEM;

  /* Allocate a TD from the free list */

  td = stm32_tdalloc();
  if (td != NULL)
    {
      /* Initialize the allocated TD and link it before the common tail TD. */

      td->hw.ctrl         = (GTD_STATUS_R | dirpid | TD_DELAY(0) | toggle | GTD_STATUS_CC_MASK);
      TDTAIL->hw.ctrl     = 0;
      td->hw.cbp          = (uint32_t)buffer;
      TDTAIL->hw.cbp      = 0;
      td->hw.nexttd       = (uint32_t)TDTAIL;
      TDTAIL->hw.nexttd   = 0;
      td->hw.be           = (uint32_t)(buffer + (buflen - 1));
      TDTAIL->hw.be       = 0;

      /* Configure driver-only fields in the extended TD structure */

      td->ed              = ed;

      /* Link the td to the head of the ED's TD list */

      ed->hw.headp        = (uint32_t)td | ((ed->hw.headp) & ED_HEADP_C);
      ed->hw.tailp        = (uint32_t)TDTAIL;

      ret                 = OK;
    }

  return ret;
}

/*******************************************************************************
 * Name: stm32_wdhwait
 *
 * Description:
 *   Set the request for the Writeback Done Head event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 *******************************************************************************/

static int stm32_wdhwait(struct stm32_usbhost_s *priv, struct stm32_ed_s *ed)
{
  irqstate_t flags = irqsave();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set wdhwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
       */

      ed->wdhwait = true;
      ret         = OK;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_ctrltd
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  This function
 *   will enqueue the request and wait for it to complete.  Only one transfer
 *   may be queued; Neither these methods nor the transfer() method can be
 *   called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 *******************************************************************************/

static int stm32_ctrltd(struct stm32_usbhost_s *priv, uint32_t dirpid,
                        uint8_t *buffer, size_t buflen)
{
  uint32_t toggle;
  uint32_t regval;
  int ret;

  /* Set the request for the Writeback Done Head event well BEFORE enabling the
   * transfer.
   */

  ret = stm32_wdhwait(priv, EDCTRL);
  if (ret != OK)
    {
      udbg("ERROR: Device disconnected\n");
      return ret;
    }

  /* Configure the toggle field in the TD */

  if (dirpid == GTD_STATUS_DP_SETUP)
    {
      toggle = GTD_STATUS_T_DATA0;
    }
  else
    {
      toggle = GTD_STATUS_T_DATA1;
    }

  /* Then enqueue the transfer */

  EDCTRL->tdstatus = TD_CC_NOERROR;
  ret = stm32_enqueuetd(priv, EDCTRL, dirpid, toggle, buffer, buflen);
  if (ret == OK)
    {
      /* Set ControlListFilled.  This bit is used to indicate whether there are
       * TDs on the Control list.
       */

      regval = stm32_getreg(STM32_USBHOST_CMDST);
      regval |= OHCI_CMDST_CLF;
      stm32_putreg(regval, STM32_USBHOST_CMDST);

      /* Wait for the Writeback Done Head interrupt */

      stm32_takesem(&EDCTRL->wdhsem);

      /* Check the TD completion status bits */

      if (EDCTRL->tdstatus == TD_CC_NOERROR)
        {
          ret = OK;
        }
      else 
        {
          uvdbg("Bad TD completion status: %d\n", EDCTRL->tdstatus);
          ret = -EIO;
        }
    }

  /* Make sure that there is no outstanding request on this endpoint */

  EDCTRL->wdhwait = false;
  return ret;
}

/*******************************************************************************
 * Name: stm32_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int stm32_usbinterrupt(int irq, FAR void *context)
{
  struct stm32_usbhost_s *priv = &g_usbhost;
  uint32_t intst;
  uint32_t pending;
  uint32_t regval;

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  intst  = stm32_getreg(STM32_USBHOST_INTST);
  regval = stm32_getreg(STM32_USBHOST_INTEN);
  ullvdbg("INST: %08x INTEN: %08x\n", intst, regval);

  pending = intst & regval;
  if (pending != 0)
    {
      /* Root hub status change interrupt */

      if ((pending & OHCI_INT_RHSC) != 0)
        {
          uint32_t rhportst1 = stm32_getreg(STM32_USBHOST_RHPORTST1);
          ullvdbg("Root Hub Status Change, RHPORTST1: %08x\n", rhportst1);

          if ((rhportst1 & OHCI_RHPORTST_CSC) != 0)
            {
              uint32_t rhstatus = stm32_getreg(STM32_USBHOST_RHSTATUS);
              ullvdbg("Connect Status Change, RHSTATUS: %08x\n", rhstatus);

              /* If DRWE is set, Connect Status Change indicates a remote wake-up event */

              if (rhstatus & OHCI_RHSTATUS_DRWE)
                {
                  ullvdbg("DRWE: Remote wake-up\n");
                }

              /* Otherwise... Not a remote wake-up event */

              else
                {
                  /* Check current connect status */

                  if ((rhportst1 & OHCI_RHPORTST_CCS) != 0)
                    {
                      /* Connected ... Did we just become connected? */

                      if (!priv->connected)
                        {
                          /* Yes.. connected. */

                          ullvdbg("Connected\n");
                          priv->connected = true;

                          /* Notify any waiters */

                          if (priv->rhswait)
                            {
                              stm32_givesem(&priv->rhssem);
                              priv->rhswait = false;
                            }
                        }
                      else
                        {
                          ulldbg("Spurious status change (connected)\n");
                        }

                      /* The LSDA (Low speed device attached) bit is valid
                       * when CCS == 1.
                       */

                      priv->lowspeed = (rhportst1 & OHCI_RHPORTST_LSDA) != 0;
                      ullvdbg("Speed:%s\n", priv->lowspeed ? "LOW" : "FULL");
                    }

                  /* Check if we are now disconnected */
 
                  else if (priv->connected)
                    {
                      /* Yes.. disconnect the device */

                      ullvdbg("Disconnected\n");
                      priv->connected = false;
                      priv->lowspeed  = false;

                      /* Are we bound to a class instance? */

                      if (priv->class)
                        {
                          /* Yes.. Disconnect the class */

                          CLASS_DISCONNECTED(priv->class);
                          priv->class = NULL;
                        }

                      /* Notify any waiters for the Root Hub Status change event */

                      if (priv->rhswait)
                        {
                          stm32_givesem(&priv->rhssem);
                          priv->rhswait = false;
                        }
                    }
                  else
                    {
                       ulldbg("Spurious status change (disconnected)\n");
                    }
                }

              /* Clear the status change interrupt */

              stm32_putreg(OHCI_RHPORTST_CSC, STM32_USBHOST_RHPORTST1);
            }

          /* Check for port reset status change */

          if ((rhportst1 & OHCI_RHPORTST_PRSC) != 0)
            {
              /* Release the RH port from reset */

              stm32_putreg(OHCI_RHPORTST_PRSC, STM32_USBHOST_RHPORTST1);
            }
        }

      /* Writeback Done Head interrupt */
 
      if ((pending & OHCI_INT_WDH) != 0)
        {
          struct stm32_gtd_s *td;
          struct stm32_gtd_s *next;

          /* The host controller just wrote the list of finished TDs into the HCCA
           * done head.  This may include multiple packets that were transferred
           * in the preceding frame.
           *
           * Remove the TD(s) from the Writeback Done Head in the HCCA and return
           * them to the free list.  Note that this is safe because the hardware
           * will not modify the writeback done head again until the WDH bit is
           * cleared in the interrupt status register.
           */

          td = (struct stm32_gtd_s *)HCCA->donehead;
          HCCA->donehead = 0;

          /* Process each TD in the write done list */

          for (; td; td = next)
            {
              /* Get the ED in which this TD was enqueued */

              struct stm32_ed_s *ed = td->ed;
              DEBUGASSERT(ed != NULL);

              /* Save the condition code from the (single) TD status/control
               * word.
               */

              ed->tdstatus = (td->hw.ctrl & GTD_STATUS_CC_MASK) >> GTD_STATUS_CC_SHIFT;

#ifdef CONFIG_DEBUG_USB
              if (ed->tdstatus != TD_CC_NOERROR)
                {
                  /* The transfer failed for some reason... dump some diagnostic info. */

                  ulldbg("ERROR: ED xfrtype:%d TD CTRL:%08x/CC:%d RHPORTST1:%08x\n",
                         ed->xfrtype, td->hw.ctrl, ed->tdstatus,
                         stm32_getreg(STM32_USBHOST_RHPORTST1));
                }
#endif

              /* Return the TD to the free list */

              next = (struct stm32_gtd_s *)td->hw.nexttd;
              stm32_tdfree(td);

              /* And wake up the thread waiting for the WDH event */

              if (ed->wdhwait)
                {
                  stm32_givesem(&ed->wdhsem);
                  ed->wdhwait = false;
                }
            }
        }

#ifdef CONFIG_DEBUG_USB
      if ((pending & STM32_DEBUG_INTS) != 0)
        {
          ulldbg("ERROR: Unhandled interrupts INTST:%08x\n", intst);
        }
#endif

      /* Clear interrupt status register */

      stm32_putreg(intst, STM32_USBHOST_INTST);
    }

  return OK;
}

/*******************************************************************************
 * USB Host Controller Operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_wait
 *
 * Description:
 *   Wait for a device to be connected or disconneced.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   connected - TRUE: Wait for device to be connected; FALSE: wait for device
 *      to be disconnected
 *
 * Returned Values:
 *   Zero (OK) is returned when a device in connected. This function will not
 *   return until either (1) a device is connected or (2) some failure occurs.
 *   On a failure, a negated errno value is returned indicating the nature of
 *   the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_wait(FAR struct usbhost_driver_s *drvr, bool connected)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  irqstate_t flags;

  /* Are we already connected? */

  flags = irqsave();
  while (priv->connected == connected)
    {
      /* No... wait for the connection/disconnection */

      priv->rhswait = true;
      stm32_takesem(&priv->rhssem);
    }
  irqrestore(flags);

  udbg("Connected:%s\n", priv->connected ? "YES" : "NO");
  return OK;
}

/*******************************************************************************
 * Name: stm32_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the configdesc() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_enumerate(FAR struct usbhost_driver_s *drvr)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      udbg("Not connected\n");
      return -ENODEV;
    }
 
  /* USB 2.0 spec says at least 50ms delay before port reset */

  up_mdelay(100);

  /* Put RH port 1 in reset (the STM32 supports only a single downstream port) */

  stm32_putreg(OHCI_RHPORTST_PRS, STM32_USBHOST_RHPORTST1);

  /* Wait for the port reset to complete */

  while ((stm32_getreg(STM32_USBHOST_RHPORTST1) & OHCI_RHPORTST_PRS) != 0);

  /* Release RH port 1 from reset and wait a bit */

  stm32_putreg(OHCI_RHPORTST_PRSC, STM32_USBHOST_RHPORTST1);
  up_mdelay(200);

  /* Let the common usbhost_enumerate do all of the real work.  Note that the
   * FunctionAddress (USB address) is hardcoded to one.
   */

  uvdbg("Enumerate the device\n");
  return usbhost_enumerate(drvr, 1, &priv->class);
}

/************************************************************************************
 * Name: stm32_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;

  DEBUGASSERT(drvr && funcaddr < 128 && maxpacketsize < 2048);

  /* We must have exclusive access to EP0 and the control list */

  stm32_takesem(&priv->exclsem);

  /* Set the EP0 ED control word */

  EDCTRL->hw.ctrl = (uint32_t)funcaddr << ED_CONTROL_FA_SHIFT | 
                    (uint32_t)maxpacketsize << ED_CONTROL_MPS_SHIFT;

  if (priv->lowspeed)
   {
     EDCTRL->hw.ctrl |= ED_CONTROL_S;
   }

  /* Set the transfer type to control */

  EDCTRL->xfrtype = USB_EP_ATTR_XFER_CONTROL;
  stm32_givesem(&priv->exclsem);

  uvdbg("EP0 CTRL:%08x\n", EDCTRL->hw.ctrl);
  return OK;
}

/************************************************************************************
 * Name: stm32_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint desciptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  struct stm32_ed_s      *ed;
  int                     ret  = -ENOMEM;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(priv && epdesc && ep && priv->connected);

  /* We must have exclusive access to the ED pool, the bulk list, the periodic list
   * and the interrupt table.
   */

  stm32_takesem(&priv->exclsem);

  /* Take the next ED from the beginning of the free list */

  ed = (struct stm32_ed_s *)g_edfree;
  if (ed)
    {
      /* Remove the ED from the freelist */

      g_edfree = ((struct stm32_list_s*)ed)->flink;

      /* Configure the endpoint descriptor. */
 
      memset((void*)ed, 0, sizeof(struct stm32_ed_s));
      ed->hw.ctrl = (uint32_t)(epdesc->funcaddr)     << ED_CONTROL_FA_SHIFT | 
                    (uint32_t)(epdesc->addr)         << ED_CONTROL_EN_SHIFT |
                    (uint32_t)(epdesc->mxpacketsize) << ED_CONTROL_MPS_SHIFT;

      /* Get the direction of the endpoint */

      if (epdesc->in)
        {
          ed->hw.ctrl |= ED_CONTROL_D_IN;
        }
      else
        {
          ed->hw.ctrl |= ED_CONTROL_D_OUT;
        }

      /* Check for a low-speed device */

      if (priv->lowspeed)
        {
          ed->hw.ctrl |= ED_CONTROL_S;
        }

      /* Set the transfer type */

      ed->xfrtype = epdesc->xfrtype;

      /* Special Case isochronous transfer types */

#if 0 /* Isochronous transfers not yet supported */
      if (ed->xfrtype == USB_EP_ATTR_XFER_ISOC)
        {
          ed->hw.ctrl |= ED_CONTROL_F;
        }
#endif
      uvdbg("EP%d CTRL:%08x\n", epdesc->addr, ed->hw.ctrl);

      /* Initialize the semaphore that is used to wait for the endpoint
       * WDH event.
       */

      sem_init(&ed->wdhsem, 0, 0);

      /* Link the common tail TD to the ED's TD list */

      ed->hw.headp = (uint32_t)TDTAIL;
      ed->hw.tailp = (uint32_t)TDTAIL;

      /* Now add the endpoint descriptor to the appropriate list */

      switch (ed->xfrtype)
        {
        case USB_EP_ATTR_XFER_BULK:
          ret = stm32_addbulked(priv, ed);
          break;

        case USB_EP_ATTR_XFER_INT:
          ret = stm32_addinted(priv, epdesc, ed);
          break;

        case USB_EP_ATTR_XFER_ISOC:
          ret = stm32_addisoced(priv, epdesc, ed);
          break;

        case USB_EP_ATTR_XFER_CONTROL:
        default:
          ret = -EINVAL;
          break;
        }

      /* Was the ED successfully added? */

      if (ret != OK)
        {
          /* No.. destroy it and report the error */

          udbg("ERROR: Failed to queue ED for transfer type: %d\n", ed->xfrtype);
          sem_destroy(&ed->wdhsem);
          stm32_edfree(ed);
        }
      else
        {
          /* Yes.. return an opaque reference to the ED */

          *ep = (usbhost_ep_t)ed;
        }
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

/************************************************************************************
 * Name: stm32_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpint to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  struct stm32_ed_s      *ed   = (struct stm32_ed_s *)ep;
  int                     ret;

  /* There should not be any pending, real TDs linked to this ED */

  DEBUGASSERT(ed && (ed->hw.headp & ED_HEADP_ADDR_MASK) == STM32_TDTAIL_ADDR);

  /* We must have exclusive access to the ED pool, the bulk list, the periodic list
   * and the interrupt table.
   */

  stm32_takesem(&priv->exclsem);

  /* Remove the ED to the correct list depending on the trasfer type */

  switch (ed->xfrtype)
    {
    case USB_EP_ATTR_XFER_BULK:
      ret = stm32_rembulked(priv, ed);
      break;

    case USB_EP_ATTR_XFER_INT:
      ret = stm32_reminted(priv, ed);
      break;

    case USB_EP_ATTR_XFER_ISOC:
      ret = stm32_remisoced(priv, ed);
      break;

    case USB_EP_ATTR_XFER_CONTROL:
    default:
      ret = -EINVAL;
      break;
    }

  /* Destroy the semaphore */

  sem_destroy(&ed->wdhsem);

  /* Put the ED back into the free list */

  stm32_edfree(ed);
  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  DEBUGASSERT(priv && buffer && maxlen);
  int ret = -ENOMEM;

  /* We must have exclusive access to the transfer buffer pool */

  stm32_takesem(&priv->exclsem);

  *buffer = stm32_tballoc();
  if (*buffer)
    {
      *maxlen = CONFIG_USBHOST_TDBUFSIZE;
      ret = OK;
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  DEBUGASSERT(buffer);

  /* We must have exclusive access to the transfer buffer pool */

  stm32_takesem(&priv->exclsem);
  stm32_tbfree(buffer);
  stm32_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: stm32_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t buflen)
{
  DEBUGASSERT(drvr && buffer);

#if STM32_IOBUFFERS > 0
  if (buflen <= CONFIG_USBHOST_IOBUFSIZE)
    {
      FAR uint8_t *alloc = stm32_allocio();
      if (alloc)
        {
          *buffer = alloc;
          return OK;
        }
    }
  return -ENOMEM;
#else
  return -ENOSYS;
#endif
}

/************************************************************************************
 * Name: stm32_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

#if STM32_IOBUFFERS > 0
  stm32_freeio(buffer);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: stm32_ctrlin and stm32_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  uint16_t len;
  int  ret;

  DEBUGASSERT(drvr && req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to EP0 and the control list */

  stm32_takesem(&priv->exclsem);

  len = stm32_getle16(req->len);
  ret = stm32_ctrltd(priv, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_IN, buffer, len);
        }

      if (ret == OK)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_OUT, NULL, 0);
        }
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

static int stm32_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  uint16_t len;
  int  ret;

  DEBUGASSERT(drvr && req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to EP0 and the control list */

  stm32_takesem(&priv->exclsem);

  len = stm32_getle16(req->len);
  ret = stm32_ctrltd(priv, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_OUT, (uint8_t*)buffer, len);
        }

      if (ret == OK)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_IN, NULL, 0);
        }
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  Only one transfer may be
 *   queued; Neither this method nor the ctrlin or ctrlout methods can be called
 *   again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/
 
static int stm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                          FAR uint8_t *buffer, size_t buflen)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  struct stm32_ed_s *ed = (struct stm32_ed_s *)ep;
  uint32_t dirpid;
  uint32_t regval;
#if STM32_IOBUFFERS > 0
  uint8_t *origbuf = NULL;
#endif
  bool in;
  int ret;

  DEBUGASSERT(priv && ed && buffer && buflen > 0);

  in = (ed->hw.ctrl  & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN;
  uvdbg("EP%d %s toggle:%d maxpacket:%d buflen:%d\n",
        (ed->hw.ctrl  & ED_CONTROL_EN_MASK) >> ED_CONTROL_EN_SHIFT, 
        in ? "IN" : "OUT",
        (ed->hw.headp & ED_HEADP_C) != 0 ? 1 : 0,
        (ed->hw.ctrl  & ED_CONTROL_MPS_MASK) >> ED_CONTROL_MPS_SHIFT, 
        buflen);

  /* We must have exclusive access to the endpoint, the TD pool, the I/O buffer
   * pool, the bulk and interrupt lists, and the HCCA interrupt table.
   */

  stm32_takesem(&priv->exclsem);

  /* Allocate an IO buffer if the user buffer does not lie in AHB SRAM */

#if STM32_IOBUFFERS > 0
  if ((uintptr_t)buffer < STM32_SRAM_BANK0 ||
      (uintptr_t)buffer >= (STM32_SRAM_BANK0 + STM32_BANK0_SIZE + STM32_BANK1_SIZE))
    {
      /* Will the transfer fit in an IO buffer? */

      if (buflen > CONFIG_USBHOST_IOBUFSIZE)
        {
          uvdbg("buflen (%d) > IO buffer size (%d)\n",
                 buflen, CONFIG_USBHOST_IOBUFSIZE);
          ret = -ENOMEM;
          goto errout;
        }

      /* Allocate an IO buffer in AHB SRAM */

      origbuf = buffer;
      buffer  = stm32_allocio();
      if (!buffer)
        {
          uvdbg("IO buffer allocation failed\n");
          ret = -ENOMEM;
          goto errout;
        }

      /* If this is an OUT transaction, copy the user data into the AHB
       * SRAM IO buffer.  Sad... so inefficient.  But without exposing
       * the AHB SRAM to the final, end-user client I don't know of any
       * way around this copy.
       */

      if (!in)
        {
          memcpy(buffer, origbuf, buflen);
        }
    }
#endif

  /* Set the request for the Writeback Done Head event well BEFORE enabling the
   * transfer.
   */

  ret = stm32_wdhwait(priv, ed);
  if (ret != OK)
    {
      udbg("ERROR: Device disconnected\n");
      goto errout;
    }

  /* Get the direction of the endpoint */

  if (in)
    {
      dirpid    = GTD_STATUS_DP_IN;
    }
  else
    {
      dirpid    = GTD_STATUS_DP_OUT;
    }

  /* Then enqueue the transfer */

  ed->tdstatus = TD_CC_NOERROR;
  ret = stm32_enqueuetd(priv, ed, dirpid, GTD_STATUS_T_TOGGLE, buffer, buflen);
  if (ret == OK)
    {
      /* BulkListFilled. This bit is used to indicate whether there are any
       * TDs on the Bulk list.
       */
 
      regval  = stm32_getreg(STM32_USBHOST_CMDST);
      regval |= OHCI_CMDST_BLF;
      stm32_putreg(regval, STM32_USBHOST_CMDST);

      /* Wait for the Writeback Done Head interrupt */

      stm32_takesem(&ed->wdhsem);

      /* Check the TD completion status bits */

      if (ed->tdstatus == TD_CC_NOERROR)
        {
          ret = OK;
        }
      else 
        {
          uvdbg("Bad TD completion status: %d\n", ed->tdstatus);
          ret = -EIO;
        }
    }

errout:
  /* Make sure that there is no outstanding request on this endpoint */

  ed->wdhwait = false;

  /* Free any temporary IO buffers */

#if STM32_IOBUFFERS > 0
  if (buffer && origbuf)
    {
      /* If this is an IN transaction, get the user data from the AHB
       * SRAM IO buffer.  Sad... so inefficient.  But without exposing
       * the AHB SRAM to the final, end-user client I don't know of any
       * way around this copy.
       */

      if (in && ret == OK)
        {
          memcpy(origbuf, buffer, buflen);
        }

      /* Then free the temporary I/O buffer */

      stm32_freeio(buffer);
    }
#endif

  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void stm32_disconnect(FAR struct usbhost_driver_s *drvr)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  priv->class = NULL;
}
  
/*******************************************************************************
 * Initialization
 *******************************************************************************/
/*******************************************************************************
 * Name: stm32_ep0init
 *
 * Description:
 *   Initialize ED for EP0, add it to the control ED list, and enable control
 *   transfers.
 *
 * Input Parameters:
 *   priv - private driver state instance.
 *
 * Returned Values:
 *   None
 *
 *******************************************************************************/

static inline void stm32_ep0init(struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  /* Set up some default values */

  (void)stm32_ep0configure(&priv->drvr, 1, 8);

  /* Initialize the common tail TD. */

  memset(TDTAIL, 0, sizeof(struct stm32_gtd_s));
  TDTAIL->ed              = EDCTRL;

  /* Link the common tail TD to the ED's TD list */

  memset(EDCTRL, 0, sizeof(struct stm32_ed_s));
  EDCTRL->hw.headp        = (uint32_t)TDTAIL;
  EDCTRL->hw.tailp        = (uint32_t)TDTAIL;

  /* Set the head of the control list to the EP0 EDCTRL (this would have to
   * change if we want more than on control EP queued at a time).
   */

  stm32_putreg(STM32_EDCTRL_ADDR, STM32_USBHOST_CTRLHEADED);

  /* ControlListEnable.  This bit is set to enable the processing of the
   * Control list.  Note: once enabled, it remains enabled and we may even
   * complete list processing before we get the bit set.  We really
   * should never modify the control list while CLE is set.
   */

  regval = stm32_getreg(STM32_USBHOST_CTRL);
  regval |= OHCI_CTRL_CLE;
  stm32_putreg(regval, STM32_USBHOST_CTRL);
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being intialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 *******************************************************************************/

FAR struct usbhost_driver_s *usbhost_initialize(int controller)
{
  struct stm32_usbhost_s *priv = &g_usbhost;
  uint32_t regval;
  uint8_t *buffer;
  irqstate_t flags;
  int i;

  /* Sanity checks.  NOTE: If certain OS features are enabled, it may be
   * necessary to increase the size of STM32_ED/TD_SIZE in stm32_ohciram.h
   */

  DEBUGASSERT(controller == 0);
  DEBUGASSERT(sizeof(struct stm32_ed_s)  <= STM32_ED_SIZE);
  DEBUGASSERT(sizeof(struct stm32_gtd_s) <= STM32_TD_SIZE);

  /* Initialize the state data structure */

  sem_init(&priv->rhssem,  0, 0);
  sem_init(&priv->exclsem, 0, 1);

#ifndef CONFIG_USBHOST_INT_DISABLE
  priv->ininterval  = MAX_PERINTERVAL;
  priv->outinterval = MAX_PERINTERVAL;
#endif

  /* Enable power by setting PCUSB in the PCONP register.  Disable interrupts
   * because this register may be shared with other drivers.
   */

  flags   = irqsave();
  regval  = stm32_getreg(STM32_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUSB;
  stm32_putreg(regval, STM32_SYSCON_PCONP);
  irqrestore(flags);

  /* Enable clocking on USB (USB PLL clocking was initialized in very low-
   * evel clock setup logic (see stm32_clockconfig.c)).  We do still need
   * to set up USBOTG CLKCTRL to enable clocking.
   *
   * NOTE: The PORTSEL clock needs to be enabled only when accessing OTGSTCTRL
   */

  stm32_putreg(STM32_CLKCTRL_ENABLES, STM32_USBOTG_CLKCTRL);

  /* Then wait for the clocks to be reported as "ON" */

  do
    {
      regval = stm32_getreg(STM32_USBOTG_CLKST);
    }
  while ((regval & STM32_CLKCTRL_ENABLES) != STM32_CLKCTRL_ENABLES);

  /* Set the OTG status and control register.  Bits 0:1 apparently mean:
   *
   *   00: U1=device, U2=host
   *   01: U1=host, U2=host
   *   10: reserved
   *   11: U1=host, U2=device
   *
   * We need only select U1=host (Bit 0=1, Bit 1 is not used on STM32);
   * NOTE: The PORTSEL clock needs to be enabled when accessing OTGSTCTRL
   */

  stm32_putreg(1, STM32_USBOTG_STCTRL);

  /* Now we can turn off the PORTSEL clock */

  stm32_putreg((STM32_CLKCTRL_ENABLES & ~USBOTG_CLK_PORTSELCLK), STM32_USBOTG_CLKCTRL);

  /* Configure I/O pins */

  usbhost_dumpgpio();
  stm32_configgpio(GPIO_USB_DP);      /* Positive differential data */
  stm32_configgpio(GPIO_USB_DM);      /* Negative differential data */
  stm32_configgpio(GPIO_USB_UPLED);   /* GoodLink LED control signal */
  stm32_configgpio(GPIO_USB_PPWR);    /* Port Power enable signal for USB port */
  stm32_configgpio(GPIO_USB_PWRD);    /* Power Status for USB port (host power switch) */
  stm32_configgpio(GPIO_USB_OVRCR);   /* USB port Over-Current status */
  usbhost_dumpgpio();

  udbg("Initializing Host Stack\n");

  /* Show AHB SRAM memory map */

#if 0 /* Useful if you have doubts about the layout */
  uvdbg("AHB SRAM:\n");
  uvdbg("  HCCA:   %08x %d\n", STM32_HCCA_BASE,   STM32_HCCA_SIZE);
  uvdbg("  TDTAIL: %08x %d\n", STM32_TDTAIL_ADDR, STM32_TD_SIZE);
  uvdbg("  EDCTRL: %08x %d\n", STM32_EDCTRL_ADDR, STM32_ED_SIZE);
  uvdbg("  EDFREE: %08x %d\n", STM32_EDFREE_BASE, STM32_ED_SIZE);
  uvdbg("  TDFREE: %08x %d\n", STM32_TDFREE_BASE, STM32_EDFREE_SIZE);
  uvdbg("  TBFREE: %08x %d\n", STM32_TBFREE_BASE, STM32_TBFREE_SIZE);
  uvdbg("  IOFREE: %08x %d\n", STM32_IOFREE_BASE, STM32_IOBUFFERS * CONFIG_USBHOST_IOBUFSIZE);
#endif

  /* Initialize all the TDs, EDs and HCCA to 0 */

  memset((void*)HCCA,   0, sizeof(struct ohci_hcca_s));
  memset((void*)TDTAIL, 0, sizeof(struct ohci_gtd_s));
  memset((void*)EDCTRL, 0, sizeof(struct stm32_ed_s));
  sem_init(&EDCTRL->wdhsem, 0, 0);

  /* Initialize user-configurable EDs */

  buffer = (uint8_t *)STM32_EDFREE_BASE;
  for (i = 0; i < CONFIG_USBHOST_NEDS; i++)
    {
      /* Put the ED in a free list */

      stm32_edfree((struct stm32_ed_s *)buffer);
      buffer += STM32_ED_SIZE;
    }

  /* Initialize user-configurable TDs */

  buffer = (uint8_t *)STM32_TDFREE_BASE;
  for (i = 0; i < CONFIG_USBHOST_NTDS; i++)
    {
      /* Put the ED in a free list */

      stm32_tdfree((struct stm32_gtd_s *)buffer);
      buffer += STM32_TD_SIZE;
    }

  /* Initialize user-configurable request/descriptor transfer buffers */

  buffer = (uint8_t *)STM32_TBFREE_BASE;
  for (i = 0; i < CONFIG_USBHOST_TDBUFFERS; i++)
    {
      /* Put the TD buffer in a free list */

      stm32_tbfree(buffer);
      buffer += CONFIG_USBHOST_TDBUFSIZE;
    }

#if STM32_IOBUFFERS > 0
  /* Initialize user-configurable IO buffers */

  buffer = (uint8_t *)STM32_IOFREE_BASE;
  for (i = 0; i < STM32_IOBUFFERS; i++)
    {
      /* Put the IO buffer in a free list */

      stm32_freeio(buffer);
      buffer += CONFIG_USBHOST_IOBUFSIZE;
    }
#endif

  /* Wait 50MS then perform hardware reset */

  up_mdelay(50);

  stm32_putreg(0, STM32_USBHOST_CTRL);        /* Hardware reset */
  stm32_putreg(0, STM32_USBHOST_CTRLHEADED);  /* Initialize control list head to Zero */
  stm32_putreg(0, STM32_USBHOST_BULKHEADED);  /* Initialize bulk list head to Zero */

  /* Software reset */

  stm32_putreg(OHCI_CMDST_HCR, STM32_USBHOST_CMDST);
  
  /* Write Fm interval (FI), largest data packet counter (FSMPS), and
   * periodic start.
   */
  
  stm32_putreg(DEFAULT_FMINTERVAL, STM32_USBHOST_FMINT);
  stm32_putreg(DEFAULT_PERSTART, STM32_USBHOST_PERSTART);

  /* Put HC in operational state */

  regval  = stm32_getreg(STM32_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_HCFS_MASK;
  regval |= OHCI_CTRL_HCFS_OPER;
  stm32_putreg(regval, STM32_USBHOST_CTRL);

  /* Set global power in HcRhStatus */

  stm32_putreg(OHCI_RHSTATUS_SGP, STM32_USBHOST_RHSTATUS);

  /* Set HCCA base address */

  stm32_putreg((uint32_t)HCCA, STM32_USBHOST_HCCA);

  /* Set up EP0 */

  stm32_ep0init(priv);

  /* Clear pending interrupts */

  regval = stm32_getreg(STM32_USBHOST_INTST);
  stm32_putreg(regval, STM32_USBHOST_INTST);

  /* Enable OHCI interrupts */

  stm32_putreg((STM32_ALL_INTS|OHCI_INT_MIE), STM32_USBHOST_INTEN);

  /* Attach USB host controller interrupt handler */

  if (irq_attach(STM32_IRQ_USB, stm32_usbinterrupt) != 0)
    {
      udbg("Failed to attach IRQ\n");
      return NULL;
    }

  /* Enable USB interrupts at the SYCON controller.  Disable interrupts
   * because this register may be shared with other drivers.
   */

  flags   = irqsave();
  regval  = stm32_getreg(STM32_SYSCON_USBINTST);
  regval |= SYSCON_USBINTST_ENINTS;
  stm32_putreg(regval, STM32_SYSCON_USBINTST);
  irqrestore(flags);

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  regval          = stm32_getreg(STM32_USBHOST_RHPORTST1);
  priv->connected = ((regval & OHCI_RHPORTST_CCS) != 0);

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(STM32_IRQ_USB); /* enable USB interrupt */
  udbg("USB host Initialized, Device connected:%s\n",
       priv->connected ? "YES" : "NO");

  return &priv->drvr;
}
