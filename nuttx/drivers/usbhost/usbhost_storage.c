/****************************************************************************
 * drivers/usbhost/usbhost_storage.c
 *
 *   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/scsi.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/storage.h>

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_BULK_DISABLE) && \
   !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* If the create() method is called by the USB host device driver from an
 * interrupt handler, then it will be unable to call kmalloc() in order to
 * allocate a new class instance.  If the create() method is called from the
 * interrupt level, then class instances must be pre-allocated.
 */

#ifndef CONFIG_USBHOST_NPREALLOC
#  define CONFIG_USBHOST_NPREALLOC 0
#endif

#if CONFIG_USBHOST_NPREALLOC > 26
#  error "Currently limited to 26 devices /dev/sda-z"
#endif

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/sd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/sd%c"
#define DEV_NAMELEN         10

/* Used in usbhost_connect() */

#define USBHOST_IFFOUND     0x01
#define USBHOST_BINFOUND    0x02
#define USBHOST_BOUTFOUND   0x04
#define USBHOST_ALLFOUND    0x07

#define USBHOST_MAX_RETRIES 100
#define USBHOST_MAX_CREFS   0x7fff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains the internal, private state of the USB host mass
 * storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide to the mass storage class */

  char                    sdchar;       /* Character identifying the /dev/sd[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  uint8_t                 ifno;         /* Interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  uint16_t                blocksize;    /* Block size of USB mass storage device */
  uint32_t                nblocks;      /* Number of blocks on the USB mass storage device */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  struct work_s           work;         /* For interacting with the worker thread */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  usbhost_ep_t            bulkin;       /* Bulk IN endpoint */
  usbhost_ep_t            bulkout;      /* Bulk OUT endpoint */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname);

/* CBW/CSW debug helpers */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
static void usbhost_dumpcbw(FAR struct usbmsc_cbw_s *cbw);
static void usbhost_dumpcsw(FAR struct usbmsc_csw_s *csw);
#else
#  define usbhost_dumpcbw(cbw);
#  define usbhost_dumpcsw(csw);
#endif

/* CBW helpers */

static inline void usbhost_requestsensecbw(FAR struct usbmsc_cbw_s *cbw);
static inline void usbhost_testunitreadycbw(FAR struct usbmsc_cbw_s *cbw);
static inline void usbhost_readcapacitycbw(FAR struct usbmsc_cbw_s *cbw);
static inline void usbhost_inquirycbw (FAR struct usbmsc_cbw_s *cbw);
static inline void usbhost_readcbw (size_t startsector, uint16_t blocksize,
                                    unsigned int nsectors,
                                    FAR struct usbmsc_cbw_s *cbw);
static inline void usbhost_writecbw(size_t startsector, uint16_t blocksize,
                                    unsigned int nsectors,
                                    FAR struct usbmsc_cbw_s *cbw);
/* Command helpers */

static inline int usbhost_maxlunreq(FAR struct usbhost_state_s *priv);
static inline int usbhost_testunitready(FAR struct usbhost_state_s *priv);
static inline int usbhost_requestsense(FAR struct usbhost_state_s *priv);
static inline int usbhost_readcapacity(FAR struct usbhost_state_s *priv);
static inline int usbhost_inquiry(FAR struct usbhost_state_s *priv);

/* Worker thread actions */

static void usbhost_destroy(FAR void *arg);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr);
static inline int usbhost_initvolume(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline uint16_t usbhost_getbe16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline void usbhost_putbe16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
static inline uint32_t usbhost_getbe32(const uint8_t *val);
static void usbhost_putle32(uint8_t *dest, uint32_t val);
static void usbhost_putbe32(uint8_t *dest, uint32_t val);

/* Transfer descriptor memory management */

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tfree(FAR struct usbhost_state_s *priv);
static FAR struct usbmsc_cbw_s *usbhost_cbwalloc(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* struct block_operations methods */

static int usbhost_open(FAR struct inode *inode);
static int usbhost_close(FAR struct inode *inode);
static ssize_t usbhost_read(FAR struct inode *inode, FAR unsigned char *buffer,
                            size_t startsector, unsigned int nsectors);
#ifdef CONFIG_FS_WRITABLE
static ssize_t usbhost_write(FAR struct inode *inode,
                             FAR const unsigned char *buffer, size_t startsector,
                             unsigned int nsectors);
#endif
static int usbhost_geometry(FAR struct inode *inode,
                            FAR struct geometry *geometry);
static int usbhost_ioctl(FAR struct inode *inode, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID informatino that will  be
 * used to associate the USB host mass storage class to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_id =
{
  USB_CLASS_MASS_STORAGE, /* base     */
  USBMSC_SUBCLASS_SCSI,  /* subclass */
  USBMSC_PROTO_BULKONLY, /* proto    */
  0,                      /* vid      */
  0                       /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_storage =
{
  NULL,                   /* flink    */
  usbhost_create,         /* create   */
  1,                      /* nids     */
  &g_id                   /* id[]     */
};

/* Block driver operations.  This is the interface exposed to NuttX by the
 * class that permits it to behave like a block driver.
 */

static const struct block_operations g_bops =
{
  usbhost_open,           /* open     */
  usbhost_close,          /* close    */
  usbhost_read,           /* read     */
#ifdef CONFIG_FS_WRITABLE
  usbhost_write,          /* write    */
#else
  NULL,                   /* write    */
#endif
  usbhost_geometry,       /* geometry */
  usbhost_ioctl           /* ioctl    */
};

/* This is an array of pre-allocated USB host storage class instances */

#if CONFIG_USBHOST_NPREALLOC > 0
static struct usbhost_state_s g_prealloc[CONFIG_USBHOST_NPREALLOC];
#endif

/* This is a list of free, pre-allocated USB host storage class instances */

#if CONFIG_USBHOST_NPREALLOC > 0
static struct usbhost_state_s *g_freelist;
#endif

/* This is a bitmap that is used to allocate device names /dev/sda-z. */

static uint32_t g_devinuse;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void usbhost_takesem(sem_t *sem)
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
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

#if CONFIG_USBHOST_NPREALLOC > 0
static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  struct usbhost_state_s *priv;
  irqstate_t flags;

  /* We may be executing from an interrupt handler so we need to take one of
   * our pre-allocated class instances from the free list.
   */

  flags = irqsave();
  priv = g_freelist;
  if (priv)
    {
      g_freelist        = priv->class.flink;
      priv->class.flink = NULL;
    }
  irqrestore(flags);
  ullvdbg("Allocated: %p\n", priv);;
  return priv;
}
#else
static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  /* We are not executing from an interrupt handler so we can just call
   * kmalloc() to get memory for the class instance.
   */

  DEBUGASSERT(!up_interrupt_context());
  priv = (FAR struct usbhost_state_s *)kmalloc(sizeof(struct usbhost_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
}
#endif

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if CONFIG_USBHOST_NPREALLOC > 0
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class)
{
  irqstate_t flags;
  DEBUGASSERT(class != NULL);

  ullvdbg("Freeing: %p\n", class);;

  /* Just put the pre-allocated class structure back on the freelist */

  flags = irqsave();
  class->class.flink = g_freelist;
  g_freelist = class;
  irqrestore(flags);
}
#else
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance (calling sched_free() in case we are executing
   * from an interrupt handler.
   */

  uvdbg("Freeing: %p\n", class);;
  kfree(class);
}
#endif

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of mass storage device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = irqsave();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->sdchar = 'a' + devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->sdchar;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv, char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->sdchar);
}

/****************************************************************************
 * Name: CBW/CSW debug helpers
 *
 * Description:
 *   The following functions are helper functions used to dump CBWs and CSWs.
 *
 * Input Parameters:
 *   cbw/csw - A reference to the CBW/CSW to dump.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
static void usbhost_dumpcbw(FAR struct usbmsc_cbw_s *cbw)
{
  int i;

  uvdbg("CBW:\n");
  uvdbg("  signature: %08x\n", usbhost_getle32(cbw->signature));
  uvdbg("  tag:       %08x\n", usbhost_getle32(cbw->tag));
  uvdbg("  datlen:    %08x\n", usbhost_getle32(cbw->datlen));
  uvdbg("  flags:     %02x\n", cbw->flags);
  uvdbg("  lun:       %02x\n", cbw->lun);
  uvdbg("  cdblen:    %02x\n", cbw->cdblen);

  uvdbg("CDB:\n");
  for (i = 0; i < cbw->cdblen; i += 8)
    {
      uvdbg("  %02x %02x %02x %02x %02x %02x %02x %02x\n",
            cbw->cdb[i],   cbw->cdb[i+1], cbw->cdb[i+2], cbw->cdb[i+3],
            cbw->cdb[i+4], cbw->cdb[i+5], cbw->cdb[i+6], cbw->cdb[i+7]);
    }
}

static void usbhost_dumpcsw(FAR struct usbmsc_csw_s *csw)
{
  uvdbg("CSW:\n");
  uvdbg("  signature: %08x\n", usbhost_getle32(csw->signature));
  uvdbg("  tag:       %08x\n", usbhost_getle32(csw->tag));
  uvdbg("  residue:   %08x\n", usbhost_getle32(csw->residue));
  uvdbg("  status:    %02x\n", csw->status);
}
#endif

/****************************************************************************
 * Name: CBW helpers
 *
 * Description:
 *   The following functions are helper functions used to format CBWs.
 *
 * Input Parameters:
 *   cbw - A reference to allocated and initialized CBW to be built.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_requestsensecbw(FAR struct usbmsc_cbw_s *cbw)
{
  FAR struct scsicmd_requestsense_s *reqsense;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, SCSIRESP_FIXEDSENSEDATA_SIZEOF);
  cbw->flags         = USBMSC_CBWFLAG_IN;
  cbw->cdblen        = SCSICMD_REQUESTSENSE_SIZEOF;

  /* Format the CDB */

  reqsense           = (FAR struct scsicmd_requestsense_s *)cbw->cdb;
  reqsense->opcode   = SCSI_CMD_REQUESTSENSE;
  reqsense->alloclen = SCSIRESP_FIXEDSENSEDATA_SIZEOF;

  usbhost_dumpcbw(cbw);
}

static inline void usbhost_testunitreadycbw(FAR struct usbmsc_cbw_s *cbw)
{
  /* Format the CBW */

  cbw->cdblen = SCSICMD_TESTUNITREADY_SIZEOF;

  /* Format the CDB */

  cbw->cdb[0] = SCSI_CMD_TESTUNITREADY;

  usbhost_dumpcbw(cbw);
}

static inline void usbhost_readcapacitycbw(FAR struct usbmsc_cbw_s *cbw)
{
  FAR struct scsicmd_readcapacity10_s *rcap10;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, SCSIRESP_READCAPACITY10_SIZEOF);
  cbw->flags     = USBMSC_CBWFLAG_IN;
  cbw->cdblen    = SCSICMD_READCAPACITY10_SIZEOF;

  /* Format the CDB */

  rcap10         = (FAR struct scsicmd_readcapacity10_s *)cbw->cdb;
  rcap10->opcode = SCSI_CMD_READCAPACITY10;

  usbhost_dumpcbw(cbw);
}

static inline void usbhost_inquirycbw (FAR struct usbmsc_cbw_s *cbw)
{
  FAR struct scscicmd_inquiry_s *inq;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, SCSIRESP_INQUIRY_SIZEOF);
  cbw->flags    = USBMSC_CBWFLAG_IN;
  cbw->cdblen   = SCSICMD_INQUIRY_SIZEOF;

  /* Format the CDB */

  inq           = (FAR struct scscicmd_inquiry_s *)cbw->cdb;
  inq->opcode   = SCSI_CMD_INQUIRY;
  usbhost_putbe16(inq->alloclen, SCSIRESP_INQUIRY_SIZEOF);

  usbhost_dumpcbw(cbw);
}

static inline void
usbhost_readcbw (size_t startsector, uint16_t blocksize,
                 unsigned int nsectors, FAR struct usbmsc_cbw_s *cbw)
{
  FAR struct scsicmd_read10_s *rd10;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, blocksize * nsectors);
  cbw->flags   = USBMSC_CBWFLAG_IN;
  cbw->cdblen  = SCSICMD_READ10_SIZEOF;

  /* Format the CDB */

  rd10 = (FAR struct scsicmd_read10_s *)cbw->cdb;
  rd10->opcode = SCSI_CMD_READ10;
  usbhost_putbe32(rd10->lba, startsector);
  usbhost_putbe16(rd10->xfrlen, nsectors);

  usbhost_dumpcbw(cbw);
}

static inline void
usbhost_writecbw(size_t startsector, uint16_t blocksize,
                 unsigned int nsectors, FAR struct usbmsc_cbw_s *cbw)
{
  FAR struct scsicmd_write10_s *wr10;

  /* Format the CBW */

  usbhost_putle32(cbw->datlen, blocksize * nsectors);
  cbw->cdblen  = SCSICMD_WRITE10_SIZEOF;

  /* Format the CDB */

  wr10 = (FAR struct scsicmd_write10_s *)cbw->cdb;
  wr10->opcode = SCSI_CMD_WRITE10;
  usbhost_putbe32(wr10->lba, startsector);
  usbhost_putbe16(wr10->xfrlen, nsectors);

  usbhost_dumpcbw(cbw);
}

/****************************************************************************
 * Name: Command helpers
 *
 * Description:
 *   The following functions are helper functions used to send commands.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_maxlunreq(FAR struct usbhost_state_s *priv)
{
  FAR struct usb_ctrlreq_s *req = (FAR struct usb_ctrlreq_s *)priv->tbuffer;
  DEBUGASSERT(priv && priv->tbuffer);
  int ret;

  /* Request maximum logical unit number.  NOTE: On an IN transaction, The
   * req and buffer pointers passed to DRVR_CTRLIN may refer to the same
   * allocated memory.
   */

  uvdbg("Request maximum logical unit number\n");
  memset(req, 0, sizeof(struct usb_ctrlreq_s));
  req->type    = USB_DIR_IN|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE;
  req->req     = USBMSC_REQ_GETMAXLUN;
  usbhost_putle16(req->len, 1);

  ret = DRVR_CTRLIN(priv->drvr, req, priv->tbuffer);
  if (ret != OK)
    {
      /* Devices that do not support multiple LUNs may stall this command.
       * On a failure, a single LUN is assumed.
       */

      *(priv->tbuffer) = 0;
    }
  return OK;
}

static inline int usbhost_testunitready(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_cbw_s *cbw;
  int result;

  /* Initialize a CBW (re-using the allocated transfer buffer) */

  cbw = usbhost_cbwalloc(priv);
  if (!cbw)
    {
      udbg("ERROR: Failed to create CBW\n");
      return -ENOMEM;
    }

  /* Construct and send the CBW */

  usbhost_testunitreadycbw(cbw);
  result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                        (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
  if (result == OK)
    {
      /* Receive the CSW */

      result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                             priv->tbuffer, USBMSC_CSW_SIZEOF);
      if (result == OK)
        {
          usbhost_dumpcsw((FAR struct usbmsc_csw_s *)priv->tbuffer);
        }
    }
  return result;
}

static inline int usbhost_requestsense(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_cbw_s *cbw;
  int result;

  /* Initialize a CBW (re-using the allocated transfer buffer) */

  cbw = usbhost_cbwalloc(priv);
  if (!cbw)
    {
      udbg("ERROR: Failed to create CBW\n");
      return -ENOMEM;
    }

  /* Construct and send the CBW */

  usbhost_requestsensecbw(cbw);
  result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                        (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
  if (result == OK)
    {
      /* Receive the sense data response */

      result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                             priv->tbuffer, SCSIRESP_FIXEDSENSEDATA_SIZEOF);
      if (result == OK)
        {
          /* Receive the CSW */

          result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                 priv->tbuffer, USBMSC_CSW_SIZEOF);
          if (result == OK)
            {
              usbhost_dumpcsw((FAR struct usbmsc_csw_s *)priv->tbuffer);
            }
        }
    }

  return result;
}

static inline int usbhost_readcapacity(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_cbw_s *cbw;
  FAR struct scsiresp_readcapacity10_s *resp;
  int result;

  /* Initialize a CBW (re-using the allocated transfer buffer) */

  cbw = usbhost_cbwalloc(priv);
  if (!cbw)
    {
      udbg("ERROR: Failed to create CBW\n");
      return -ENOMEM;
    }

  /* Construct and send the CBW */

  usbhost_readcapacitycbw(cbw);
  result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                        (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
  if (result == OK)
    {
      /* Receive the read capacity CBW IN response */

      result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                             priv->tbuffer, SCSIRESP_READCAPACITY10_SIZEOF);
      if (result == OK)
        {
          /* Save the capacity information */

          resp            = (FAR struct scsiresp_readcapacity10_s *)priv->tbuffer;
          priv->nblocks   = usbhost_getbe32(resp->lba) + 1;
          priv->blocksize = usbhost_getbe32(resp->blklen);

          /* Receive the CSW */

          result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                 priv->tbuffer, USBMSC_CSW_SIZEOF);
          if (result == OK)
            {
              usbhost_dumpcsw((FAR struct usbmsc_csw_s *)priv->tbuffer);
            }
        }
    }

  return result;
}

static inline int usbhost_inquiry(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_cbw_s *cbw;
  FAR struct scsiresp_inquiry_s *resp;
  int result;

  /* Initialize a CBW (re-using the allocated transfer buffer) */

  cbw = usbhost_cbwalloc(priv);
  if (!cbw)
    {
      udbg("ERROR: Failed to create CBW\n");
      return -ENOMEM;
    }

  /* Construct and send the CBW */

  usbhost_inquirycbw(cbw);
  result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                         (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
  if (result == OK)
    {
      /* Receive the CBW IN response */

      result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                             priv->tbuffer, SCSIRESP_INQUIRY_SIZEOF);
      if (result == OK)
        {
          /* TODO: If USB debug is enabled, dump the response data here */

          resp = (FAR struct scsiresp_inquiry_s *)priv->tbuffer;

          /* Receive the CSW */

          result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                 priv->tbuffer, USBMSC_CSW_SIZEOF);
          if (result == OK)
            {
              usbhost_dumpcsw((FAR struct usbmsc_csw_s *)priv->tbuffer);
            }
        }
    }

  return result;
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB mass storage device has been disconnected and the refernce count
 *   on the USB host class instance has gone to 1.. Time to destroy the USB
 *   host class instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);

  /* Unregister the block driver */

  usbhost_mkdevname(priv, devname);
  (void)unregister_blockdriver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the bulk endpoints */

  if (priv->bulkout)
    {
      DRVR_EPFREE(priv->drvr, priv->bulkout);
    }

  if (priv->bulkin)
    {
      DRVR_EPFREE(priv->drvr, priv->bulkin);
    }

  /* Free any transfer buffers */

  usbhost_tfree(priv);

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->drvr);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s bindesc;
  FAR struct usbhost_epdesc_s boutdesc;
  int remaining;
  uint8_t found = 0;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
  */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more dscriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            FAR struct usb_ifdesc_s *ifdesc = (FAR struct usb_ifdesc_s *)configdesc;

            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Save the interface number and mark ONLY the interface found */

            priv->ifno = ifdesc->ifno;
            found      = USBHOST_IFFOUND;
          }
          break;

        /* Endpoint descriptor.  We expect two bulk endpoints, an IN and an
         * OUT.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for a bulk endpoint.  We only support the bulk-only
             * protocol so I suppose anything else should really be an error.
             */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_BULK)
              {
                /* Yes.. it is a bulk endpoint.  IN or OUT? */

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an OUT bulk endpoint.  There should be only one
                     * bulk OUT endpoint.
                     */

                    if ((found & USBHOST_BOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BOUTFOUND;

                    /* Save the bulk OUT endpoint information */

                    boutdesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    boutdesc.in           = false;
                    boutdesc.funcaddr     = funcaddr;
                    boutdesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    boutdesc.interval     = epdesc->interval;
                    boutdesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk OUT EP addr:%d mxpacketsize:%d\n",
                          boutdesc.addr, boutdesc.mxpacketsize);
                  }
                else
                  {
                    /* It is an IN bulk endpoint.  There should be only one
                     * bulk IN endpoint.
                     */

                    if ((found & USBHOST_BINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know
                         * what to do with this.
                         */

                        return -EINVAL;
                      }
                    found |= USBHOST_BINFOUND;

                    /* Save the bulk IN endpoint information */

                    bindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    bindesc.in           = 1;
                    bindesc.funcaddr     = funcaddr;
                    bindesc.xfrtype      = USB_EP_ATTR_XFER_BULK;
                    bindesc.interval     = epdesc->interval;
                    bindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);
                    uvdbg("Bulk IN EP addr:%d mxpacketsize:%d\n",
                          bindesc.addr, bindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          break;
        }

      /* If we found everything we need with this interface, then break out
       * of the loop early.
       */

      if (found == USBHOST_ALLFOUND)
        {
          break;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? Hmmm..  I wonder..
   * can we work read-only or write-only if only one bulk endpoint found?
   */

  if (found != USBHOST_ALLFOUND)
    {
      ulldbg("ERROR: Found IF:%s BIN:%s BOUT:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_BINFOUND) != 0 ? "YES" : "NO",
             (found & USBHOST_BOUTFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints */

  ret = DRVR_EPALLOC(priv->drvr, &boutdesc, &priv->bulkout);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk OUT endpoint\n");
      return ret;
    }

  ret = DRVR_EPALLOC(priv->drvr, &bindesc, &priv->bulkin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate Bulk IN endpoint\n");
      (void)DRVR_EPFREE(priv->drvr, priv->bulkout);
      return ret;
    }

  ullvdbg("Endpoints allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_initvolume
 *
 * Description:
 *   The USB mass storage device has been successfully connected.  This
 *   completes the initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_initvolume(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_csw_s *csw;
  unsigned int retries;
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  /* Set aside a transfer buffer for exclusive use by the mass storage driver */

  ret = usbhost_talloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Request the maximum logical unit number */

  uvdbg("Get max LUN\n");
  ret = usbhost_maxlunreq(priv);

  /* Wait for the unit to be ready */

  for (retries = 0; retries < USBHOST_MAX_RETRIES && ret == OK; retries++)
    {
      uvdbg("Test unit ready, retries=%d\n", retries);

      /* Send TESTUNITREADY to see the unit is ready */

      ret = usbhost_testunitready(priv);
      if (ret == OK)
        {
          /* Is the unit is ready */

          csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
          if (csw->status == 0)
            {
              /* Yes... break out of the loop */

              break;
            }

          /* No.. Request mode sense information.  The REQUEST SENSE command
           * is sent only "to clear interlocked unit attention conditions."
           * The returned status is ignored here.
           */

          uvdbg("Request sense\n");
          ret = usbhost_requestsense(priv);
        }
    }

  /* Did the unit become ready?  Did an error occur? Or did we time out? */

  if (retries >= USBHOST_MAX_RETRIES)
    {
      udbg("ERROR: Timeout!\n");
      ret = -ETIMEDOUT;
    }

  if (ret == OK)
    {
      /* Get the capacity of the volume */

      uvdbg("Read capacity\n");
      ret = usbhost_readcapacity(priv);
      if (ret == OK)
        {
          /* Check the CSW for errors */

          csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
          if (csw->status != 0)
            {
              udbg("ERROR: CSW status error: %d\n", csw->status);
              ret = -ENODEV;
            }
        }
    }

  /* Get information about the volume */

  if (ret == OK)
    {
      /* Inquiry */

      uvdbg("Inquiry\n");
      ret = usbhost_inquiry(priv);
      if (ret == OK)
        {
          /* Check the CSW for errors */

          csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
          if (csw->status != 0)
            {
              udbg("ERROR: CSW status error: %d\n", csw->status);
              ret = -ENODEV;
            }
        }
    }

  /* Register the block driver */

  if (ret == OK)
    {
      char devname[DEV_NAMELEN];

      uvdbg("Register block driver\n");
      usbhost_mkdevname(priv, devname);
      ret = register_blockdriver(devname, &g_bops, 0, priv);
    }

  /* Check if we successfully initialized. We now have to be concerned
   * about asynchronous modification of crefs because the block
   * driver has been registerd.
   */

  if (ret == OK)
    {
      usbhost_takesem(&priv->exclsem);
      DEBUGASSERT(priv->crefs >= 2);

      /* Decrement the reference count */

      priv->crefs--;

      /* Handle a corner case where (1) open() has been called so the
       * reference count was > 2, but the device has been disconnected.
       * In this case, the class instance needs to persist until close()
       * is called.
       */

      if (priv->crefs <= 1 && priv->disconnected)
        {
          /* The will cause the enumeration logic to disconnect
           * the class driver.
           */

          ret = -ENODEV;
        }

      /* Release the semaphore... there is a race condition here.
       * Decrementing the reference count and releasing the semaphore
       * allows usbhost_destroy() to execute (on the worker thread);
       * the class driver instance could get destoryed before we are
       * ready to handle it!
       */

       usbhost_givesem(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_getbe16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit big endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the big endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getbe16(const uint8_t *val)
{
  return (uint16_t)val[0] << 8 | (uint16_t)val[1];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_putbe16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbe16(uint8_t *dest, uint16_t val)
{
  dest[0] = val >> 8; /* Big endian means MS byte first in byte stream */
  dest[1] = val & 0xff;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 | (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_getbe32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getbe32(const uint8_t *val)
{
  /* Big endian means MS halfword first in byte stream */

  return (uint32_t)usbhost_getbe16(val) << 16 | (uint32_t)usbhost_getbe16(&val[2]);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest+2, (uint16_t)(val >> 16));
}

/****************************************************************************
 * Name: usbhost_putbe32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit big endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbe32(uint8_t *dest, uint32_t val)
{
  /* Big endian means MS halfword first in byte stream */

  usbhost_putbe16(dest, (uint16_t)(val >> 16));
  usbhost_putbe16(dest+2, (uint16_t)(val & 0xffff));
}

/****************************************************************************
 * Name: usbhost_talloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_talloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tfree(FAR struct usbhost_state_s *priv)
{
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->drvr);
      result         = DRVR_FREE(priv->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }
  return result;
}

/****************************************************************************
 * Name: usbhost_cbwalloc
 *
 * Description:
 *   Initialize a CBW (re-using the allocated transfer buffer). Upon
 *   successful return, the CBW is cleared and has the CBW signature in place.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static FAR struct usbmsc_cbw_s *usbhost_cbwalloc(FAR struct usbhost_state_s *priv)
{
  FAR struct usbmsc_cbw_s *cbw = NULL;

  DEBUGASSERT(priv->tbuffer && priv->tbuflen >= sizeof(struct usbmsc_cbw_s))

  /* Intialize the CBW sructure */

  cbw = (FAR struct usbmsc_cbw_s *)priv->tbuffer;
  memset(cbw, 0, sizeof(struct usbmsc_cbw_s));
  usbhost_putle32(cbw->signature, USBMSC_CBW_SIGNATURE);
  return cbw;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct usbhost_registry_s.
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   drvr - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the drvr input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                                  FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host mass storage class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
         /* Initialize class method function pointers */

          priv->class.connect      = usbhost_connect;
          priv->class.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by the driver */

          priv->crefs              = 1;

          /* Initialize semphores (this works okay in the interrupt context) */

          sem_init(&priv->exclsem, 0, 1);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* NOTE: We do not yet know the geometry of the USB mass storage device */

          /* Return the instance of the USB mass storage class */

          return &priv->class;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }
  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the bulk I/O endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the LUNs and register the block driver(s) */

      ret = usbhost_initvolume(priv);
      if (ret != OK)
        {
          udbg("usbhost_initvolume() failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *class)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the mass storage device that the device
   * is no longer available.
   */

  flags              = irqsave();
  priv->disconnected = true;

  /* Now check the number of references on the class instance.  If it is one,
   * then we can free the class instance now.  Otherwise, we will have to
   * wait until the holders of the references free them by closing the
   * block driver.
   */

  ullvdbg("crefs: %d\n", priv->crefs);
  if (priv->crefs == 1)
    {
      /* Destroy the class instance.  If we are executing from an interrupt
       * handler, then defer the destruction to the worker thread.
       * Otherwise, destroy the instance now.
       */

      if (up_interrupt_context())
        {
          /* Destroy the instance on the worker thread. */

          uvdbg("Queuing destruction: worker %p->%p\n", priv->work.worker, usbhost_destroy);
          DEBUGASSERT(priv->work.worker == NULL);
          (void)work_queue(&priv->work, usbhost_destroy, priv, 0);
       }
      else
        {
          /* Do the work now */

          usbhost_destroy(priv);
        }
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * struct block_operations methods
 ****************************************************************************/
/****************************************************************************
 * Name: usbhost_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int usbhost_open(FAR struct inode *inode)
{
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  usbhost_takesem(&priv->exclsem);

  /* Check if the mass storage device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous disconnect
   * events.
   */

  flags = irqsave();
  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any further
       * attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
      ret = OK;
    }
  irqrestore(flags);

  usbhost_givesem(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: usbhost_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int usbhost_close(FAR struct inode *inode)
{
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;

  /* Decrement the reference count on the block driver */

  DEBUGASSERT(priv->crefs > 1);
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;

  /* Release the semaphore.  The following operations when crefs == 1 are
   * safe because we know that there is no outstanding open references to
   * the block driver.
   */

  usbhost_givesem(&priv->exclsem);

  /* We need to disable interrupts momentarily to assure that there are
   * no asynchronous disconnect events.
   */

  flags = irqsave();

  /* Check if the USB mass storage device is still connected.  If the
   * storage device is not connected and the reference count just
   * decremented to one, then unregister the block driver and free
   * the class instance.
   */

  if (priv->crefs <= 1 && priv->disconnected)
    {
      /* Destroy the class instance */

      DEBUGASSERT(priv->crefs == 1);
      usbhost_destroy(priv);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Read the specified numer of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct inode *inode, unsigned char *buffer,
                            size_t startsector, unsigned int nsectors)
{
  FAR struct usbhost_state_s *priv;
  ssize_t ret = 0;
  int result;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;
  uvdbg("startsector: %d nsectors: %d sectorsize: %d\n",
        startsector, nsectors, priv->blocksize);

  /* Check if the mass storage device is still connected */

  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any attempt to read
       * from the device.
       */

      ret = -ENODEV;
    }
  else if (nsectors > 0)
    {
      FAR struct usbmsc_cbw_s *cbw;

      usbhost_takesem(&priv->exclsem);

      /* Assume allocation failure */

      ret = -ENOMEM;

      /* Initialize a CBW (re-using the allocated transfer buffer) */

      cbw = usbhost_cbwalloc(priv);
      if (cbw)
        {
          /* Loop in the event that EAGAIN is returned (mean that the
           * transaction was NAKed and we should try again.
           */

          do
            {
              /* Assume some device failure */

              ret = -ENODEV;

              /* Construct and send the CBW */

              usbhost_readcbw(startsector, priv->blocksize, nsectors, cbw);
              result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                                     (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
              if (result == OK)
                {
                  /* Receive the user data */

                  result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                         buffer, priv->blocksize * nsectors);
                  if (result == OK)
                    {
                      /* Receive the CSW */

                      result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                             priv->tbuffer, USBMSC_CSW_SIZEOF);
                      if (result == OK)
                        {
                          FAR struct usbmsc_csw_s *csw;

                          /* Check the CSW status */

                          csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
                          if (csw->status == 0)
                            {
                              ret = nsectors;
                            }
                        }
                    }
                }
            } while (result == -EAGAIN);
        }

      usbhost_givesem(&priv->exclsem);
    }

  /* On success, return the number of blocks read */

  return ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_WRITABLE
static ssize_t usbhost_write(FAR struct inode *inode, const unsigned char *buffer,
                           size_t startsector, unsigned int nsectors)
{
  FAR struct usbhost_state_s *priv;
  ssize_t ret;
  int result;

  uvdbg("sector: %d nsectors: %d sectorsize: %d\n");
  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct usbhost_state_s *)inode->i_private;

  /* Check if the mass storage device is still connected */

  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any attempt to
       * write to the device.
       */

      ret = -ENODEV;
    }
  else
    {
      FAR struct usbmsc_cbw_s *cbw;

      usbhost_takesem(&priv->exclsem);

     /* Assume allocation failure */

      ret = -ENOMEM;

      /* Initialize a CBW (re-using the allocated transfer buffer) */

      cbw = usbhost_cbwalloc(priv);
      if (cbw)
        {
          /* Assume some device failure */

          ret = -ENODEV;

          /* Construct and send the CBW */

          usbhost_writecbw(startsector, priv->blocksize, nsectors, cbw);
          result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                                 (uint8_t*)cbw, USBMSC_CBW_SIZEOF);
          if (result == OK)
            {
              /* Send the user data */

              result = DRVR_TRANSFER(priv->drvr, priv->bulkout,
                                     (uint8_t*)buffer, priv->blocksize * nsectors);
              if (result == OK)
                {
                  /* Receive the CSW */

                  result = DRVR_TRANSFER(priv->drvr, priv->bulkin,
                                         priv->tbuffer, USBMSC_CSW_SIZEOF);
                  if (result == OK)
                    {
                      FAR struct usbmsc_csw_s *csw;

                      /* Check the CSW status */

                      csw = (FAR struct usbmsc_csw_s *)priv->tbuffer;
                      if (csw->status == 0)
                        {
                          ret = nsectors;
                        }
                    }
                }
            }
        }

      usbhost_givesem(&priv->exclsem);
    }

  /* On success, return the number of blocks written */

  return ret;
}
#endif

/****************************************************************************
 * Name: usbhost_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int usbhost_geometry(FAR struct inode *inode, struct geometry *geometry)
{
  FAR struct usbhost_state_s *priv;
  int ret = -EINVAL;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  /* Check if the mass storage device is still connected */

  priv = (FAR struct usbhost_state_s *)inode->i_private;
  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse to return any
       * geometry info.
       */

      ret = -ENODEV;
    }
  else if (geometry)
    {
      /* Return the geometry of the USB mass storage device */

      usbhost_takesem(&priv->exclsem);

      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
#ifdef CONFIG_FS_WRITABLE
      geometry->geo_writeenabled  = true;
#else
      geometry->geo_writeenabled  = false;
#endif
      geometry->geo_nsectors      = priv->nblocks;
      geometry->geo_sectorsize    = priv->blocksize;
      usbhost_givesem(&priv->exclsem);

      uvdbg("nsectors: %ld sectorsize: %d\n",
             (long)geometry->geo_nsectors, geometry->geo_sectorsize);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usbhost_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int usbhost_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct usbhost_state_s *priv;
  int ret;

  uvdbg("Entry\n");
  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct usbhost_state_s *)inode->i_private;

  /* Check if the mass storage device is still connected */

  if (priv->disconnected)
    {
      /* No... the block driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse to process any
       * ioctl commands.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Process the IOCTL by command */

      usbhost_takesem(&priv->exclsem);
      switch (cmd)
        {
        /* Add support for ioctl commands here */

        default:
          ret = -ENOTTY;
          break;
        }
      usbhost_givesem(&priv->exclsem);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_storageinit
 *
 * Description:
 *   Initialize the USB host storage class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_storageinit(void)
{
  /* If we have been configured to use pre-allocated storage class instances,
   * then place all of the pre-allocated USB host storage class instances
   * into a free list.
   */

#if CONFIG_USBHOST_NPREALLOC > 0
  int i;

  g_freelist = NULL;
  for (i = 0; i < CONFIG_USBHOST_NPREALLOC; i++)
    {
      struct usbhost_state_s *class = &g_prealloc[i];
      class->class.flink = g_freelist;
      g_freelist         = class;
    }
#endif

  /* Advertise our availability to support (certain) mass storage devices */

  return usbhost_registerclass(&g_storage);
}

#endif  /* CONFIG_USBHOST && !CONFIG_USBHOST_BULK_DISABLE && !CONFIG_DISABLE_MOUNTPOINT && CONFIG_NFILE_DESCRIPTORS > 0 */
