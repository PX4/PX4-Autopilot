/****************************************************************************
 * drivers/usbdev/usbmsc_scsi.c
 *
 *   Copyright (C) 2008-2010, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Mass storage class device.  Bulk-only with SCSI subclass.
 *
 * References:
 *   "Universal Serial Bus Mass Storage Class, Specification Overview,"
 *   Revision 1.2,  USB Implementer's Forum, June 23, 2003.
 *
 *   "Universal Serial Bus Mass Storage Class, Bulk-Only Transport,"
 *   Revision 1.0, USB Implementer's Forum, September 31, 1999.
 *
 *   "SCSI Primary Commands - 3 (SPC-3),"  American National Standard
 *   for Information Technology, May 4, 2005
 *
 *   "SCSI Primary Commands - 4 (SPC-4),"  American National Standard
 *   for Information Technology, July 19, 2008
 *
 *   "SCSI Block Commands -2 (SBC-2)," American National Standard
 *   for Information Technology, November 13, 2004
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
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/scsi.h>
#include <nuttx/usb/storage.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "usbmsc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Race condition workaround found by David Hewson.  This race condition
 * "seems to relate to stalling the endpoint when a short response is
 * generated which causes a residue to exist when the CSW would be returned.
 * I think there's two issues here.  The first being if the transfer which
 * had just been queued before the stall had not completed then it wouldn’t
 * then complete once the endpoint was stalled?  The second is that the
 * subsequent transfer for the CSW would be dropped on the floor (by the
 * epsubmit() function) if the end point was still stalled as the control
 * transfer to resume it hadn't occurred."
 */

#define CONFIG_USBMSC_RACEWAR 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Debug ********************************************************************/

#if defined(CONFIG_DEBUG_VERBOSE) && defined (CONFIG_DEBUG_USB)
static void     usbmsc_dumpdata(const char *msg, const uint8_t *buf,
                  int buflen);
#else
#  define usbmsc_dumpdata(msg, buf, len)
#endif

/* Utility Support Functions ************************************************/

static uint16_t usbmsc_getbe16(uint8_t *buf);
static uint32_t usbmsc_getbe32(uint8_t *buf);
static void     usbmsc_putbe16(uint8_t * buf, uint16_t val);
static void     usbmsc_putbe24(uint8_t *buf, uint32_t val);
static void     usbmsc_putbe32(uint8_t *buf, uint32_t val);
#if 0 /* not used */
static uint16_t usbmsc_getle16(uint8_t *buf);
#endif
static uint32_t usbmsc_getle32(uint8_t *buf);
#if 0 /* not used */
static void     usbmsc_putle16(uint8_t * buf, uint16_t val);
#endif
static void     usbmsc_putle32(uint8_t *buf, uint32_t val);

/* SCSI Command Processing **************************************************/

static inline int usbmsc_cmdtestunitready(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdrequestsense(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf);
static inline int usbmsc_cmdread6(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdwrite6(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdinquiry(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf);
static inline int usbmsc_cmdmodeselect6(FAR struct usbmsc_dev_s *priv);
static int    usbmsc_modepage(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf, uint8_t pcpgcode, int *mdlen);
static inline int usbmsc_cmdmodesense6(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf);
static inline int usbmsc_cmdstartstopunit(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdpreventmediumremoval(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdreadformatcapacity(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf);
static inline int usbmsc_cmdreadcapacity10(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf);
static inline int usbmsc_cmdread10(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdwrite10(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdverify10(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdsynchronizecache10(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdmodeselect10(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdmodesense10(FAR struct usbmsc_dev_s *priv,
                FAR uint8_t *buf);
static inline int usbmsc_cmdread12(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_cmdwrite12(FAR struct usbmsc_dev_s *priv);
static inline int usbmsc_setupcmd(FAR struct usbmsc_dev_s *priv,
                uint8_t cdblen, uint8_t flags);

/* SCSI Worker Thread *******************************************************/

static int    usbmsc_idlestate(FAR struct usbmsc_dev_s *priv);
static int    usbmsc_cmdparsestate(FAR struct usbmsc_dev_s *priv);
static int    usbmsc_cmdreadstate(FAR struct usbmsc_dev_s *priv);
static int    usbmsc_cmdwritestate(FAR struct usbmsc_dev_s *priv);
static int    usbmsc_cmdfinishstate(FAR struct usbmsc_dev_s *priv);
static int    usbmsc_cmdstatusstate(FAR struct usbmsc_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Debug
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_dumpdata
 ****************************************************************************/

#if defined(CONFIG_DEBUG_VERBOSE) && defined (CONFIG_DEBUG_USB)
static void usbmsc_dumpdata(const char *msg, const uint8_t *buf, int buflen)
{
  int i;

  dbgprintf("%s:", msg);
  for (i = 0; i < buflen; i++)
    {
      dbgprintf(" %02x", buf[i]);
    }
  dbgprintf("\n");
}
#endif

/****************************************************************************
 * Utility Support Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_getbe16
 *
 * Description:
 *   Get a 16-bit big-endian value reference by the byte pointer
 *
 ****************************************************************************/

static uint16_t usbmsc_getbe16(uint8_t *buf)
{
  return ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
}

/****************************************************************************
 * Name: usbmsc_getbe32
 *
 * Description:
 *   Get a 32-bit big-endian value reference by the byte pointer
 *
 ****************************************************************************/

static uint32_t usbmsc_getbe32(uint8_t *buf)
{
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3]);
}

/****************************************************************************
 * Name: usbmsc_putbe16
 *
 * Description:
 *   Store a 16-bit value in big-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmsc_putbe16(uint8_t * buf, uint16_t val)
{
  buf[0] = val >> 8;
  buf[1] = val;
}

/****************************************************************************
 * Name: usbmsc_putbe24
 *
 * Description:
 *   Store a 32-bit value in big-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmsc_putbe24(uint8_t *buf, uint32_t val)
{
  buf[0] = val >> 16;
  buf[1] = val >> 8;
  buf[2] = val;
}

/****************************************************************************
 * Name: usbmsc_putbe32
 *
 * Description:
 *   Store a 32-bit value in big-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmsc_putbe32(uint8_t *buf, uint32_t val)
{
  buf[0] = val >> 24;
  buf[1] = val >> 16;
  buf[2] = val >> 8;
  buf[3] = val;
}

/****************************************************************************
 * Name: usbmsc_getle16
 *
 * Description:
 *   Get a 16-bit little-endian value reference by the byte pointer
 *
 ****************************************************************************/

#if 0 /* not used */
static uint16_t usbmsc_getle16(uint8_t *buf)
{
  return ((uint16_t)buf[1] << 8) | ((uint16_t)buf[0]);
}
#endif

/****************************************************************************
 * Name: usbmsc_getle32
 *
 * Description:
 *   Get a 32-bit little-endian value reference by the byte pointer
 *
 ****************************************************************************/

static uint32_t usbmsc_getle32(uint8_t *buf)
{
  return ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[1] << 8) | ((uint32_t)buf[0]);
}

/****************************************************************************
 * Name: usbmsc_putle16
 *
 * Description:
 *   Store a 16-bit value in little-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

#if 0 /* not used */
static void usbmsc_putle16(uint8_t * buf, uint16_t val)
{
  buf[0] = val;
  buf[1] = val >> 8;
}
#endif

/****************************************************************************
 * Name: usbmsc_putle32
 *
 * Description:
 *   Store a 32-bit value in little-endian order to the location specified by
 *   a byte pointer
 *
 ****************************************************************************/

static void usbmsc_putle32(uint8_t *buf, uint32_t val)
{
  buf[0] = val;
  buf[1] = val >> 8;
  buf[2] = val >> 16;
  buf[3] = val >> 24;
}

/****************************************************************************
 * SCSI Worker Thread
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_cmdtestunitready
 *
 * Description:
 *  Handle the SCSI_CMD_TESTUNITREADY command
 *
 ****************************************************************************/

static inline int usbmsc_cmdtestunitready(FAR struct usbmsc_dev_s *priv)
{
  int ret;

  priv->u.alloclen = 0;
  ret = usbmsc_setupcmd(priv, 6, USBMSC_FLAGS_DIRNONE);
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdrequestsense
 *
 * Description:
 *  Handle the SCSI_CMD_REQUESTSENSE command
 *
 ****************************************************************************/

static inline int usbmsc_cmdrequestsense(FAR struct usbmsc_dev_s *priv,
                                         FAR uint8_t *buf)
{
  FAR struct scsicmd_requestsense_s *request = (FAR struct scsicmd_requestsense_s *)priv->cdb;
  FAR struct scsiresp_fixedsensedata_s *response = (FAR struct scsiresp_fixedsensedata_s *)buf;
  FAR struct usbmsc_lun_s *lun;
  uint32_t sd;
  uint32_t sdinfo;
  uint8_t  cdblen;
  int      ret;

  /* Extract the host allocation length */

  priv->u.alloclen = request->alloclen;

  /* Get the expected length of the command (with hack for MS-Windows 12-byte
   * REQUEST SENSE command.
   */

  cdblen = SCSICMD_REQUESTSENSE_SIZEOF;
  if (cdblen != priv->cdblen)
    {
      /* Try MS-Windows REQUEST SENSE with cbw->cdblen == 12 */

      cdblen  = SCSICMD_REQUESTSENSE_MSSIZEOF;
    }

  ret = usbmsc_setupcmd(priv, cdblen,
                        USBMSC_FLAGS_DIRDEVICE2HOST|USBMSC_FLAGS_LUNNOTNEEDED|
                        USBMSC_FLAGS_UACOKAY|USBMSC_FLAGS_RETAINSENSEDATA);
  if (ret == OK)
    {
      lun = priv->lun;
      if (!lun)
        {
          sd     = SCSI_KCQIR_INVALIDLUN;
          sdinfo = 0;
        }
      else
        {
          /* Get the saved sense data from the LUN structure */

          sd     = lun->sd;
          sdinfo = lun->sdinfo;

          /* Discard the sense data */

          lun->sd = SCSI_KCQ_NOSENSE;
          lun->sdinfo = 0;
        }

      /* Create the fixed sense data response */

      memset(response, 0, SCSIRESP_FIXEDSENSEDATA_SIZEOF);

      response->code  = SCSIRESP_SENSEDATA_RESPVALID|SCSIRESP_SENSEDATA_CURRENTFIXED;
      response->flags = (uint8_t)(sd >> 16);
      usbmsc_putbe32(response->info, sdinfo);
      response->len   = SCSIRESP_FIXEDSENSEDATA_SIZEOF - 7;
      response->code2 = (uint8_t)(sd >> 8);
      response->qual2 = (uint8_t)sd;

      priv->nreqbytes = SCSIRESP_FIXEDSENSEDATA_SIZEOF;
      ret             = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdread6
 *
 * Description:
 *  Handle the SCSI_CMD_READ6 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdread6(FAR struct usbmsc_dev_s *priv)
{
  FAR struct scsicmd_read6_s *read6 = (FAR struct scsicmd_read6_s*)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.xfrlen = (uint16_t)read6->xfrlen;
  if (priv->u.xfrlen == 0)
    {
      priv->u.xfrlen = 256;
    }

  ret = usbmsc_setupcmd(priv, SCSICMD_READ6_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST|USBMSC_FLAGS_BLOCKXFR);
  if (ret == OK)
    {
      /* Get the Logical Block Address (LBA) from cdb[] as the starting sector */

      priv->sector = (uint32_t)(read6->mslba & SCSICMD_READ6_MSLBAMASK) << 16 | (uint32_t)usbmsc_getbe16(read6->lslba);

      /* Verify that a block driver has been bound to the LUN */

      if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ6MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret     = -EINVAL;
        }

      /* Verify that sector lies in the range supported by the block driver */

      else if (priv->sector >= lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ6LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
          ret     = -EINVAL;
        }

      /* Looks like we are good to go */

      else
        {
          usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDREAD6), priv->cdb[0]);
          priv->thstate = USBMSC_STATE_CMDREAD;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdwrite6
 *
 * Description:
 *  Handle the SCSI_CMD_WRITE6 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdwrite6(FAR struct usbmsc_dev_s *priv)
{
  FAR struct scsicmd_write6_s *write6 = (FAR struct scsicmd_write6_s *)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.xfrlen = (uint16_t)write6->xfrlen;
  if (priv->u.xfrlen == 0)
    {
      priv->u.xfrlen = 256;
    }

  ret = usbmsc_setupcmd(priv, SCSICMD_WRITE6_SIZEOF, USBMSC_FLAGS_DIRHOST2DEVICE|USBMSC_FLAGS_BLOCKXFR);
  if (ret == OK)
    {
      /* Get the Logical Block Address (LBA) from cdb[] as the starting sector */

      priv->sector = (uint32_t)(write6->mslba & SCSICMD_WRITE6_MSLBAMASK) << 16 | (uint32_t)usbmsc_getbe16(write6->lslba);

      /* Verify that a block driver has been bound to the LUN */

      if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE6MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret     = -EINVAL;
        }

      /* Check for attempts to write to a read-only device */

      else if (lun->readonly)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE6READONLY), 0);
          lun->sd = SCSI_KCQWP_COMMANDNOTALLOWED;
          ret     = -EINVAL;
        }

      /* Verify that sector lies in the range supported by the block driver */

      else if (priv->sector >= lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE6LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
          ret     = -EINVAL;
        }

      /* Looks like we are good to go */

      else
        {
          usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDWRITE6), priv->cdb[0]);
          priv->thstate = USBMSC_STATE_CMDWRITE;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdinquiry
 *
 * Description:
 *   Handle SCSI_CMD_INQUIRY command
 *
 ****************************************************************************/

static inline int usbmsc_cmdinquiry(FAR struct usbmsc_dev_s *priv,
                                    FAR uint8_t *buf)
{
  FAR struct scscicmd_inquiry_s *inquiry = (FAR struct scscicmd_inquiry_s *)priv->cdb;
  FAR struct scsiresp_inquiry_s *response = (FAR struct scsiresp_inquiry_s *)buf;
  int len;
  int ret;

  priv->u.alloclen = usbmsc_getbe16(inquiry->alloclen);
  ret = usbmsc_setupcmd(priv, SCSICMD_INQUIRY_SIZEOF,
                         USBMSC_FLAGS_DIRDEVICE2HOST|USBMSC_FLAGS_LUNNOTNEEDED|USBMSC_FLAGS_UACOKAY);
  if (ret == OK)
    {
      if (!priv->lun)
        {
          response->qualtype = SCSIRESP_INQUIRYPQ_NOTCAPABLE|SCSIRESP_INQUIRYPD_UNKNOWN;
         }
      else if ((inquiry->flags != 0) || (inquiry->pagecode != 0))
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INQUIRYFLAGS), 0);
          priv->lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret = -EINVAL;
        }
      else
        {
          memset(response, 0, SCSIRESP_INQUIRY_SIZEOF);
          priv->nreqbytes = SCSIRESP_INQUIRY_SIZEOF;

#ifdef CONFIG_USBMSC_REMOVABLE
          response->flags1   = SCSIRESP_INQUIRYFLAGS1_RMB;
#endif
          response->version  = 2; /* SCSI-2 */
          response->flags2   = 2; /* SCSI-2 INQUIRY response data format */
          response->len      = SCSIRESP_INQUIRY_SIZEOF - 5; 

          /* Strings */

          memset(response->vendorid, ' ', 8+16+4);

          len = strlen(g_mscvendorstr);
          DEBUGASSERT(len <= 8);
          memcpy(response->vendorid, g_mscvendorstr, len);

          len = strlen(g_mscproductstr);
          DEBUGASSERT(len <= 16);
          memcpy(response->productid, g_mscproductstr, len);

          len = strlen(g_mscserialstr);
          DEBUGASSERT(len <= 4);
          memcpy(response->revision, g_mscserialstr, len);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdmodeselect6
 *
 * Description:
 *   Handle SCSI_CMD_MODESELECT6 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdmodeselect6(FAR struct usbmsc_dev_s *priv)
{
  FAR struct scsicmd_modeselect6_s *modeselect = (FAR struct scsicmd_modeselect6_s *)priv->cdb;

  priv->u.alloclen = modeselect->plen;
  (void)usbmsc_setupcmd(priv, SCSICMD_MODESELECT6_SIZEOF, USBMSC_FLAGS_DIRHOST2DEVICE);

  /* Not supported */

  priv->lun->sd = SCSI_KCQIR_INVALIDCOMMAND;
  return -EINVAL;
}

/****************************************************************************
 * Name: usbmsc_modepage
 *
 * Description:
 *   Common logic for usbmsc_cmdmodesense6() and usbmsc_cmdmodesense10()
 *
 ****************************************************************************/

static int usbmsc_modepage(FAR struct usbmsc_dev_s *priv, FAR uint8_t *buf,
                           uint8_t pcpgcode, int *mdlen)
{
  FAR struct scsiresp_cachingmodepage_s *cmp = (FAR struct scsiresp_cachingmodepage_s *)buf;

  /* Saving parms not supported */

  if ((pcpgcode & SCSICMD_MODESENSE_PCMASK) == SCSICMD_MODESENSE_PCSAVED)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PCSAVED), 0);
      priv->lun->sd = SCSI_KCQIR_SAVINGPARMSNOTSUPPORTED;
      return -EINVAL;
    }

  /* Only the caching mode page is supported: */

  if ((pcpgcode & SCSICMD_MODESENSE_PGCODEMASK) == SCSIRESP_MODESENSE_PGCCODE_CACHING ||
      (pcpgcode & SCSICMD_MODESENSE_PGCODEMASK) == SCSIRESP_MODESENSE_PGCCODE_RETURNALL)
    {
      memset(cmp, 0, 12);
      cmp->pgcode = SCSIRESP_MODESENSE_PGCCODE_CACHING;
      cmp->len    = 10; /* n-2 */

      /* None of the fields are changeable */

      if (((pcpgcode & SCSICMD_MODESENSE_PCMASK) != SCSICMD_MODESENSE_PCCHANGEABLE))
        {
          cmp->flags1    = SCSIRESP_CACHINGMODEPG_WCE; /* Write cache enable */
          cmp->dpflen[0] = 0xff;  /* Disable prefetch transfer length = 0xffffffff */
          cmp->dpflen[1] = 0xff;
          cmp->maxpf[0]  = 0xff;  /* Maximum pre-fetch  = 0xffffffff */
          cmp->maxpf[1]  = 0xff;
          cmp->maxpfc[0] = 0xff;  /* Maximum pref-fetch ceiling  = 0xffffffff */
          cmp->maxpfc[1] = 0xff;
        }

       /* Return the mode data length */

      *mdlen = 12; /* Only the first 12-bytes of caching mode page sent */
      return OK;
    }
  else
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_MODEPAGEFLAGS), pcpgcode);
      priv->lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: usbmsc_cmdmodesense6
 *
 * Description:
 *   Handle SCSI_CMD_MODESENSE6 command
 *
 ****************************************************************************/

static int inline usbmsc_cmdmodesense6(FAR struct usbmsc_dev_s *priv,
                                       FAR uint8_t *buf)
{
  FAR struct scsicmd_modesense6_s *modesense = (FAR struct scsicmd_modesense6_s *)priv->cdb;
  FAR struct scsiresp_modeparameterhdr6_s *mph = (FAR struct scsiresp_modeparameterhdr6_s *)buf;
  int mdlen;
  int ret;

  priv->u.alloclen = modesense->alloclen;
  ret = usbmsc_setupcmd(priv, SCSICMD_MODESENSE6_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST);
  if (ret == OK)
    {
      if ((modesense->flags & ~SCSICMD_MODESENSE6_DBD) != 0 || modesense->subpgcode != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_MODESENSE6FLAGS), 0);
          priv->lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret = -EINVAL;
        }
      else
        {
          /* The response consists of:
           *
           * (1) A MODESENSE6-specific mode parameter header,
           * (2) A variable length list of block descriptors, and
           * (3) A variable length list of mode page formats
           */

          mph->type  = 0; /* Medium type */
          mph->param = (priv->lun->readonly ? SCSIRESP_MODEPARMHDR_DAPARM_WP : 0x00);
          mph->bdlen = 0; /* Block descriptor length */

          /* There are no block descriptors, only the following mode page: */

          ret = usbmsc_modepage(priv, &buf[SCSIRESP_MODEPARAMETERHDR6_SIZEOF], modesense->pcpgcode, &mdlen);
          if (ret == OK)
            {
               /* Store the mode data length and return the total message size */

               mph->mdlen      = mdlen - 1;
               priv->nreqbytes = mdlen + SCSIRESP_MODEPARAMETERHDR6_SIZEOF;
            }
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdstartstopunit
 *
 * Description:
 *   Handle SCSI_CMD_STARTSTOPUNIT command
 *
 ****************************************************************************/

static inline int usbmsc_cmdstartstopunit(FAR struct usbmsc_dev_s *priv)
{
  int ret;

  priv->u.alloclen = 0;
  ret = usbmsc_setupcmd(priv, SCSICMD_STARTSTOPUNIT_SIZEOF, USBMSC_FLAGS_DIRNONE);
  if (ret == OK)
    {
#ifndef CONFIG_USBMSC_REMOVABLE
      /* This command is not valid if the media is not removable */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_NOTREMOVABLE), 0);
      lun->sd = SCSI_KCQIR_INVALIDCOMMAND;
      ret = -EINVAL;
#endif
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdpreventmediumremoval
 *
 * Description:
 *   Handle SCSI_CMD_PREVENTMEDIAREMOVAL command
 *
 ****************************************************************************/

static inline int usbmsc_cmdpreventmediumremoval(FAR struct usbmsc_dev_s *priv)
{
#ifdef CONFIG_USBMSC_REMOVABLE
  FAR struct scsicmd_preventmediumremoval_s *pmr = (FAR struct scsicmd_preventmediumremoval_s *)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
#endif
  int ret;

  priv->u.alloclen = 0;
  ret = usbmsc_setupcmd(priv, SCSICMD_PREVENTMEDIUMREMOVAL_SIZEOF, USBMSC_FLAGS_DIRNONE);
  if (ret == OK)
    {
#ifndef CONFIG_USBMSC_REMOVABLE
      lun->sd = SCSI_KCQIR_INVALIDCOMMAND;
      ret = -EINVAL;
#else
      if ((pmr->prevent & ~SCSICMD_PREVENTMEDIUMREMOVAL_TRANSPORT) != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PREVENTMEDIUMREMOVALPREVENT), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret = -EINVAL;
        }

      lun->locked = pmr->prevent & SCSICMD_PREVENTMEDIUMREMOVAL_TRANSPORT;
#endif
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdreadformatcapacity
 *
 * Description:
 *   Handle SCSI_CMD_READFORMATCAPACITIES command (MMC)
 *
 ****************************************************************************/

static inline int usbmsc_cmdreadformatcapacity(FAR struct usbmsc_dev_s *priv,
                                               FAR uint8_t *buf)
{
  FAR struct scsicmd_readformatcapcacities_s *rfc = (FAR struct scsicmd_readformatcapcacities_s *)priv->cdb;
  FAR struct scsiresp_readformatcapacities_s *hdr;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.alloclen = usbmsc_getbe16(rfc->alloclen);
  ret = usbmsc_setupcmd(priv, SCSICMD_READFORMATCAPACITIES_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST);
  if (ret == OK)
    {
      hdr = (FAR struct scsiresp_readformatcapacities_s *)buf;
      memset(hdr, 0, SCSIRESP_READFORMATCAPACITIES_SIZEOF);
      hdr->listlen = SCSIRESP_CURRCAPACITYDESC_SIZEOF;

      /* Only the Current/Maximum Capacity Descriptor follows the header */

      usbmsc_putbe32(hdr->nblocks, lun->nsectors);
      hdr->type = SCIRESP_RDFMTCAPACITIES_FORMATED;
      usbmsc_putbe24(hdr->blocklen, lun->sectorsize);
      priv->nreqbytes = SCSIRESP_READFORMATCAPACITIES_SIZEOF;
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdreadcapacity10
 *
 * Description:
 *   Handle SCSI_CMD_READCAPACITY10 command
 *
 ****************************************************************************/

static int inline usbmsc_cmdreadcapacity10(FAR struct usbmsc_dev_s *priv,
                                           FAR uint8_t *buf)
{
  FAR struct scsicmd_readcapacity10_s *rcc = (FAR struct scsicmd_readcapacity10_s *)priv->cdb;
  FAR struct scsiresp_readcapacity10_s *rcr = (FAR struct scsiresp_readcapacity10_s *)buf;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  uint32_t lba;
  int ret;

  priv->u.alloclen = SCSIRESP_READCAPACITY10_SIZEOF; /* Fake the allocation length */
  ret = usbmsc_setupcmd(priv, SCSICMD_READCAPACITY10_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST);
  if (ret == OK)
    {
      /* Check the PMI and LBA fields */

      lba = usbmsc_getbe32(rcc->lba);

      if (rcc->pmi > 1 || (rcc->pmi == 0 && lba != 0))
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READCAPACITYFLAGS), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret = -EINVAL;
        }
      else
        {
          usbmsc_putbe32(rcr->lba, lun->nsectors - 1);
          usbmsc_putbe32(&buf[4], lun->sectorsize);
          priv->nreqbytes = SCSIRESP_READCAPACITY10_SIZEOF;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdread10
 *
 * Description:
 *   Handle SCSI_CMD_READ10 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdread10(FAR struct usbmsc_dev_s *priv)
{
  struct scsicmd_read10_s *read10 = (struct scsicmd_read10_s*)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.xfrlen = usbmsc_getbe16(read10->xfrlen);
  ret = usbmsc_setupcmd(priv, SCSICMD_READ10_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST|USBMSC_FLAGS_BLOCKXFR);
  if (ret == OK)
    {
      /* Get the Logical Block Address (LBA) from cdb[] as the starting sector */

      priv->sector = usbmsc_getbe32(read10->lba);

      /* Verify that we can support this read command */

      if ((read10->flags & ~(SCSICMD_READ10FLAGS_DPO|SCSICMD_READ10FLAGS_FUA)) != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ10FLAGS), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret     = -EINVAL;
        }

      /* Verify that a block driver has been bound to the LUN */

      else if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ10MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret     = -EINVAL;
        }

      /* Verify that LBA lies in the range supported by the block driver */

      else if (priv->sector >= lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ10LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
          ret     = -EINVAL;
       }

      /* Looks like we are good to go */

      else
        {
          usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDREAD10), priv->cdb[0]);
          priv->thstate = USBMSC_STATE_CMDREAD;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdwrite10
 *
 * Description:
 *   Handle SCSI_CMD_WRITE10 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdwrite10(FAR struct usbmsc_dev_s *priv)
{
  struct scsicmd_write10_s *write10 = (struct scsicmd_write10_s *)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.xfrlen = usbmsc_getbe16(write10->xfrlen);
  ret = usbmsc_setupcmd(priv, SCSICMD_WRITE10_SIZEOF, USBMSC_FLAGS_DIRHOST2DEVICE|USBMSC_FLAGS_BLOCKXFR);
  if (ret == OK)
    {
      /* Get the Logical Block Address (LBA) from cdb[] as the starting sector */

      priv->sector = usbmsc_getbe32(write10->lba);

      /* Verify that we can support this write command */

      if ((write10->flags & ~(SCSICMD_WRITE10FLAGS_DPO|SCSICMD_WRITE10FLAGS_FUA)) != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE10FLAGS), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret     = -EINVAL;
        }

      /* Verify that a block driver has been bound to the LUN */

      else if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE10MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret     = -EINVAL;
        }

      /* Check for attempts to write to a read-only device */

      else if (lun->readonly)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE10READONLY), 0);
          lun->sd = SCSI_KCQWP_COMMANDNOTALLOWED;
          ret     = -EINVAL;
        }

     /* Verify that LBA lies in the range supported by the block driver */

     else if (priv->sector >= lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE10LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
          ret     = -EINVAL;
       }

      /* Looks like we are good to go */

      else
        {
          usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDWRITE10), priv->cdb[0]);
          priv->thstate = USBMSC_STATE_CMDWRITE;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdverify10
 *
 * Description:
 *   Handle SCSI_CMD_VERIFY10 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdverify10(FAR struct usbmsc_dev_s *priv)
{
  FAR struct scsicmd_verify10_s *verf = (FAR struct scsicmd_verify10_s *)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  uint32_t  lba;
  uint16_t  blocks;
  size_t  sector;
  ssize_t nread;
  int     ret;
  int     i;

  priv->u.alloclen = 0;
  ret = usbmsc_setupcmd(priv, SCSICMD_VERIFY10_SIZEOF, USBMSC_FLAGS_DIRNONE);
  if (ret == OK)
    {
      /* Verify the starting and ending LBA */

      lba    = usbmsc_getbe32(verf->lba);
      blocks = usbmsc_getbe16(verf->len);

      if ((verf->flags & ~SCSICMD_VERIFY10_DPO) != 0 || verf->groupno != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_VERIFY10FLAGS), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret     = -EINVAL;
        }
      else if (blocks == 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_VERIFY10NOBLOCKS), 0);
          ret = -EIO; /* No reply */
        }

      /* Verify that a block driver has been bound to the LUN */

      else if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_VERIFY10MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret = -EINVAL;
        }

      /* Verify that LBA lies in the range supported by the block driver */

      else if (lba >= lun->nsectors || lba + blocks > lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_VERIFY10LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
          ret     = -EINVAL;
        }
      else
        {
          /* Try to read the requested blocks */

          for (i = 0, sector = lba + lun->startsector; i < blocks; i++, sector++)
            {
              nread = USBMSC_DRVR_READ(lun, priv->iobuffer, sector, 1);
              if (nread < 0)
                {
                  usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_VERIFY10READFAIL), i);
                  lun->sd     = SCSI_KCQME_UNRRE1;
                  lun->sdinfo = sector;
                  ret         = -EIO;
                  break;
                }
            }
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdsynchronizecache10
 *
 * Description:
 *   Handle SCSI_CMD_SYNCHCACHE10 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdsynchronizecache10(FAR struct usbmsc_dev_s *priv)
{
  int ret;

  priv->u.alloclen = 0;

  /* Verify that we have the LUN structure and the block driver has been bound */

  if (!priv->lun->inode)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SYNCCACHEMEDIANOTPRESENT), 0);
      priv->lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
      ret = -EINVAL;
    }
  else
    {
      ret = usbmsc_setupcmd(priv, SCSICMD_SYNCHRONIZECACHE10_SIZEOF, USBMSC_FLAGS_DIRNONE);
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdmodeselect10
 *
 * Description:
 *   Handle SCSI_CMD_MODESELECT10 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdmodeselect10(FAR struct usbmsc_dev_s *priv)
{
  FAR struct scsicmd_modeselect10_s *modeselect = (FAR struct scsicmd_modeselect10_s *)priv->cdb;

  priv->u.alloclen = usbmsc_getbe16(modeselect->parmlen);
  (void)usbmsc_setupcmd(priv, SCSICMD_MODESELECT10_SIZEOF, USBMSC_FLAGS_DIRHOST2DEVICE);

  /* Not supported */

  priv->lun->sd = SCSI_KCQIR_INVALIDCOMMAND;
  return -EINVAL;
}

/****************************************************************************
 * Name: usbmsc_cmdmodesense10
 *
 * Description:
 *   Handle SCSI_CMD_MODESENSE10 command
 *
 ****************************************************************************/

static int inline usbmsc_cmdmodesense10(FAR struct usbmsc_dev_s *priv,
                                        FAR uint8_t *buf)
{
  FAR struct scsicmd_modesense10_s *modesense = (FAR struct scsicmd_modesense10_s *)priv->cdb;
  FAR struct scsiresp_modeparameterhdr10_s *mph = (FAR struct scsiresp_modeparameterhdr10_s *)buf;
  int mdlen;
  int ret;

  priv->u.alloclen = usbmsc_getbe16(modesense->alloclen);
  ret = usbmsc_setupcmd(priv, SCSICMD_MODESENSE10_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST);
  if (ret == OK)
    {
      if ((modesense->flags & ~SCSICMD_MODESENSE10_DBD) != 0 || modesense->subpgcode != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_MODESENSE10FLAGS), 0);
          priv->lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret = -EINVAL;
        }
      else
        {
          /* The response consists of:
           *
           * (1) A MODESENSE6-specific mode parameter header,
           * (2) A variable length list of block descriptors, and
           * (3) A variable lengtth list of mode page formats
           */

          memset(mph, 0, SCSIRESP_MODEPARAMETERHDR10_SIZEOF);
          mph->param = (priv->lun->readonly ? SCSIRESP_MODEPARMHDR_DAPARM_WP : 0x00);

          /* There are no block descriptors, only the following mode page: */

          ret = usbmsc_modepage(priv, &buf[SCSIRESP_MODEPARAMETERHDR10_SIZEOF], modesense->pcpgcode, &mdlen);
          if (ret == OK)
            {
               /* Store the mode data length and return the total message size */

              usbmsc_putbe16(mph->mdlen, mdlen - 2);
              priv->nreqbytes = mdlen + SCSIRESP_MODEPARAMETERHDR10_SIZEOF;
            }
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdread12
 *
 * Description:
 *   Handle SCSI_CMD_READ12 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdread12(FAR struct usbmsc_dev_s *priv)
{
  struct scsicmd_read12_s *read12 = (struct scsicmd_read12_s*)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.xfrlen = usbmsc_getbe32(read12->xfrlen);
  ret = usbmsc_setupcmd(priv, SCSICMD_READ12_SIZEOF, USBMSC_FLAGS_DIRDEVICE2HOST|USBMSC_FLAGS_BLOCKXFR);
  if (ret == OK)
    {
      /* Get the Logical Block Address (LBA) from cdb[] as the starting sector */

      priv->sector = usbmsc_getbe32(read12->lba);

      /* Verify that we can support this read command */

      if ((read12->flags & ~(SCSICMD_READ12FLAGS_DPO|SCSICMD_READ12FLAGS_FUA)) != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ12FLAGS), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
          ret     = -EINVAL;
        }

      /* Verify that a block driver has been bound to the LUN */

      else if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ12MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret     = -EINVAL;
        }

      /* Verify that LBA lies in the range supported by the block driver */

      else if (priv->sector >= lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_READ12LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
          ret     = -EINVAL;
        }

      /* Looks like we are good to go */

      else
        {
          usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDREAD12), priv->cdb[0]);
          priv->thstate = USBMSC_STATE_CMDREAD;
        }
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdwrite12
 *
 * Description:
 *   Handle SCSI_CMD_WRITE12 command
 *
 ****************************************************************************/

static inline int usbmsc_cmdwrite12(FAR struct usbmsc_dev_s *priv)
{
  struct scsicmd_write12_s *write12 = (struct scsicmd_write12_s *)priv->cdb;
  FAR struct usbmsc_lun_s *lun = priv->lun;
  int ret;

  priv->u.xfrlen = usbmsc_getbe32(write12->xfrlen);
  ret = usbmsc_setupcmd(priv, SCSICMD_WRITE12_SIZEOF, USBMSC_FLAGS_DIRHOST2DEVICE|USBMSC_FLAGS_BLOCKXFR);
  if (ret == OK)
    {
      /* Get the Logical Block Address (LBA) from cdb[] as the starting sector */

      priv->sector = usbmsc_getbe32(write12->lba);

      /* Verify that we can support this write command */

      if ((write12->flags & ~(SCSICMD_WRITE12FLAGS_DPO|SCSICMD_WRITE12FLAGS_FUA)) != 0)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE12FLAGS), 0);
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
        }

      /* Verify that a block driver has been bound to the LUN */

      else if (!lun->inode)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE12MEDIANOTPRESENT), 0);
          lun->sd = SCSI_KCQNR_MEDIANOTPRESENT;
          ret = -EINVAL;
        }

      /* Check for attempts to write to a read-only device */

      else if (lun->readonly)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE12READONLY), 0);
          lun->sd = SCSI_KCQWP_COMMANDNOTALLOWED;
        }

      /* Verify that LBA lies in the range supported by the block driver */

      else if (priv->sector >= lun->nsectors)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_WRITE12LBARANGE), 0);
          lun->sd = SCSI_KCQIR_LBAOUTOFRANGE;
        }

      /* Looks like we are good to go */

      else
        {
          usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDWRITE12), priv->cdb[0]);
          priv->thstate = USBMSC_STATE_CMDWRITE;
          return OK;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: usbmsc_setupcmd
 *
 * Description:
 *   Called after each SCSI command is identified in order to perform setup
 *   and verification operations that are common to all SCSI commands.  This
 *   function performs the following common setup operations:
 *
 *   1. Determine the direction of the response
 *   2. Verify lengths
 *   3. Setup and verify the LUN
 *
 *   Includes special logic for INQUIRY and REQUESTSENSE commands
 *
 ****************************************************************************/

static int inline usbmsc_setupcmd(FAR struct usbmsc_dev_s *priv, uint8_t cdblen, uint8_t flags)
{
  FAR struct usbmsc_lun_s *lun = NULL;
  uint32_t datlen;
  uint8_t  dir = flags & USBMSC_FLAGS_DIRMASK;
  int    ret = OK;

  /* Verify the LUN and set up the current LUN reference in the
   * device structure
   */

  if (priv->cbwlun < priv->nluns)
    {
      /* LUN number is valid in a valid range, but the LUN is not necessarily
       * bound to a block driver.  That will be checked as necessary in each
       * individual command.
       */

      lun       = &priv->luntab[priv->cbwlun];
      priv->lun = lun;
    }

  /* Only a few commands may specify unsupported LUNs */

  else if ((flags & USBMSC_FLAGS_LUNNOTNEEDED) == 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDBADLUN), priv->cbwlun);
      ret = -EINVAL;
    }

  /* Extract the direction and data transfer length */

  dir    = flags & USBMSC_FLAGS_DIRMASK;      /* Expected data direction */
  datlen = 0;
  if ((flags & USBMSC_FLAGS_BLOCKXFR) == 0)
    {
      /* Non-block transfer.  Data length: Host allocation to receive data
       * (only for device-to-host transfers.  At present, alloclen is set
       * to zero for all host-to-device, non-block transfers.
       */

      datlen = priv->u.alloclen;
    }
  else if (lun)
    {
      /* Block transfer: Calculate the total size of all sectors to be transferred */

      datlen = priv->u.alloclen * lun->sectorsize;
    }

  /* Check the data direction.  This was set up at the end of the
   * IDLE state when the CBW was parsed, but if there is no data,
   * then the direction is none.
   */

  if (datlen == 0)
    {
      /* No data.. then direction is none */

      dir = USBMSC_FLAGS_DIRNONE;
    }

  /* Compare the expected data length in the command to the data length
   * in the CBW.
   */

  else if (priv->cbwlen < datlen)
    {
      /* Clip to the length in the CBW and declare a phase error */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PHASEERROR1), priv->cdb[0]);
      if ((flags & USBMSC_FLAGS_BLOCKXFR) != 0)
        {
          priv->u.alloclen = priv->cbwlen;
        }
      else
        {
          priv->u.xfrlen = priv->cbwlen / lun->sectorsize;
        }

      priv->phaseerror = 1;
    }

  /* Initialize the residue */

  priv->residue = priv->cbwlen;

  /* Conflicting data directions is a phase error */

  if (priv->cbwdir != dir && datlen > 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PHASEERROR2), priv->cdb[0]);
      priv->phaseerror = 1;
      ret             = -EINVAL;
    }

  /* Compare the length of data in the cdb[] with the expected length
   * of the command.
   */

  if (cdblen != priv->cdblen)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_PHASEERROR3), priv->cdb[0]);
      priv->phaseerror = 1;
      ret              = -EINVAL;
    }

  if (lun)
    {
      /* Retain the sense data for the REQUEST SENSE command */

      if ((flags & USBMSC_FLAGS_RETAINSENSEDATA) == 0)
        {
          /* Discard the sense data */

          lun->sd     = SCSI_KCQ_NOSENSE;
          lun->sdinfo = 0;
        }

      /* If a unit attention condition exists, then only a restricted set of
       * commands is permitted.
       */

      if (lun->uad != SCSI_KCQ_NOSENSE && (flags & USBMSC_FLAGS_UACOKAY) != 0)
        {
          /* Command not permitted */

          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDUNEVIOLATION), priv->cbwlun);
          lun->sd  = lun->uad;
          lun->uad = SCSI_KCQ_NOSENSE;
          ret      = -EINVAL;
        }
    }

  /* The final, 1-byte field of every SCSI command is the Control field which
   * must be zero
   */

  if (priv->cdb[cdblen-1] != 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SCSICMDCONTROL), 0);
      if (lun)
        {
          lun->sd = SCSI_KCQIR_INVALIDFIELDINCBA;
        }
      ret     = -EINVAL;
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_idlestate
 *
 * Description:
 *   Called from the worker thread in the USBMSC_STATE_IDLE state.  Checks
 *   for the receipt of a bulk CBW.
 *
 * Returned value:
 *   If no new, valid CBW is available, this function returns a negated errno.
 *   Otherwise, when a new CBW is successfully parsed, this function sets
 *   priv->thstate to USBMSC_STATE_CMDPARSE and returns OK.
 *
 ****************************************************************************/

static int usbmsc_idlestate(FAR struct usbmsc_dev_s *priv)
{
  FAR struct usbmsc_req_s *privreq;
  FAR struct usbdev_req_s *req;
  FAR struct usbmsc_cbw_s *cbw;
  irqstate_t flags;
  int ret = -EINVAL;

  /* Take a request from the rdreqlist */

  flags = irqsave();
  privreq = (FAR struct usbmsc_req_s *)sq_remfirst(&priv->rdreqlist);
  irqrestore(flags);

  /* Has anything been received? If not, just return an error.
   * This will cause us to remain in the IDLE state.  When a USB request is
   * received, the worker thread will be awakened in the USBMSC_STATE_IDLE
   * and we will be called again.
   */

  if (!privreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_IDLERDREQLISTEMPTY), 0);
      return -ENOMEM;
    }

  req = privreq->req;
  cbw = (FAR struct usbmsc_cbw_s *)req->buf;

  /* Handle the CBW */

  usbmsc_dumpdata("SCSCI CBW", (uint8_t*)cbw, USBMSC_CBW_SIZEOF - USBMSC_MAXCDBLEN);
  usbmsc_dumpdata("      CDB", cbw->cdb, MIN(cbw->cdblen, USBMSC_MAXCDBLEN));

  /* Check for properly formatted CBW? */

  if (req->xfrd != USBMSC_CBW_SIZEOF ||
      cbw->signature[0] != 'U' ||
      cbw->signature[1] != 'S' ||
      cbw->signature[2] != 'B' ||
      cbw->signature[3] != 'C')
    {
      /* CBS BAD: Stall the bulk endpoints.  If the CBW is bad we must stall the
       * bulk IN endpoint and either (1) stall the bulk OUT endpoint, or
       * (2) discard data from the endpoint.
       */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INVALIDCBWSIGNATURE), 0);
      EP_STALL(priv->epbulkout);
      EP_STALL(priv->epbulkin);
    }

  /* Is the CBW meaningful? */

  else if (cbw->lun >= priv->nluns || (cbw->flags & ~USBMSC_CBWFLAG_IN) != 0 ||
           cbw->cdblen < 6 || cbw->cdblen > USBMSC_MAXCDBLEN)
    {
      /* CBS BAD: Stall the bulk endpoints.  If the CBW is bad we must stall the
       * bulk IN endpoint and either (1) stall the bulk OUT endpoint, or
       * (2) discard data from the endpoint.
       */

      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INVALIDCBWCONTENT), 0);
      EP_STALL(priv->epbulkout);
      EP_STALL(priv->epbulkin);
    }

  /* Save the information from the CBW */

  else
    {
      /* Extract the CDB and copy the whole CBD[] for later use */

      priv->cdblen = cbw->cdblen;
      memcpy(priv->cdb, cbw->cdb, priv->cdblen);

      /* Extract the data direction */

      if ((cbw->flags & USBMSC_CBWFLAG_IN) != 0)
        {
          /* IN: Device-to-host */

          priv->cbwdir = USBMSC_FLAGS_DIRDEVICE2HOST;
        }
      else
        {
          /* OUT: Host-to-device */

          priv->cbwdir = USBMSC_FLAGS_DIRHOST2DEVICE;
        }

      /* Get the max size of the data response */

      priv->cbwlen = usbmsc_getle32(cbw->datlen);
      if (priv->cbwlen == 0)
        {
          /* No length?  Then no direction either */

          priv->cbwdir = USBMSC_FLAGS_DIRNONE;
        }

      /* Extract and save the LUN index and TAG value */

      priv->cbwlun = cbw->lun;
      priv->cbwtag = usbmsc_getle32(cbw->tag);

      /* Return the read request to the bulk out endpoint for re-filling */

      req           = privreq->req;
      req->len      = CONFIG_USBMSC_BULKOUTREQLEN;
      req->priv     = privreq;
      req->callback = usbmsc_rdcomplete;

      if (EP_SUBMIT(priv->epbulkout, req) != OK)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_IDLERDSUBMIT), (uint16_t)-ret);
        }

      /* Change to the CMDPARSE state and return success */

      usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_IDLECMDPARSE), priv->cdb[0]);
      priv->thstate = USBMSC_STATE_CMDPARSE;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdparsestate
 *
 * Description:
 *   Called from the worker thread in the USBMSC_STATE_CMDPARSE state.
 *   This state is entered when usbmsc_idlestate obtains a valid CBW
 *   containing SCSI commands.  This function processes those SCSI commands.
 *
 * Returned value:
 *   If no write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMSC_STATE_CMDPARSE
 *   state.  Otherwise, when the new CBW is successfully process, this
 *   function sets priv->thstate to one of USBMSC_STATE_CMDREAD,
 *   USBMSC_STATE_CMDWRITE, or USBMSC_STATE_CMDFINISH and returns OK.
 *
 ****************************************************************************/

static int usbmsc_cmdparsestate(FAR struct usbmsc_dev_s *priv)
{
  FAR struct usbmsc_req_s *privreq;
  FAR uint8_t *buf;
  int ret = -EINVAL;

  usbmsc_dumpdata("SCSCI CDB", priv->cdb, priv->cdblen);

  /* Check if there is a request in the wrreqlist that we will be able to
   * use for data or status.
   */

  privreq = (FAR struct usbmsc_req_s *)sq_peek(&priv->wrreqlist);

  /* If there no request structures available, then just return an error.
   * This will cause us to remain in the CMDPARSE state.  When a request is
   * returned, the worker thread will be awakened in the USBMSC_STATE_CMDPARSE
   * and we will be called again.
   */

  if (!privreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDPARSEWRREQLISTEMPTY), 0);
      return -ENOMEM;
    }

  DEBUGASSERT(privreq->req && privreq->req->buf);
  buf                = privreq->req->buf;

  /* Assume that no errors will be encountered */

  priv->phaseerror   = 0;
  priv->shortpacket  = 0;

  /* No data is buffered */

  priv->nsectbytes   = 0;
  priv->nreqbytes    = 0;

  /* Get exclusive access to the block driver */

  pthread_mutex_lock(&priv->mutex);
  switch (priv->cdb[0])
    {
    case SCSI_CMD_TESTUNITREADY:                  /* 0x00 Mandatory */
      ret = usbmsc_cmdtestunitready(priv);
      break;

 /* case SCSI_CMD_REZEROUNIT:                      * 0x01 Obsolete
  *                                                * 0x02 Vendor-specific */

    case SCSI_CMD_REQUESTSENSE:                   /* 0x03 Mandatory */
      ret = usbmsc_cmdrequestsense(priv, buf);
      break;

 /* case SCSI_CMD_FORMAT_UNIT:                     * 0x04 Mandatory, but not implemented
  *                                                * 0x05 Vendor specific
  *                                                * 0x06 Vendor specific
  * case SCSI_CMD_REASSIGNBLOCKS:                  * 0x07 Optional */

    case SCSI_CMD_READ6:                          /* 0x08 Mandatory */
      ret = usbmsc_cmdread6(priv);
      break;

 /*                                                * 0x09 Vendor specific */

    case SCSI_CMD_WRITE6:                         /* 0x0a Optional */
      ret = usbmsc_cmdwrite6(priv);
      break;

 /* case SCSI_CMD_SEEK6:                           * 0x0b Obsolete
  *                                                * 0x0c-0x10 Vendor specific
  * case SCSI_CMD_SPACE6:                          * 0x11 Vendor specific */

    case SCSI_CMD_INQUIRY:                        /* 0x12 Mandatory */
      ret = usbmsc_cmdinquiry(priv, buf);
      break;

 /*                                                * 0x13-0x14 Vendor specific */

    case SCSI_CMD_MODESELECT6:                    /* 0x15 Optional */
      ret = usbmsc_cmdmodeselect6(priv);
      break;

 /* case SCSI_CMD_RESERVE6:                        * 0x16 Obsolete
  * case SCSI_CMD_RELEASE6:                        * 0x17 Obsolete
  * case SCSI_CMD_COPY:                            * 0x18 Obsolete
  *                                                * 0x19 Vendor specific */

    case SCSI_CMD_MODESENSE6:                     /* 0x1a Optional */
      ret = usbmsc_cmdmodesense6(priv, buf);
      break;

    case SCSI_CMD_STARTSTOPUNIT:                  /* 0x1b Optional */
      ret = usbmsc_cmdstartstopunit(priv);
      break;

 /* case SCSI_CMD_RECEIVEDIAGNOSTICRESULTS:        * 0x1c Optional
  * case SCSI_CMD_SENDDIAGNOSTIC:                  * 0x1d Mandatory, but not implemented */

    case SCSI_CMD_PREVENTMEDIAREMOVAL:            /* 0x1e Optional */
      ret = usbmsc_cmdpreventmediumremoval(priv);
      break;

 /*                                                * 0x20-22 Vendor specific */
    case SCSI_CMD_READFORMATCAPACITIES:           /* 0x23 Vendor-specific (defined in MMC spec) */
      ret = usbmsc_cmdreadformatcapacity(priv, buf);
      break;
 /*                                                * 0x24 Vendor specific */

    case SCSI_CMD_READCAPACITY10:                 /* 0x25 Mandatory */
      ret = usbmsc_cmdreadcapacity10(priv, buf);
      break;

 /*                                                * 0x26-27 Vendor specific */
    case SCSI_CMD_READ10:                         /* 0x28 Mandatory */
      ret = usbmsc_cmdread10(priv);
      break;

 /*                                                * 0x29 Vendor specific */

    case SCSI_CMD_WRITE10:                        /* 0x2a Optional */
      ret = usbmsc_cmdwrite10(priv);
      break;

 /* case SCSI_CMD_SEEK10:                          * 0x2b Obsolete
  *                                                * 0x2c-2d Vendor specific
  * case SCSI_CMD_WRITEANDVERIFY:                  * 0x2e Optional */

    case SCSI_CMD_VERIFY10:                       /* 0x2f Optional, but needed my MS Windows */
      ret = usbmsc_cmdverify10(priv);
      break;

 /* case SCSI_CMD_SEARCHDATAHIGH:                  * 0x30 Obsolete
  * case SCSI_CMD_SEARCHDATAEQUAL:                 * 0x31 Obsolete
  * case SCSI_CMD_SEARCHDATALOW:                   * 0x32 Obsolete
  * case SCSI_CMD_SETLIMITS10:                     * 0x33 Obsolete
  * case SCSI_CMD_PREFETCH10:                      * 0x34 Optional */

    case SCSI_CMD_SYNCHCACHE10:                   /* 0x35 Optional */
      ret = usbmsc_cmdsynchronizecache10(priv);
      break;

 /* case SCSI_CMD_LOCKCACHE:                       * 0x36 Obsolete
  * case SCSI_CMD_READDEFECTDATA10:                * 0x37 Optional
  * case SCSI_CMD_COMPARE:                         * 0x39 Obsolete
  * case SCSI_CMD_COPYANDVERIFY:                   * 0x3a Obsolete
  * case SCSI_CMD_WRITEBUFFER:                     * 0x3b Optional
  * case SCSI_CMD_READBUFFER:                      * 0x3c Optional
  * case SCSI_CMD_READLONG10:                      * 0x3e Optional
  * case SCSI_CMD_WRITELONG10:                     * 0x3f Optional
  * case SCSI_CMD_CHANGEDEFINITION:                * 0x40 Obsolete
  * case SCSI_CMD_WRITESAME10:                     * 0x41 Optional
  * case SCSI_CMD_LOGSELECT:                       * 0x4c Optional
  * case SCSI_CMD_LOGSENSE:                        * 0x4d Optional
  * case SCSI_CMD_XDWRITE10:                       * 0x50 Optional
  * case SCSI_CMD_XPWRITE10:                       * 0x51 Optional
  * case SCSI_CMD_XDREAD10:                        * 0x52 Optional */

    case SCSI_CMD_MODESELECT10:                   /* 0x55 Optional */
      ret = usbmsc_cmdmodeselect10(priv);
      break;

 /* case SCSI_CMD_RESERVE10:                       * 0x56 Obsolete
  * case SCSI_CMD_RELEASE10:                       * 0x57 Obsolete */

    case SCSI_CMD_MODESENSE10:                    /* 0x5a Optional */
      ret = usbmsc_cmdmodesense10(priv, buf);
      break;

 /* case SCSI_CMD_PERSISTENTRESERVEIN:             * 0x5e Optional
  * case SCSI_CMD_PERSISTENTRESERVEOUT:            * 0x5f Optional
  * case SCSI_CMD_32:                              * 0x7f Optional
  * case SCSI_CMD_XDWRITEEXTENDED:                 * 0x80 Obsolete
  * case SCSI_CMD_REBUILD:                         * 0x81 Obsolete
  * case SCSI_CMD_REGENERATE:                      * 0x82 Obsolete
  * case SCSI_CMD_EXTENDEDCOPY:                    * 0x83 Optional
  * case SCSI_CMD_COPYRESULTS:                     * 0x84 Optional
  * case SCSI_CMD_ACCESSCONTROLIN:                 * 0x86 Optional
  * case SCSI_CMD_ACCESSCONTROLOUT:                * 0x87 Optional
  * case SCSI_CMD_READ16:                          * 0x88 Optional
  * case SCSI_CMD_WRITE16:                         * 0x8a Optional
  * case SCSI_CMD_READATTRIBUTE:                   * 0x8c Optional
  * case SCSI_CMD_WRITEATTRIBUTE:                  * 0x8d Optional
  * case SCSI_CMD_WRITEANDVERIFY16:                * 0x8e Optional
  * case SCSI_CMD_SYNCHCACHE16:                    * 0x91 Optional
  * case SCSI_CMD_LOCKUNLOCKACACHE:                * 0x92 Optional
  * case SCSI_CMD_WRITESAME16:                     * 0x93 Optional
  * case SCSI_CMD_READCAPACITY16:                  * 0x9e Optional
  * case SCSI_CMD_READLONG16:                      * 0x9e Optional
  * case SCSI_CMD_WRITELONG16                      * 0x9f Optional
  * case SCSI_CMD_REPORTLUNS:                      * 0xa0 Mandatory, but not implemented
  * case SCSI_CMD_MAINTENANCEIN:                   * 0xa3 Optional (SCCS==0)
  * case SCSI_CMD_MAINTENANCEOUT:                  * 0xa4 Optional (SCCS==0)
  * case SCSI_CMD_MOVEMEDIUM:                      * 0xa5 ?
  * case SCSI_CMD_MOVEMEDIUMATTACHED:              * 0xa7 Optional (MCHNGR==0) */

    case SCSI_CMD_READ12:                         /* 0xa8 Optional */
      ret = usbmsc_cmdread12(priv);
      break;

    case SCSI_CMD_WRITE12:                        /* 0xaa Optional */
      ret = usbmsc_cmdwrite12(priv);
      break;

 /* case SCSI_CMD_READMEDIASERIALNUMBER:           * 0xab Optional
  * case SCSI_CMD_WRITEANDVERIFY12:                * 0xae Optional
  * case SCSI_CMD_VERIFY12:                        * 0xaf Optional
  * case SCSI_CMD_SETLIMITS12                      * 0xb3 Obsolete
  * case SCSI_CMD_READELEMENTSTATUS:               * 0xb4 Optional (MCHNGR==0)
  * case SCSI_CMD_READDEFECTDATA12:                * 0xb7 Optional
  * case SCSI_CMD_REDUNDANCYGROUPIN:               * 0xba Optional
  * case SCSI_CMD_REDUNDANCYGROUPOUT:              * 0xbb Optional
  * case SCSI_CMD_SPAREIN:                         * 0xbc Optional (SCCS==0)
  * case SCSI_CMD_SPAREOUT:                        * 0xbd Optional (SCCS==0)
  * case SCSI_CMD_VOLUMESETIN:                     * 0xbe Optional (SCCS==0)
  * case SCSI_CMD_VOLUMESETOUT:                    * 0xbe Optional (SCCS==0)
  *                                                * 0xc0-0xff Vendor specific */

    default:
      priv->u.alloclen = 0;
      if (ret == OK)
        {
          priv->lun->sd = SCSI_KCQIR_INVALIDCOMMAND;
          ret = -EINVAL;
        }
      break;
    }
  pthread_mutex_unlock(&priv->mutex);
 
  /* Is a response required?  (Not for read6/10/12 and write6/10/12). */

  if (priv->thstate == USBMSC_STATE_CMDPARSE)
    {
      /* All commands come through this path (EXCEPT read6/10/12 and write6/10/12). 
       * For all other commands, the following setup is expected for the response
       * based on data direction:
       *
       * For direction NONE:
       *   1. priv->u.alloclen == 0
       *   2. priv->nreqbytes == 0
       *
       * For direction = device-to-host:
       *   1. priv->u.alloclen == allocation length; space set aside by the
       *      host to receive the device data.  The size of the response
       *      cannot exceed this value.
       *   2. response data is in the request currently at the head of
       *      the priv->wrreqlist queue.  priv->nreqbytes is set to the length
       *      of data in the response.
       *
       * For direction host-to-device
       *   At present, there are no supported commands that should have host-to-device
       *   transfers (except write6/10/12 and that command logic does not take this
       *   path.  The 'residue' is left at the full host-to-device data size.
       *
       * For all:
       *   ret set to <0 if an error occurred in parsing the commands.
       */

      /* For from device-to-hose operations, the residue is the expected size
       * (the initial value of 'residue') minus the amount actually returned
       * in the response:
       */

      if (priv->cbwdir == USBMSC_FLAGS_DIRDEVICE2HOST)
        {
          /* The number of bytes in the response cannot exceed the host
           * 'allocation length' in the command.
           */

          if (priv->nreqbytes > priv->u.alloclen)
            {
              priv->nreqbytes = priv->u.alloclen;
            }

          /* The residue is then the number of bytes that were not sent */

          priv->residue -= priv->nreqbytes;
        }

      /* On return, we need the following:
       *
       *   1. Setup for CMDFINISH state if appropriate
       *   2. priv->thstate set to either CMDPARSE if no buffer was available or
       *      CMDFINISH to send the response
       *   3. Return OK to continue; <0 to wait for the next event
       */

      usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDPARSECMDFINISH), priv->cdb[0]);
      priv->thstate    = USBMSC_STATE_CMDFINISH;
      ret = OK;
    }
  return ret;
}

/****************************************************************************
 * Name: usbmsc_cmdreadstate
 *
 * Description:
 *   Called from the worker thread in the USBMSC_STATE_CMDREAD state.
 *   The USBMSC_STATE_CMDREAD state is entered when usbmsc_cmdparsestate
 *   obtains a valid SCSI read command. This state is really a continuation
 *   of the USBMSC_STATE_CMDPARSE state that handles extended SCSI read
 *   command handling.
 *
 * Returned value:
 *   If no USBDEV write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMSC_STATE_CMDREAD
 *   state.  Otherwise, when the new SCSI read command is fully processed,
 *   this function sets priv->thstate to USBMSC_STATE_CMDFINISH and returns OK.
 *
 * State variables:
 *   xfrlen     - holds the number of sectors read to be read.
 *   sector     - holds the sector number of the next sector to be read
 *   nsectbytes - holds the number of bytes buffered for the current sector
 *   nreqbytes  - holds the number of bytes currently buffered in the request
 *                at the head of the wrreqlist.
 *
 ****************************************************************************/

static int usbmsc_cmdreadstate(FAR struct usbmsc_dev_s *priv)
{
  FAR struct usbmsc_lun_s *lun = priv->lun;
  FAR struct usbmsc_req_s *privreq;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  ssize_t nread;
  uint8_t *src;
  uint8_t *dest;
  int nbytes;
  int ret;

  /* Loop transferring data until either (1) all of the data has been
   * transferred, or (2) we have used up all of the write requests that we have
   * available.
   */

  while (priv->u.xfrlen > 0 || priv->nsectbytes > 0)
    {
      usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDREAD), priv->u.xfrlen);

      /* Is the I/O buffer empty? */

      if (priv->nsectbytes <= 0)
        {
          /* Yes.. read the next sector */

          nread = USBMSC_DRVR_READ(lun, priv->iobuffer, priv->sector, 1);
          if (nread < 0)
            {
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDREADREADFAIL), -nread);
              lun->sd     = SCSI_KCQME_UNRRE1;
              lun->sdinfo = priv->sector;
              break;
            }

          priv->nsectbytes = lun->sectorsize;
          priv->u.xfrlen--;
          priv->sector++;
        }

      /* Check if there is a request in the wrreqlist that we will be able to
       * use for data transfer.
       */

      privreq = (FAR struct usbmsc_req_s *)sq_peek(&priv->wrreqlist);

      /* If there no request structures available, then just return an error.
       * This will cause us to remain in the CMDREAD state.  When a request is
       * returned, the worker thread will be awakened in the USBMSC_STATE_CMDREAD
       * and we will be called again.
       */

      if (!privreq)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDREADWRRQEMPTY), 0);
          priv->nreqbytes = 0;
          return -ENOMEM;
        }
      req = privreq->req;

      /* Transfer all of the data that will (1) fit into the request buffer, OR (2)
       * all of the data available in the sector buffer.
       */

      src    = &priv->iobuffer[lun->sectorsize - priv->nsectbytes];
      dest   = &req->buf[priv->nreqbytes];

      nbytes = MIN(CONFIG_USBMSC_BULKINREQLEN - priv->nreqbytes, priv->nsectbytes);

      /* Copy the data from the sector buffer to the USB request and update counts */

      memcpy(dest, src, nbytes);
      priv->nreqbytes  += nbytes;
      priv->nsectbytes -= nbytes;

      /* If (1) the request buffer is full OR (2) this is the final request full of data,
       * then submit the request
       */

      if (priv->nreqbytes >= CONFIG_USBMSC_BULKINREQLEN ||
          (priv->u.xfrlen <= 0 && priv->nsectbytes <= 0))
        {
          /* Remove the request that we just filled from wrreqlist (we've already checked
           * that is it not NULL
           */

          flags = irqsave();
          privreq = (FAR struct usbmsc_req_s *)sq_remfirst(&priv->wrreqlist);
          irqrestore(flags);

          /* And submit the request to the bulk IN endpoint */

          req->len      = priv->nreqbytes;
          req->priv     = privreq;
          req->callback = usbmsc_wrcomplete;
          req->flags    = 0;

          ret           = EP_SUBMIT(priv->epbulkin, req);
          if (ret != OK)
            {
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDREADSUBMIT), (uint16_t)-ret);
              lun->sd     = SCSI_KCQME_UNRRE1;
              lun->sdinfo = priv->sector;
              break;
            }

          /* Assume success... residue should probably really be decremented in
           * wrcomplete when we know that the transfer completed successfully.
           */

          priv->residue  -= priv->nreqbytes;
          priv->nreqbytes = 0;
        }
    }

  usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDREADCMDFINISH), priv->u.xfrlen);
  priv->thstate  = USBMSC_STATE_CMDFINISH;
  return OK;
}

/****************************************************************************
 * Name: usbmsc_cmdwritestate
 *
 * Description:
 *   Called from the worker thread in the USBMSC_STATE_CMDWRITE state.
 *   The USBMSC_STATE_CMDWRITE state is entered when usbmsc_cmdparsestate
 *   obtains a valid SCSI write command. This state is really a continuation
 *   of the USBMSC_STATE_CMDPARSE state that handles extended SCSI write
 *   command handling.
 *
 * Returned value:
 *   If no USBDEV write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMSC_STATE_CMDWRITE
 *   state.  Otherwise, when the new SCSI write command is fully processed,
 *   this function sets priv->thstate to USBMSC_STATE_CMDFINISH and returns OK.
 *
 * State variables:
 *   xfrlen     - holds the number of sectors read to be written.
 *   sector     - holds the sector number of the next sector to write
 *   nsectbytes - holds the number of bytes buffered for the current sector
 *   nreqbytes  - holds the number of untransferred bytes currently in the
 *                request at the head of the rdreqlist.
 *
 ****************************************************************************/

static int usbmsc_cmdwritestate(FAR struct usbmsc_dev_s *priv)
{
  FAR struct usbmsc_lun_s *lun = priv->lun;
  FAR struct usbmsc_req_s *privreq;
  FAR struct usbdev_req_s *req;
  ssize_t nwritten;
  uint16_t xfrd;
  uint8_t *src;
  uint8_t *dest;
  int nbytes;
  int ret;

  /* Loop transferring data until either (1) all of the data has been
   * transferred, or (2) we have written all of the data in the available
   * read requests.
   */

  while (priv->u.xfrlen > 0)
    {
      usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDWRITE), priv->u.xfrlen);

      /* Check if there is a request in the rdreqlist containing additional
       * data to be written.
       */

      privreq = (FAR struct usbmsc_req_s *)sq_remfirst(&priv->rdreqlist);

      /* If there no request data available, then just return an error.
       * This will cause us to remain in the CMDWRITE state.  When a filled request is
       * received, the worker thread will be awakened in the USBMSC_STATE_CMDWRITE
       * and we will be called again.
       */

      if (!privreq)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDWRITERDRQEMPTY), 0);
          priv->nreqbytes = 0;
          return -ENOMEM;
        }

      req             = privreq->req;
      xfrd            = req->xfrd;
      priv->nreqbytes = xfrd;

      /* Now loop until all of the data in the read request has been tranferred
       * to the block driver OR all of the request data has been transferred.
       */

      while (priv->nreqbytes > 0 && priv->u.xfrlen > 0)
        {
          /* Copy the data received in the read request into the sector I/O buffer */

          src  = &req->buf[xfrd - priv->nreqbytes];
          dest = &priv->iobuffer[priv->nsectbytes];

          nbytes = MIN(lun->sectorsize - priv->nsectbytes, priv->nreqbytes);

          /* Copy the data from the sector buffer to the USB request and update counts */

          memcpy(dest, src, nbytes);
          priv->nsectbytes += nbytes;
          priv->nreqbytes  -= nbytes;

          /* Is the I/O buffer full? */

          if (priv->nsectbytes >= lun->sectorsize)
            {
              /* Yes.. Write the next sector */

              nwritten = USBMSC_DRVR_WRITE(lun, priv->iobuffer, priv->sector, 1);
              if (nwritten < 0)
                {
                  usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDWRITEWRITEFAIL), -nwritten);
                  lun->sd     = SCSI_KCQME_WRITEFAULTAUTOREALLOCFAILED;
                  lun->sdinfo = priv->sector;
                  goto errout;
                }

              priv->nsectbytes = 0;
              priv->residue   -= lun->sectorsize;
              priv->u.xfrlen--;
              priv->sector++;
            }
        }

      /* In either case, we are finished with this read request and can return it
       * to the endpoint.  Then we will go back to the top of the top and attempt
       * to get the next read request.
       */

      req->len      = CONFIG_USBMSC_BULKOUTREQLEN;
      req->priv     = privreq;
      req->callback = usbmsc_rdcomplete;

      ret = EP_SUBMIT(priv->epbulkout, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDWRITERDSUBMIT), (uint16_t)-ret);
        }

      /* Did the host decide to stop early? */

      if (xfrd != CONFIG_USBMSC_BULKOUTREQLEN)
        {
          priv->shortpacket = 1;
          goto errout;
        }
    }

errout:
  usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDWRITECMDFINISH), priv->u.xfrlen);
  priv->thstate  = USBMSC_STATE_CMDFINISH;
  return OK;
}

/****************************************************************************
 * Name: usbmsc_cmdfinishstate
 *
 * Description:
 *   Called from the worker thread in the USBMSC_STATE_CMDFINISH state.
 *   The USBMSC_STATE_CMDFINISH state is entered when processing of a
 *   command has finished but before status has been returned.
 *
 * Returned value:
 *   If no USBDEV write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMSC_STATE_CMDFINISH
 *   state.  Otherwise, when the command is fully processed, this function
 *   sets priv->thstate to USBMSC_STATE_CMDSTATUS and returns OK.
 *
 ****************************************************************************/

static int usbmsc_cmdfinishstate(FAR struct usbmsc_dev_s *priv)
{
  FAR struct usbmsc_req_s *privreq;
  irqstate_t flags;
  int ret = OK;

  /* Check if there is a request in the wrreqlist that we will be able to
   * use for data transfer.
   */

  privreq = (FAR struct usbmsc_req_s *)sq_peek(&priv->wrreqlist);

  /* If there no request structures available, then just return an error.
   * This will cause us to remain in the CMDREAD state.  When a request is
   * returned, the worker thread will be awakened in the USBMSC_STATE_CMDREAD
   * and we will be called again.
   */

  if (!privreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDFINISHRQEMPTY), 0);
      return -ENOMEM;
    }

  /* Finish the final stages of the reply */

  switch (priv->cbwdir)
    {
      /* Device-to-host: All but the last data buffer has been sent */

    case USBMSC_FLAGS_DIRDEVICE2HOST:
      if (priv->cbwlen > 0)
        {
          /* On most commands (the exception is outgoing, write commands),
           * the data has not not yet been sent.
           */

          if (priv->nreqbytes > 0)
            {
              struct usbdev_req_s *req;

              /* Take a request from the wrreqlist (we've already checked
               * that is it not NULL)
               */

              flags = irqsave();
              privreq = (FAR struct usbmsc_req_s *)sq_remfirst(&priv->wrreqlist);
              irqrestore(flags);

              /* Send the write request */

              req           = privreq->req;
              req->len      = priv->nreqbytes;
              req->callback = usbmsc_wrcomplete;
              req->priv     = privreq;
              req->flags    = USBDEV_REQFLAGS_NULLPKT;

              ret           = EP_SUBMIT(priv->epbulkin, privreq->req);
              if (ret < 0)
                {
                  usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDFINISHSUBMIT), (uint16_t)-ret);
                }
             }

          /* Stall the BULK In endpoint if there is a residue */

          if (priv->residue > 0)
            {
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDFINISHRESIDUE), (uint16_t)priv->residue);

#ifdef CONFIG_USBMSC_RACEWAR
              /* (See description of the workaround at the top of the file).
               * First, wait for the transfer to complete, then stall the endpoint
               */

              usleep (100000);
              (void)EP_STALL(priv->epbulkin);

              /* now wait for stall to go away .... */

              usleep (100000);
#else
              (void)EP_STALL(priv->epbulkin);
#endif
            }
        }
      break;

      /* Host-to-device: We have processed all we want of the host data. */

    case USBMSC_FLAGS_DIRHOST2DEVICE:
      if (priv->residue > 0)
        {
          /* Did the host stop sending unexpectedly early? */

          flags = irqsave();
          if (priv->shortpacket)
            {
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDFINISHSHORTPKT), (uint16_t)priv->residue);
            }

          /* Unprocessed incoming data: STALL and cancel requests. */

          else 
            {
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDFINSHSUBMIT), (uint16_t)priv->residue);
              EP_STALL(priv->epbulkout);
           }

           priv->theventset |= USBMSC_EVENT_ABORTBULKOUT;
           irqrestore(flags);
        }
      break;

    default:
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDFINSHDIR), priv->cbwdir);
    case USBMSC_FLAGS_DIRNONE: /* Nothing to send */
      break;
    }

  /* Return to the IDLE state */

  usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDFINISHCMDSTATUS), 0);
  priv->thstate = USBMSC_STATE_CMDSTATUS;
  return OK;
}

/****************************************************************************
 * Name: usbmsc_cmdstatusstate
 *
 * Description:
 *   Called from the worker thread in the USBMSC_STATE_CMDSTATUS state.
 *   That state is after a CBW has been fully processed.  This function sends
 *   the concluding CSW.
 *
 * Returned value:
 *   If no write request is available or certain other errors occur, this
 *   function returns a negated errno and stays in the USBMSC_STATE_CMDSTATUS
 *   state.  Otherwise, when the SCSI statis is successfully returned, this
 *   function sets priv->thstate to USBMSC_STATE_IDLE and returns OK.
 *
 ****************************************************************************/

static int usbmsc_cmdstatusstate(FAR struct usbmsc_dev_s *priv)
{
  FAR struct usbmsc_lun_s *lun = priv->lun;
  FAR struct usbmsc_req_s *privreq;
  FAR struct usbdev_req_s *req;
  FAR struct usbmsc_csw_s *csw;
  irqstate_t flags;
  uint32_t sd;
  uint8_t status = USBMSC_CSWSTATUS_PASS;
  int ret;

  /* Take a request from the wrreqlist */

  flags = irqsave();
  privreq = (FAR struct usbmsc_req_s *)sq_remfirst(&priv->wrreqlist);
  irqrestore(flags);

  /* If there no request structures available, then just return an error.
   * This will cause us to remain in the CMDSTATUS status.  When a request is
   * returned, the worker thread will be awakened in the USBMSC_STATE_CMDSTATUS
   * and we will be called again.
   */

  if (!privreq)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_CMDSTATUSRDREQLISTEMPTY), 0);
      return -ENOMEM;
    }

  req = privreq->req;
  csw = (struct usbmsc_csw_s*)req->buf;

  /* Extract the sense data from the LUN structure */

  if (lun)
    {
      sd = lun->sd;
    }
  else 
    {
      sd = SCSI_KCQIR_INVALIDLUN;
    }

  /* Determine the CSW status: PASS, PHASEERROR, or FAIL */

  if (priv->phaseerror)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SNDPHERROR), 0);
      status = USBMSC_CSWSTATUS_PHASEERROR;
      sd     = SCSI_KCQIR_INVALIDCOMMAND;
    }
  else if (sd != SCSI_KCQ_NOSENSE)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SNDCSWFAIL), 0);
      status = USBMSC_CSWSTATUS_FAIL;
    }

  /* Format and submit the CSW */

  csw->signature[0] = 'U';
  csw->signature[1] = 'S';
  csw->signature[2] = 'B';
  csw->signature[3] = 'S';
  usbmsc_putle32(csw->tag, priv->cbwtag);
  usbmsc_putle32(csw->residue, priv->residue);
  csw->status       = status;

  usbmsc_dumpdata("SCSCI CSW", (uint8_t*)csw, USBMSC_CSW_SIZEOF);

  req->len       = USBMSC_CSW_SIZEOF;
  req->callback  = usbmsc_wrcomplete;
  req->priv      = privreq;
  req->flags     = USBDEV_REQFLAGS_NULLPKT;

  ret            = EP_SUBMIT(priv->epbulkin, req);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_SNDSTATUSSUBMIT), (uint16_t)-ret);
      flags = irqsave();
      (void)sq_addlast((sq_entry_t*)privreq, &priv->wrreqlist);
      irqrestore(flags);
    }

  /* Return to the IDLE state */

  usbtrace(TRACE_CLASSSTATE(USBMSC_CLASSSTATE_CMDSTATUSIDLE), 0);
  priv->thstate = USBMSC_STATE_IDLE;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_workerthread
 *
 * Description:
 *   This is the main function of the USB storage worker thread.  It loops
 *   until USB-related events occur, then processes those events accordingly
 *
 ****************************************************************************/

void *usbmsc_workerthread(void *arg)
{
  struct usbmsc_dev_s *priv = (struct usbmsc_dev_s *)arg;
  irqstate_t flags;
  uint16_t eventset;
  int ret;

  /* This thread is started before the USB storage class is fully initialized.
   * wait here until we are told to begin.  Start in the NOTINITIALIZED state
   */

  pthread_mutex_lock(&priv->mutex);
  priv->thstate = USBMSC_STATE_STARTED;
  while ((priv->theventset & USBMSC_EVENT_READY) != 0 &&
         (priv->theventset & USBMSC_EVENT_TERMINATEREQUEST) != 0)
    {
      pthread_cond_wait(&priv->cond, &priv->mutex);
    }

  /* Transition to the INITIALIZED/IDLE state */

  priv->thstate    = USBMSC_STATE_IDLE;
  eventset         = priv->theventset;
  priv->theventset = USBMSC_EVENT_NOEVENTS;
  pthread_mutex_unlock(&priv->mutex);

  /* Then loop until we are asked to terminate */

  while ((eventset & USBMSC_EVENT_TERMINATEREQUEST) == 0)
    {
      /* Wait for some interesting event.  Note that we must both take the
       * lock (to eliminate race conditions with other threads) and disable
       * interrupts (to eliminate race conditions with USB interrupt handling.
       */

      pthread_mutex_lock(&priv->mutex);
      flags = irqsave();
      if (priv->theventset == USBMSC_EVENT_NOEVENTS)
        {
          pthread_cond_wait(&priv->cond, &priv->mutex);
        }

      /* Sample any events before re-enabling interrupts.  Any events that
       * occur after re-enabling interrupts will have to be handled in the
       * next time through the loop.
       */

      eventset         = priv->theventset;
      priv->theventset = USBMSC_EVENT_NOEVENTS;
      pthread_mutex_unlock(&priv->mutex);

      /* Were we awakened by some event that requires immediate action?
       *
       * - The USBMSC_EVENT_DISCONNECT is signalled from the disconnect method
       *   after all transfers have been stopped, when the host is disconnected.
       *
       * - The CUSBMSC_EVENT_RESET is signalled when the bulk-storage-specific
       *   USBMSC_REQ_MSRESET EP0 setup received.  We must stop the current
       *   operation and reinialize state.
       *
       * - The USBMSC_EVENT_CFGCHANGE is signaled when the EP0 setup logic
       *   receives a valid USB_REQ_SETCONFIGURATION request
       *
       * - The USBMSC_EVENT_IFCHANGE is signaled when the EP0 setup logic
       *   receives a valid USB_REQ_SETINTERFACE request
       *
       * - The USBMSC_EVENT_ABORTBULKOUT event is signalled by the CMDFINISH
       *   logic when there is a residue after processing a host-to-device
       *   transfer.  We need to discard all incoming request.
       *
       * All other events are just wakeup calls and are intended only
       * drive the state machine.
       */

      if ((eventset & (USBMSC_EVENT_DISCONNECT|USBMSC_EVENT_RESET|USBMSC_EVENT_CFGCHANGE|
                       USBMSC_EVENT_IFCHANGE|USBMSC_EVENT_ABORTBULKOUT)) != 0)
        {
          /* These events require that the current configuration be reset */

          if ((eventset & USBMSC_EVENT_IFCHANGE) != 0)
            {
              usbmsc_resetconfig(priv);
            }

          /* These events require that a new configuration be established */

          if ((eventset & (USBMSC_EVENT_CFGCHANGE|USBMSC_EVENT_IFCHANGE)) != 0)
            {
              usbmsc_setconfig(priv, priv->thvalue);
            }

          /* These events required that we send a deferred EP0 setup response */

          if ((eventset & (USBMSC_EVENT_RESET|USBMSC_EVENT_CFGCHANGE|USBMSC_EVENT_IFCHANGE)) != 0)
            {
              usbmsc_deferredresponse(priv, false);
            }

          /* For all of these events... terminate any transactions in progress */

          priv->thstate = USBMSC_STATE_IDLE;
        }
      irqrestore(flags);

      /* Loop processing each SCSI command state.  Each state handling
       * function will do the following:
       *
       * - If it must block for an event, it will return a negated errno value
       * - If it completes the processing for that state, it will (1) set
       *   the next appropriate state value and (2) return OK.
       *
       * So the following will loop until we must block for an event in
       * a particular state.  When we are awakened by an event (above) we
       * will resume processing in the same state.
       */

      do
        {
          switch (priv->thstate)
            {
            case USBMSC_STATE_IDLE:             /* Started and waiting for commands */
               ret = usbmsc_idlestate(priv);
               break;

            case USBMSC_STATE_CMDPARSE:         /* Parsing the received a command */
               ret = usbmsc_cmdparsestate(priv);
               break;

            case USBMSC_STATE_CMDREAD:          /* Continuing to process a SCSI read command */
               ret = usbmsc_cmdreadstate(priv);
               break;

            case USBMSC_STATE_CMDWRITE:         /* Continuing to process a SCSI write command */
               ret = usbmsc_cmdwritestate(priv);
               break;

            case USBMSC_STATE_CMDFINISH:        /* Finish command processing */
               ret = usbmsc_cmdfinishstate(priv);
               break;

            case USBMSC_STATE_CMDSTATUS:        /* Processing the status phase of a command */
              ret = usbmsc_cmdstatusstate(priv);
              break;

            case USBMSC_STATE_NOTSTARTED:       /* Thread has not yet been started */
            case USBMSC_STATE_STARTED:          /* Started, but is not yet initialized */
            case USBMSC_STATE_TERMINATED:       /* Thread has exitted */
            default:
              usbtrace(TRACE_CLSERROR(USBMSC_TRACEERR_INVALIDSTATE), priv->thstate);
              priv->thstate = USBMSC_STATE_IDLE;
              ret           = OK;
              break;
            }
        }
      while (ret == OK);
    }

  /* Transition to the TERMINATED state and exit */

  priv->thstate = USBMSC_STATE_TERMINATED;
  return NULL;
}
