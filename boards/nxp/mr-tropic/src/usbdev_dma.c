/****************************************************************************
*
* Copyright 2026 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*
****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>

#include <nuttx/mm/gran.h>
#include <nuttx/usb/usbdev.h>

#if defined(CONFIG_USBDEV_DMA) && defined(CONFIG_USBDEV_DMAMEMORY)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

extern uint8_t _sdtcm[];
extern uint8_t _edtcm[];

#define USBDMA_GRAN_LOG2   5   /* 2^5 = 32-byte granule */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static GRAN_HANDLE g_usbdma_gran;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void usbdev_dma_initialize(void)
{
  if (g_usbdma_gran == NULL)
    {
      size_t heapsize = (size_t)(_edtcm - _sdtcm);

      g_usbdma_gran = gran_initialize((void *)_sdtcm, heapsize,
                                      USBDMA_GRAN_LOG2, USBDMA_GRAN_LOG2);
      DEBUGASSERT(g_usbdma_gran != NULL);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *usbdev_dma_alloc(size_t size)
{
  usbdev_dma_initialize();
  return gran_alloc(g_usbdma_gran, size);
}

void usbdev_dma_free(FAR void *memory)
{
  if (g_usbdma_gran != NULL && memory != NULL)
    {
      gran_free(g_usbdma_gran, memory, (1 << USBDMA_GRAN_LOG2));
    }
}

#endif /* CONFIG_USBDEV_DMA && CONFIG_USBDEV_DMAMEMORY */
