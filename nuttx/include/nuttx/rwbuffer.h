/****************************************************************************
 * include/nuttx/rwbuffer.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_RWBUFFER_H
#define __INCLUDE_NUTTX_RWBUFFER_H

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <semaphore.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

/**********************************************************************
 * Public Types
 **********************************************************************/

/* Data transfer callouts.  These must be provided by the block driver
 * logic in order to flush the write buffer when appropriate or to
 * reload the read-ahead buffer, when appropriate.
 */

typedef ssize_t (*rwbreload_t)(FAR void *dev, FAR uint8_t *buffer,
                               off_t startblock, size_t nblocks);
typedef ssize_t (*rwbflush_t)(FAR void *dev, FAR const uint8_t *buffer,
                              off_t startblock, size_t nblocks);

/* This structure holds the state of the buffers.  In typical usage,
 * an instance of this structure is declared within each block driver
 * status structure like:
 *
 * struct foo_dev_s
 * {
 *   ...
 *   struct rwbuffer_s rwbuffer;
 *   ...
 * };
 *
 * Note that this supports buffering for multiple block devices or for
 * multiple instances of same block device, because each rwbuffer instance
 * supports independent buffering.
 *
 * A reference to the struct rwbuffer_s instance is then passed to each
 * interface like:
 *
 *  struct foo_dev_s *priv;
 *  ...
 *  ... [Setup blocksize, nblocks, dev, wrmaxblocks, wrflush,
 *       rhmaxblocks, rhreload] ...
 *  ret = rwb_initialize(&priv->rwbuffer);
 */

struct rwbuffer_s
{
  /********************************************************************/
  /* These values must be provided by the user prior to calling
   * rwb_initialize()
   */

  /* Supported geometry */

  uint16_t      blocksize;       /* The size of one block */
  size_t        nblocks;         /* The total number blocks supported */
  FAR void     *dev;             /* Device state passed to callout functions */

  /* Write buffer setup.  If CONFIG_FS_WRITEBUFFER is defined, but you
   * want read-ahead-only operation, (1) set wrmaxblocks to zero and do
   * not use rwb_write().
   */

#ifdef CONFIG_FS_WRITEBUFFER
  uint16_t      wrmaxblocks;     /* The number of blocks to buffer in memory */
  rwbflush_t    wrflush;         /* Callout to flush the write buffer */
#endif

  /* Read-ahead buffer setup.  If CONFIG_FS_READAHEAD is defined but you
   * want write-buffer-only operation, then (1) set rhmaxblocks to zero and
   * do not use rwb_read().
   */

#ifdef CONFIG_FS_READAHEAD
  uint16_t      rhmaxblocks;     /* The number of blocks to buffer in memory */
  rwbreload_t   rhreload;        /* Callout to reload the read-ahead buffer */
#endif

  /********************************************************************/
  /* The user should never modify any of the remaing fields */

  /* This is the state of the write buffer */

#ifdef CONFIG_FS_WRITEBUFFER
  sem_t         wrsem;           /* Enforces exclusive access to the write buffer */
  struct work_s work;            /* Delayed work to flush buffer after adelay with no activity */
  uint8_t      *wrbuffer;        /* Allocated write buffer */
  uint16_t      wrnblocks;       /* Number of blocks in write buffer */
  off_t         wrblockstart;    /* First block in write buffer */
  off_t         wrexpectedblock; /* Next block expected */
#endif

  /* This is the state of the read-ahead buffer */

#ifdef CONFIG_FS_READAHEAD
  sem_t         rhsem;           /* Enforces exclusive access to the write buffer */
  uint8_t      *rhbuffer;        /* Allocated read-ahead buffer */
  uint16_t      rhnblocks;       /* Number of blocks in read-ahead buffer */
  off_t         rhblockstart;    /* First block in read-ahead buffer */
#endif
};

/**********************************************************************
 * Global Variables
 **********************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN EXTERN "C"
EXTERN "C" {
#else
#define EXTERN extern
#endif

/**********************************************************************
 * Global Function Prototypes
 **********************************************************************/

/* Buffer initialization */

EXTERN int rwb_initialize(FAR struct rwbuffer_s *rwb);
EXTERN void rwb_uninitialize(FAR struct rwbuffer_s *rwb);

/* Buffer transfers */

EXTERN ssize_t rwb_read(FAR struct rwbuffer_s *rwb, off_t startblock,
                        size_t blockcount, FAR uint8_t *rdbuffer);
EXTERN ssize_t rwb_write(FAR struct rwbuffer_s *rwb,
                         off_t startblock, size_t blockcount,
                         FAR const uint8_t *wrbuffer);
EXTERN int rwb_mediaremoved(FAR struct rwbuffer_s *rwb);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_FS_WRITEBUFFER || CONFIG_READAHEAD_BUFFER */
#endif /* __INCLUDE_NUTTX_RWBUFFER_H */
