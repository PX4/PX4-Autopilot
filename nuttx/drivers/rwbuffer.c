/****************************************************************************
 * drivers/rwbuffer.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/rwbuffer.h>

#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_FS_WRDELAY
#  define CONFIG_FS_WRDELAY 350
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rwb_semtake
 ****************************************************************************/

static void rwb_semtake(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: rwb_semgive
 ****************************************************************************/

#define rwb_semgive(s) sem_post(s)

/****************************************************************************
 * Name: rwb_overlap
 ****************************************************************************/

static inline bool rwb_overlap(off_t blockstart1, size_t nblocks1,
                               off_t blockstart2, size_t nblocks2)
{
  off_t blockend1 = blockstart1 + nblocks1;
  off_t blockend2 = blockstart2 + nblocks2;

  /* If the buffer 1 is wholly outside of buffer 2, return false */

  if ((blockend1   < blockstart2) || /* Wholly "below" */
      (blockstart1 > blockend2))     /* Wholly "above" */
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: rwb_resetwrbuffer
 ****************************************************************************/

#ifdef CONFIG_FS_WRITEBUFFER
static inline void rwb_resetwrbuffer(struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the wrsem */

  rwb->wrnblocks       = 0;
  rwb->wrblockstart    = (off_t)-1;
  rwb->wrexpectedblock = (off_t)-1;
}
#endif

/****************************************************************************
 * Name: rwb_wrflush
 ****************************************************************************/

#ifdef CONFIG_FS_WRITEBUFFER
static void rwb_wrflush(struct rwbuffer_s *rwb)
{
  int ret;

  fvdbg("Timeout!\n");
  
  rwb_semtake(&rwb->wrsem);
  if (rwb->wrnblocks)
    {
      fvdbg("Flushing: blockstart=0x%08lx nblocks=%d from buffer=%p\n",
      (long)rwb->wrblockstart, rwb->wrnblocks, rwb->wrbuffer);

      /* Flush cache.  On success, the flush method will return the number
       * of blocks written.  Anything other than the number requested is
       * an error.
       */

      ret = rwb->wrflush(rwb->dev, rwb->wrbuffer, rwb->wrblockstart, rwb->wrnblocks);
      if (ret != rwb->wrnblocks)
        {
          fdbg("ERROR: Error flushing write buffer: %d\n", ret);
        }
      
      rwb_resetwrbuffer(rwb);
    }

  rwb_semgive(&rwb->wrsem);
}
#endif

/****************************************************************************
 * Name: rwb_wrtimeout
 ****************************************************************************/

static void rwb_wrtimeout(FAR void *arg)
{
  /* The following assumes that the size of a pointer is 4-bytes or less */

  FAR struct rwbuffer_s *rwb = (struct rwbuffer_s *)arg;
  DEBUGASSERT(rwb != NULL);

  /* If a timeout elpases with with write buffer activity, this watchdog
   * handler function will be evoked on the thread of execution of the
   * worker thread.
   */

  rwb_wrflush(rwb);
}

/****************************************************************************
 * Name: rwb_wrstarttimeout
 ****************************************************************************/

static void rwb_wrstarttimeout(FAR struct rwbuffer_s *rwb)
{
  /* CONFIG_FS_WRDELAY provides the delay period in milliseconds. CLK_TCK
   * provides the clock tick of the system (frequency in Hz).
   */

  int ticks = (CONFIG_FS_WRDELAY + CLK_TCK/2) / CLK_TCK;
  (void)work_queue(LPWORK, &rwb->work, rwb_wrtimeout, (FAR void *)rwb, ticks);
}

/****************************************************************************
 * Name: rwb_wrcanceltimeout
 ****************************************************************************/

static inline void rwb_wrcanceltimeout(struct rwbuffer_s *rwb)
{
  (void)work_cancel(LPWORK, &rwb->work);
}

/****************************************************************************
 * Name: rwb_writebuffer
 ****************************************************************************/

#ifdef CONFIG_FS_WRITEBUFFER
static ssize_t rwb_writebuffer(FAR struct rwbuffer_s *rwb,
                               off_t startblock, uint32_t nblocks,
                               FAR const uint8_t *wrbuffer)
{
  int ret;

  /* Write writebuffer Logic */

  rwb_wrcanceltimeout(rwb);
  
  /* First: Should we flush out our cache? We would do that if (1) we already
   * buffering blocks and the next block writing is not in the same sequence,
   * or (2) the number of blocks would exceed our allocated buffer capacity
   */

  if (((startblock != rwb->wrexpectedblock) && (rwb->wrnblocks)) ||
      ((rwb->wrnblocks + nblocks) > rwb->wrmaxblocks))
    {
      fvdbg("writebuffer miss, expected: %08x, given: %08x\n",
            rwb->wrexpectedblock, startblock);
    
      /* Flush the write buffer */

      ret = rwb->wrflush(rwb, rwb->wrbuffer, rwb->wrblockstart, rwb->wrnblocks);
      if (ret < 0)
        {
          fdbg("ERROR: Error writing multiple from cache: %d\n", -ret);
          return ret;
        }

      rwb_resetwrbuffer(rwb);
    }
  
  /* writebuffer is empty? Then initialize it */

  if (!rwb->wrnblocks)
    {
      fvdbg("Fresh cache starting at block: 0x%08x\n", startblock);
      rwb->wrblockstart = startblock;
    }
  
  /* Add data to cache */

  fvdbg("writebuffer: copying %d bytes from %p to %p\n",
        nblocks * wrb->blocksize, wrbuffer,
        &rwb->wrbuffer[rwb->wrnblocks * rwb->blocksize]);
  memcpy(&rwb->wrbuffer[rwb->wrnblocks * rwb->blocksize],
         wrbuffer, nblocks * rwb->blocksize);

  rwb->wrnblocks      += nblocks;
  rwb->wrexpectedblock = rwb->wrblockstart + rwb->wrnblocks;
  rwb_wrstarttimeout(rwb);
  return nblocks;
}
#endif

/****************************************************************************
 * Name: rwb_resetrhbuffer
 ****************************************************************************/

#ifdef CONFIG_FS_READAHEAD
static inline void rwb_resetrhbuffer(struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the readAheadBufferSemphore */

  rwb->rhnblocks    = 0;
  rwb->rhblockstart = (off_t)-1;
}
#endif

/****************************************************************************
 * Name: rwb_bufferread
 ****************************************************************************/

#ifdef CONFIG_FS_READAHEAD
static inline void
rwb_bufferread(struct rwbuffer_s *rwb,  off_t startblock,
               size_t nblocks, uint8_t **rdbuffer)
{
  /* We assume that (1) the caller holds the readAheadBufferSemphore, and (2)
   * that the caller already knows that all of the blocks are in the
   * read-ahead buffer.
   */

  /* Convert the units from blocks to bytes */

  off_t  blockoffset = startblock - rwb->rhblockstart;
  off_t  byteoffset  = rwb->blocksize * blockoffset;
  size_t nbytes      = rwb->blocksize * nblocks;

  /* Get the byte address in the read-ahead buffer */

  uint8_t *rhbuffer    = rwb->rhbuffer + byteoffset;

  /* Copy the data from the read-ahead buffer into the IO buffer */

  memcpy(*rdbuffer, rhbuffer, nbytes);

  /* Update the caller's copy for the next address */

  *rdbuffer += nbytes;
}
#endif

/****************************************************************************
 * Name: rwb_rhreload
 ****************************************************************************/

#ifdef CONFIG_FS_READAHEAD
static int rwb_rhreload(struct rwbuffer_s *rwb, off_t startblock)
{
  /* Get the block number +1 of the last block that will fit in the
   * read-ahead buffer
   */

  off_t  endblock  = startblock + rwb->rhmaxblocks;
  size_t nblocks;
  int    ret;

  /* Reset the read buffer */

  rwb_resetrhbuffer(rwb);

  /* Make sure that we don't read past the end of the device */

  if (endblock > rwb->nblocks)
    {
      endblock = rwb->nblocks;
    }

  nblocks = endblock - startblock;

  /* Now perform the read */

  ret = rwb->rhreload(rwb->dev, rwb->rhbuffer, startblock, nblocks);
  if (ret == nblocks)
    {
      /* Update information about what is in the read-ahead buffer */

      rwb->rhnblocks    = nblocks;
      rwb->rhblockstart = startblock;

      /* The return value is not the number of blocks we asked to be loaded. */

      return nblocks;
    }

  return -EIO;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: rwb_initialize
 ****************************************************************************/

int rwb_initialize(FAR struct rwbuffer_s *rwb)
{
  uint32_t allocsize;

  /* Sanity checking */

  DEBUGASSERT(rwb != NULL);
  DEBUGASSERT(rwb->blocksize > 0);
  DEBUGASSERT(rwb->nblocks > 0);
  DEBUGASSERT(rwb->dev != NULL);

  /* Setup so that rwb_uninitialize can handle a failure */

#ifdef CONFIG_FS_WRITEBUFFER
  DEBUGASSERT(rwb->wrflush!= NULL);
  rwb->wrbuffer = NULL;
#endif
#ifdef CONFIG_FS_READAHEAD
  DEBUGASSERT(rwb->rhreload != NULL);
  rwb->rhbuffer = NULL;
#endif

#ifdef CONFIG_FS_WRITEBUFFER
  fvdbg("Initialize the write buffer\n");

  /* Initialize the write buffer access semaphore */

  sem_init(&rwb->wrsem, 0, 1);

  /* Initialize write buffer parameters */

  rwb_resetwrbuffer(rwb);

  /* Allocate the write buffer */

  rwb->wrbuffer = NULL;
  if (rwb->wrmaxblocks > 0)
    {
      allocsize     = rwb->wrmaxblocks * rwb->blocksize;
      rwb->wrbuffer = kmalloc(allocsize);
      if (!rwb->wrbuffer)
        {
          fdbg("Write buffer kmalloc(%d) failed\n", allocsizee);
          return -ENOMEM;
        }
    }
  
  fvdbg("Write buffer size: %d bytes\n", allocsize);
#endif /* CONFIG_FS_WRITEBUFFER */

#ifdef CONFIG_FS_READAHEAD
  fvdbg("Initialize the read-ahead buffer\n");

  /* Initialize the read-ahead buffer access semaphore */

  sem_init(&rwb->rhsem, 0, 1);

  /* Initialize read-ahead buffer parameters */

  rwb_resetrhbuffer(rwb);

  /* Allocate the read-ahead buffer */

  rwb->rhbuffer = NULL;
  if (rwb->rhmaxblocks > 0)
    {
      allocsize     = rwb->rhmaxblocks * rwb->blocksize;
      rwb->rhbuffer = kmalloc(allocsize);
      if (!rwb->rhbuffer)
        {
          fdbg("Read-ahead buffer kmalloc(%d) failed\n", allocsize);
          return -ENOMEM;
        }
    }
  
  fvdbg("Read-ahead buffer size: %d bytes\n", allocsize);
#endif /* CONFIG_FS_READAHEAD */
  return 0;
}

/****************************************************************************
 * Name: rwb_uninitialize
 ****************************************************************************/

void rwb_uninitialize(FAR struct rwbuffer_s *rwb)
{
#ifdef CONFIG_FS_WRITEBUFFER
  rwb_wrcanceltimeout(rwb);
  sem_destroy(&rwb->wrsem);
  if (rwb->wrbuffer)
    {
      kfree(rwb->wrbuffer);
    }
#endif

#ifdef CONFIG_FS_READAHEAD
  sem_destroy(&rwb->rhsem);
  if (rwb->rhbuffer)
    {
      kfree(rwb->rhbuffer);
    }
#endif
}

/****************************************************************************
 * Name: rwb_read
 ****************************************************************************/

int rwb_read(FAR struct rwbuffer_s *rwb, off_t startblock, uint32_t nblocks,
             FAR uint8_t *rdbuffer)
{
  uint32_t remaining;
 
  fvdbg("startblock=%ld nblocks=%ld rdbuffer=%p\n",
        (long)startblock, (long)nblocks, rdbuffer);

#ifdef CONFIG_FS_WRITEBUFFER
  /* If the new read data overlaps any part of the write buffer, then
   * flush the write data onto the physical media before reading.  We
   * could attempt some more exotic handling -- but this simple logic
   * is well-suited for simple streaming applications.
   */

  if (rwb->wrmaxblocks > 0)
    {
      /* If the write buffer overlaps the block(s) requested, then flush the
       * write buffer.
       */

      rwb_semtake(&rwb->wrsem);
      if (rwb_overlap(rwb->wrblockstart, rwb->wrnblocks, startblock, nblocks))
        {
          rwb_wrflush(rwb);
        }
      rwb_semgive(&rwb->wrsem);
    }
#endif

#ifdef CONFIG_FS_READAHEAD
  /* Loop until we have read all of the requested blocks */

  rwb_semtake(&rwb->rhsem);
  for (remaining = nblocks; remaining > 0;)
    {
      /* Is there anything in the read-ahead buffer? */

      if (rwb->rhnblocks > 0)
        {
          off_t  startblock = startblock;
          size_t nbufblocks = 0;
          off_t  bufferend;

          /* Loop for each block we find in the read-head buffer.  Count the
           * number of buffers that we can read from read-ahead buffer.
           */

          bufferend = rwb->rhblockstart + rwb->rhnblocks;

          while ((startblock >= rwb->rhblockstart) &&
                 (startblock < bufferend) &&
                 (remaining > 0))
            {
              /* This is one more that we will read from the read ahead buffer */

              nbufblocks++;

              /* And one less that we will read from the media */

              startblock++;
              remaining--;
            }

          /* Then read the data from the read-ahead buffer */

          rwb_bufferread(rwb, startblock, nbufblocks, &rdbuffer);
        }

      /* If we did not get all of the data from the buffer, then we have to refill
       * the buffer and try again.
       */

      if (remaining > 0)
        {
          int ret = rwb_rhreload(rwb, startblock);
          if (ret < 0)
            {
              fdbg("ERROR: Failed to fill the read-ahead buffer: %d\n", -ret);
              return ret;
            }
        }
    }

  /* On success, return the number of blocks that we were requested to read.
   * This is for compatibility with the normal return of a block driver read
   * method
   */

  rwb_semgive(&rwb->rhsem);
  return 0;
#else
  return rwb->rhreload(rwb->dev, startblock, nblocks, rdbuffer);
#endif
}

/****************************************************************************
 * Name: rwb_write
 ****************************************************************************/

int rwb_write(FAR struct rwbuffer_s *rwb, off_t startblock,
              size_t nblocks, FAR const uint8_t *wrbuffer)
{
  int ret;

#ifdef CONFIG_FS_READAHEAD
  /* If the new write data overlaps any part of the read buffer, then
   * flush the data from the read buffer.  We could attempt some more
   * exotic handling -- but this simple logic is well-suited for simple
   * streaming applications.
   */

  rwb_semtake(&rwb->rhsem);
  if (rwb_overlap(rwb->rhblockstart, rwb->rhnblocks, startblock, nblocks))
    {
      rwb_resetrhbuffer(rwb);
    }
  rwb_give(&rwb->rhsem);
#endif

#ifdef CONFIG_FS_WRITEBUFFER
  fvdbg("startblock=%d wrbuffer=%p\n", startblock, wrbuffer);

  /* Use the block cache unless the buffer size is bigger than block cache */   
     
  if (nblocks > rwb->wrmaxblocks)
    {
      /* First flush the cache */

      rwb_semtake(&rwb->wrsem);
      rwb_wrflush(rwb);
      rwb_semgive(&rwb->wrsem);

      /* Then transfer the data directly to the media */

      ret = rwb->wrflush(rwb->dev, startblock, nblocks, wrbuffer);
    }
  else
    {
      /* Buffer the data in the write buffer */

      ret = rwb_writebuffer(rwb, startblock, nblocks, wrbuffer);
    }

  /* On success, return the number of blocks that we were requested to write.
   * This is for compatibility with the normal return of a block driver write
   * method
   */

  return ret;

#else

  return rwb->wrflush(rwb->dev, startblock, nblocks, wrbuffer);

#endif
}

/****************************************************************************
 * Name: rwb_mediaremoved
 *
 * Description:
 *   The following function is called when media is removed
 *
 ****************************************************************************/

int rwb_mediaremoved(FAR struct rwbuffer_s *rwb)
{
#ifdef CONFIG_FS_WRITEBUFFER
  rwb_semtake(&rwb->wrsem);
  rwb_resetwrbuffer(rwb);
  rwb_semgive(&rwb->wrsem);
#endif

#ifdef CONFIG_FS_READAHEAD
  rwb_semtake(&rwb->rhsem);
  rwb_resetrhbuffer(rwb);
  rwb_semgive(&rwb->rhsem);
#endif
  return 0;
}

#endif /* CONFIG_FS_WRITEBUFFER || CONFIG_FS_READAHEAD */

