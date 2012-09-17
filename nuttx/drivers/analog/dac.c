/****************************************************************************
 * drivers/analog/dac.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-04 initial version
 * 
 * Derived from drivers/can.c
 *
 *   Copyright (C) 2008-2009Gregory Nutt. All rights reserved.
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
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HALF_SECOND_MSEC 500
#define HALF_SECOND_USEC 500000L


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     dac_open(FAR struct file *filep);
static int     dac_close(FAR struct file *filep);
static ssize_t dac_read(FAR struct file *, FAR char *, size_t);
static ssize_t dac_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int     dac_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations dac_fops =
{
  dac_open,
  dac_close,
  dac_read,
  dac_write,
  0,
  dac_ioctl
#ifndef CONFIG_DISABLE_POLL
  , 0
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/************************************************************************************
 * Name: dac_open
 *
 * Description:
 *   This function is called whenever the DAC device is opened.
 *
 ************************************************************************************/

static int dac_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct dac_dev_s *dev   = inode->i_private;
  uint8_t               tmp;
  int                   ret   = OK;

  /* If the port is the middle of closing, wait until the close is finished */

  if (sem_wait(&dev->ad_closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then initialize
       * the device.
       */

      tmp = dev->ad_ocount + 1;
      if (tmp == 0)
        {
          /* More than 255 opens; uint8_t overflows to zero */

          ret = -EMFILE;
        }
      else
        {
          /* Check if this is the first time that the driver has been opened. */

          if (tmp == 1)
            {
              /* Yes.. perform one time hardware initialization. */

              irqstate_t flags = irqsave();
              ret = dev->ad_ops->ao_setup(dev);
              if (ret == OK)
                {
                  /* Mark the FIFOs empty */

                  dev->ad_xmit.af_head = 0;
                  dev->ad_xmit.af_tail = 0;

                  /* Save the new open count on success */

                  dev->ad_ocount = tmp;
                }
              irqrestore(flags);
            }
        }
      sem_post(&dev->ad_closesem);
    }
  return ret;
}

/************************************************************************************
 * Name: dac_close
 *
 * Description:
 *   This routine is called when the DAC device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ************************************************************************************/

static int dac_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct dac_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret = OK;

  if (sem_wait(&dev->ad_closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->ad_ocount > 1)
        {
          dev->ad_ocount--;
          sem_post(&dev->ad_closesem);
        }
      else
        {
          /* There are no more references to the port */

          dev->ad_ocount = 0;

          /* Now we wait for the transmit FIFO to clear */

          while (dev->ad_xmit.af_head != dev->ad_xmit.af_tail)
            {
#ifndef CONFIG_DISABLE_SIGNALS
               usleep(HALF_SECOND_USEC);
#else
               up_mdelay(HALF_SECOND_MSEC);
#endif
            }

          /* Free the IRQ and disable the DAC device */

          flags = irqsave();       /* Disable interrupts */
          dev->ad_ops->ao_shutdown(dev);       /* Disable the DAC */
          irqrestore(flags);

          sem_post(&dev->ad_closesem);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: dac_read
 ****************************************************************************/

static ssize_t dac_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: dac_xmit
 *
 * Description:
 *   Send the message at the head of the ad_xmit FIFO
 *
 * Assumptions:
 *   Called with interrupts disabled
 *
 ************************************************************************************/

static int dac_xmit(FAR struct dac_dev_s *dev)
{
  bool enable = false;
  int ret = OK;

  /* Check if the xmit FIFO is empty */

  if (dev->ad_xmit.af_head != dev->ad_xmit.af_tail)
    {
      /* Send the next message at the head of the FIFO */

      ret = dev->ad_ops->ao_send(dev, &dev->ad_xmit.af_buffer[dev->ad_xmit.af_head]);

      /* Make sure the TX done interrupts are enabled */

      enable = (ret == OK ? true : false);
    }
  dev->ad_ops->ao_txint(dev, enable);
  return ret;
}

/************************************************************************************
 * Name: dac_write
 ************************************************************************************/

static ssize_t dac_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct dac_dev_s  *dev   = inode->i_private;
  FAR struct dac_fifo_s *fifo  = &dev->ad_xmit;
  FAR struct dac_msg_s  *msg;
  bool                   empty = false;
  ssize_t                nsent = 0;
  irqstate_t             flags;
  int                    nexttail;
  int                    msglen;
  int                    ret   = 0;

  /* Interrupts must disabled throughout the following */

  flags = irqsave();

  /* Check if the TX FIFO was empty when we started.  That is a clue that we have
   * to kick off a new TX sequence.
   */

  empty = (fifo->af_head == fifo->af_tail);

  /* Add the messages to the FIFO.  Ignore any trailing messages that are
   * shorter than the minimum.
   */

  if (buflen % 5 ==0 )
    msglen=5;
  else if (buflen % 4 ==0 )
    msglen=4;
  else if (buflen % 3 ==0 )
    msglen=3;
  else if (buflen % 2 ==0 )
    msglen=2;
  else if (buflen == 1)
    msglen=1;
  else
    msglen=5;

  while ((buflen - nsent) >= msglen )
    {
      /* Check if adding this new message would over-run the drivers ability to enqueue
       * xmit data.
       */

      nexttail = fifo->af_tail + 1;
      if (nexttail >= CONFIG_DAC_FIFOSIZE)
        {
          nexttail = 0;
        }

      /* If the XMIT fifo becomes full, then wait for space to become available */

      while (nexttail == fifo->af_head)
        {
          /* The transmit FIFO is full  -- was non-blocking mode selected? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              if (nsent == 0)
                {
                  ret = -EAGAIN;
                }
              else
                {
                  ret = nsent;
                }
              goto return_with_irqdisabled;
            }

          /* If the FIFO was empty when we started, then we will have
           * start the XMIT sequence to clear the FIFO.
           */

          if (empty)
            {
              dac_xmit(dev);
            }

          /* Wait for a message to be sent */

          do
            {
              ret = sem_wait(&fifo->af_sem);
              if (ret < 0 && errno != EINTR)
                {
                  ret = -errno;
                  goto return_with_irqdisabled;
                }
            }
          while (ret < 0);

          /* Re-check the FIFO state */

          empty = (fifo->af_head == fifo->af_tail);
        }

      /* We get here if there is space at the end of the FIFO.  Add the new
       * CAN message at the tail of the FIFO.
       */

     if (msglen==5)
     {
        msg    = (FAR struct dac_msg_s *)&buffer[nsent];
        memcpy(&fifo->af_buffer[fifo->af_tail], msg, msglen);
     }
     else if(msglen == 4)
     {
         fifo->af_buffer[fifo->af_tail].am_channel=buffer[nsent];
         fifo->af_buffer[fifo->af_tail].am_data=*(uint32_t *)&buffer[nsent];
         fifo->af_buffer[fifo->af_tail].am_data&=0xffffff00;
     }
     else if(msglen == 3)
     {
         fifo->af_buffer[fifo->af_tail].am_channel=buffer[nsent];
         fifo->af_buffer[fifo->af_tail].am_data=(*(uint16_t *)&buffer[nsent+1]);
         fifo->af_buffer[fifo->af_tail].am_data<<=16;
     }
     else if(msglen == 2)
     {
         fifo->af_buffer[fifo->af_tail].am_channel=0;
         fifo->af_buffer[fifo->af_tail].am_data=(*(uint16_t *)&buffer[nsent]);
         fifo->af_buffer[fifo->af_tail].am_data<<=16;
     }
     else if(msglen == 1)
     {
         fifo->af_buffer[fifo->af_tail].am_channel=0;
         fifo->af_buffer[fifo->af_tail].am_data=buffer[nsent];
         fifo->af_buffer[fifo->af_tail].am_data<<=24;
     }
      /* Increment the tail of the circular buffer */

      fifo->af_tail = nexttail;

      /* Increment the number of bytes that were sent */

      nsent += msglen;
    }

 /* We get here after all messages have been added to the FIFO.  Check if
  * we need to kick of the XMIT sequence.
  */

 if (empty)
   {
     dac_xmit(dev);
   }

  /* Return the number of bytes that were sent */

  ret = nsent;

return_with_irqdisabled:
  irqrestore(flags);
  return ret;
}

/************************************************************************************
 * Name: dac_ioctl
 ************************************************************************************/

static int dac_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct dac_dev_s *dev   = inode->i_private;
  int               ret   = OK;

  ret = dev->ad_ops->ao_ioctl(dev, cmd, arg);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: dac_txdone
 *
 * Description:
 *   Called from the DAC interrupt handler at the completion of a send operation.
 *
 * Return:
 *   OK on success; a negated errno on failure.
 *
 ************************************************************************************/

int dac_txdone(FAR struct dac_dev_s *dev)
{
  int ret = -ENOENT;

  /* Verify that the xmit FIFO is not empty */

  if (dev->ad_xmit.af_head != dev->ad_xmit.af_tail)
    {
      /* Remove the message at the head of the xmit FIFO */

      if (++dev->ad_xmit.af_head >= CONFIG_DAC_FIFOSIZE)
        {
          dev->ad_xmit.af_head = 0;
        }

      /* Send the next message in the FIFO */

      ret = dac_xmit(dev);
      if (ret == OK)
        {
          /* Inform any waiting threads that new xmit space is available */

          ret = sem_post(&dev->ad_xmit.af_sem);
        }
    }
  return ret;
}

int dac_register(FAR const char *path, FAR struct dac_dev_s *dev)
{
  /* Initialize the DAC device structure */

  dev->ad_ocount = 0;

  sem_init(&dev->ad_xmit.af_sem, 0, 0);
  sem_init(&dev->ad_closesem, 0, 1);

  dev->ad_ops->ao_reset(dev);

  return register_driver(path, &dac_fops, 0555, dev);
}

