/****************************************************************************
 * drivers/can.c
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/can.h>

#include <arch/irq.h>

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing CAN */

#ifdef CONFIG_DEBUG_CAN
#  define candbg    dbg
#  define canvdbg   vdbg
#  define canlldbg  lldbg
#  define canllvdbg llvdbg
#else
#  define candbg(x...)
#  define canvdbg(x...)
#  define canlldbg(x...)
#  define canllvdbg(x...)
#endif

/* Timing Definitions *******************************************************/

#define HALF_SECOND_MSEC 500
#define HALF_SECOND_USEC 500000L

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int            can_open(FAR struct file *filep);
static int            can_close(FAR struct file *filep);
static ssize_t        can_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static int            can_xmit(FAR struct can_dev_s *dev);
static ssize_t        can_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static inline ssize_t can_rtrread(FAR struct can_dev_s *dev, FAR struct canioctl_rtr_s *rtr);
static int            can_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_canops =
{
  can_open,  /* open */
  can_close, /* close */
  can_read,  /* read */
  can_write, /* write */
  0,         /* seek */
  can_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0        /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: can_open
 *
 * Description:
 *   This function is called whenever the CAN device is opened.
 *
 ************************************************************************************/

static int can_open(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  uint8_t               tmp;
  int                   ret   = OK;

  canvdbg("ocount: %d\n", dev->cd_ocount);

  /* If the port is the middle of closing, wait until the close is finished */

  if (sem_wait(&dev->cd_closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Increment the count of references to the device.  If this the first
       * time that the driver has been opened for this device, then initialize
       * the device.
       */

      tmp = dev->cd_ocount + 1;
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
              ret = dev_setup(dev);
              if (ret == OK)
                {
                  /* Mark the FIFOs empty */

                  dev->cd_xmit.tx_head  = 0;
                  dev->cd_xmit.tx_queue = 0;
                  dev->cd_xmit.tx_tail  = 0;
                  dev->cd_recv.rx_head  = 0;
                  dev->cd_recv.rx_tail  = 0;

                  /* Finally, Enable the CAN RX interrupt */

                  dev_rxint(dev, true);

                  /* Save the new open count on success */

                  dev->cd_ocount = tmp;
                }
              irqrestore(flags);
            }
        }
      sem_post(&dev->cd_closesem);
    }
  return ret;
}

/************************************************************************************
 * Name: can_close
 *
 * Description:
 *   This routine is called when the CAN device is closed.
 *   It waits for the last remaining data to be sent.
 *
 ************************************************************************************/

static int can_close(FAR struct file *filep)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  irqstate_t            flags;
  int                   ret = OK;

  canvdbg("ocount: %d\n", dev->cd_ocount);

  if (sem_wait(&dev->cd_closesem) != OK)
    {
      ret = -errno;
    }
  else
    {
      /* Decrement the references to the driver.  If the reference count will
       * decrement to 0, then uninitialize the driver.
       */

      if (dev->cd_ocount > 1)
        {
          dev->cd_ocount--;
          sem_post(&dev->cd_closesem);
        }
      else
        {
          /* There are no more references to the port */

          dev->cd_ocount = 0;

          /* Stop accepting input */

          dev_rxint(dev, false);

          /* Now we wait for the transmit FIFO to clear */

          while (dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail)
            {
#ifndef CONFIG_DISABLE_SIGNALS
               usleep(HALF_SECOND_USEC);
#else
               up_mdelay(HALF_SECOND_MSEC);
#endif
            }

          /* And wait for the TX hardware FIFO to drain */

          while (!dev_txempty(dev))
            {
#ifndef CONFIG_DISABLE_SIGNALS
              usleep(HALF_SECOND_USEC);
#else
              up_mdelay(HALF_SECOND_MSEC);
#endif
            }

          /* Free the IRQ and disable the CAN device */

          flags = irqsave();       /* Disable interrupts */
          dev_shutdown(dev);       /* Disable the CAN */
          irqrestore(flags);

          sem_post(&dev->cd_closesem);
        }
    }
  return ret;
}

/************************************************************************************
 * Name: can_read
 *
 * Description:
 *   Read standard CAN messages
 *
 ************************************************************************************/

static ssize_t can_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  size_t                nread;
  irqstate_t            flags;
  int                   ret   = 0;

  canvdbg("buflen: %d\n", buflen);

  /* The caller must provide enough memory to catch the smallest possible message
   * This is not a system error condition, but we won't permit it,  Hence we return 0.
   */

  if (buflen >= CAN_MSGLEN(0))
    {
      /* Interrupts must be disabled while accessing the cd_recv FIFO */

      flags = irqsave();
      while (dev->cd_recv.rx_head == dev->cd_recv.rx_tail)
        {
          /* The receive FIFO is empty -- was non-blocking mode selected? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto return_with_irqdisabled;
            }

          /* Wait for a message to be received */

          ret = sem_wait(&dev->cd_recv.rx_sem);
          if (ret < 0)
            {
              ret = -errno;
              goto return_with_irqdisabled;
            }
        }

      /* The cd_recv FIFO is not empty.  Copy all buffered data that will fit
       * in the user buffer.
       */

      nread = 0;
      do
        {
          /* Will the next message in the FIFO fit into the user buffer? */

          FAR struct can_msg_s *msg = &dev->cd_recv.rx_buffer[dev->cd_recv.rx_head];
          int msglen = CAN_MSGLEN(msg->cm_hdr.ch_dlc);

          if (nread + msglen > buflen)
           {
             break;
           }

          /* Copy the message to the user buffer */

          memcpy(&buffer[nread], msg, msglen);
          nread += msglen;

          /* Increment the head of the circular message buffer */

          if (++dev->cd_recv.rx_head >= CONFIG_CAN_FIFOSIZE)
            {
              dev->cd_recv.rx_head = 0;
            }
        }
      while (dev->cd_recv.rx_head != dev->cd_recv.rx_tail);

      /* All on the messages have bee transferred.  Return the number of bytes
       * that were read.
       */

      ret = nread;

return_with_irqdisabled:
      irqrestore(flags);
    }
  return ret;
}

/************************************************************************************
 * Name: can_xmit
 *
 * Description:
 *   Send the message at the head of the cd_xmit FIFO
 *
 * Assumptions:
 *   Called with interrupts disabled
 *
 ************************************************************************************/

static int can_xmit(FAR struct can_dev_s *dev)
{
  int tmpndx;
  int ret = -EBUSY;

  canllvdbg("xmit head: %d queue: %d tail: %d\n",
            dev->cd_xmit.tx_head, dev->cd_xmit.tx_queue, dev->cd_xmit.tx_tail);

  /* If there is nothing to send, then just disable interrupts and return */

  if (dev->cd_xmit.tx_head == dev->cd_xmit.tx_tail)
    {
      DEBUGASSERT(dev->cd_xmit.tx_queue == dev->cd_xmit.tx_head);
      dev_txint(dev, false);
      return -EIO;
    }

  /* Check if we have already queued all of the data in the TX fifo.
   *
   * tx_tail:  Incremented in can_write each time a message is queued in the FIFO
   * tx_head:  Incremented in can_txdone each time a message completes
   * tx_queue: Incremented each time that a message is sent to the hardware.
   *
   * Logically (ignoring buffer wrap-around): tx_head <= tx_queue <= tx_tail
   * tx_head == tx_queue == tx_tail means that the FIFO is empty
   * tx_head < tx_queue == tx_tail means that all data has been queued, but
   * we are still waiting for transmissions to complete.
   */

  while (dev->cd_xmit.tx_queue != dev->cd_xmit.tx_tail && dev_txready(dev))
    {
      /* No.. The fifo should not be empty in this case */

      DEBUGASSERT(dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail);

      /* Increment the FIFO queue index before sending (because dev_send()
       * might call can_txdone().
       */

      tmpndx = dev->cd_xmit.tx_queue;
      if (++dev->cd_xmit.tx_queue >= CONFIG_CAN_FIFOSIZE)
        {
          dev->cd_xmit.tx_queue = 0;
        }

      /* Send the next message at the FIFO queue index */

      ret = dev_send(dev, &dev->cd_xmit.tx_buffer[tmpndx]);
      if (ret != OK)
        {
          candbg("dev_send failed: %d\n", ret);
          break;
        }
    }

  /* Make sure that TX interrupts are enabled */

  dev_txint(dev, true);
  return ret;
}

/************************************************************************************
 * Name: can_write
 ************************************************************************************/

static ssize_t can_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct can_dev_s    *dev   = inode->i_private;
  FAR struct can_txfifo_s *fifo  = &dev->cd_xmit;
  FAR struct can_msg_s    *msg;
  bool                     inactive;
  ssize_t                  nsent = 0;
  irqstate_t               flags;
  int                      nexttail;
  int                      msglen;
  int                      ret   = 0;

  canvdbg("buflen: %d\n", buflen);

  /* Interrupts must disabled throughout the following */

  flags = irqsave();

  /* Check if the TX is inactive when we started. In certain race conditionas, there
   * may be a pending interrupt to kick things back off, but we will here that there
   * is not.  That the hardware is IDLE and will need to be kick-started.
   */

  inactive = dev_txempty(dev);

  /* Add the messages to the FIFO.  Ignore any trailing messages that are
   * shorter than the minimum.
   */

  while ((buflen - nsent) >= CAN_MSGLEN(0))
    {
      /* Check if adding this new message would over-run the drivers ability to enqueue
       * xmit data.
       */

      nexttail = fifo->tx_tail + 1;
      if (nexttail >= CONFIG_CAN_FIFOSIZE)
        {
          nexttail = 0;
        }

      /* If the XMIT fifo becomes full, then wait for space to become available */

      while (nexttail == fifo->tx_head)
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

          /* If the TX hardware was inactive when we started, then we will have
           * start the XMIT sequence generate the TX done interrrupts needed
           * to clear the FIFO.
           */

          if (inactive)
            {
              can_xmit(dev);
            }

          /* Wait for a message to be sent */

          do
            {
              DEBUGASSERT(dev->cd_ntxwaiters < 255);
              dev->cd_ntxwaiters++;
              ret = sem_wait(&fifo->tx_sem);
              dev->cd_ntxwaiters--;

              if (ret < 0 && errno != EINTR)
                {
                  ret = -errno;
                  goto return_with_irqdisabled;
                }
            }
          while (ret < 0);

          /* Re-check the FIFO state */

          inactive = dev_txempty(dev);
        }

      /* We get here if there is space at the end of the FIFO.  Add the new
       * CAN message at the tail of the FIFO.
       */

      msg    = (FAR struct can_msg_s *)&buffer[nsent];
      msglen = CAN_MSGLEN(msg->cm_hdr.ch_dlc);
      memcpy(&fifo->tx_buffer[fifo->tx_tail], msg, msglen);

      /* Increment the tail of the circular buffer */

      fifo->tx_tail = nexttail;

      /* Increment the number of bytes that were sent */

      nsent += msglen;
    }

 /* We get here after all messages have been added to the FIFO.  Check if
  * we need to kick of the XMIT sequence.
  */

 if (inactive)
   {
     can_xmit(dev);
   }

  /* Return the number of bytes that were sent */

  ret = nsent;

return_with_irqdisabled:
  irqrestore(flags);
  return ret;
}

/************************************************************************************
 * Name: can_rtrread
 *
 * Description:
 *   Read RTR messages.  The RTR message is a special message -- it is an outgoing
 *   message that says "Please re-transmit the message with the same identifier as
 *   this message.  So the RTR read is really a send-wait-receive operation.
 *
 ************************************************************************************/

static inline ssize_t can_rtrread(FAR struct can_dev_s *dev, FAR struct canioctl_rtr_s *rtr)
{
  FAR struct can_rtrwait_s *wait = NULL;
  irqstate_t                flags;
  int                       i;
  int                       ret = -ENOMEM;

  /* Disable interrupts through this operation */

  flags = irqsave();

  /* Find an avaiable slot in the pending RTR list */

  for (i = 0; i < CONFIG_CAN_NPENDINGRTR; i++)
    {
      FAR struct can_rtrwait_s *tmp = &dev->cd_rtr[i];
      if (!rtr->ci_msg)
        {
          tmp->cr_id  = rtr->ci_id;
          tmp->cr_msg = rtr->ci_msg;
          dev->cd_npendrtr++;
          wait        = tmp;
          break;
        }
    }

  if (wait)
    {
      /* Send the remote transmission request */

      ret = dev_remoterequest(dev, wait->cr_id);
      if (ret == OK)
        {
          /* Then wait for the response */

          ret = sem_wait(&wait->cr_sem);
        }
    }
  irqrestore(flags);
  return ret;
}

/************************************************************************************
 * Name: can_ioctl
 ************************************************************************************/

static int can_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode     *inode = filep->f_inode;
  FAR struct can_dev_s *dev   = inode->i_private;
  int               ret   = OK;

  canvdbg("cmd: %d arg: %ld\n", cmd, arg);

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* CANIOCTL_RTR: Send the remote transmission request and wait for the response.
       * Argument is a reference to struct canioctl_rtr_s (casting to uintptr_t first
       * eliminates complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      case CANIOCTL_RTR:
        ret = can_rtrread(dev, (struct canioctl_rtr_s*)((uintptr_t)arg));
        break;

      /* Not a "built-in" ioctl command.. perhaps it is unique to this device driver */

      default:
        ret = dev_ioctl(dev, cmd, arg);
        break;
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: can_register
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ************************************************************************************/

int can_register(FAR const char *path, FAR struct can_dev_s *dev)
{
  int i;

  /* Initialize the CAN device structure */

  dev->cd_ocount = 0;

  sem_init(&dev->cd_xmit.tx_sem, 0, 0);
  sem_init(&dev->cd_recv.rx_sem, 0, 0);
  sem_init(&dev->cd_closesem, 0, 1);

  for (i = 0; i < CONFIG_CAN_NPENDINGRTR; i++)
    {
      sem_init(&dev->cd_rtr[i].cr_sem, 0, 0);
      dev->cd_rtr[i].cr_msg = NULL;
      dev->cd_npendrtr--;
    }

  /* Initialize/reset the CAN hardware */

  dev_reset(dev);

  /* Register the CAN device */

  canvdbg("Registering %s\n", path);
  return register_driver(path, &g_canops, 0666, dev);
}

/************************************************************************************
 * Name: can_receive
 *
 * Description:
 *   Called from the CAN interrupt handler when new read data is available
 *
 * Parameters:
 *   dev  - CAN driver state structure
 *   hdr  - CAN message header
 *   data - CAN message data (if DLC > 0)
 *
 * Assumptions:
 *   CAN interrupts are disabled.
 *
 ************************************************************************************/

int can_receive(FAR struct can_dev_s *dev, FAR struct can_hdr_s *hdr, FAR uint8_t *data)
{
  FAR struct can_rxfifo_s *fifo = &dev->cd_recv;
  FAR uint8_t             *dest;
  int                      nexttail;
  int                      err = -ENOMEM;
  int                      i;

  canllvdbg("ID: %d DLC: %d\n", hdr->ch_id, hdr->ch_dlc);

  /* Check if adding this new message would over-run the drivers ability to enqueue
   * read data.
   */

  nexttail = fifo->rx_tail + 1;
  if (nexttail >= CONFIG_CAN_FIFOSIZE)
    {
      nexttail = 0;
    }

  /* First, check if this response matches any RTR response that we may be waiting for */

  if (dev->cd_npendrtr > 0)
    {
      /* There are pending RTR requests -- search the lists of requests
       * and see any any matches this new message.
       */

      for (i = 0; i < CONFIG_CAN_NPENDINGRTR; i++)
        {
          FAR struct can_rtrwait_s *rtr = &dev->cd_rtr[i];
          FAR struct can_msg_s     *msg = rtr->cr_msg;

          /* Check if the entry is valid and if the ID matches.  A valid entry has
           * a non-NULL receiving address
           */

          if (msg && hdr->ch_id == rtr->cr_id)
            {
              /* We have the response... copy the data to the user's buffer */

              memcpy(&msg->cm_hdr, hdr, sizeof(struct can_hdr_s));
              for (i = 0, dest = msg->cm_data; i < hdr->ch_dlc; i++)
                {
                  *dest++ = *data++;
                }

              /* Mark the entry unused */

              rtr->cr_msg = NULL;

              /* And restart the waiting thread */

              sem_post(&rtr->cr_sem);
            }
        }
    }

  /* Refuse the new data if the FIFO is full */

  if (nexttail != fifo->rx_head)
    {
      /* Add the new, decoded CAN message at the tail of the FIFO */

      memcpy(&fifo->rx_buffer[fifo->rx_tail].cm_hdr, hdr, sizeof(struct can_hdr_s));
      for (i = 0, dest = fifo->rx_buffer[fifo->rx_tail].cm_data; i < hdr->ch_dlc; i++)
        {
          *dest++ = *data++;
        }

      /* Increment the tail of the circular buffer */

      fifo->rx_tail = nexttail;

      /* The increment the counting semaphore. The maximum value should be
       * CONFIG_CAN_FIFOSIZE -- one possible count for each allocated message buffer.
       */

      sem_post(&fifo->rx_sem);
      err = OK;
    }
  return err;
}

/************************************************************************************
 * Name: can_txdone
 *
 * Description:
 *   Called from the CAN interrupt handler at the completion of a send operation.
 *
 * Parameters:
 *   dev  - The specific CAN device
 *   hdr  - The 16-bit CAN header
 *   data - An array contain the CAN data.
 *
 * Return:
 *   OK on success; a negated errno on failure.
 *
 ************************************************************************************/

int can_txdone(FAR struct can_dev_s *dev)
{
  int ret = -ENOENT;

  canllvdbg("xmit head: %d queue: %d tail: %d\n",
            dev->cd_xmit.tx_head, dev->cd_xmit.tx_queue, dev->cd_xmit.tx_tail);

  /* Verify that the xmit FIFO is not empty */

  if (dev->cd_xmit.tx_head != dev->cd_xmit.tx_tail)
    {
      DEBUGASSERT(dev->cd_xmit.tx_head != dev->cd_xmit.tx_queue);

      /* Remove the message at the head of the xmit FIFO */

      if (++dev->cd_xmit.tx_head >= CONFIG_CAN_FIFOSIZE)
        {
          dev->cd_xmit.tx_head = 0;
        }

      /* Send the next message in the FIFO */

      ret = can_xmit(dev);

      /* Are there any threads waiting for space in the TX FIFO? */

      if (ret == OK && dev->cd_ntxwaiters > 0)
        {
          /* Yes.. Inform them that new xmit space is available */

          ret = sem_post(&dev->cd_xmit.tx_sem);
        }
    }
  return ret;
}

#endif /* CONFIG_CAN */
