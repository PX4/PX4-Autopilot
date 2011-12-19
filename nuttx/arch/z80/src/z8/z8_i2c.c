/****************************************************************************
 * arch/z80/src/z8/z8_i2c.c
 *
 *   Copyright(C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
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
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/i2c.h>
#include <arch/board/board.h>

#include <eZ8.h>  /* eZ8 Register definitions */
#include "chip.h" /* Register bit definitions */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct z8_i2cdev_s
{
  const struct i2c_ops_s *ops; /* I2C vtable */
  uint16_t brg;                /* Baud rate generator value */
  uint8_t  addr;               /* 8-bit address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc. Helpers */

static void i2c_waittxempty(void);
static void i2c_waitrxavail(void);
static void i2c_setbrg(uint16_t brg);
static uint16_t i2c_getbrg(uint32_t frequency);

/* I2C methods */

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency);
static int i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen);
static int i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* This function is normally prototyped int the ZiLOG header file sio.h */

extern uint32_t get_freq(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_currbrg;      /* Current BRG setting */
static bool     g_initialized;  /* true:I2C has been initialized */
static sem_t    g_i2csem;       /* Serialize I2C transfers */

const struct i2c_ops_s g_ops =
{
  i2c_setfrequency,
  i2c_setaddress,
  i2c_write,
  i2c_read,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: i2c_semtake/i2c_semgive
 *
 * Description:
 *   Take/Give the I2C semaphore.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_semtake(void)
{
  /* Take the I2C semaphore (perhaps waiting) */

  while (sem_wait(&g_i2csem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

#define i2c_semgive() sem_post(&g_i2csem)

/****************************************************************************
 * Name: i2c_waittxempty
 *
 * Description:
 *   Wait for the transmit data register to become empty.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_waittxempty(void)
{
  int i;  
  for (i = 0; i < 10000 && (I2CSTAT & I2C_STAT_TDRE) == 0;  i++);
}

/****************************************************************************
 * Name: i2c_waitrxavail
 *
 * Description:
 *   Wait until we have received a full byte of data.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_waitrxavail(void)  
{
  int i;
  for (i = 0; i <= 10000 && (I2CSTAT & (I2C_STAT_RDRF | I2C_STAT_NCKI)) == 0; i++);
}

/****************************************************************************
 * Name: i2c_setbrg
 *
 * Description:
 *   Set the current BRG value for this transaction
 *
 * Input Parameters:
 *   brg - BRG to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_setbrg(uint16_t brg)
{
  if (g_currbrg != brg)
    {
      I2CBRH    = (uint8_t)(brg >> 8);
      I2CBRL    = (uint8_t)(brg & 0xff);
      g_currbrg = brg;
    }
}

/****************************************************************************
 * Name: i2c_getbrg
 *
 * Description:
 *   Calculate the BRG value
 *
 * Input Parameters:
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint16_t i2c_getbrg(uint32_t frequency)
{
  uint32_t sysclock = get_freq();

  /* Max is 400 Kb/sec */

  if (frequency > 400*1000)
    {
      dbg("Invalid inputs\n");
      frequency = 400*1000;
    }

  /* BRG = sysclock / (4 * frequency) */

  return ((sysclock >> 2) + (frequency >> 1)) / frequency;
}

/****************************************************************************
 * Name: i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency. This frequency will be retained in the struct
 *   i2c_dev_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency)
{
  FAR struct z8_i2cdev_s *priv = (FAR struct z8_i2cdev_s *)dev;

  /* Sanity Check */

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      dbg("Invalid inputs\n");
      return -EINVAL;
    }
#endif

  /* Calculate and save the BRG (we won't apply it until the first transfer) */

  priv->brg = i2c_getbrg(frequency);
  return OK;
}

/****************************************************************************
 * Name: i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address. This frequency will be retained in the struct
 *   i2c_dev_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev -     Device-specific state data
 *   address - The I2C slave address
 *   nbits -   The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static int i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
  FAR struct z8_i2cdev_s *priv = (FAR struct z8_i2cdev_s *)dev;

  /* Sanity Check */

#ifdef CONFIG_DEBUG
  if (!dev || (unsigned)addr > 0x7f)
    {
      dbg("Invalid inputs\n");
      return -EINVAL;
    }
#endif

  /* Save the 7-bit address (10-bit address not yet supported) */

  DEBUGASSERT(nbits == 7);
  priv->addr = (uint8_t)addr;
  return OK;
}

/****************************************************************************
 * Name: i2c_write
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen)
{
  FAR struct z8_i2cdev_s *priv = (FAR struct z8_i2cdev_s *)dev;
  const uint8_t *ptr;
  int retry;
  int count;

#ifdef CONFIG_DEBUG
  if (!priv || !buffer || buflen < 1)
    {
      dbg("Invalid inputs\n");
      return -EINVAL;
    }
#endif

  /* Get exclusive access */

  i2c_semtake();

  /* Set the frequency */

  i2c_setbrg(priv->brg);

  /* Retry as necessary to send this whole message */

  for (retry = 0; retry < 100; retry++)
    {
      /* Load the address into the transmit register.  It is not sent
       * until the START bit is set.
       */

      I2CD    = I2C_WRITEADDR8(priv->addr);
      I2CCTL |= I2C_CTL_START;
 
      /* Wait for the xmt buffer to become empty */

      i2c_waittxempty();

      /* Then send all of the bytes in the buffer */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Send a byte of data and wait for it to be sent */

          I2CD = *ptr++;
          i2c_waittxempty();

          /* If this was the last byte, then send STOP immediately.  This
           * is because the ACK will not be valid until the STOP clocks out
           * the last bit.. Hmmm.  If this true then we will never be
           * able to send more than one data byte???
           */
 
          if (count == 1)
            {
              I2CCTL |= I2C_CTL_STOP;

              /* If this last byte was ACKed, then the whole buffer
               * was successfully sent and we can return success.
               */

              if ((I2CSTAT & I2C_STAT_ACK) != 0)
                {
                  i2c_semgive();
                  return OK;
                }

              /* If was was not ACKed, then this inner loop will
               * terminated (because count will decrement to zero
               * and the whole message will be resent
               */
            }

          /* Not the last byte... was this byte ACKed? */

          else if ((I2CSTAT & I2C_STAT_ACK) == 0)
            {
              /* No, flush the buffer and toggle the I2C on and off */

              I2CCTL |= I2C_CTL_FLUSH;
              I2CCTL &= ~I2C_CTL_IEN;
              I2CCTL |= I2C_CTL_IEN;

              /* Break out of the loop early and try again */

              break;
            }
        }
    }
  i2c_semgive();
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: i2c_read
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
 *
 * Input Parameters:
 *   dev -   Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

static int i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen)
{
  FAR struct z8_i2cdev_s *priv = (FAR struct z8_i2cdev_s *)dev;
  uint8_t *ptr;
  int retry;
  int count;

#ifdef CONFIG_DEBUG
  if (!priv || !buffer || buflen < 1)
    {
      dbg("Invalid inputs\n");
      return -EINVAL;
    }
#endif

  /* Get exclusive access */

  i2c_semtake();

  /* Set the frequency */

  i2c_setbrg(priv->brg);

  /* Retry as necessary to receive the whole message */

  for (retry = 0; retry < 100; retry++)    
    {
      /* Load the address into the transmit register.  It is not sent
       * until the START bit is set.
       */

      I2CD = I2C_READADDR8(priv->addr);

      /* If we want only a single byte of data, then set the NACK
       * bit now.
       */
 
      I2CCTL |= I2C_CTL_NAK;

      /* The START bit begins the transaction */

      I2CCTL |= I2C_CTL_START;

      /* Now loop to receive each data byte */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Wait for the receive buffer to fill */

          i2c_waitrxavail();

          /* Did we get a byte?  Or did an error occur? */

          if (I2CSTAT & I2C_STAT_RDRF)
            {
              /* Save the data byte */

              *ptr++ = I2CD;

              /* If the next byte is the last byte, then set NAK now */

              if (count == 2)
                {
                  I2CCTL |= I2C_CTL_NAK;
                }

              /* If this was the last byte, then set STOP and return success */

              else if (count == 1)
                {
                  I2CCTL |= I2C_CTL_STOP;
                  i2c_semgive();
                  return OK;
                }
            }
          /* An error occurred.  Clear byte bus and break out of the loop
           * to retry now.
           */

          else
            {
              /* No, flush the buffer and toggle the I2C on and off */

              I2CCTL |= I2C_CTL_FLUSH;
              I2CCTL &= ~I2C_CTL_IEN;
              I2CCTL |= I2C_CTL_IEN;

              /* Break out of the loop early and try again */

              break;
            }
        }
    }
  i2c_semgive();
  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_dev_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a 
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct i2c_dev_s *up_i2cinitialize(int port)
{
  FAR struct z8_i2cdev_s *i2c;
 
  if (!g_initialized)
    {
      /* Set up some initial BRG value */

      uint16_t brg = i2c_getbrg(100*1000);
      i2c_setbrg(brg);

      /* Make sure that GPIOs are configured for the alternate function (this
       * varies with silicon revisions).
       */
  
      PAADDR = 0x02;
      PACTL |= 0xc0;

      /* This semaphore enforces serialized access for I2C transfers */

      sem_init(&g_i2csem, 0, 1);

      /* Enable I2C -- no interrupts */

      I2CCTL = I2C_CTL_IEN;
    }

  /* Now, allocate an I2C instance for this caller */

  i2c = (FAR struct z8_i2cdev_s *)malloc(sizeof(FAR struct z8_i2cdev_s));
  if (i2c)
    {
      /* Initialize the allocated instance */

      i2c->ops = &g_ops;
      i2c->brg = g_currbrg;
    }
  return (FAR struct i2c_dev_s *)i2c;
} 
