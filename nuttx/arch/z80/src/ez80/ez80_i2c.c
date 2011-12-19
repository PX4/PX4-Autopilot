/****************************************************************************
 * arch/z80/src/ez80/ez80_i2c.c
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
#include <arch/io.h>
#include <arch/board/board.h>

#include "ez80f91.h"
#include "ez80f91_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ez80_i2cdev_s
{
  const struct i2c_ops_s *ops; /* I2C vtable */
  uint16_t ccr;                /* Clock control register value */
  uint16_t addr;               /* 7- or 10-bit address */
  uint8_t  addr10 : 1;         /* 1=Address is 10-bit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Misc. Helpers */

static void i2c_setccr(uint16_t ccr);
static uint16_t i2c_getccr(uint32_t frequency);
static uint8_t i2c_waitiflg(void);
static void i2c_clriflg(void);
static void i2c_start(void);
static void i2c_stop(void);
static int  i2c_sendaddr(struct ez80_i2cdev_s *priv, uint8_t readbit);

/* I2C methods */

static uint32_t i2c_setfrequency(FAR struct i2c_dev_s *dev, uint32_t frequency);
static int  i2c_setaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
static int  i2c_write(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen);
static int  i2c_read(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* This function is normally prototyped int the ZiLOG header file sio.h */

extern uint32_t get_freq(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_currccr;      /* Current setting of I2C CCR register */
static bool    g_initialized;  /* true:I2C has been initialized */
static sem_t   g_i2csem;       /* Serialize I2C transfers */

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
 * Name: i2c_setccr
 *
 * Description:
 *   Set the current BRG value for this transaction
 *
 * Input Parameters:
 *   ccr - BRG to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_setccr(uint16_t ccr)
{
  if (g_currccr != ccr)
    {
      outp(EZ80_I2C_CCR, ccr);
      g_currccr = ccr;
    }
}

/****************************************************************************
 * Name: i2c_getccr
 *
 * Description:
 *   Calculate the BRG value
 *
 * Input Parameters:
 *   fscl - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint16_t i2c_getccr(uint32_t fscl)
{
  uint32_t fsamp;
  uint32_t ftmp;
  uint8_t  n;
  uint8_t  m;

  /* The sampling frequency is given by:
   *
   *   fsamp = sysclock / 2**N
   *
   * And the I2C clock is determined by:
   *
   *   fscl = sysclock / 10 / (M + 1) / 2**N
   *        = fsamp / 10 / (M + 1)
   *
   * The fsmp must be >= 10 * fscl.  The best solution is the smallest value of
   * N so that the sampling rate is the highest subject to:
   *
   * The minimum value of the fsamp is given by:
   */

   fsamp = 10 * fscl;

   /* Now, serarch for the smallest value of N that results in the actual
    * fsamp >= the ideal fsamp.  Fortunately, we only have to check at most
    * eight values.
    */

   if (fsamp >= EZ80_SYS_CLK_FREQ)
     {
       ftmp = EZ80_SYS_CLK_FREQ / 10;
       n    = 0;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 1))
     {
       ftmp = (EZ80_SYS_CLK_FREQ >> 1) / 10;
       n    = 1;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 2))
     {
       ftmp = (EZ80_SYS_CLK_FREQ >> 2) / 10;
       n    = 2;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 3))
     {
       ftmp = (EZ80_SYS_CLK_FREQ >> 3) / 10;
       n    = 3;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 4))
     {
       ftmp = (EZ80_SYS_CLK_FREQ >> 4) / 10;
       n     = 4;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 5))
     {
       ftmp = (EZ80_SYS_CLK_FREQ >> 5) / 10;
       n    = 5;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 6))
     {
       ftmp = (EZ80_SYS_CLK_FREQ >> 6) / 10;
       n     = 6;
     }
   else if (fsamp >= (EZ80_SYS_CLK_FREQ >> 7))
     {
       ftmp  = (EZ80_SYS_CLK_FREQ >> 7) / 10;
       n     = 7;
     }
   else
     {
       ftmp  = (EZ80_SYS_CLK_FREQ >> 7) / 10;
       fscl  = ftmp;
       n     = 7;
     }
  
  /* Finally, get M:
   *
   *   M = (fsamp / 10) / fscl - 1 = ftmp / fscl - 1
   */

  m = ftmp / fscl;
  if (m > 0)
  {
    if (--m > 15)
      {
        m = 15;
      }
  }

  /* Return the value for CCR */

  return (n << I2C_CCR_NSHIFT) | (m << I2C_CCR_MSHIFT);
}

/****************************************************************************
 * Name: i2c_waitiflg
 *
 * Description:
 *   In polled mode, we have to spin until the IFLG bit in the xxx register
 *   goes to 1, signalling that the last send or receive is complete.  This
 *   could be used to generate an interrupt for a non-polled driver.
 *
 * Input Parameters:
 *   priv -    Device-specific state data
 *   readbit - 0 or I2C_READBIT
 *
 * Returned Value:
 *   The contents of the I2C_SR register at the time that IFLG became 1.
 *
 ****************************************************************************/

static uint8_t i2c_waitiflg(void)
{
  while ((inp(EZ80_I2C_CTL) & I2C_CTL_IFLG) != 0);
  return inp(EZ80_I2C_SR);
}

/****************************************************************************
 * Name: i2c_clriflg
 *
 * Description:
 *   Clear the IFLAG bit in the I2C_CTL register, acknowledging the event.
 *   If interrupts are enabled, this would clear the interrupt status.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_clriflg(void)
{
  uint8_t regval = inp(EZ80_I2C_CTL);
  regval &= ~I2C_CTL_IFLG;
  outp(EZ80_I2C_CTL, regval);
}

/****************************************************************************
 * Name: i2c_start
 *
 * Description:
 *   Send the START bit.  IFLAG must be zero; it will go to 1 when it is
 *   time to send the address.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
 
static void i2c_start(void)
{
  uint8_t regval  = inp(EZ80_I2C_CTL);
  regval |= I2C_CTL_STA;
  outp(EZ80_I2C_CTL, regval);
}

/****************************************************************************
 * Name: i2c_stop
 *
 * Description:
 *   Send the STOP bit.  This terminates the I2C transfer and reverts back
 *   to IDLE mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
 
static void i2c_stop(void)
{
  uint8_t regval  = inp(EZ80_I2C_CTL);
  regval |= I2C_CTL_STP;
  outp(EZ80_I2C_CTL, regval);
}

/****************************************************************************
 * Name: i2c_sendaddr
 *
 * Description:
 *   Send the 8- or 11-bit address for either a read or a write transaction.
 *
 * Input Parameters:
 *   priv -    Device-specific state data
 *   readbit - 0 or I2C_READBIT
 *
 * Returned Value:
 *   0: Success, IFLG is set and DATA can be sent or received.
 
 *   Or <0: Negated error value.  IFLG is cleared.
 *
 *   -EIO: Irrecoverable (or unexpected) error occured
 *   -EAGAIN: And 
 *
 ****************************************************************************/

static int i2c_sendaddr(struct ez80_i2cdev_s *priv, uint8_t readbit)
{
  uint8_t sr;
  int ret = OK;

  /* Wait for the IFLG bit to transition to 1.  At this point, we should
   * have status == 8 meaning that the start bit was sent successfully.
   */

  sr = i2c_waitiflg();
#ifdef CONFIG_DEBUG
  if (sr != I2C_SR_MSTART)
    {
      /* This error should never occur */

      dbg("Bad START status: %02x\n", sr);
      i2c_clriflg();
      return -EIO;
    }
#endif

  /* Now send the address */

  if (!priv->addr10)
    {
      /* Load the I2C_DR with the 8-bit I2C slave address and clear the
       * IFLG.  Clearing the IFLAG will cause the address to be transferred.
       */

      outp(EZ80_I2C_DR, (uint8_t)I2C_ADDR8(priv->addr) | readbit);
      i2c_clriflg();
 
      /* And wait for the address transfer to complete */

      sr = i2c_waitiflg();
      if (sr != I2C_SR_MADDRWRACK && sr != I2C_SR_MADDRWR)
        {
          dbg("Bad ADDR8 status: %02x\n", sr);
          goto failure;
        }
    }
  else
    {
      /* Load the I2C_DR with upper part of the 10->16-bit I2C slave address
       * and clear the IFLG.  Clearing the IFLAG will cause the address to
       * be transferred.
       */

      outp(EZ80_I2C_DR, (uint8_t)I2C_ADDR10H(priv->addr) | readbit);
      i2c_clriflg();

      /* And wait for the address transfer to complete */

      sr = i2c_waitiflg();
      if (sr != I2C_SR_MADDRWRACK && sr != I2C_SR_MADDRWR)
        {
         dbg("Bad ADDR10H status: %02x\n", sr);
         goto failure;
        }

      /* Now send the lower 8 bits of the 10-bit address */

      outp(EZ80_I2C_DR, (uint8_t)I2C_ADDR10L(priv->addr));
      i2c_clriflg();

      /* And wait for the address transfer to complete */

      sr = i2c_waitiflg();
      if (sr != I2C_SR_MADDR2WRACK && sr != I2C_SR_MADDR2WR)
        {
          dbg("Bad ADDR10L status: %02x\n", sr);
          goto failure;
        }
    }

  return OK;

  /* We don't attempt any fancy status-based error recovery */

failure:
#ifdef CONFIG_DEBUG
  switch (sr)
    {
      case I2C_SR_ARBLOST1: /* Arbitration lost in address or data byte */
      case I2C_SR_ARBLOST2: /* Arbitration lost in address as master, slave address and Write bit received, ACK transmitted */
      case I2C_SR_ARBLOST3: /* Arbitration lost in address as master, General Call address received, ACK transmitted */
      case I2C_SR_ARBLOST4: /* Arbitration lost in address as master, slave address and Read bit received, ACK transmitted */
        dbg("Arbitration lost: %02x\n", sr);
        i2c_clriflg();
        return -EAGAIN;

      default:
        dbg("Unexpected status: %02x\n", sr);
        i2c_clriflg();
        return -EIO;
    }
#else
  i2c_clriflg();
  return -EAGAIN;
#endif
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
  FAR struct ez80_i2cdev_s *priv = (FAR struct ez80_i2cdev_s *)dev;

  /* Sanity Check */

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      dbg("Invalid inputs\n");
      return -EINVAL;
    }
#endif

  /* Calculate and save the BRG (we won't apply it until the first transfer) */

  priv->ccr = i2c_getccr(frequency);
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
  FAR struct ez80_i2cdev_s *priv = (FAR struct ez80_i2cdev_s *)dev;

  /* Sanity Check */

#ifdef CONFIG_DEBUG
  if (!dev || (unsigned)addr > 0x7f || (nbits != 7 && nbits != 10))
    {
      dbg("Invalid inputs\n");
      return -EINVAL;
    }
#endif

  /* Save the 7- or 10-bit address */

  priv->addr   = addr;
  priv->addr10 = (nbits == 10);
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
  FAR struct ez80_i2cdev_s *priv = (FAR struct ez80_i2cdev_s *)dev;
  const uint8_t *ptr;
  uint8_t sr;
  int retry;
  int count;
  int ret;

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

  i2c_setccr(priv->ccr);

  /* Retry as necessary to send this whole message */

  for (retry = 0; retry < 100; retry++)
    {
      /* Enter MASTER TRANSMIT mode by setting the STA bit in the I2C_CTL
       * register to 1. The I2C then tests the I2C bus and transmits a START
       * condition when the bus is free.
       */

      i2c_start();

      /* When a START condition is transmitted, the IFLG bit is 1.  Then we may
       * send the I2C slave address.
       */

      ret = i2c_sendaddr(priv, 0);
      if (ret < 0)
        {
          if (ret == -EAGAIN)
            {
              continue;
            }
          else
            {
              goto failure;
            }
        }

      /* Then send all of the bytes in the buffer */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Load the I2C_DR with next data byte and clear the IFLG.  Clearing
           * the IFLAG will cause the data to be transferred.
           */

          outp(EZ80_I2C_DR, *ptr++);
          i2c_clriflg();

          /* And wait for the data transfer to complete */

          sr = i2c_waitiflg();
          if (sr != I2C_SR_MDATAWRACK && sr != I2C_SR_MDATAWR)
            {
              dbg("Bad DATA status: %02x\n", sr);
              i2c_clriflg();
              if (sr == I2C_SR_ARBLOST1)
                {
                   /* Arbitration lost, break out of the inner loop and
                    * try sending the message again 
                    */

                   break;
                }

              /* Otherwise, it is fatal (shouldn't happen) */

              ret = -EIO;
              goto failure;
            }

          /* Data byte was sent successfully.  Was that the last byte? */

          else if (count <= 1)
            {
              /* When all bytes are transmitted, the microcontroller must
               * write a 1 to the STP bit in the I2C_CTL register. The
               * I2C then transmits a STOP condition, clears the STP bit
               * and returns to an idle state.
               */

              i2c_stop();

              ret = OK;
              goto success;
            }
        }
    }

  /* If we get here, we timed out without successfully sending the message */

  ret = -ETIMEDOUT;

success:
failure:
  i2c_semgive();
  return ret;
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
  FAR struct ez80_i2cdev_s *priv = (FAR struct ez80_i2cdev_s *)dev;
  uint8_t *ptr;
  uint8_t regval;
  int retry;
  int count;
  int ret;

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

  i2c_setccr(priv->ccr);

  /* Retry as necessary to receive the whole message */

  for (retry = 0; retry < 100; retry++)    
    {
       /* Enter MASTER TRANSMIT mode by setting the STA bit in the I2C_CTL
        * register to 1. The I2C then tests the I2C bus and transmits a START
        * condition when the bus is free.
        */

      i2c_start();

      /* When a START condition is transmitted, the IFLG bit is 1.  Then we may
       * send the I2C slave address.
       */

      ret = i2c_sendaddr(priv, 0);
      if (ret < 0)
        {
          if (ret == -EAGAIN)
            {
              continue;
            }
          else
            {
              goto failure;
            }
        }

      /* Now loop to receive each data byte */

      ptr = buffer;
      for (count = buflen; count; count--)
        {
          /* Is this the last byte? If so, we must NACK it */

          regval  = inp(EZ80_I2C_CTL);
          if (count <= 1)
            {
              /* If the AAK bit is cleared to 0 during a transfer, the I2C
               * transmits a NACK bit after the next byte is received.
               */
             
              regval &= ~I2C_CTL_AAK;
            }
          else
            {
              /* If the AAK bit in the I2C_CTL register is set to 1 then an
               * ACK bit is transmitted and the IFLG bit is set after each
               * byte is received.
               */

              regval |= I2C_CTL_AAK;
            }
          outp(EZ80_I2C_CTL, regval);

          /* Wait for IFLG to be set meaning that incoming data is 
           * available in the I2C_DR registers.
           */

          regval = i2c_waitiflg();

          /* Data byte received in MASTER mode, ACK transmitted */

          if (regval == I2C_SR_MDATARDACK)
          {
            /* Since we just ACKed the incoming byte, it must NOT be the last */

            DEBUGASSERT(count > 1);

            /* Receive the data and clear the IFLGS */

            *ptr++ = inp(EZ80_I2C_DR);
            i2c_clriflg();
          }

        /* Data byte received in MASTER mode, NACK transmitted */

        else if (regval == I2C_SR_MDATARDNAK)
          {
            /* Since we just NACKed the incoming byte, it must be the last */

            DEBUGASSERT(count <= 1);

            /* When all bytes are received and the NACK has been sent, then the
             * microcontroller must write 1 to the STP bit in the I2C_CTL
             * register. The I2C then transmits a STOP condition, clears
             * the STP bit and returns to an idle state.
             */

            i2c_stop();
            i2c_clriflg();

            ret = OK;
            goto success;
          }

        /* Arbitration lost in address or data byte */

        else if (regval == I2C_SR_ARBLOST1)
          {
            /* Clear the IFLG and break out of the inner loop.
             * this will cause the whole transfer to start over
             */

            dbg("Arbitration lost: %02x\n", regval);
            i2c_clriflg();
            break;
          }
          
        /* Unexpected status response */

        else
          {
            dbg("Unexpected status: %02x\n", regval);
            i2c_clriflg();
            ret = -EIO;
            goto failure;
          }
        }
    }
  ret = -ETIMEDOUT;

success:
failure:
  i2c_semgive();
  return ret;
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
  FAR struct ez80_i2cdev_s *i2c;
  uint16_t ccr;
  uint8_t  regval;
 
  if (!g_initialized)
    {
      /* Set up some initial BRG value */

      ccr = i2c_getccr(100*1000);
      i2c_setccr(ccr);

      /* No GPIO setup is required -- I2C pints, SCL/SDA are not multiplexed */
  
      /* This semaphore enforces serialized access for I2C transfers */

      sem_init(&g_i2csem, 0, 1);

      /* Enable I2C -- but not interrupts */

      regval  = inp(EZ80_I2C_CTL);
      regval |= I2C_CTL_ENAB;
      outp(EZ80_I2C_CTL, regval);
    }

  /* Now, allocate an I2C instance for this caller */

  i2c = (FAR struct ez80_i2cdev_s *)malloc(sizeof(FAR struct ez80_i2cdev_s));
  if (i2c)
    {
      /* Initialize the allocated instance */

      i2c->ops = &g_ops;
      i2c->ccr = g_currccr;
    }
  return (FAR struct i2c_dev_s *)i2c;
} 
