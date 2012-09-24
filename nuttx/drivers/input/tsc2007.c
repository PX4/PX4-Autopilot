/****************************************************************************
 * drivers/input/tsc2007.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "1.2V to 3.6V, 12-Bit, Nanopower, 4-Wire Micro TOUCH SCREEN CONTROLLER
 *    with I2C Interface," SBAS405A March 2007, Revised, March 2009, Texas
 *    Instruments Incorporated
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

/* The TSC2007 is an analog interface circuit for a human interface touch
 * screen device. All peripheral functions are controlled through the command
 * byte and onboard state machines.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c.h>
#include <nuttx/wqueue.h>

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/tsc2007.h>

#include "tsc2007.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Reference counting is partially implemented, but not needed in the
 * current design.
 */

#undef CONFIG_TSC2007_REFCNT

/* I don't think that it is necessary to activate the converters before
 * making meaurements.  However, I will keep this functionality enabled
 * until I have a change to prove that that activation is unnecessary.
 */

#undef  CONFIG_TSC2007_ACTIVATE
#define CONFIG_TSC2007_ACTIVATE 1

/* Driver support ***********************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* Commands *****************************************************************/

#define TSC2007_SETUP           (TSC2007_CMD_FUNC_SETUP)
#ifdef CONFIG_TSC2007_8BIT
#  define TSC2007_ACTIVATE_Y    (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YON)
#  define TSC2007_MEASURE_Y     (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YPOS)
#  define TSC2007_ACTIVATE_X    (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_XON)
#  define TSC2007_MEASURE_X     (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_XPOS)
#  define TSC2007_ACTIVATE_Z    (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YXON)
#  define TSC2007_MEASURE_Z1    (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_Z1POS)
#  define TSC2007_MEASURE_Z2    (TSC2007_CMD_8BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_Z2POS)
#  define TSC2007_ENABLE_PENIRQ (TSC2007_CMD_8BIT | TSC2007_CMD_PWRDN_IRQEN)
#else
#  define TSC2007_ACTIVATE_Y    (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YON)
#  define TSC2007_MEASURE_Y     (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YPOS)
#  define TSC2007_ACTIVATE_X    (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_XON)
#  define TSC2007_MEASURE_X     (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_XPOS)
#  define TSC2007_ACTIVATE_Z    (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_YXON)
#  define TSC2007_MEASURE_Z1    (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_Z1POS)
#  define TSC2007_MEASURE_Z2    (TSC2007_CMD_12BIT | TSC2007_CMD_ADCON_IRQDIS | TSC2007_CMD_FUNC_Z2POS)
#  define TSC2007_ENABLE_PENIRQ (TSC2007_CMD_12BIT | TSC2007_CMD_PWRDN_IRQEN)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes the state of one contact */

enum tsc2007_contact_3
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one TSC2007 sample */

struct tsc2007_sample_s
{
  uint8_t  id;                         /* Sampled touch point ID */
  uint8_t  contact;                    /* Contact state (see enum tsc2007_contact_e) */
  bool     valid;                      /* True: x,y,pressure contain valid, sampled data */
  uint16_t x;                          /* Measured X position */
  uint16_t y;                          /* Measured Y position */
  uint16_t pressure;                   /* Calculated pressure */
};

/* This structure describes the state of one TSC2007 driver instance */

struct tsc2007_dev_s
{
#ifdef CONFIG_TSC2007_MULTIPLE
  FAR struct tsc2007_dev_s *flink;     /* Supports a singly linked list of drivers */
#endif
#ifdef CONFIG_TSC2007_REFCNT
  uint8_t crefs;                       /* Number of times the device has been opened */
#endif
  uint8_t nwaiters;                    /* Number of threads waiting for TSC2007 data */
  uint8_t id;                          /* Current touch point ID */
  volatile bool penchange;             /* An unreported event is buffered */
  sem_t devsem;                        /* Manages exclusive access to this structure */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  FAR struct tsc2007_config_s *config; /* Board configuration data */
  FAR struct i2c_dev_s *i2c;           /* Saved I2C driver instance */
  struct work_s work;                  /* Supports the interrupt handling "bottom half" */
  struct tsc2007_sample_s sample;      /* Last sampled touch point data */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_TSC2007_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tsc2007_notify(FAR struct tsc2007_dev_s *priv);
static int tsc2007_sample(FAR struct tsc2007_dev_s *priv,
                          FAR struct tsc2007_sample_s *sample);
static int tsc2007_waitsample(FAR struct tsc2007_dev_s *priv,
                              FAR struct tsc2007_sample_s *sample);
#ifdef CONFIG_TSC2007_ACTIVATE
static int tsc2007_activate(FAR struct tsc2007_dev_s *priv, uint8_t cmd);
#endif
static int tsc2007_transfer(FAR struct tsc2007_dev_s *priv, uint8_t cmd);
static void tsc2007_worker(FAR void *arg);
static int tsc2007_interrupt(int irq, FAR void *context);

/* Character driver methods */

static int tsc2007_open(FAR struct file *filep);
static int tsc2007_close(FAR struct file *filep);
static ssize_t tsc2007_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int tsc2007_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int tsc2007_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the the vtable that supports the character driver interface */

static const struct file_operations tsc2007_fops =
{
  tsc2007_open,    /* open */
  tsc2007_close,   /* close */
  tsc2007_read,    /* read */
  0,               /* write */
  0,               /* seek */
  tsc2007_ioctl    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , tsc2007_poll   /* poll */
#endif
};

/* If only a single TSC2007 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_TSC2007_MULTIPLE
static struct tsc2007_dev_s g_tsc2007;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct tsc2007_dev_s *g_tsc2007list;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsc2007_notify
 ****************************************************************************/

static void tsc2007_notify(FAR struct tsc2007_dev_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the TSC2007
       * is no longer available.
       */

      sem_post(&priv->waitsem); 
    }

  /* If there are threads waiting on poll() for TSC2007 data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_TSC2007_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          ivdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
}

/****************************************************************************
 * Name: tsc2007_sample
 ****************************************************************************/

static int tsc2007_sample(FAR struct tsc2007_dev_s *priv,
                          FAR struct tsc2007_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = irqsave();

  /* Is there new TSC2007 sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct tsc2007_sample_s ));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* Next.. no contact.  Increment the ID so that next contact ID
           * will be unique.  X/Y positions are no longer valid.
           */

          priv->sample.contact = CONTACT_NONE;
          priv->sample.valid   = false;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
       {
          /* First report -- next report will be a movement */

         priv->sample.contact = CONTACT_MOVE;
       }

      priv->penchange = false;
      ret = OK;
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: tsc2007_waitsample
 ****************************************************************************/

static int tsc2007_waitsample(FAR struct tsc2007_dev_s *priv,
                              FAR struct tsc2007_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   *
   * In addition, we will also disable pre-emption to prevent other threads
   * from getting control while we muck with the semaphores.
   */

  sched_lock();
  flags = irqsave();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  sem_post(&priv->devsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is availble.
   */

  while (tsc2007_sample(priv, sample) < 0)
    {
      /* Wait for a change in the TSC2007 state */
 
      priv->nwaiters++;
      ret = sem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          /* If we are awakened by a signal, then we need to return
           * the failure now.
           */

          DEBUGASSERT(errno == EINTR);
          ret = -EINTR;
          goto errout;
        }
    }

  /* Re-acquire the the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = sem_wait(&priv->devsem);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  irqrestore(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the TSC2007 for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: tsc2007_activate
 ****************************************************************************/

#ifdef CONFIG_TSC2007_ACTIVATE
static int tsc2007_activate(FAR struct tsc2007_dev_s *priv, uint8_t cmd)
{
   struct i2c_msg_s msg;
   uint8_t data;
   int ret;

  /* Send the setup command (with no ACK) followed by the A/D converter
   * activation command (ACKed).
   */

   data = TSC2007_SETUP;

   msg.addr   = priv->config->address; /* 7-bit address */
   msg.flags  = 0;                     /* Write transaction, beginning with START */
   msg.buffer = &data;                 /* Transfer from this address */
   msg.length = 1;                     /* Send one byte following the address */
 
   /* Ignore errors from the setup command (because it is not ACKed) */

   (void)I2C_TRANSFER(priv->i2c, &msg, 1);

   /* Now activate the A/D converter */

   data = cmd;

   msg.addr   = priv->config->address; /* 7-bit address */
   msg.flags  = 0;                     /* Write transaction, beginning with START */
   msg.buffer = &data;                 /* Transfer from this address */
   msg.length = 1;                     /* Send one byte following the address */
 
   ret = I2C_TRANSFER(priv->i2c, &msg, 1);
   if (ret < 0)
     {
       idbg("I2C_TRANSFER failed: %d\n", ret);
     }
   return ret;
}
#else
#  define tsc2007_activate(p,c)
#endif

/****************************************************************************
 * Name: tsc2007_transfer
 ****************************************************************************/

static int tsc2007_transfer(FAR struct tsc2007_dev_s *priv, uint8_t cmd)
{
   struct i2c_msg_s msg;
   uint8_t data12[2];
   int ret;

  /* "A conversion/write cycle begins when the master issues the address
   *  byte containing the slave address of the TSC2007, with the eighth bit
   *  equal to a 0 (R/W = 0)... Once the eighth bit has been received...
   *  the TSC2007 issues an acknowledge.
   *
   * "When the master receives the acknowledge bit from the TSC2007, the
   *  master writes the command byte to the slave... After the command byte
   *  is received by the slave, the slave issues another acknowledge bit.
   *  The master then ends the write cycle by issuing a repeated START or a
   *  STOP condition...
   */

   msg.addr   = priv->config->address; /* 7-bit address */
   msg.flags  = 0;                     /* Write transaction, beginning with START */
   msg.buffer = &cmd;                  /* Transfer from this address */
   msg.length = 1;                     /* Send one byte following the address */
 
   ret = I2C_TRANSFER(priv->i2c, &msg, 1);
   if (ret < 0)
     {
       idbg("I2C_TRANSFER failed: %d\n", ret);
       return ret;
     }

  /* "The input multiplexer channel for the A/D converter is selected when
   *  bits C3 through C0 are clocked in. If the selected channel is an X-,Y-,
   *  or Z-position measurement, the appropriate drivers turn on once the
   *  acquisition period begins.
   *
   * "... the input sample acquisition period starts on the falling edge of
   *  SCL when the C0 bit of the command byte has been latched, and ends
   *  when a STOP or repeated START condition has been issued. A/D conversion
   *  starts immediately after the acquisition period...
   *
   * "For best performance, the I2C bus should remain in an idle state while
   *  an A/D conversion is taking place. ... The master should wait for at
   *  least 10ms before attempting to read data from the TSC2007...
   */

  usleep(10*1000);

  /* "Data access begins with the master issuing a START condition followed
   *  by the address byte ... with R/W = 1.
   *
   * "When the eighth bit has been received and the address matches, the
   *  slave issues an acknowledge. The first byte of serial data then follows
   *  (D11-D4, MSB first).
   *
   * "After the first byte has been sent by the slave, it releases the SDA line
   *  for the master to issue an acknowledge.  The slave responds with the
   *  second byte of serial data upon receiving the acknowledge from the master
   *  (D3-D0, followed by four 0 bits). The second byte is followed by a NOT
   *  acknowledge bit (ACK = 1) from the master to indicate that the last
   *  data byte has been received...
   */

   msg.addr   = priv->config->address; /* 7-bit address */
   msg.flags  = I2C_M_READ;            /* Read transaction, beginning with START */
   msg.buffer = data12;                /* Transfer to this address */
   msg.length = 2;                     /* Read two bytes following the address */
 
   ret = I2C_TRANSFER(priv->i2c, &msg, 1);
   if (ret < 0)
     {
       idbg("I2C_TRANSFER failed: %d\n", ret);
       return ret;
     }

   /* Get the MS 8 bits from the first byte and the remaining LS 4 bits from
    * the second byte.  The valid range of data is then from 0 to 4095 with
    * the LSB unit corresponding to Vref/4096.
    */

   ret = (unsigned int)data12[0] << 4 | (unsigned int)data12[1] >> 4;
   ivdbg("data: 0x%04x\n", ret);
   return ret;
}

/****************************************************************************
 * Name: tsc2007_worker
 ****************************************************************************/

static void tsc2007_worker(FAR void *arg)
{
  FAR struct tsc2007_dev_s    *priv = (FAR struct tsc2007_dev_s *)arg;
  FAR struct tsc2007_config_s *config;   /* Convenience pointer */
  bool                         pendown;  /* true: pend is down */
  uint16_t                     x;        /* X position */
  uint16_t                     y;        /* Y position */
  uint16_t                     z1;       /* Z1 position */
  uint16_t                     z2;       /* Z2 position */
  uint32_t                     pressure; /* Measured pressure */

  ASSERT(priv != NULL);

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Check for pen up or down by reading the PENIRQ GPIO. */

  pendown = config->pendown(config);

  /* Handle the change from pen down to pen up */

  if (!pendown)
    {
      /* Ignore the interrupt if the pen was already down (CONTACT_NONE == pen up and
       * already reported.  CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact == CONTACT_NONE)
        {
          goto errout;
        }
    }

  /* It is a pen down event.  If the last loss-of-contact event has not been
   * processed yet, then we have to ignore the pen down event (or else it will
   * look like a drag event)
   */

  else if (priv->sample.contact == CONTACT_UP)
    {
       goto errout;
    }
  else
    {
      /* Handle all pen down events.  First, sample X, Y, Z1, and Z2 values.
       *
       * "A resistive touch screen operates by applying a voltage across a
       *  resistor network and measuring the change in resistance at a given
       *  point on the matrix where the screen is touched by an input (stylus,
       *  pen, or finger). The change in the resistance ratio marks the location
       *  on the touch screen.
       *
       * "The 4-wire touch screen panel works by applying a voltage across the
       *  vertical or horizontal resistive network.  The A/D converter converts
       *  the voltage measured at the point where the panel is touched. A measurement
       *  of the Y position of the pointing device is made by connecting the X+
       *  input to a data converter chip, turning on the Y+ and Y– drivers, and
       *  digitizing the voltage seen at the X+ input ..."
       *
       * "... it is recommended that whenever the host writes to the TSC2007, the
       *  master processor masks the interrupt associated to PENIRQ. This masking
       *  prevents false triggering of interrupts when the PENIRQ line is disabled
       *  in the cases previously listed."
       */

      (void)tsc2007_activate(priv, TSC2007_ACTIVATE_X);
      y = tsc2007_transfer(priv, TSC2007_MEASURE_Y);


      /* "Voltage is then applied to the other axis, and the A/D converter
       *  converts the voltage representing the X position on the screen. This
       *  process provides the X and Y coordinates to the associated processor."
       */

      (void)tsc2007_activate(priv, TSC2007_ACTIVATE_Y);
      x = tsc2007_transfer(priv, TSC2007_MEASURE_X);

      /* "... To determine pen or finger touch, the pressure of the touch must be
       *  determined. ... There are several different ways of performing this
       *  measurement. The TSC2007 supports two methods. The first method requires
       *  knowing the X-plate resistance, the measurement of the X-position, and two
       *  additional cross panel measurements (Z2 and Z1) of the touch screen."
       *
       *  Rtouch = Rxplate * (X / 4096)* (Z2/Z1 - 1)
       *
       * "The second method requires knowing both the X-plate and Y-plate
       *  resistance, measurement of X-position and Y-position, and Z1 ..."
       *
       *  Rtouch = Rxplate * (X / 4096) * (4096/Z1 - 1) - Ryplate * (1 - Y/4096)
       *
       * Read Z1 and Z2 values.
       */

      (void)tsc2007_activate(priv, TSC2007_ACTIVATE_Z);
      z1 = tsc2007_transfer(priv, TSC2007_MEASURE_Z1);
      (void)tsc2007_activate(priv, TSC2007_ACTIVATE_Z);
      z2 = tsc2007_transfer(priv, TSC2007_MEASURE_Z2);

      /* Power down ADC and enable PENIRQ */

     (void)tsc2007_transfer(priv, TSC2007_ENABLE_PENIRQ);

      /* Now calculate the pressure using the first method, reduced to:
       *
       * Rtouch = X * Rxplate *(Z2 - Z1) * / Z1 / 4096
       */

      if (z1 == 0)
        {
          idbg("Z1 zero\n");
          pressure = 0;
        }
      else
        {
          pressure = (x * config->rxplate * (z2 - z1)) / z1;
          pressure = (pressure + 2048) >> 12;

          ivdbg("Position: (%d,%4d) pressure: %u z1/2: (%d,%d)\n",
                x, y, pressure, z1, z2);

          /* Ignore out of range caculcations */

          if (pressure > 0x0fff)
            {
              idbg("Dropped out-of-range pressure: %d\n", pressure);
              pressure = 0;
            }
        }

      /* Save the measurements */

      priv->sample.x        = x;
      priv->sample.y        = y;
      priv->sample.pressure = pressure;
      priv->sample.valid    = true;
    }

  /* Note the availability of new measurements */

  if (pendown)
    {
      /* If this is the first (acknowledged) pend down report, then report
       * this as the first contact.  If contact == CONTACT_DOWN, it will be
       * set to set to CONTACT_MOVE after the contact is first sampled.
       */

      if (priv->sample.contact != CONTACT_MOVE)
        {
          /* First contact */

          priv->sample.contact = CONTACT_DOWN;
        }
    }
  else /* if (priv->sample.contact != CONTACT_NONE) */
    {
      /* The pen is up.  NOTE: We know from a previous test, that this is a
       * loss of contact condition.  This will be changed to CONTACT_NONE
       * after the loss of contact is sampled.
       */

       priv->sample.contact = CONTACT_UP;
    }

  /* Indicate the availability of new sample data for this ID */

  priv->sample.id = priv->id;
  priv->penchange = true;

  /* Notify any waiters that nes TSC2007 data is available */

  tsc2007_notify(priv);

  /* Exit, re-enabling TSC2007 interrupts */

errout:
  config->enable(config, true);
}

/****************************************************************************
 * Name: tsc2007_interrupt
 ****************************************************************************/

static int tsc2007_interrupt(int irq, FAR void *context)
{
  FAR struct tsc2007_dev_s    *priv;
  FAR struct tsc2007_config_s *config;
  int                          ret;

  /* Which TSC2007 device caused the interrupt? */

#ifndef CONFIG_TSC2007_MULTIPLE
  priv = &g_tsc2007;
#else
  for (priv = g_tsc2007list;
       priv && priv->configs->irq != irq;
       priv = priv->flink);

  ASSERT(priv != NULL);
#endif

  /* Get a pointer the callbacks for convenience (and so the code is not so
   * ugly).
   */

  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* Disable further interrupts */

  config->enable(config, false);

  /* Transfer processing to the worker thread.  Since TSC2007 interrupts are
   * disabled while the work is pending, no special action should be required
   * to protected the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(HPWORK, &priv->work, tsc2007_worker, priv, 0);
  if (ret != 0)
    {
      illdbg("Failed to queue work: %d\n", ret);
    }

  /* Clear any pending interrupts and return success */

  config->clear(config);
  return OK;
}

/****************************************************************************
 * Name: tsc2007_open
 ****************************************************************************/

static int tsc2007_open(FAR struct file *filep)
{
#ifdef CONFIG_TSC2007_REFCNT
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  sem_post(&priv->devsem);
  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: tsc2007_close
 ****************************************************************************/

static int tsc2007_close(FAR struct file *filep)
{
#ifdef CONFIG_TSC2007_REFCNT
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  sem_post(&priv->devsem);
#endif
  return OK;
}

/****************************************************************************
 * Name: tsc2007_read
 ****************************************************************************/

static ssize_t tsc2007_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct tsc2007_dev_s  *priv;
  FAR struct touch_sample_s *report;
  struct tsc2007_sample_s    sample;
  int                        ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Try to read sample data. */

  ret = tsc2007_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
       }

      /* Wait for sample data */

      ret = tsc2007_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  /* In any event, we now have sampled TSC2007 data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = priv->id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;
  report->point[0].pressure  = sample.pressure;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
       /* Pen is now up.  Is the positional data valid?  This is important to
        * know because the release will be sent to the window based on its
       * last positional data.
       */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else
    {
      if (sample.contact == CONTACT_DOWN)
        {
          /* First contact */

          report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else /* if (sample->contact == CONTACT_MOVE) */
        {
          /* Movement of the same contact */

          report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }

      /* A pressure measurement of zero means that pressure is not available */

      if (report->point[0].pressure != 0)
        {
          report->point[0].flags  |= TOUCH_PRESSURE_VALID;
        }
    }

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name:tsc2007_ioctl
 ****************************************************************************/

static int tsc2007_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;

  ivdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      case TSIOC_SETCALIB:  /* arg: Pointer to int calibration value */
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          priv->config->rxplate = *ptr;
        }
        break;

      case TSIOC_GETCALIB:  /* arg: Pointer to int calibration value */
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          *ptr = priv->config->rxplate;
        }
        break;

      case TSIOC_SETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          priv->config->frequency = I2C_SETFREQUENCY(priv->i2c, *ptr);
        }
        break;

      case TSIOC_GETFREQUENCY:  /* arg: Pointer to uint32_t frequency value */
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          DEBUGASSERT(priv->config != NULL && ptr != NULL);
          *ptr = priv->config->frequency;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: tsc2007_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int tsc2007_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode         *inode;
  FAR struct tsc2007_dev_s *priv;
  int                       ret;
  int                       i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tsc2007_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was canceled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          idbg("Missing POLLIN: revents: %08x\n", fds->revents);
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_TSC2007_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_TSC2007_NPOLLWAITERS)
        {
          idbg("No availabled slot found: %d\n", i);
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          tsc2007_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsc2007_register
 *
 * Description:
 *   Configure the TSC2007 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   config  - Persistant board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int tsc2007_register(FAR struct i2c_dev_s *dev,
                     FAR struct tsc2007_config_s *config, int minor)
{
  FAR struct tsc2007_dev_s *priv;
  char devname[DEV_NAMELEN];
#ifdef CONFIG_TSC2007_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  ivdbg("dev: %p minor: %d\n", dev, minor);

  /* Debug-only sanity checks */

  DEBUGASSERT(dev != NULL && config != NULL && minor >= 0 && minor < 100);
  DEBUGASSERT((config->address & 0xfc) == 0x48);
  DEBUGASSERT(config->attach != NULL && config->enable  != NULL &&
              config->clear  != NULL && config->pendown != NULL);

  /* Create and initialize a TSC2007 device driver instance */

#ifndef CONFIG_TSC2007_MULTIPLE
  priv = &g_tsc2007;
#else
  priv = (FAR struct tsc2007_dev_s *)kmalloc(sizeof(struct tsc2007_dev_s));
  if (!priv)
    {
      idbg("kmalloc(%d) failed\n", sizeof(struct tsc2007_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the TSC2007 device driver instance */

  memset(priv, 0, sizeof(struct tsc2007_dev_s));
  priv->i2c    = dev;             /* Save the I2C device handle */
  priv->config = config;          /* Save the board configuration */
  sem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */
  sem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  /* Set the I2C frequency (saving the actual frequency) */

  config->frequency = I2C_SETFREQUENCY(dev, config->frequency);

  /* Set the I2C address and address size */

  ret = I2C_SETADDRESS(dev, config->address, 7);
  if (ret < 0)
    {
      idbg("I2C_SETADDRESS failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Make sure that interrupts are disabled */

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, tsc2007_interrupt);
  if (ret < 0)
    {
      idbg("Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Power down the ADC and enable PENIRQ.  This is the normal state while
   * waiting for a touch event.
   */

  ret = tsc2007_transfer(priv, TSC2007_ENABLE_PENIRQ);
  if (ret < 0)
    {
      idbg("tsc2007_transfer failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Register the device as an input device */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ivdbg("Registering %s\n", devname);

  ret = register_driver(devname, &tsc2007_fops, 0666, priv);
  if (ret < 0)
    {
      idbg("register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* If multiple TSC2007 devices are supported, then we will need to add
   * this new instance to a list of device instances so that it can be
   * found by the interrupt handler based on the recieved IRQ number.
   */

#ifdef CONFIG_TSC2007_MULTIPLE
  flags         = irqsave();
  priv->flink   = g_tsc2007list;
  g_tsc2007list = priv;
  irqrestore(flags);
#endif

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  ret = work_queue(HPWORK, &priv->work, tsc2007_worker, priv, 0);
  if (ret != 0)
    {
      idbg("Failed to queue work: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success (?) */

  return OK;

errout_with_priv:
  sem_destroy(&priv->devsem);
#ifdef CONFIG_TSC2007_MULTIPLE
  kfree(priv);
#endif
  return ret;
}
