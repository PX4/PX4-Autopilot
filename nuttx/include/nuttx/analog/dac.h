/************************************************************************************
 * include/nuttx/analog/dac.h
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-04 initial version
 *
 * Derived from include/nuttx/can.h
 *
 *   Copyright (C) 2008, 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_ANALOG_DAC_H
#define __INCLUDE_NUTTX_ANALOG_DAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Default configuration settings that may be overridden in the board configuration.
 * file.  The configured size is limited to 255 to fit into a uint8_t.
 */

#if !defined(CONFIG_DAC_FIFOSIZE)
#  define CONFIG_DAC_FIFOSIZE 8
#elif CONFIG_DAC_FIFOSIZE > 255
#  undef  CONFIG_DAC_FIFOSIZE
#  define CONFIG_DAC_FIFOSIZE 255
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct dac_msg_s
{
  uint8_t      am_channel;               /* The 8-bit DAC Channel */
  int32_t      am_data;                  /* DAC convert result (4 bytes) */
};

struct dac_fifo_s
{
  sem_t         af_sem;                  /* Counting semaphore */
  uint8_t       af_head;                 /* Index to the head [IN] index in the circular buffer */
  uint8_t       af_tail;                 /* Index to the tail [OUT] index in the circular buffer */
                                         /* Circular buffer of CAN messages */
  struct dac_msg_s af_buffer[CONFIG_DAC_FIFOSIZE];
};

/* This structure defines all of the operations providd by the architecture specific
 * logic.  All fields must be provided with non-NULL function pointers by the
 * caller of dac_register().
 */

struct dac_dev_s;
struct dac_ops_s
{
  /* Reset the DAC device.  Called early to initialize the hardware. This
   * is called, before ao_setup() and on error conditions.
   */

  CODE void (*ao_reset)(FAR struct dac_dev_s *dev);

  /* Configure the DAC. This method is called the first time that the DAC
   * device is opened.  This will occur when the port is first opened. 
   * This setup includes configuring and attaching DAC interrupts.  Interrupts
   * are all disabled upon return.
   */

  CODE int (*ao_setup)(FAR struct dac_dev_s *dev);

  /* Disable the DAC.  This method is called when the DAC device is closed.
   * This method reverses the operation the setup method.
   */
  CODE void (*ao_shutdown)(FAR struct dac_dev_s *dev);

  /* Call to enable or disable TX interrupts */

  CODE void (*ao_txint)(FAR struct dac_dev_s *dev, bool enable);

  /* This method will send one message on the DAC */

  CODE int (*ao_send)(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);

  /* All ioctl calls will be routed through this method */

  CODE int (*ao_ioctl)(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);

};

/* This is the device structure used by the driver.  The caller of
 * dac_register() must allocate and initialize this structure.  The
 * calling logic need only set all fields to zero except:
 *
 *   The elements of 'ad_ops', and 'ad_priv'
 *
 * The common logic will initialize all semaphores.
 */

struct dac_dev_s
{
  uint8_t                 ad_ocount;   /* The number of times the device has been opened */
  uint8_t                 ad_nchannel; /* Number of dac channel */
  sem_t                   ad_closesem; /* Locks out new opens while close is in progress */
  struct dac_fifo_s       ad_xmit;     /* Describes receive FIFO */
  const struct dac_ops_s *ad_ops;      /* Arch-specific operations */
  void                   *ad_priv;     /* Used by the arch-specific logic */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/************************************************************************************
 * Name: dac_register
 *
 * Description:
 *   Register a dac driver.
 *
 * Input Parameters:
 *    path - The full path to the DAC device to be registered.  This could be, as an
 *      example, "/dev/dac0"
 *    dev - An instance of the device-specific DAC interface
 *
 * Returned Value:
 *    Zero on success; A negated errno value on failure.
 *
 ************************************************************************************/

int dac_register(FAR const char *path, FAR struct dac_dev_s *dev);

/************************************************************************************
 * Name: dac_txdone
 *
 * Description:
 *   Called from the DAC interrupt handler at the completion of a send operation.
 *
 * Input Parameters:
 *    dev - An instance of the device-specific DAC interface
 *
 * Return:
 *   OK on success; a negated errno on failure.
 *
 ************************************************************************************/

int dac_txdone(FAR struct dac_dev_s *dev);

/************************************************************************************
 * DAC Initialization functions
 *
 * Architecture-specific versions will have prototypes in architect-specific header
 * files.  Common DAC implementations in drivers/analog will have prototypes listed
 * below.
 *
 ************************************************************************************/

FAR struct dac_dev_s *up_ad5410initialize(FAR struct spi_dev_s *spi, unsigned int devno);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_DAC_H */
