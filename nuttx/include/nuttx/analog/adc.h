/************************************************************************************
 * include/nuttx/analog/adc.h
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-04 initial version
 *            0.2 remove ao_read
 *
 * Derived from include/nuttx/can.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADC_H
#define __INCLUDE_NUTTX_ANALOG_ADC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

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

#if !defined(CONFIG_ADC_FIFOSIZE)
#  define CONFIG_ADC_FIFOSIZE 8
#elif CONFIG_ADC_FIFOSIZE > 255
#  undef  CONFIG_ADC_FIFOSIZE
#  define CONFIG_ADC_FIFOSIZE 255
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

struct adc_msg_s
{
  uint8_t      am_channel;               /* The 8-bit ADC Channel */
  int32_t      am_data;                  /* ADC convert result (4 bytes) */
} packed_struct;

struct adc_fifo_s
{
  sem_t        af_sem;                   /* Counting semaphore */
  uint8_t      af_head;                  /* Index to the head [IN] index in the circular buffer */
  uint8_t      af_tail;                  /* Index to the tail [OUT] index in the circular buffer */
                                         /* Circular buffer of CAN messages */
  struct adc_msg_s af_buffer[CONFIG_ADC_FIFOSIZE];
};

/* This structure defines all of the operations providd by the architecture specific
 * logic.  All fields must be provided with non-NULL function pointers by the
 * caller of can_register().
 */

struct adc_dev_s;
struct adc_ops_s
{
  /* Reset the ADC device.  Called early to initialize the hardware. This
   * is called, before ao_setup() and on error conditions.
   */

  CODE void (*ao_reset)(FAR struct adc_dev_s *dev);

  /* Configure the ADC. This method is called the first time that the ADC
   * device is opened.  This will occur when the port is first opened. 
   * This setup includes configuring and attaching ADC interrupts.  Interrupts
   * are all disabled upon return.
   */

  CODE int (*ao_setup)(FAR struct adc_dev_s *dev);

  /* Disable the ADC.  This method is called when the ADC device is closed.
   * This method reverses the operation the setup method.
   */

  CODE void (*ao_shutdown)(FAR struct adc_dev_s *dev);

  /* Call to enable or disable RX interrupts */

  CODE void (*ao_rxint)(FAR struct adc_dev_s *dev, bool enable);
 
  /* All ioctl calls will be routed through this method */

  CODE int (*ao_ioctl)(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

};

/* This is the device structure used by the driver.  The caller of
 * can_register() must allocate and initialize this structure.  The
 * calling logic need only set all fields to zero except:
 *
 *   The elements of 'ad_ops', and 'ad_priv'
 *
 * The common logic will initialize all semaphores.
 */

struct adc_dev_s
{
  uint8_t                 ad_ocount;     /* The number of times the device has been opened */
  uint8_t                 ad_nrxwaiters; /* Number of threads waiting to enqueue a message */
  sem_t                   ad_closesem;   /* Locks out new opens while close is in progress */
  sem_t                   ad_recvsem;    /* Used to wakeup user waiting for space in ad_recv.buffer */
  struct adc_fifo_s       ad_recv;       /* Describes receive FIFO */
  const struct adc_ops_s *ad_ops;        /* Arch-specific operations */
  void                   *ad_priv;       /* Used by the arch-specific logic */
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
 * "Upper-Half" ADC Driver Interfaces
 ************************************************************************************/
/************************************************************************************
 * Name: adc_register
 *
 * Description:
 *   Register a ADC driver. This function binds an instance of a "lower half" ADC
 *   driver with the "upper half" ADC device and registers that device so that can
 *   be used by application code.
 *
 * Input parameters:
 *   path - The full path to the driver to be registers in the NuttX pseudo-
 *     filesystem.  The recommended convention is to name all PWM drivers
 *     as "/dev/adc", "/dev/adc1", etc.  where the driver path differs only
 *     in the "minor" number at the end of the device name.
 *   dev - A pointer to an instance of lower half ADC driver.  This instance
 *     is bound to the upper half ADC driver and must persists as long as the
 *     upper half driver driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ************************************************************************************/

int adc_register(FAR const char *path, FAR struct adc_dev_s *dev);

/************************************************************************************
 * Name: adc_receive
 *
 * Description:
 *   This function is called from the lower half, platform-specific ADC logic when
 *   new ADC sample data is available.
 *
 * Input Parameters:
 *   dev - The ADC device structure that was previously registered by adc_register()
 *   ch  - And ID for the ADC channel number that generated the data
 *   data - The actualy converted data from the channel.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ************************************************************************************/

int adc_receive(FAR struct adc_dev_s *dev, uint8_t ch, int32_t data);

/************************************************************************************
 * Platform-Independent "Lower Half" ADC Driver Interfaces
 ************************************************************************************/
/************************************************************************************
 * Name: up_ads1255initialize
 *
 * Description:
 *   Initialize the TI ADS 125X lower half driver
 *
 ************************************************************************************/

FAR struct adc_dev_s *up_ads1255initialize(FAR struct spi_dev_s *spi, unsigned int devno);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ANALOG_ADC_H */
