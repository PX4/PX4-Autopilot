/************************************************************************************
 * arch/arm/src/stm32/stm32_qencoder.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/qencoder.h>

#include "up_arch.h"
#include "stm32_qencoder.h"

#ifdef CONFIG_QENCODER

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct stm32_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;  /* Lower half callback structure */

  /* STM32 driver-specific fields: */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Lower-half Quadrature Encoder Driver Methods */

static int stm32_setup(FAR struct qe_lowerhalf_s *lower);
static int stm32_shutdown(FAR struct qe_lowerhalf_s *lower);
static int stm32_position(FAR struct qe_lowerhalf_s *lower, int32_t *pos);
static int stm32_reset(FAR struct qe_lowerhalf_s *lower);
static int stm32_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd, unsigned long arg);

 /************************************************************************************
 * Private Data
 ************************************************************************************/
/* The lower half callback structure */

FAR const struct qe_ops_s g_qecallbacks =
{
  .setup    = stm32_setup,
  .shutdown = stm32_shutdown,
  .position = stm32_position,
  .reset    = stm32_reset,
  .ioctl    = stm32_ioctl,
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *   
 *
 ************************************************************************************/

static int stm32_setup(FAR struct qe_lowerhalf_s *lower)
{
#warning "Missing logic"
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware, and
 *   put the system into the lowest possible power usage state *   
 *
 ************************************************************************************/

static int stm32_shutdown(FAR struct qe_lowerhalf_s *lower)
{
#warning "Missing logic"
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ************************************************************************************/

static int stm32_position(FAR struct qe_lowerhalf_s *lower, int32_t *pos)
{
#warning "Missing logic"
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ************************************************************************************/

static int stm32_reset(FAR struct qe_lowerhalf_s *lower)
{
#warning "Missing logic"
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ************************************************************************************/

static int stm32_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd, unsigned long arg)
{
  /* No ioctl commands supported */

  return -ENOTTY;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called from
 *   board-specific logic after input pins have been configured.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *
 * Returned Values:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ************************************************************************************/

int stm32_qeinitialize(FAR const char *devpath)
{
  FAR struct stm32_lowerhalf_s *lower;
  int ret;

  /* Allocate an instance to the device-specific, lower-half state structure */

  lower = (FAR struct stm32_lowerhalf_s *)kmalloc(sizeof(struct stm32_lowerhalf_s));
  if (lower)
    {
      return -ENOMEM;
    }

  /* Initialize the allocated state structure */

  lower->ops = &g_qecallbacks;

  /* Register the lower-half driver */

  ret = qe_register(devpath, (FAR struct qe_lowerhalf_s *)lower);
  if (ret < 0)
    {
      kfree(lower);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_QENCODER */
