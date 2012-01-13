/************************************************************************************
 * arch/arm/src/stm32/stm32_tim.h
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_TIM_H
#define __ARCH_ARM_SRC_STM32_STM32_TIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/stm32_tim.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Helpers **************************************************************************/

#define STM32_TIM_SETMODE(d,mode)       ((d)->ops->setmode(d,mode))
#define STM32_TIM_SETCLOCK(d,freq)      ((d)->ops->setclock(d,freq))
#define STM32_TIM_SETPERIOD(d,period)   ((d)->ops->setperiod(d,period))
#define STM32_TIM_SETCHANNEL(d,ch,mode) ((d)->ops->setchannel(d,ch,mode))
#define STM32_TIM_SETCOMPARE(d,ch,comp) ((d)->ops->setcompare(d,ch,comp))
#define STM32_TIM_GETCAPTURE(d,ch)      ((d)->ops->getcapture(d,ch))
#define STM32_TIM_SETISR(d,hnd,s)       ((d)->ops->setisr(d,hnd,s))
#define STM32_TIM_ENABLEINT(d,s)        ((d)->ops->enableint(d,s))
#define STM32_TIM_DISABLEINT(d,s)       ((d)->ops->disableint(d,s))
#define STM32_TIM_ACKINT(d,s)           ((d)->ops->ackint(d,s))

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* TIM Device Structure */

struct stm32_tim_dev_s
{
  struct stm32_tim_ops_s *ops;
};

/* TIM Modes of Operation */

typedef enum
{
  STM32_TIM_MODE_UNUSED       = -1,

  /* One of the following */

  STM32_TIM_MODE_MASK         = 0x0310,
  STM32_TIM_MODE_DISABLED     = 0x0000,
  STM32_TIM_MODE_UP           = 0x0100,
  STM32_TIM_MODE_DOWN         = 0x0110,
  STM32_TIM_MODE_UPDOWN       = 0x0200,
  STM32_TIM_MODE_PULSE        = 0x0300,

  /* One of the following */

  STM32_TIM_MODE_CK_INT       = 0x0000,
//STM32_TIM_MODE_CK_INT_TRIG  = 0x0400,
//STM32_TIM_MODE_CK_EXT       = 0x0800,
//STM32_TIM_MODE_CK_EXT_TRIG  = 0x0C00,

  /* Clock sources, OR'ed with CK_EXT */

//STM32_TIM_MODE_CK_CHINVALID = 0x0000,
//STM32_TIM_MODE_CK_CH1       = 0x0001,
//STM32_TIM_MODE_CK_CH2       = 0x0002,
//STM32_TIM_MODE_CK_CH3       = 0x0003,
//STM32_TIM_MODE_CK_CH4       = 0x0004

  /* Todo: external trigger block */

} stm32_tim_mode_t;

/* TIM Channel Modes */

typedef enum
{
  STM32_TIM_CH_DISABLED       = 0x00,

  /* Common configuration */

  STM32_TIM_CH_POLARITY_POS   = 0x00,
  STM32_TIM_CH_POLARITY_NEG   = 0x01,

  /* MODES: */

  STM32_TIM_CH_MODE_MASK      = 0x06,

  /* Output Compare Modes */

  STM32_TIM_CH_OUTPWM         = 0x04,     /** Enable standard PWM mode, active high when counter < compare */
//STM32_TIM_CH_OUTCOMPARE     = 0x06,

  // TODO other modes ... as PWM capture, ENCODER and Hall Sensor
//STM32_TIM_CH_INCAPTURE      = 0x10,
//STM32_TIM_CH_INPWM          = 0x20
//STM32_TIM_CH_DRIVE_OC   -- open collector mode

} stm32_tim_channel_t;

/* TIM Operations */

struct stm32_tim_ops_s
{
  /* Basic Timers */

  int  (*setmode)(FAR struct stm32_tim_dev_s *dev, stm32_tim_mode_t mode);
  int  (*setclock)(FAR struct stm32_tim_dev_s *dev, uint32_t freq);
  void (*setperiod)(FAR struct stm32_tim_dev_s *dev, uint32_t period);

  /* General and Advanced Timers Adds */

  int  (*setchannel)(FAR struct stm32_tim_dev_s *dev, uint8_t channel, stm32_tim_channel_t mode);
  int  (*setcompare)(FAR struct stm32_tim_dev_s *dev, uint8_t channel, uint32_t compare);
  int  (*getcapture)(FAR struct stm32_tim_dev_s *dev, uint8_t channel);

  int  (*setisr)(FAR struct stm32_tim_dev_s *dev, int (*handler)(int irq, void *context), int source);
  void (*enableint)(FAR struct stm32_tim_dev_s *dev, int source);
  void (*disableint)(FAR struct stm32_tim_dev_s *dev, int source);
  void (*ackint)(FAR struct stm32_tim_dev_s *dev, int source);
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/* Power-up timer and get its structure */

EXTERN FAR struct stm32_tim_dev_s * stm32_tim_init(int timer);

/* Power-down timer, mark it as unused */

EXTERN int stm32_tim_deinit(FAR struct stm32_tim_dev_s * dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_TIM_H */
