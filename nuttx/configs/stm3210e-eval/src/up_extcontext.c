/************************************************************************************
 * configs/stm3210e-eval/src/up_extcontext.c
 * arch/arm/src/board/up_extcontext.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm3210e-internal.h"

#ifdef CONFIG_STM32_FSMC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_extcontextsave
 *
 * Description:
 *  Save current GPIOs that will used by external memory configurations
 *
 ************************************************************************************/

void stm32_extcontextsave(struct extmem_save_s *save)
{
  DEBUGASSERT(save != NULL);
  save->gpiod_crl = getreg32(STM32_GPIOE_CRL);
  save->gpiod_crh = getreg32(STM32_GPIOE_CRH);
  save->gpioe_crl = getreg32(STM32_GPIOD_CRL);
  save->gpioe_crh = getreg32(STM32_GPIOD_CRH);
  save->gpiof_crl = getreg32(STM32_GPIOF_CRL);
  save->gpiof_crh = getreg32(STM32_GPIOF_CRH);
  save->gpiog_crl = getreg32(STM32_GPIOG_CRL);
  save->gpiog_crh = getreg32(STM32_GPIOG_CRH);
}

/************************************************************************************
 * Name: stm32_extcontextrestore
 *
 * Description:
 *  Restore GPIOs that were used by external memory configurations
 *
 ************************************************************************************/

void stm32_extcontextrestore(struct extmem_save_s *restore)
{
  DEBUGASSERT(restore != NULL);
  putreg32(restore->gpiod_crl, STM32_GPIOE_CRL);
  putreg32(restore->gpiod_crh, STM32_GPIOE_CRH);
  putreg32(restore->gpioe_crl, STM32_GPIOD_CRL);
  putreg32(restore->gpioe_crh, STM32_GPIOD_CRH);
  putreg32(restore->gpiof_crl, STM32_GPIOF_CRL);
  putreg32(restore->gpiof_crh, STM32_GPIOF_CRH);
  putreg32(restore->gpiog_crl, STM32_GPIOG_CRL);
  putreg32(restore->gpiog_crh, STM32_GPIOG_CRH);
}

#endif /* CONFIG_STM32_FSMC */


