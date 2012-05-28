/************************************************************************************
 * configs/stm3220g-eval/src/up_extmem.c
 * arch/arm/src/board/up_extmem.c
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
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_fsmc.h"
#include "stm32_gpio.h"
#include "stm32_internal.h"
#include "stm3220g-internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  warning "FSMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

#define STM32_FSMC_NADDRCONFIGS 26
#define STM32_FSMC_NDATACONFIGS 16

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_addressconfig[STM32_FSMC_NADDRCONFIGS] =
{
  GPIO_FSMC_A0,  GPIO_FSMC_A1 , GPIO_FSMC_A2,  GPIO_FSMC_A3,  GPIO_FSMC_A4 , GPIO_FSMC_A5,
  GPIO_FSMC_A6,  GPIO_FSMC_A7,  GPIO_FSMC_A8,  GPIO_FSMC_A9,  GPIO_FSMC_A10, GPIO_FSMC_A11,
  GPIO_FSMC_A12, GPIO_FSMC_A13, GPIO_FSMC_A14, GPIO_FSMC_A15, GPIO_FSMC_A16, GPIO_FSMC_A17,
  GPIO_FSMC_A18, GPIO_FSMC_A19, GPIO_FSMC_A20, GPIO_FSMC_A21, GPIO_FSMC_A22, GPIO_FSMC_A23,
  GPIO_FSMC_A24, GPIO_FSMC_A25
};

static const uint32_t g_dataconfig[STM32_FSMC_NDATACONFIGS] =
{
  GPIO_FSMC_D0,  GPIO_FSMC_D1 , GPIO_FSMC_D2,  GPIO_FSMC_D3,  GPIO_FSMC_D4 , GPIO_FSMC_D5,
  GPIO_FSMC_D6,  GPIO_FSMC_D7,  GPIO_FSMC_D8,  GPIO_FSMC_D9,  GPIO_FSMC_D10, GPIO_FSMC_D11,
  GPIO_FSMC_D12, GPIO_FSMC_D13, GPIO_FSMC_D14, GPIO_FSMC_D15
};

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
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ************************************************************************************/

void stm32_extmemgpios(const uint32_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < ngpios; i++)
    {
      stm32_configgpio(gpios[i]);
    }
}

/************************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize adress line GPIOs for external memory access
 *
 ************************************************************************************/

void stm32_extmemaddr(int naddrs)
{
  stm32_extmemgpios(g_addressconfig, naddrs);
}

/************************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ************************************************************************************/

void stm32_extmemdata(int ndata)
{
  stm32_extmemgpios(g_dataconfig, ndata);
}

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

void stm32_enablefsmc(void)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHB3ENR);
  regval |= RCC_AHB3ENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);
}

/************************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

void stm32_disablefsmc(void)
{
  uint32_t regval;

  /* Disable AHB clocking to the FSMC */

  regval  = getreg32(STM32_RCC_AHB3ENR);
  regval &= ~RCC_AHB3ENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHB3ENR);
}
