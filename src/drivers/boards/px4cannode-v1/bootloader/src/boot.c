/************************************************************************************
 * configs/olimexino-stm32/src/bl_boot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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
#include "app_config.h"

#include <debug.h>
#include <arch/board/board.h>

#include <nuttx/board.h>


#include "board_config.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
  putreg32(getreg32(STM32_RCC_APB1ENR) | RCC_APB1ENR_CAN1EN, STM32_RCC_APB1ENR);
  stm32_configgpio(GPIO_CAN1_RX);
  stm32_configgpio(GPIO_CAN1_TX);
  stm32_configgpio(GPIO_CAN_CTRL);
  putreg32(getreg32(STM32_RCC_APB1RSTR) | RCC_APB1RSTR_CAN1RST,
           STM32_RCC_APB1RSTR);
  putreg32(getreg32(STM32_RCC_APB1RSTR) & ~RCC_APB1RSTR_CAN1RST,
           STM32_RCC_APB1RSTR);

#if defined(OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO)
  stm32_configgpio(GPIO_GETNODEINFO_JUMPER);
#endif

}

void stm32_boarddeinitialize(void)
{

  putreg32(getreg32(STM32_RCC_APB1RSTR) | RCC_APB1RSTR_CAN1RST,
           STM32_RCC_APB1RSTR);
}

#include <nuttx/config.h>
#include "app_config.h"

#include <stdint.h>

#include "stm32.h"



uint8_t board_get_product_name(uint8_t * product_name)
{
  product_name[0] = 'h';
  product_name[1] = 'i';
  product_name[2] = '!';
  return 3u;
}

void board_get_hardware_version(uavcan_hardwareversion_t * hw_version)
{
  uint32_t i;
  volatile uint8_t *stm32f_uid = (volatile uint8_t *)STM32_UNIQUE_DEVICE_ID;

  hw_version->major = 1u;
  hw_version->minor = 0u;

  for (i = 0u; i < 12u; i++)
    {
      hw_version->unique_id[i] = stm32f_uid[i];
    }
  for (; i < 16u; i++)
    {
      hw_version->unique_id[i] = 0u;
    }

  for (i = 0u; i < 255u; i++)
    {
      hw_version->certificate_of_authenticity[i] = 0;
    }

  hw_version->certificate_of_authenticity_length = 0u;
}

void board_indicate_reset(void)
{

}

void board_indicate_autobaud_start(void)
{

}

void board_indicate_autobaud_end(void)
{

}

void board_indicate_allocation_start(void)
{

}

void board_indicate_allocation_end(void)
{

}

void board_indicate_fw_update_start(void)
{

}

void board_indicate_fw_update_erase_fail(void)
{

}

void board_indicate_fw_update_invalid_response(void)
{

}

void board_indicate_fw_update_timeout(void)
{

}

void board_indicate_fw_update_invalid_crc(void)
{

}

void board_indicate_jump_to_app(void)
{

}

uint8_t board_get_wait_for_getnodeinfo_flag(void)
{
  return 1u;
}
