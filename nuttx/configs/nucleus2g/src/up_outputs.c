/****************************************************************************
 * configs/nucleus2g/src/up_outputs.c
 * arch/arm/src/board/up_outputs.c
 *
 *   Copyright (C) 2012 Hal Glenn. All rights reserved.
 *   Author: Hal Glenn <hglenn@2g-eng.com>
 *
 * This file is part of NuttX:
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_gpio.h"

#include "nucleus2g_internal.h"

#ifdef CONFIG_ARCH_BOARD_NUCLEUS2G_BMS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Private Data
 ****************************************************************************/
 
/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
/****************************************************************************
 * Public Functions
 ****************************************************************************/
 
/****************************************************************************
 * Name: nucleus_bms_relay 1-4
 *
 * Description:
 *   Once booted these functions control the 4 isolated FET outputs from the
 *   master BMS controller
 *
 ***************************************************************************/
 
void nucleus_bms_relay1(enum output_state state)
{
  bool value   = (state == (enum output_state)RELAY_OPEN);
  lpc17_gpiowrite(NUCLEUS_BMS_RELAY1, value);
}

void nucleus_bms_relay2(enum output_state state)
{
  bool value   = (state == (enum output_state)RELAY_OPEN);
  lpc17_gpiowrite(NUCLEUS_BMS_RELAY2, value);
}

void nucleus_bms_relay3(enum output_state state)
{
  bool value   = (state == (enum output_state)RELAY_OPEN);
  lpc17_gpiowrite(NUCLEUS_BMS_RELAY3, value);
}

void nucleus_bms_relay4(enum output_state state)
{
  bool value   = (state == (enum output_state)RELAY_OPEN);
  lpc17_gpiowrite(NUCLEUS_BMS_RELAY4, value);
}

/***************************************************************************
 * Name: up_relayinit
 *
 * Description:
 *  This function is called on boot to init the GPIO for relay control
 *
 ***************************************************************************/

void up_relayinit(void)
{
  lpc17_configgpio(NUCLEUS_BMS_RELAY1);
  lpc17_configgpio(NUCLEUS_BMS_RELAY2);
  lpc17_configgpio(NUCLEUS_BMS_RELAY3);
  lpc17_configgpio(NUCLEUS_BMS_RELAY4);
}

#endif /* CONFIG_ARCH_BOARD_NUCLEUS2G_BMS */

