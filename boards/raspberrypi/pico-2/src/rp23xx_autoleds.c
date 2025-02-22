/****************************************************************************
 * boards/arm/rp23xx/raspberrypi-pico-2/src/rp23xx_autoleds.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* There are four LED status indicators located on the EVK Board.  The
 * functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/rp23xx_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------- ------
 *   SYMBOL               Meaning                 LED
 *   -------------------- ----------------------- ------
 *
 *   LED_STARTED       0  NuttX has been started  OFF
 *   LED_HEAPALLOCATE  0  Heap has been allocated OFF
 *   LED_IRQSENABLED   0  Interrupts enabled      OFF
 *   LED_STACKCREATED  1  Idle stack created      ON
 *   LED_INIRQ         2  In an interrupt         N/C
 *   LED_SIGNAL        2  In a signal handler     N/C
 *   LED_ASSERTION     2  An assertion failed     N/C
 *   LED_PANIC         3  The system has crashed  FLASH
 *   LED_IDLE             Not used
 *
 * Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "rp23xx_gpio.h"

#include "rp23xx_pico.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIO for output */

  rp23xx_gpio_init(GPIO_LED1);
  rp23xx_gpio_setdir(GPIO_LED1, true);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turn on the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledon = true;

  switch (led)
    {
      case 0:  /* LED Off */
        ledon = false;
        break;

      case 2:  /* LED No change */
        return;

      case 1:  /* LED On */
      case 3:  /* LED On */
        break;
    }

  rp23xx_gpio_put(GPIO_LED1, ledon); /* High illuminates */
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn off the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case 0:  /* LED Off */
      case 1:  /* LED Off */
      case 3:  /* LED Off */
        break;

      case 2:  /* LED No change */
        return;
    }

  rp23xx_gpio_put(GPIO_LED1, false); /* High illuminates */
}

#endif /* CONFIG_ARCH_LEDS */
