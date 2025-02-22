/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_common_initialize.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp23xx_gpio.h"
#include "rp23xx_uniqueid.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_common_earlyinitialize
 *
 * Description:
 *  This is the early initialization common to all RP23XX boards.
 *  It configures the UART pins so the system console can be used.
 ****************************************************************************/

void rp23xx_common_earlyinitialize(void)
{
  rp23xx_gpio_initialize();

  /* Disable IE on GPIO 26-29 */

  clrbits_reg32(RP23XX_PADS_BANK0_GPIO_IE, RP23XX_PADS_BANK0_GPIO(26));
  clrbits_reg32(RP23XX_PADS_BANK0_GPIO_IE, RP23XX_PADS_BANK0_GPIO(27));
  clrbits_reg32(RP23XX_PADS_BANK0_GPIO_IE, RP23XX_PADS_BANK0_GPIO(28));
  clrbits_reg32(RP23XX_PADS_BANK0_GPIO_IE, RP23XX_PADS_BANK0_GPIO(29));

  /* Set default UART pin */

#ifdef CONFIG_RP23XX_UART0
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_TX_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* TX */
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_RX_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* RX */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_CTS_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* CTS */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_RTS_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* RTS */
#endif
#endif

#ifdef CONFIG_RP23XX_UART1
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_TX_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* TX */
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_RX_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* RX */
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_CTS_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* CTS */
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_RTS_GPIO,
                           RP23XX_GPIO_FUNC_UART);      /* RTS */
#endif
#endif

#if defined(CONFIG_RP23XX_CLK_GPOUT0)
  rp23xx_gpio_set_function(RP23XX_GPIO_PIN_CLK_GPOUT0,
                           RP23XX_GPIO_FUNC_GPCK);
#endif
#if defined(CONFIG_RP23XX_CLK_GPOUT1)
  rp23xx_gpio_set_function(RP23XX_GPIO_PIN_CLK_GPOUT1,
                           RP23XX_GPIO_FUNC_GPCK);
#endif
#if defined(CONFIG_RP23XX_CLK_GPOUT2)
  rp23xx_gpio_set_function(RP23XX_GPIO_PIN_CLK_GPOUT2,
                           RP23XX_GPIO_FUNC_GPCK);
#endif
#if defined(CONFIG_RP23XX_CLK_GPOUT3)
  rp23xx_gpio_set_function(RP23XX_GPIO_PIN_CLK_GPOUT3,
                           RP23XX_GPIO_FUNC_GPCK);
#endif
}

/****************************************************************************
 * Name: rp23xx_common_initialize
 *
 * Description:
 *  It configures the pin assignments that were not done in the early
 *  initialization.
 ****************************************************************************/

void rp23xx_common_initialize(void)
{
#ifdef CONFIG_BOARDCTL_UNIQUEID
  rp23xx_uniqueid_initialize();
#endif

  /* Set default I2C pin */

#ifdef CONFIG_RP23XX_I2C0
  rp23xx_gpio_set_function(CONFIG_RP23XX_I2C0_SDA_GPIO,
                           RP23XX_GPIO_FUNC_I2C);       /* SDA */
  rp23xx_gpio_set_function(CONFIG_RP23XX_I2C0_SCL_GPIO,
                           RP23XX_GPIO_FUNC_I2C);       /* SCL */

  rp23xx_gpio_set_pulls(CONFIG_RP23XX_I2C0_SDA_GPIO, true, false);  /* Pull up */
  rp23xx_gpio_set_pulls(CONFIG_RP23XX_I2C0_SCL_GPIO, true, false);
#endif

#ifdef CONFIG_RP23XX_I2C1
  rp23xx_gpio_set_function(CONFIG_RP23XX_I2C1_SDA_GPIO,
                           RP23XX_GPIO_FUNC_I2C);       /* SDA */
  rp23xx_gpio_set_function(CONFIG_RP23XX_I2C1_SCL_GPIO,
                           RP23XX_GPIO_FUNC_I2C);       /* SCL */

  rp23xx_gpio_set_pulls(CONFIG_RP23XX_I2C1_SDA_GPIO, true, false);  /* Pull up */
  rp23xx_gpio_set_pulls(CONFIG_RP23XX_I2C1_SCL_GPIO, true, false);
#endif

  /* Set default SPI pin */

#ifdef CONFIG_RP23XX_SPI0
  rp23xx_gpio_set_function(CONFIG_RP23XX_SPI0_RX_GPIO,
                           RP23XX_GPIO_FUNC_SPI);       /* RX */
  rp23xx_gpio_set_function(CONFIG_RP23XX_SPI0_SCK_GPIO,
                           RP23XX_GPIO_FUNC_SPI);       /* SCK */
  rp23xx_gpio_set_function(CONFIG_RP23XX_SPI0_TX_GPIO,
                           RP23XX_GPIO_FUNC_SPI);       /* TX */

  /* CSn is controlled by board-specific logic */

  rp23xx_gpio_init(CONFIG_RP23XX_SPI0_CS_GPIO);        /* CSn */
  rp23xx_gpio_setdir(CONFIG_RP23XX_SPI0_CS_GPIO, true);
  rp23xx_gpio_put(CONFIG_RP23XX_SPI0_CS_GPIO, true);
#endif

#ifdef CONFIG_RP23XX_SPI1
  rp23xx_gpio_set_function(CONFIG_RP23XX_SPI1_RX_GPIO,
                           RP23XX_GPIO_FUNC_SPI);       /* RX */
  rp23xx_gpio_set_function(CONFIG_RP23XX_SPI1_SCK_GPIO,
                           RP23XX_GPIO_FUNC_SPI);       /* SCK */
  rp23xx_gpio_set_function(CONFIG_RP23XX_SPI1_TX_GPIO,
                           RP23XX_GPIO_FUNC_SPI);       /* TX */

  /* CSn is controlled by board-specific logic */

  rp23xx_gpio_init(CONFIG_RP23XX_SPI1_CS_GPIO);        /* CSn */
  rp23xx_gpio_setdir(CONFIG_RP23XX_SPI1_CS_GPIO, true);
  rp23xx_gpio_put(CONFIG_RP23XX_SPI1_CS_GPIO, true);
#endif
}
