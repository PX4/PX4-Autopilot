/****************************************************************************
 * boards/nxp/mr-navq95/src/mr-navq95.h
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

#ifndef __BOARDS_NXP_MR_NAVQ95_SRC_MR_NAVQ95_H
#define __BOARDS_NXP_MR_NAVQ95_SRC_MR_NAVQ95_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: imx95_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imx95_bringup(void);
#endif

/****************************************************************************
 * Name: imx95_pwm_setup
 *
 * Description:
 *   Initialize PWM outputs
 *
 ****************************************************************************/

#if defined(CONFIG_PWM)
int imx95_pwm_setup(void);
#endif

/****************************************************************************
 * Name: imx95_i2c_setup
 *
 * Description:
 *   Initialize I2C devices and driver
 *
 ****************************************************************************/

int imx95_i2c_initialize(void);

/****************************************************************************
 * Name: imx95_spi_setup
 *
 * Description:
 *   Initialize SPI devices and driver
 *
 ****************************************************************************/

int imx95_spi_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_NXP_MR_NAVQ95_SRC_MR_NAVQ95_H */
