/****************************************************************************
 * boards/arm/rp23xx/common/include/rp23xx_common_initialize.h
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

#ifndef __BOARDS_ARM_RP23XX_COMMON_INCLUDE_RP23XX_COMMON_INITIALIZE_H
#define __BOARDS_ARM_RP23XX_COMMON_INCLUDE_RP23XX_COMMON_INITIALIZE_H

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_common_earlyinitialize
 *
 * This is the early initialization common to all RP23XX boards.
 * It configures the GPIO, SPI, and I2C pins.
 ****************************************************************************/

int rp23xx_common_earlyinitialize(void);

/****************************************************************************
 * Name: rp23xx_common_initialize
 *
 * Description:
 *  It configures the pin assignments that were not done in the early
 *  initialization.
 ****************************************************************************/

void rp23xx_common_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RP23XX_COMMON_INCLUDE_RP23XX_COMMON_INITIALIZE_H */
