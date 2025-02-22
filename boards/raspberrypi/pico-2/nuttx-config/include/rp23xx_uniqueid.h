/****************************************************************************
 * boards/arm/rp23xx/common/include/rp23xx_uniqueid.h
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

#ifndef __BOARDS_ARM_RP23XX_COMMON_INCLUDE_RP23XX_UNIQUEID_H
#define __BOARDS_ARM_RP23XX_COMMON_INCLUDE_RP23XX_UNIQUEID_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_uniqueid_initialize
 *
 * Description:
 *   The RP23XX doesn't have a unique ID, so we load the ID from the
 *   connected flash chip.  We use the flash ID to seed a simple xorshift
 *   PRNG.  The PRNG then generates CONFIG_BOARDCTL_UNIQUEID_SIZE bytes,
 *   which we will use as the board's unique ID.
 *
 *   Retrieving the flash id is somewhat slow and complex, so we only do
 *   this during initialization and store the result for later use.
 *
 * Assumptions/Limitations:
 *   This uniqueid implementation requires a flash chip.  It should not be
 *   used on boards without flash.
 *
 ****************************************************************************/

void rp23xx_uniqueid_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RP23XX_COMMON_INCLUDE_RP23XX_UNIQUEID_H */
