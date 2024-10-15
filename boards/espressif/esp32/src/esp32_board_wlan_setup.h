/****************************************************************************
 * boards/xtensa/esp32/common/include/esp32_board_wlan.h
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

#ifndef __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_BOARD_WLAN_H
#define __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_BOARD_WLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32_WIRELESS

/****************************************************************************
 * Name: board_wlan_init
 *
 * Description:
 *   Configure the wireless subsystem.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_wlan_init(void);

#endif /* CONFIG_ESP32_WIRELESS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_BOARD_WLAN_H */
