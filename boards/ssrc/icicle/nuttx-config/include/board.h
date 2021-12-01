/****************************************************************************
 * boards/risc-v/mpfs/icicle/include/board.h
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

#ifndef __BOARDS_RISCV_ICICLE_MPFS_INCLUDE_BOARD_H
#define __BOARDS_RISCV_ICICLE_MPFS_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "mpfs_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking TODO: */

#define MPFS_MSS_EXT_SGMII_REF_CLK (125000000UL)
#define MPFS_MSS_COREPLEX_CPU_CLK  (600000000UL)
#define MPFS_MSS_SYSTEM_CLK        (600000000UL)
#define MPFS_MSS_RTC_TOGGLE_CLK      (1000000UL)
#define MPFS_MSS_AXI_CLK           (300000000UL)
#define MPFS_MSS_APB_AHB_CLK       (150000000UL)
#define MPFS_FPGA_BCLK               (3000000UL)

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

#define BOARD_LED_ORANGE1 BOARD_LED1
#define BOARD_LED_ORANGE2 BOARD_LED2
#define BOARD_LED_RED1    BOARD_LED3
#define BOARD_LED_RED2    BOARD_LED4

#define LED_STARTED       0 /* LED1 */
#define LED_HEAPALLOCATE  1 /* LED2 */
#define LED_IRQSENABLED   2 /* LED1 + LED2 */
#define LED_STACKCREATED  3 /* LED3 */
#define LED_INIRQ         4 /* LED3 + LED1 */
#define LED_SIGNAL        5 /* LED3 + LED2 */
#define LED_ASSERTION     6 /* LED3 + LED2 + LED1 */
#define LED_PANIC         7 /* LED4 */

/* Button definitions *******************************************************/

/* The Icicle supports 3 buttons: */

#define BUTTON_USER1 0
#define BUTTON_USER2 1
#define BUTTON_USER3 2
#define NUM_BUTTONS  3
#define BUTTON_USER1_BIT (1 << BUTTON_USER1)
#define BUTTON_USER2_BIT (1 << BUTTON_USER2)
#define BUTTON_USER3_BIT (1 << BUTTON_USER3)

#ifndef __ASSEMBLY__

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

/****************************************************************************
 * Name: mpfs_boardinitialize
 ****************************************************************************/

void mpfs_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_ICICLE_MPFS_INCLUDE_BOARD_H  */
