/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david.sidrane@nscdg.com>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
#include <nuttx/config.h>
#include "arm_internal.h"
#include "chip.h"

#define STM32_IWDG_BASE          0x58004800
#define STM32_IWDG_KR_OFFSET     0x0000  /* Key register (32-bit) */
#define STM32_IWDG_PR_OFFSET     0x0004  /* Prescaler register (32-bit) */
#define STM32_IWDG_RLR_OFFSET    0x0008  /* Reload register (32-bit) */
#define STM32_IWDG_SR_OFFSET     0x000c  /* Status register (32-bit) */

#define STM32_IWDG_KR            (STM32_IWDG_BASE+STM32_IWDG_KR_OFFSET)
#define STM32_IWDG_PR            (STM32_IWDG_BASE+STM32_IWDG_PR_OFFSET)
#define STM32_IWDG_RLR           (STM32_IWDG_BASE+STM32_IWDG_RLR_OFFSET)
#define STM32_IWDG_SR            (STM32_IWDG_BASE+STM32_IWDG_SR_OFFSET)

#define IWDG_KR_KEY_ENABLE       (0x5555)  /* Enable register access */
#define IWDG_KR_KEY_DISABLE      (0x0000)  /* Disable register access */
#define IWDG_KR_KEY_RELOAD       (0xaaaa)  /* Reload the counter */
#define IWDG_KR_KEY_START        (0xcccc)  /* Start the watchdog */

#define IWDG_PR_SHIFT            (0)       /* Bits 2-0: Prescaler divider */
#define IWDG_PR_MASK             (7 << IWDG_PR_SHIFT)
#  define IWDG_PR_DIV16          (2 << IWDG_PR_SHIFT) /* 010: divider /16 */
#  define IWDG_PR_DIV256         (6 << IWDG_PR_SHIFT) /* 11x: divider /256 */

#define IWDG_RLR_RL_SHIFT        (0)       /* Bits11:0 RL[11:0]: Watchdog counter reload value */
#define IWDG_RLR_RL_MASK         (0x0fff << IWDG_RLR_RL_SHIFT)
#define IWDG_RLR_MAX             (0xfff)

/****************************************************************************
 * Name: watchdog_pet()
 *
 * Description:
 *   This function resets the Independent watchdog (IWDG)
 *
 *
 * Input Parameters:
 *   none.
 *
 * Returned value:
 *   none.
 *
 ****************************************************************************/

void watchdog_pat(void)
{
	putreg32(IWDG_KR_KEY_RELOAD, STM32_IWDG_KR);
}

/****************************************************************************
 * Name: watchdog_init()
 *
 * Description:
 *   This function initialize the Independent watchdog (IWDG)
 *
 *
 * Input Parameters:
 *   none.
 *
 * Returned value:
 *   none.
 *
 ****************************************************************************/


void watchdog_init(void)
{
	putreg32(IWDG_KR_KEY_ENABLE, STM32_IWDG_KR);
	putreg32(IWDG_PR_DIV16, STM32_IWDG_PR);
	putreg32(IWDG_RLR_MAX, STM32_IWDG_RLR);
	putreg32(IWDG_KR_KEY_START, STM32_IWDG_KR);

	watchdog_pat();
}
