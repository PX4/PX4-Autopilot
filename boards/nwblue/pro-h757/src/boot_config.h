/****************************************************************************
 *
 *   Copyright (c) 2021, 2024 PX4 Development Team. All rights reserved.
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

/*
 * @file boot_config.h
 *
 * CAN bootloader definitions that configure the behavior and options
 * of the Boot loader for the NWBlue Pro H757
 */

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "board_config.h"
#include "uavcan.h"
#include <nuttx/compiler.h>

#include <stdint.h>

/* STM32H747XI has 2MB flash (16 x 128KB sectors).
 * Use ifndef guards since NuttX may define these from memory map headers.
 */
#ifndef STM32_FLASH_BASE
#define STM32_FLASH_BASE     0x08000000
#endif
#ifndef STM32_FLASH_SIZE
#define STM32_FLASH_SIZE     (2 * 1024 * 1024)  /* 2MB */
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPT_PREFERRED_NODE_ID ANY_NODE_ID

/* Timing parameters */
#define OPT_TBOOT_MS                    3000
#define OPT_NODE_STATUS_RATE_MS         800
#define OPT_NODE_INFO_RATE_MS           50
#define OPT_BL_NUMBER_TIMERS            7

/*
 * Wait for GetNodeInfo policy
 * When set to 0, the bootloader will boot to the application after tboot timeout.
 * When set to 1, the bootloader waits indefinitely for GetNodeInfo.
 */
#define OPT_WAIT_FOR_GETNODEINFO                    0
#define OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT 1

/* Enable watchdog - app must call watchdog_init() early and watchdog_pet() in main loop */
#define OPT_ENABLE_WD           1

/* Timeout for restart after firmware update */
#define OPT_RESTART_TIMEOUT_MS          20000

/* Reserved for the Bootloader - STM32H7 has 128KB sectors */
#define OPT_BOOTLOADER_SIZE_IN_K        (1024 * 128)

/* Reserved for the application out of the total
 * system flash minus the BOOTLOADER_SIZE_IN_K
 */
#define OPT_APPLICATION_RESERVER_IN_K    0

#define OPT_APPLICATION_IMAGE_OFFSET    OPT_BOOTLOADER_SIZE_IN_K
#define OPT_APPLICATION_IMAGE_LENGTH    (FLASH_SIZE - (OPT_BOOTLOADER_SIZE_IN_K + OPT_APPLICATION_RESERVER_IN_K))

#define FLASH_BASE              STM32_FLASH_BASE
#define FLASH_SIZE              STM32_FLASH_SIZE

#define APPLICATION_LOAD_ADDRESS (FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET)
#define APPLICATION_SIZE (FLASH_SIZE - OPT_APPLICATION_IMAGE_OFFSET - (2 * 128 * 1024)) /* Exclude last 2 sectors (params) */
#define APPLICATION_LAST_8BIT_ADDRRESS  ((uint8_t *)((APPLICATION_LOAD_ADDRESS + APPLICATION_SIZE) - sizeof(uint8_t)))
#define APPLICATION_LAST_32BIT_ADDRRESS ((uint32_t *)((APPLICATION_LOAD_ADDRESS + APPLICATION_SIZE) - sizeof(uint32_t)))
#define APPLICATION_LAST_64BIT_ADDRRESS ((uint64_t *)((APPLICATION_LOAD_ADDRESS + APPLICATION_SIZE) - sizeof(uint64_t)))

/* Use yield for large flash sectors */
#define OPT_USE_YIELD

/* STM32H7 flash requires 32-byte (256-bit) aligned writes.
 * Must save and write back 8 words (32 bytes) at a time.
 */
#define OPT_LATER_FLAHSED_WORDS 8

/* UAVCAN hardware name */
#define HW_UAVCAN_NAME                  "org.nwblue.proh757"
#define HW_VERSION_MAJOR                1
#define HW_VERSION_MINOR                0
