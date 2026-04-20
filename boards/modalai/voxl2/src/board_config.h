/****************************************************************************
 *
 *   Copyright (c) 2022-2026 ModalAI, Inc. All rights reserved.
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

/**
 * @file board_config.h
 *
 * VOXL2 internal definitions
 */

#pragma once

#define CONFIG_BOARDCTL_RESET
#define BOARD_HAS_NO_BOOTLOADER

/*
 * SPI buses (shared)
 */
#define CONFIG_SPI 1
#define BOARD_SPI_BUS_MAX_BUS_ITEMS 1

#ifdef __PX4_QURT
/*
 * QURT (DSP) specific defines
 */

#define CONFIG_I2C 1
#define PX4_NUMBER_I2C_BUSES    4

#include <system_config.h>
#include <px4_platform_common/board_common.h>

#define VOXL_ESC_DEFAULT_PORT 	"2"
#define GHST_RC_DEFAULT_PORT 	"7"
#define VOXL2_IO_DEFAULT_PORT 	"2"

/* M0065 PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS 4
#define MAX_IO_TIMERS 3

#endif /* __PX4_QURT */

#if defined(__PX4_POSIX) && !defined(__PX4_QURT)
/*
 * POSIX (apps processor) specific defines
 */

/* I2C clock init not required on Linux */
#define BOARD_I2C_BUS_CLOCK_INIT

#define CONFIG_I2C 1
#define PX4_NUMBER_I2C_BUSES    1

#include <system_config.h>
#include <px4_platform_common/board_common.h>

#define BOARD_OVERRIDE_UUID "MODALAIVOXL20000" // must be of length 16
#define PX4_SOC_ARCH_ID PX4_SOC_ARCH_ID_VOXL2

#define VOXL_ESC_DEFAULT_PORT 	"2"
#define VOXL2_IO_DEFAULT_PORT 	"2"

#endif /* __PX4_POSIX && !__PX4_QURT */
