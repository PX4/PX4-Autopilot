/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file mpl3115a2.h
 *
 * Shared defines for the mpl3115a2 driver.
 */

#pragma once
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <math.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/device/Device.hpp>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/err.h>

#include "board_config.h"

#define MPL3115A2_REG_WHO_AM_I   0x0c
#define MPL3115A2_WHO_AM_I       0xC4

#define OUT_P_MSB                0x01

#define MPL3115A2_CTRL_REG1      0x26
#  define CTRL_REG1_ALT          (1 << 7)
#  define CTRL_REG1_RAW          (1 << 6)
#  define CTRL_REG1_OS_SHIFTS    (3)
#  define CTRL_REG1_OS_MASK      (0x7 << CTRL_REG1_OS_SHIFTS)
#  define CTRL_REG1_OS(n)        (((n)& 0x7) << CTRL_REG1_OS_SHIFTS)
#  define CTRL_REG1_RST          (1 << 2)
#  define CTRL_REG1_OST          (1 << 1)
#  define CTRL_REG1_SBYB         (1 << 0)

/* interface ioctls */
#define IOCTL_RESET         1
#define IOCTL_MEASURE       2

typedef begin_packed_struct struct MPL3115A2_data_t {
	union {
		uint32_t q;
		uint16_t w[sizeof(q) / sizeof(uint16_t)];
		uint8_t  b[sizeof(q) / sizeof(uint8_t)];
	} pressure;

	union {
		uint16_t w;
		uint8_t  b[sizeof(w)];
	} temperature;
} end_packed_struct MPL3115A2_data_t;

/* interface factories */
extern device::Device *MPL3115A2_i2c_interface(uint8_t busnum);
extern device::Device *MPL3115A2_sim_interface(uint8_t busnum);
typedef device::Device *(*MPL3115A2_constructor)(uint8_t busnum);
