/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * PX4FMUv3 internal definitions: identical to fmuv2 except for usable flash
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

/* Run time Hardware detection */
#define BOARD_HAS_SIMPLE_HW_VERSIONING 1
#define HW_VER_PB4             (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)
#define HW_VER_PB12            (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12)
#define HW_VER_PB4_INIT        (GPIO_EXTI_ACCEL_DRDY)
#define HW_VER_PB12_INIT       (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN12)
#define HW_VER_FMUV2_STATE     0x8 /* PB12:PU:1 PB12:PD:0 PB4:PU:0 PB4PD:0 */
#define HW_VER_FMUV3_STATE     0xE /* PB12:PU:1 PB12:PD:1 PB4:PU:1 PB4PD:0 */
#define HW_VER_FMUV2MINI_STATE 0xA /* PB12:PU:1 PB12:PD:0 PB4:PU:1 PB4PD:0 */
#define HW_VER_TYPE_INIT {'V','2',0, 0}

#include "../px4fmu-v2/board_config.h"
#define PX4_SPIDEV_ICM_20608  PX4_SPIDEV_ACCEL_MAG // PixhawkMini has ICM_20608 on GPIO_SPI_CS_ACCEL_MAG
