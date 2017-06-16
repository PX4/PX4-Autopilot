/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file px4_adc.h
 *
 * ADC header depending on the build target
 */

#pragma once

#include <stdint.h>

#if defined(__PX4_ROS)
#error "ADC not supported in ROS"
#elif defined(__PX4_NUTTX)
/*
 * Building for NuttX
 */
#include <nuttx/analog/adc.h>
#elif defined(__PX4_POSIX)

// FIXME - this needs to be a px4_adc_msg_s type
// Curently copied from NuttX
struct adc_msg_s {
	uint8_t      am_channel;               /* The 8-bit ADC Channel */
	int32_t      am_data;                  /* ADC convert result (4 bytes) */
} __attribute__((packed));

// Example settings
#define ADC_BATTERY_VOLTAGE_CHANNEL     10
#define ADC_BATTERY_CURRENT_CHANNEL     ((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL    11
#define ADCSIM0_DEVICE_PATH	"/dev/adc0"
#else
#error "No target platform defined"
#endif
