/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file drv_irlock.h
 *
 * IR-Lock device API
 **/

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h" // include sensor driver interfaces

#define IRLOCK_BASE_DEVICE_PATH	"/dev/irlock"
#define IRLOCK0_DEVICE_PATH	"/dev/irlock0"

#define IRLOCK_OBJECTS_MAX	5	/** up to 5 objects can be detected/reported **/

struct irlock_target_s {
	uint16_t signature;	/** target signature **/
	float pos_x;	/** x-axis distance from center of image to center of target in units of tan(theta) **/
	float pos_y;	/** y-axis distance from center of image to center of target in units of tan(theta) **/
	float size_x;	/** size of target along x-axis in units of tan(theta) **/
	float size_y;	/** size of target along y-axis in units of tan(theta) **/
};

/** irlock_s structure returned from read calls **/
struct irlock_s {
	uint64_t timestamp; /** microseconds since system start **/
	uint8_t num_targets;
	struct irlock_target_s targets[IRLOCK_OBJECTS_MAX];
};
