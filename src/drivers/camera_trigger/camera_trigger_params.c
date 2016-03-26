/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file camera_trigger_params.c
 * Camera trigger parameters
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

/**
 * Camera trigger interval
 *
 * This parameter sets the time between two consecutive trigger events
 *
 * @unit ms
 * @min 4.0
 * @max 10000.0
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_INTERVAL, 40.0f);

/**
 * Camera trigger polarity
 *
 * This parameter sets the polarity of the trigger (0 = ACTIVE_LOW, 1 = ACTIVE_HIGH )
 *
 * @value 0 ACTIVE_LOW
 * @value 1 ACTIVE_HIGH
 * @min 0
 * @max 1
 * @group Camera trigger
 */
PARAM_DEFINE_INT32(TRIG_POLARITY, 0);

/**
 * Camera trigger activation time
 *
 * This parameter sets the time the trigger needs to pulled high or low.
 *
 * @unit ms
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_ACT_TIME, 0.5f);

/**
 * Camera trigger mode
 *
 * 0 disables the trigger, 1 sets it to enabled on command, 2 always on, 3 distance based, 4 distance based enabled on command
 *
 * @value 0 Disable
 * @value 1 CMD
 * @value 2 Always
 * @value 3 Distance
 * @min 0
 * @max 4
 * @group Camera trigger
 */
PARAM_DEFINE_INT32(TRIG_MODE, 0);

/**
 * Camera trigger pin
 *
 * Selects which pin is used, ranges from 1 to 6 (AUX1-AUX6)
 *
 * @min 1
 * @max 123456
 * @group Camera trigger
 */
PARAM_DEFINE_INT32(TRIG_PINS, 12);

/**
 * Camera trigger distance
 *
 * Sets the distance at which to trigger the camera.
 *
 * @unit m
 * @min 0
 * @group Camera trigger
 */
PARAM_DEFINE_FLOAT(TRIG_DISTANCE, 25.0f);
