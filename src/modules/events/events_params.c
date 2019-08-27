/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file events_params.c
 *
 * Parameters defined by the events module.
 */

#include <px4_platform_common/config.h>
#include <parameters/param.h>

/**
 * Status Display
 *
 * Enable/disable event task for displaying the vehicle status using arm-mounted
 * LEDs. When enabled and if the vehicle supports it, LEDs will flash
 * indicating various vehicle status changes. Currently PX4 has not implemented
 * any specific status events.
 * -
 *
 * @group Events
 * @boolean
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EV_TSK_STAT_DIS, 0);

/**
 * RC Loss Alarm
 *
 * Enable/disable event task for RC Loss. When enabled, an alarm tune will be
 * played via buzzer or ESCs, if supported. The alarm will sound after a disarm,
 * if the vehicle was previously armed and only if the vehicle had RC signal at
 * some point. Particularly useful for locating crashed drones without a GPS
 * sensor.
 *
 * @group Events
 * @boolean
 * @reboot_required true
 */
PARAM_DEFINE_INT32(EV_TSK_RC_LOSS, 0);
