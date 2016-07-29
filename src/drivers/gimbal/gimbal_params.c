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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file gimbal_params.c
 *
 * Parameters for the gimbal controller.
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <px4_config.h>

#include <systemlib/param/param.h>

/**
 * Consider mount operation mode.
 *
 * If set to 1, mount mode will be enforced.
 *
 * @boolean
 * @group Gimbal
 */
PARAM_DEFINE_INT32(GMB_USE_MNT, 0);

/**
 * Auxiliary switch to set mount operation mode.
 *
 * Set to 0 to disable manual mode control.
 *
 * If set to an auxiliary switch:
 * Switch off means the gimbal is put into safe/locked position.
 * Switch on means the gimbal can move freely, and landing gear
 * will be retracted if applicable.
 *
 * @value 0 Disable
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @min 0
 * @max 3
 * @group Gimbal
 */
PARAM_DEFINE_INT32(GMB_AUX_MNT_CHN, 0);
