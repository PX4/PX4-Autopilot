/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file geofence_params.c
 *
 * Parameters for geofence
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Geofence parameters, accessible via MAVLink
 */

/**
 * Enable geofence.
 *
 * Set to 1 to enable geofence.
 * Defaults to 1 because geofence is only enabled when the geofence.txt file is present.
 *
 * @min 0
 * @max 1
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_ON, 1);

/**
 * Geofence altitude mode
 *
 * Select which altitude reference should be used
 * 0 = WGS84, 1 = AMSL
 *
 * @min 0
 * @max 1
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_ALTMODE, 0);

/**
 * Geofence source
 *
 * Select which position source should be used. Selecting GPS instead of global position makes sure that there is
 * no dependence on the position estimator
 * 0 = global position, 1 = GPS
 *
 * @min 0
 * @max 1
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_SOURCE, 0);

/**
 * Geofence counter limit
 *
 * Set how many subsequent position measurements outside of the fence are needed before geofence violation is triggered
 *
 * @min -1
 * @max 10
 * @group Geofence
 */
PARAM_DEFINE_INT32(GF_COUNT, -1);
