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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 */

#include <systemlib/param/param.h>

/**
 * VTOL number of engines
 *
 * @min 1.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_MOT_COUNT,0);

/**
 * Idle speed of VTOL when in multicopter mode
 *
 * @min 900
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_IDLE_PWM_MC,900);

/**
 * Minimum airspeed in multicopter mode
 *
 * This is the minimum speed of the air flowing over the control surfaces.
 *
 * @min 0.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_MIN,2.0f);

/**
 * Maximum airspeed in multicopter mode
 *
 * This is the maximum speed of the air flowing over the control surfaces.
 *
 * @min 0.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_MAX,30.0f);

/**
 * Trim airspeed when in multicopter mode
 *
 * This is the airflow over the control surfaces for which no airspeed scaling is applied in multicopter mode.
 *
 * @min 0.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_TRIM,10.0f);

