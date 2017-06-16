
/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file MP_att_control_params.h
 * Parameters for multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#pragma once

#define PARAM_MP_ROLL_P_DEFAULT 6.0f
#define PARAM_MP_ROLLRATE_P_DEFAULT 0.1f
#define PARAM_MP_ROLLRATE_I_DEFAULT 0.0f
#define PARAM_MP_ROLLRATE_D_DEFAULT 0.002f
#define PARAM_MP_PITCH_P_DEFAULT 6.0f
#define PARAM_MP_PITCHRATE_P_DEFAULT 0.1f
#define PARAM_MP_PITCHRATE_I_DEFAULT 0.0f
#define PARAM_MP_PITCHRATE_D_DEFAULT 0.002f
#define PARAM_MP_YAW_P_DEFAULT 2.0f
#define PARAM_MP_YAWRATE_P_DEFAULT 0.3f
#define PARAM_MP_YAWRATE_I_DEFAULT 0.0f
#define PARAM_MP_YAWRATE_D_DEFAULT 0.0f
#define PARAM_MP_YAW_FF_DEFAULT 0.5f
#define PARAM_MP_YAWRATE_MAX_DEFAULT 60.0f
#define PARAM_MP_ACRO_R_MAX_DEFAULT 35.0f
#define PARAM_MP_ACRO_P_MAX_DEFAULT 35.0f
#define PARAM_MP_ACRO_Y_MAX_DEFAULT 120.0f
