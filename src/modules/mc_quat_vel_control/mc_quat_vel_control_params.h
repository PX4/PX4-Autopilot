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
 * @file mc_quat_vel_control_params.c
 * Parameters for the quaternion based quadrotor controller for manual and trajectory tracking.
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */

#include <systemlib/param/param.h>

/**
 * Vehicle velocity control gains
 * Units in m/s or something
 */
PARAM_DEFINE_FLOAT(MC_VEL_VX_P, 0.20f);
PARAM_DEFINE_FLOAT(MC_VEL_VX_I, 0.3f);
PARAM_DEFINE_FLOAT(MC_VEL_VX_MAX, 2.0f);
PARAM_DEFINE_FLOAT(MC_VEL_VY_P, 0.30f);
PARAM_DEFINE_FLOAT(MC_VEL_VY_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_VEL_VY_MAX, 2.0f);

PARAM_DEFINE_FLOAT(MC_VEL_VZ_P, 0.1f);
PARAM_DEFINE_FLOAT(MC_VEL_VZ_I, 0.03f);
PARAM_DEFINE_FLOAT(MC_VEL_VZ_MG, 0.20f);
PARAM_DEFINE_FLOAT(MC_THR_MIN, 0.20f); /* We can read these from MPC TODO */
PARAM_DEFINE_FLOAT(MC_THR_MAX, 0.30f);

PARAM_DEFINE_FLOAT(MC_VEL_IXY_MAX, 20.0f);
PARAM_DEFINE_FLOAT(MC_VEL_IZ_MAX, 20.0f);
