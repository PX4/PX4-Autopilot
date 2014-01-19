/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *   Author: @author Tobias Naegeli <naegelit@student.ethz.ch>
 *   	     @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Anton Babushkin <anton.babushkin@me.com>
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 */

#include <systemlib/param/param.h>

PARAM_DEFINE_FLOAT(MC_YAW_P, 2.0f);
PARAM_DEFINE_FLOAT(MC_YAW_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATT_P, 6.0f);
PARAM_DEFINE_FLOAT(MC_ATT_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.3f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_P, 0.1f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_D, 0.002f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_I, 0.0f);
