/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file fw_pos_control_l1_params.c
 *
 * Parameters defined by the L1 position control task
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */

PARAM_DEFINE_FLOAT(FW_TCONST, 0.5f);
PARAM_DEFINE_FLOAT(FW_P_P, 40.0f);
PARAM_DEFINE_FLOAT(FW_P_D, 0.0f);
PARAM_DEFINE_FLOAT(FW_P_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_P_RMAX_UP, 0.0f);
PARAM_DEFINE_FLOAT(FW_P_RMAX_DN, 0.0f);
PARAM_DEFINE_FLOAT(FW_P_IMAX, 15.0f);
PARAM_DEFINE_FLOAT(FW_P_RLL, 1.0f);
PARAM_DEFINE_FLOAT(FW_R_P, 40.0f);
PARAM_DEFINE_FLOAT(FW_R_D, 0.0f);
PARAM_DEFINE_FLOAT(FW_R_I, 0.0f);
PARAM_DEFINE_FLOAT(FW_R_IMAX, 15.0f);
PARAM_DEFINE_FLOAT(FW_R_RMAX, 60);
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 9.0f);
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 12.0f);
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 18.0f);
