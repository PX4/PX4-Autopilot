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

PARAM_DEFINE_FLOAT(FW_L1_PERIOD, 25.0f);


PARAM_DEFINE_FLOAT(FW_L1_DAMPING, 0.75f);


PARAM_DEFINE_FLOAT(FW_LOITER_R, 50.0f);


PARAM_DEFINE_FLOAT(FW_THR_CRUISE, 0.7f);


PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -45.0f);


PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 45.0f);


PARAM_DEFINE_FLOAT(FW_R_LIM, 45.0f);


PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);


PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

PARAM_DEFINE_FLOAT(FW_THR_LND_MAX, 1.0f);

PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX, 5.0f);


PARAM_DEFINE_FLOAT(FW_T_SINK_MIN, 2.0f);


PARAM_DEFINE_FLOAT(FW_T_TIME_CONST, 5.0f);


PARAM_DEFINE_FLOAT(FW_T_THR_DAMP, 0.5f);


PARAM_DEFINE_FLOAT(FW_T_INTEG_GAIN, 0.1f);


PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 7.0f);


PARAM_DEFINE_FLOAT(FW_T_HGT_OMEGA, 3.0f);


PARAM_DEFINE_FLOAT(FW_T_SPD_OMEGA, 2.0f);


PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 10.0f);


PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);


PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.0f);


PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);
