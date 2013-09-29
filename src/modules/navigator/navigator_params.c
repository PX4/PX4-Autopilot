/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Jean Cyr
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
 * @file navigator_params.c
 *
 * Parameters defined by the navigator task.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Navigator parameters, accessible via MAVLink
 *
 */
PARAM_DEFINE_INT32(NAV_FNC_VTX_N, 0);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX0_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX0_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX1_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX1_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX2_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX2_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX3_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX3_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX4_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX4_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX5_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX5_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX6_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX6_LON, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX7_LAT, 0.0f);
PARAM_DEFINE_FLOAT(NAV_FNC_VTX7_LON, 0.0f);
