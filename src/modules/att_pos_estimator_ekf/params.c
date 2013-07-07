/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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

#include <systemlib/param/param.h>

/*PARAM_DEFINE_FLOAT(NAME,0.0f);*/
PARAM_DEFINE_FLOAT(KF_V_GYRO, 0.008f);
PARAM_DEFINE_FLOAT(KF_V_ACCEL, 1.0f);
PARAM_DEFINE_FLOAT(KF_R_MAG, 0.8f);
PARAM_DEFINE_FLOAT(KF_R_GPS_VEL, 0.5f);
PARAM_DEFINE_FLOAT(KF_R_GPS_POS, 2.0f);
PARAM_DEFINE_FLOAT(KF_R_GPS_ALT, 3.0f);
PARAM_DEFINE_FLOAT(KF_R_PRESS_ALT, 0.1f);
PARAM_DEFINE_FLOAT(KF_R_ACCEL, 1.0f);
PARAM_DEFINE_FLOAT(KF_FAULT_POS, 10.0f);
PARAM_DEFINE_FLOAT(KF_FAULT_ATT, 10.0f);
PARAM_DEFINE_FLOAT(KF_ENV_G, 9.765f);
PARAM_DEFINE_FLOAT(KF_ENV_MAG_DIP, 60.0f);
PARAM_DEFINE_FLOAT(KF_ENV_MAG_DEC, 0.0f);
