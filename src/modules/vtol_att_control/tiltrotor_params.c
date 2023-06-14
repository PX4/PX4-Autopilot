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
 * @file tiltrotor_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 */

/**
 * Normalized tilt in Hover
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_MC, 0.0f);

/**
 * Normalized tilt in transition to FW
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_TRANS, 0.4f);

/**
 * Normalized tilt in FW
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_FW, 1.0f);

/**
 * Tilt when disarmed and in the first second after arming
 *
 * This specific tilt during spin-up is necessary for some systems whose motors otherwise don't
 * spin-up freely.
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_SPINUP, 0.0f);

/**
 * Duration of front transition phase 2
 *
 * Time in seconds it takes to tilt form VT_TILT_TRANS to VT_TILT_FW.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_P2_DUR, 0.5f);
