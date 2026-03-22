/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
* Maximum value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(SV_CAL_MAX_VAL, 1.0f);

/**
* Minimum value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(SV_CAL_MIN_VAL, -1.0f);

/**
* Step value for servo calibration.
*
* @reboot_required true
* @group Actuators
*/
PARAM_DEFINE_FLOAT(SV_CAL_STEP_VAL, 0.1f);

/**
* Saved position for servo 1.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_1, 0.0f);

/**
* Saved position for servo 2.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_2, 0.0f);

/**
* Saved position for servo 3.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_3, 0.0f);

/**
* Saved position for servo 4.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_4, 0.0f);

/**
* Saved position for servo 5.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_5, 0.0f);

/**
* Saved position for servo 6.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_6, 0.0f);

/**
* Saved position for servo 7.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_7, 0.0f);

/**
* Saved position for servo 8.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_8, 0.0f);

/**
* Saved position for servo 9.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_9, 0.0f);

/**
* Saved position for servo 10.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_10, 0.0f);

/**
* Saved position for servo 11.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_11, 0.0f);

/**
* Saved position for servo 12.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_12, 0.0f);

/**
* Saved position for servo 13.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_13, 0.0f);

/**
* Saved position for servo 14.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_14, 0.0f);

/**
* Saved position for servo 15.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_15, 0.0f);

/**
* Saved position for servo 16.
*
* @min -1.0
* @max 1.0
* @decimal 3
* @group Servo Positions
*/
PARAM_DEFINE_FLOAT(SV_POS_SAVED_16, 0.0f);
