/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file uuv_att_control_params.c
 *
 * Parameters defined by the attitude control task for unmanned underwater vehicles (UUVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 * @author Pedro Roque <padr@kth.se>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

// Roll gains
/**
 * Roll proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_P, 4.0f);

/**
 * Roll differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_D, 1.5f);


// Pitch gains
/**
 * Pitch proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_P, 4.0f);

/**
 * Pitch differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_D, 2.0f);


// Yaw gains
/**
 * Yawh proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_YAW_P, 4.0f);

/**
 * Yaw differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_YAW_D, 2.0f);


// Gains for Manual Inputs in different Modes
/**
 * Roll gain for manual inputs in attitude control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_SGM_ROLL, 0.5f);

/**
 * Pitch gain for manual inputs in attitude control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_SGM_PITCH, 0.5f);

/**
 * Yaw gain for manual inputs in attitude control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_SGM_YAW, 0.5f);

/**
 * Throttle gain for manual inputs in attitude control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_SGM_THRTL, 0.1f);

/**
 * Roll gain for manual inputs in rate control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_RGM_ROLL, 100.0f);

/**
 * Pitch gain for manual inputs in rate control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_RGM_PITCH, 100.0f);

/**
 * Yaw gain for manual inputs in rate control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_RGM_YAW, 100.0f);

/**
 * Throttle gain for manual inputs in rate control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_RGM_THRTL, 10.0f);

/**
 * Roll gain for manual inputs in manual control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_MGM_ROLL, 0.05f);

/**
 * Pitch gain for manual inputs in manual control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_MGM_PITCH, 0.05f);

/**
 * Yaw gain for manual inputs in manual control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_MGM_YAW, 0.05f);

/**
 * Throttle gain for manual inputs in manual control mode
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_MGM_THRTL, 0.1f);

/**
 * UUV Torque setpoint Saturation
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_TORQUE_SAT, 0.3f);

/**
 * UUV Thrust setpoint Saturation
 *
 * @group UUV Attitude Control
 * @min 0.0
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_THRUST_SAT, 0.1f);

/**
 * Maximum time (in seconds) before resetting setpoint
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_SP_MAX_AGE, 2.0f);

/**
 * Stick mode selector (0=Heave/sway control, roll/pitch leveled; 1=Pitch/roll control)
 *
 * @group UUV Attitude Control
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(UUV_STICK_MODE, 0);
