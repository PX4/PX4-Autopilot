/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file pwm_params_main.c
 *
 * Parameters defined for PWM output.
 *
 */

/******************************************************************************
*                                  PWM_RATE                                   *
******************************************************************************/

/**
 * Set the PWM output frequency for the main outputs
 *
 * Set to 400 for industry default or 1000 for high frequency ESCs.
 *
 * Set to 0 for Oneshot125.
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2000
 * @unit Hz
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_RATE, 400);

/**
 * Set the minimum PWM for the main outputs
 *
 * Set to 1000 for industry default or 900 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MIN, 1000);

/**
 * Set the maximum PWM for the main outputs
 *
 * Set to 2000 for industry default or 2100 to increase servo travel.
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAX, 2000);

/**
 * Set the disarmed PWM for the main outputs
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * The main use of this parameter is to silence ESCs when they are disarmed.
 *
 * @reboot_required true
 *
 * @min 0
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_DISARMED, 900);

/******************************************************************************
*                                 PWM_MAIN_MIN                                *
******************************************************************************/
/**
 * Set the min PWM value for the main 1 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN1, -1);

/**
 * Set the min PWM value for the main 2 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN2, -1);

/**
 * Set the min PWM value for the main 3 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN3, -1);

/**
 * Set the min PWM value for the main 4 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN4, -1);

/**
 * Set the min PWM value for the main 5 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN5, -1);

/**
 * Set the min PWM value for the main 6 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN6, -1);

/**
 * Set the min PWM value for the main 7 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN7, -1);

/**
 * Set the min PWM value for the main 8 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MIN8, -1);

/******************************************************************************
*                                 PWM_MAIN_MAX                                *
******************************************************************************/
/**
 * Set the max PWM value for the main 1 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX1, -1);

/**
 * Set the max PWM value for the main 2 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX2, -1);

/**
 * Set the max PWM value for the main 3 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX3, -1);

/**
 * Set the max PWM value for the main 4 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX4, -1);

/**
 * Set the max PWM value for the main 5 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX5, -1);

/**
 * Set the max PWM value for the main 6 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX6, -1);

/**
 * Set the max PWM value for the main 7 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX7, -1);

/**
 * Set the max PWM value for the main 8 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_MAX8, -1);

/******************************************************************************
*                                PWM_MAIN_FAIL                                *
******************************************************************************/

/**
 * Set the failsafe PWM for the main 1 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL1, -1);

/**
 * Set the failsafe PWM for the main 2 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL2, -1);

/**
 * Set the failsafe PWM for the main 3 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL3, -1);

/**
 * Set the failsafe PWM for the main 4 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL4, -1);

/**
 * Set the failsafe PWM for the main 5 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL5, -1);

/**
 * Set the failsafe PWM for the main 6 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL6, -1);

/**
 * Set the failsafe PWM for the main 7 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL7, -1);

/**
 * Set the failsafe PWM for the main 8 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_FAIL8, -1);

/******************************************************************************
*                                PWM_MAIN_DIS                                 *
******************************************************************************/

/**
 * Set the disarmed PWM for the main 1 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS1, -1);

/**
 * Set the disarmed PWM for the main 2 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS2, -1);

/**
 * Set the disarmed PWM for the main 3 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS3, -1);

/**
 * Set the disarmed PWM for the main 4 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS4, -1);

/**
 * Set the disarmed PWM for the main 5 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS5, -1);

/**
 * Set the disarmed PWM for the main 6 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS6, -1);

/**
 * Set the disarmed PWM for the main 7 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS7, -1);

/**
 * Set the disarmed PWM for the main 8 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_DIS8, -1);

/******************************************************************************
*                                PWM_MAIN_REV                                 *
******************************************************************************/

/**
 * Invert direction of main output channel 1
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV1, 0);

/**
 * Invert direction of main output channel 2
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV2, 0);

/**
 * Invert direction of main output channel 3
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV3, 0);

/**
 * Invert direction of main output channel 4
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV4, 0);

/**
 * Invert direction of main output channel 5
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV5, 0);

/**
 * Invert direction of main output channel 6
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV6, 0);

/**
 * Invert direction of main output channel 7
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV7, 0);

/**
 * Invert direction of main output channel 8
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_MAIN_REV8, 0);

/******************************************************************************
*                                PWM_MAIN_TRIM                                *
******************************************************************************/

/**
 * Trim value for main output channel 1
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM1, 0);

/**
 * Trim value for main output channel 2
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM2, 0);

/**
 * Trim value for main output channel 3
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM3, 0);

/**
 * Trim value for main output channel 4
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM4, 0);

/**
 * Trim value for main output channel 5
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM5, 0);

/**
 * Trim value for main output channel 6
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM6, 0);

/**
 * Trim value for main output channel 7
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM7, 0);

/**
 * Trim value for main output channel 8
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_MAIN_TRIM8, 0);
