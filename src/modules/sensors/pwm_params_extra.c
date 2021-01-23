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
 * @file pwm_params_extra.c
 *
 * Parameters defined for PWM output.
 *
 */

/******************************************************************************
*                                PWM_EXTRA_RATE                                 *
******************************************************************************/

/**
 * Set the PWM output frequency for the extra outputs
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
PARAM_DEFINE_INT32(PWM_EXTRA_RATE, 50);

/**
 * Set the minimum PWM for the extra outputs
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
PARAM_DEFINE_INT32(PWM_EXTRA_MIN, 1000);

/**
 * Set the maximum PWM for the extra outputs
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
PARAM_DEFINE_INT32(PWM_EXTRA_MAX, 2000);

/**
 * Set the disarmed PWM for extra outputs
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
PARAM_DEFINE_INT32(PWM_EXTRA_DISARM, 1500);

/******************************************************************************
*                                 PWM_EXTRA_MIN                                *
******************************************************************************/
/**
 * Set the min PWM value for the extra 1 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN1, -1);

/**
 * Set the min PWM value for the extra 2 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN2, -1);

/**
 * Set the min PWM value for the extra 3 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN3, -1);

/**
 * Set the min PWM value for the extra 4 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN4, -1);

/**
 * Set the min PWM value for the extra 5 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN5, -1);

/**
 * Set the min PWM value for the extra 6 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN6, -1);

/**
 * Set the min PWM value for the extra 7 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN7, -1);

/**
 * Set the min PWM value for the extra 8 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MIN will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MIN8, -1);

/******************************************************************************
*                                 PWM_EXTRA_MAX                                *
******************************************************************************/
/**
 * Set the max PWM value for the extra 1 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX1, -1);

/**
 * Set the max PWM value for the extra 2 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX2, -1);

/**
 * Set the max PWM value for the extra 3 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX3, -1);

/**
 * Set the max PWM value for the extra 4 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX4, -1);

/**
 * Set the max PWM value for the extra 5 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX5, -1);

/**
 * Set the max PWM value for the extra 6 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX6, -1);

/**
 * Set the max PWM value for the extra 7 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX7, -1);

/**
 * Set the max PWM value for the extra 8 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_EXTRA_MAX will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_MAX8, -1);

/******************************************************************************
*                                 PWM_EXTRA_FAIL                                *
******************************************************************************/
/**
 * Set the failsafe PWM for the extra 1 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL1, -1);

/**
 * Set the failsafe PWM for the extra 2 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL2, -1);

/**
 * Set the failsafe PWM for the extra 3 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL3, -1);

/**
 * Set the failsafe PWM for the extra 4 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL4, -1);

/**
 * Set the failsafe PWM for the extra 5 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL5, -1);

/**
 * Set the failsafe PWM for the extra 6 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL6, -1);

/**
 * Set the failsafe PWM for the extra 7 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL7, -1);

/**
 * Set the failsafe PWM for the extra 8 output
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
PARAM_DEFINE_INT32(PWM_EXTRA_FAIL8, -1);

/******************************************************************************
*                                 PWM_EXTRA_DIS                                 *
******************************************************************************/
/**
 * Set the disarmed PWM for the extra 1 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS1, -1);

/**
 * Set the disarmed PWM for the extra 2 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS2, -1);

/**
 * Set the disarmed PWM for the extra 3 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS3, -1);

/**
 * Set the disarmed PWM for the extra 4 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS4, -1);

/**
 * Set the disarmed PWM for the extra 5 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS5, -1);

/**
 * Set the disarmed PWM for the extra 6 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS6, -1);

/**
 * Set the disarmed PWM for the extra 7 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS7, -1);

/**
 * Set the disarmed PWM for the extra 8 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_EXTRA_DISARM will be used
 *
 * @reboot_required true
 *
 * @min -1
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_DIS8, -1);

/******************************************************************************
*                                 PWM_EXTRA_REV                                 *
******************************************************************************/
/**
 * Invert direction of extra output channel 1
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV1, 0);

/**
 * Invert direction of extra output channel 2
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV2, 0);

/**
 * Invert direction of extra output channel 3
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV3, 0);

/**
 * Invert direction of extra output channel 4
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV4, 0);

/**
 * Invert direction of extra output channel 5
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV5, 0);

/**
 * Invert direction of extra output channel 6
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV6, 0);

/**
 * Invert direction of extra output channel 7
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV7, 0);

/**
 * Invert direction of extra output channel 8
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_EXTRA_REV8, 0);

/******************************************************************************
*                                PWM_EXTRA_TRIM                                 *
******************************************************************************/

/**
 * Trim value for extra output channel 1
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM1, 0);

/**
 * Trim value for extra output channel 2
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM2, 0);

/**
 * Trim value for extra output channel 3
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM3, 0);

/**
 * Trim value for extra output channel 4
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM4, 0);

/**
 * Trim value for extra output channel 5
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM5, 0);

/**
 * Trim value for extra output channel 6
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM6, 0);

/**
 * Trim value for extra output channel 7
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM7, 0);

/**
 * Trim value for extra output channel 8
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(PWM_EXTRA_TRIM8, 0);
