/*
 * iqinetics_underactuated_propeller_params.c
 *
 *  Created on: Aug 14, 2017
 *      Author: Matthew Piccoli
 */

/**
 * Propeller max pulse
 *
 * The maximum pulse voltage amplitude
 *
 * @unit volts
 * @min 0
 * @max 11.1
 * @decimal 1
 * @increment 0.1
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_PULSE, 3.0f);

/**
 * Propeller max speed
 *
 * The propeller's maximum average speed
 *
 * @unit rad/s
 * @min 0
 * @max 2800
 * @decimal 1
 * @increment 100
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_SPEED, 400.0f);

/**
 * Propeller max yaw
 *
 * The propeller's maximum speed increase/decrease from a yaw command
 *
 * @unit rad/s
 * @min 0
 * @max 2800
 * @decimal 1
 * @increment 10
 * @reboot_required true
 * @group IQinetics
 */
PARAM_DEFINE_FLOAT(PROP_MAX_YAW, 100.0f);
