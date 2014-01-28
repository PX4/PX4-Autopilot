/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file sensor_params.c
 *
 * Parameters defined by the sensors task.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/**
 * Gyro X offset
 *
 * This is an X-axis offset for the gyro. Adjust it according to the calibration data.
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_XOFF, 0.0f);

/**
 * Gyro Y offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_YOFF, 0.0f);

/**
 * Gyro Z offset
 *
 * @min -5.0
 * @max 5.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_ZOFF, 0.0f);

/**
 * Gyro X scaling
 *
 * X-axis scaling.
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_XSCALE, 1.0f);

/**
 * Gyro Y scaling
 *
 * Y-axis scaling.
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_YSCALE, 1.0f);

/**
 * Gyro Z scaling
 *
 * Z-axis scaling.
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_ZSCALE, 1.0f);

/**
 * Magnetometer X offset
 *
 * This is an X-axis offset for the magnetometer.
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_XOFF, 0.0f);

/**
 * Magnetometer Y offset
 *
 * This is an Y-axis offset for the magnetometer.
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_YOFF, 0.0f);

/**
 * Magnetometer Z offset
 *
 * This is an Z-axis offset for the magnetometer.
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_ZOFF, 0.0f);

PARAM_DEFINE_FLOAT(SENS_MAG_XSCALE, 1.0f);
PARAM_DEFINE_FLOAT(SENS_MAG_YSCALE, 1.0f);
PARAM_DEFINE_FLOAT(SENS_MAG_ZSCALE, 1.0f);

PARAM_DEFINE_FLOAT(SENS_ACC_XOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENS_ACC_YOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENS_ACC_ZOFF, 0.0f);

PARAM_DEFINE_FLOAT(SENS_ACC_XSCALE, 1.0f);
PARAM_DEFINE_FLOAT(SENS_ACC_YSCALE, 1.0f);
PARAM_DEFINE_FLOAT(SENS_ACC_ZSCALE, 1.0f);

PARAM_DEFINE_FLOAT(SENS_DPRES_OFF, 0.0f);
PARAM_DEFINE_INT32(SENS_DPRES_ANA, 0);

PARAM_DEFINE_INT32(SENS_BOARD_ROT, 0);
PARAM_DEFINE_INT32(SENS_EXT_MAG_ROT, 0);

/**
 * RC Channel 1 Minimum
 *
 * Minimum value for RC channel 1
 *
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MIN, 1000.0f);

/**
 * RC Channel 1 Trim
 *
 * Mid point value (same as min for throttle)
 *
 * @min 800.0
 * @max 2200.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_TRIM, 1500.0f);

/**
 * RC Channel 1 Maximum
 *
 * Maximum value for RC channel 1
 *
 * @min 1500.0
 * @max 2200.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_MAX, 2000.0f);

/**
 * RC Channel 1 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_REV, 1.0f);

/**
 * RC Channel 1 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_DZ, 10.0f);

/**
 * RC Channel 2 Minimum
 *
 * Minimum value for RC channel 2
 *
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MIN, 1000.0f);

/**
 * RC Channel 2 Trim
 *
 * Mid point value (same as min for throttle)
 *
 * @min 800.0
 * @max 2200.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_TRIM, 1500.0f);

/**
 * RC Channel 2 Maximum
 *
 * Maximum value for RC channel 2
 *
 * @min 1500.0
 * @max 2200.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MAX, 2000.0f);

/**
 * RC Channel 2 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_REV, 1.0f);

/**
 * RC Channel 2 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_DZ, 10.0f);

PARAM_DEFINE_FLOAT(RC3_MIN, 1000);
PARAM_DEFINE_FLOAT(RC3_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC3_MAX, 2000);
PARAM_DEFINE_FLOAT(RC3_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC3_DZ, 10.0f);

PARAM_DEFINE_FLOAT(RC4_MIN, 1000);
PARAM_DEFINE_FLOAT(RC4_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC4_MAX, 2000);
PARAM_DEFINE_FLOAT(RC4_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC4_DZ, 10.0f);

PARAM_DEFINE_FLOAT(RC5_MIN, 1000);
PARAM_DEFINE_FLOAT(RC5_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC5_MAX, 2000);
PARAM_DEFINE_FLOAT(RC5_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC5_DZ,  10.0f);

PARAM_DEFINE_FLOAT(RC6_MIN, 1000);
PARAM_DEFINE_FLOAT(RC6_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC6_MAX, 2000);
PARAM_DEFINE_FLOAT(RC6_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC6_DZ, 10.0f);

PARAM_DEFINE_FLOAT(RC7_MIN, 1000);
PARAM_DEFINE_FLOAT(RC7_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC7_MAX, 2000);
PARAM_DEFINE_FLOAT(RC7_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC7_DZ, 10.0f);

PARAM_DEFINE_FLOAT(RC8_MIN, 1000);
PARAM_DEFINE_FLOAT(RC8_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC8_MAX, 2000);
PARAM_DEFINE_FLOAT(RC8_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC8_DZ, 10.0f);

PARAM_DEFINE_FLOAT(RC9_MIN, 1000);
PARAM_DEFINE_FLOAT(RC9_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC9_MAX, 2000);
PARAM_DEFINE_FLOAT(RC9_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC9_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC10_MIN, 1000);
PARAM_DEFINE_FLOAT(RC10_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC10_MAX, 2000);
PARAM_DEFINE_FLOAT(RC10_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC10_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC11_MIN, 1000);
PARAM_DEFINE_FLOAT(RC11_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC11_MAX, 2000);
PARAM_DEFINE_FLOAT(RC11_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC11_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC12_MIN, 1000);
PARAM_DEFINE_FLOAT(RC12_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC12_MAX, 2000);
PARAM_DEFINE_FLOAT(RC12_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC12_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC13_MIN, 1000);
PARAM_DEFINE_FLOAT(RC13_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC13_MAX, 2000);
PARAM_DEFINE_FLOAT(RC13_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC13_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC14_MIN, 1000);
PARAM_DEFINE_FLOAT(RC14_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC14_MAX, 2000);
PARAM_DEFINE_FLOAT(RC14_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC14_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC15_MIN, 1000);
PARAM_DEFINE_FLOAT(RC15_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC15_MAX, 2000);
PARAM_DEFINE_FLOAT(RC15_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC15_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC16_MIN, 1000);
PARAM_DEFINE_FLOAT(RC16_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC16_MAX, 2000);
PARAM_DEFINE_FLOAT(RC16_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC16_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC17_MIN, 1000);
PARAM_DEFINE_FLOAT(RC17_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC17_MAX, 2000);
PARAM_DEFINE_FLOAT(RC17_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC17_DZ, 0.0f);

PARAM_DEFINE_FLOAT(RC18_MIN, 1000);
PARAM_DEFINE_FLOAT(RC18_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC18_MAX, 2000);
PARAM_DEFINE_FLOAT(RC18_REV, 1.0f);
PARAM_DEFINE_FLOAT(RC18_DZ, 0.0f);

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
PARAM_DEFINE_INT32(RC_RL1_DSM_VCC, 0); /* Relay 1 controls DSM VCC */
#endif
PARAM_DEFINE_INT32(RC_DSM_BIND, -1); /* -1 = Idle, 0 = Start DSM2 bind, 1 = Start DSMX bind */

PARAM_DEFINE_INT32(BAT_V_SCALE_IO, 10000);
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
PARAM_DEFINE_FLOAT(BAT_V_SCALING, 0.0082f);
#else
/* default is conversion factor for the PX4IO / PX4IOAR board, the factor for PX4FMU standalone is different */
/* PX4IOAR: 0.00838095238 */
/* FMU standalone: 1/(10 / (47+10)) * (3.3 / 4095) = 0.00459340659 */
/* FMU with PX4IOAR: (3.3f * 52.0f / 5.0f / 4095.0f) */
PARAM_DEFINE_FLOAT(BAT_V_SCALING, 0.00459340659f);
#endif
PARAM_DEFINE_FLOAT(BAT_C_SCALING, 0.0124);	/* scaling for 3DR power brick */

/**
 * Roll control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading roll inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_ROLL, 1);

/**
 * Pitch control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading pitch inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_PITCH, 2);

/**
 * Throttle control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading throttle inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 3);

/**
 * Yaw control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading yaw inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_YAW, 4);

/**
 * Mode switch channel mapping.
 *
 * This is the main flight mode selector.
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for deciding about the main mode.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_MODE_SW, 0);
PARAM_DEFINE_INT32(RC_MAP_RETURN_SW, 0);
PARAM_DEFINE_INT32(RC_MAP_ASSIST_SW, 0);
PARAM_DEFINE_INT32(RC_MAP_MISSIO_SW, 0);

//PARAM_DEFINE_INT32(RC_MAP_OFFB_SW, 0);

PARAM_DEFINE_INT32(RC_MAP_FLAPS, 0);

PARAM_DEFINE_INT32(RC_MAP_AUX1, 0);	/**< default function: camera pitch */
PARAM_DEFINE_INT32(RC_MAP_AUX2, 0);	/**< default function: camera roll */
PARAM_DEFINE_INT32(RC_MAP_AUX3, 0);	/**< default function: camera azimuth / yaw */

PARAM_DEFINE_FLOAT(RC_SCALE_ROLL, 0.6f);
PARAM_DEFINE_FLOAT(RC_SCALE_PITCH, 0.6f);
PARAM_DEFINE_FLOAT(RC_SCALE_YAW, 2.0f);

PARAM_DEFINE_INT32(RC_FS_CH, 0);	/**< RC failsafe channel, 0 = disable */
PARAM_DEFINE_INT32(RC_FS_MODE, 0);	/**< RC failsafe mode: 0 = too low means signal loss, 1 = too high means signal loss */
PARAM_DEFINE_FLOAT(RC_FS_THR, 800);	/**< RC failsafe PWM threshold */
