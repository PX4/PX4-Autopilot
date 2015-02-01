/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

/**
 * ID of the board this parameter set was calibrated on.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_BOARD_ID, 0);

/**
 * ID of the Gyro that the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_GYRO0_ID, 0);

/**
 * Gyro X-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_XOFF, 0.0f);

/**
 * Gyro Y-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_YOFF, 0.0f);

/**
 * Gyro Z-axis offset
 *
 * @min -5.0
 * @max 5.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_ZOFF, 0.0f);

/**
 * Gyro X-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_XSCALE, 1.0f);

/**
 * Gyro Y-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_YSCALE, 1.0f);

/**
 * Gyro Z-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO0_ZSCALE, 1.0f);

/**
 * ID of Magnetometer the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG0_ID, 0);

/**
 * Magnetometer X-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XOFF, 0.0f);

/**
 * Magnetometer Y-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YOFF, 0.0f);

/**
 * Magnetometer Z-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZOFF, 0.0f);

/**
 * Magnetometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_XSCALE, 1.0f);

/**
 * Magnetometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_YSCALE, 1.0f);

/**
 * Magnetometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG0_ZSCALE, 1.0f);

/**
 * ID of the Accelerometer that the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC0_ID, 0);

/**
 * Accelerometer X-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_XOFF, 0.0f);

/**
 * Accelerometer Y-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_YOFF, 0.0f);

/**
 * Accelerometer Z-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_ZOFF, 0.0f);

/**
 * Accelerometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_XSCALE, 1.0f);

/**
 * Accelerometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_YSCALE, 1.0f);

/**
 * Accelerometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC0_ZSCALE, 1.0f);

/**
 * ID of the Gyro that the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_GYRO1_ID, 0);

/**
 * Gyro X-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_XOFF, 0.0f);

/**
 * Gyro Y-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_YOFF, 0.0f);

/**
 * Gyro Z-axis offset
 *
 * @min -5.0
 * @max 5.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_ZOFF, 0.0f);

/**
 * Gyro X-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_XSCALE, 1.0f);

/**
 * Gyro Y-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_YSCALE, 1.0f);

/**
 * Gyro Z-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO1_ZSCALE, 1.0f);

/**
 * ID of Magnetometer the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG1_ID, 0);

/**
 * Magnetometer X-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XOFF, 0.0f);

/**
 * Magnetometer Y-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YOFF, 0.0f);

/**
 * Magnetometer Z-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZOFF, 0.0f);

/**
 * Magnetometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XSCALE, 1.0f);

/**
 * Magnetometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YSCALE, 1.0f);

/**
 * Magnetometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZSCALE, 1.0f);

/**
 * ID of the Accelerometer that the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC1_ID, 0);

/**
 * Accelerometer X-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_XOFF, 0.0f);

/**
 * Accelerometer Y-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_YOFF, 0.0f);

/**
 * Accelerometer Z-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_ZOFF, 0.0f);

/**
 * Accelerometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_XSCALE, 1.0f);

/**
 * Accelerometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_YSCALE, 1.0f);

/**
 * Accelerometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC1_ZSCALE, 1.0f);

/**
 * ID of the Gyro that the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_GYRO2_ID, 0);

/**
 * Gyro X-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_XOFF, 0.0f);

/**
 * Gyro Y-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_YOFF, 0.0f);

/**
 * Gyro Z-axis offset
 *
 * @min -5.0
 * @max 5.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_ZOFF, 0.0f);

/**
 * Gyro X-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_XSCALE, 1.0f);

/**
 * Gyro Y-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_YSCALE, 1.0f);

/**
 * Gyro Z-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_GYRO2_ZSCALE, 1.0f);

/**
 * ID of Magnetometer the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG2_ID, 0);

/**
 * Magnetometer X-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_XOFF, 0.0f);

/**
 * Magnetometer Y-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YOFF, 0.0f);

/**
 * Magnetometer Z-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ZOFF, 0.0f);

/**
 * Magnetometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_XSCALE, 1.0f);

/**
 * Magnetometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_YSCALE, 1.0f);

/**
 * Magnetometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG2_ZSCALE, 1.0f);

/**
 * ID of the Accelerometer that the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC2_ID, 0);

/**
 * Accelerometer X-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_XOFF, 0.0f);

/**
 * Accelerometer Y-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_YOFF, 0.0f);

/**
 * Accelerometer Z-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_ZOFF, 0.0f);

/**
 * Accelerometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_XSCALE, 1.0f);

/**
 * Accelerometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_YSCALE, 1.0f);

/**
 * Accelerometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_ACC2_ZSCALE, 1.0f);

/**
 * Differential pressure sensor offset
 *
 * The offset (zero-reading) in Pascal
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_DPRES_OFF, 0.0f);

/**
 * Differential pressure sensor analog scaling
 *
 * Pick the appropriate scaling from the datasheet.
 * this number defines the (linear) conversion from voltage
 * to Pascal (pa). For the MPXV7002DP this is 1000.
 *
 * NOTE: If the sensor always registers zero, try switching
 * the static and dynamic tubes.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_DPRES_ANSC, 0);

/**
 * QNH for barometer
 *
 * @min 500
 * @max 1500
 * @group Sensor Calibration
 * @unit hPa
 */
PARAM_DEFINE_FLOAT(SENS_BARO_QNH, 1013.25f);


/**
 * Board rotation
 *
 * This parameter defines the rotation of the FMU board relative to the platform.
 * Possible values are:
 *    0 = No rotation
 *    1 = Yaw 45°
 *    2 = Yaw 90°
 *    3 = Yaw 135°
 *    4 = Yaw 180°
 *    5 = Yaw 225°
 *    6 = Yaw 270°
 *    7 = Yaw 315°
 *    8 = Roll 180°
 *    9 = Roll 180°, Yaw 45°
 *   10 = Roll 180°, Yaw 90°
 *   11 = Roll 180°, Yaw 135°
 *   12 = Pitch 180°
 *   13 = Roll 180°, Yaw 225°
 *   14 = Roll 180°, Yaw 270°
 *   15 = Roll 180°, Yaw 315°
 *   16 = Roll 90°
 *   17 = Roll 90°, Yaw 45°
 *   18 = Roll 90°, Yaw 90°
 *   19 = Roll 90°, Yaw 135°
 *   20 = Roll 270°
 *   21 = Roll 270°, Yaw 45°
 *   22 = Roll 270°, Yaw 90°
 *   23 = Roll 270°, Yaw 135°
 *   24 = Pitch 90°
 *   25 = Pitch 270°
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_BOARD_ROT, 0);

/**
 * PX4Flow board rotation
 *
 * This parameter defines the rotation of the PX4FLOW board relative to the platform.
 * Zero rotation is defined as Y on flow board pointing towards front of vehicle
 * Possible values are:
 *    0 = No rotation
 *    1 = Yaw 45°
 *    2 = Yaw 90°
 *    3 = Yaw 135°
 *    4 = Yaw 180°
 *    5 = Yaw 225°
 *    6 = Yaw 270°
 *    7 = Yaw 315°
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_FLOW_ROT, 0);

/**
 * Board rotation Y (Pitch) offset
 *
 * This parameter defines a rotational offset in degrees around the Y (Pitch) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @group Sensor Calibration
 */
 PARAM_DEFINE_FLOAT(SENS_BOARD_Y_OFF, 0.0f);

/**
 * Board rotation X (Roll) offset
 *
 * This parameter defines a rotational offset in degrees around the X (Roll) axis It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_BOARD_X_OFF, 0.0f);

/**
 * Board rotation Z (YAW) offset
 *
 * This parameter defines a rotational offset in degrees around the Z (Yaw) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_BOARD_Z_OFF, 0.0f);

/**
 * External magnetometer rotation
 *
 * This parameter defines the rotation of the external magnetometer relative
 * to the platform (not relative to the FMU).
 * See SENS_BOARD_ROT for possible values.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_EXT_MAG_ROT, 0);

/**
* Set usage of external magnetometer
*
*  * Set to 0 (default) to auto-detect (will try to get the external as primary)
*  * Set to 1 to force the external magnetometer as primary
*  * Set to 2 to force the internal magnetometer as primary
*
* @min 0
* @max 2
* @group Sensor Calibration
*/
PARAM_DEFINE_INT32(SENS_EXT_MAG, 0);


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

/**
 * DSM binding trigger.
 *
 * -1 = Idle, 0 = Start DSM2 bind, 1 = Start DSMX bind
 *
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_DSM_BIND, -1);


/**
 * Scaling factor for battery voltage sensor on PX4IO.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_V_SCALE_IO, 10000);

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
/**
 * Scaling factor for battery voltage sensor on FMU v2.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_SCALING, 0.0082f);
#elif CONFIG_ARCH_BOARD_AEROCORE
/**
 * Scaling factor for battery voltage sensor on AeroCore.
 *
 * For R70 = 133K, R71 = 10K --> scale = 1.8 * 143 / (4096*10) = 0.0063
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_SCALING, 0.0063f);
#else
/**
 * Scaling factor for battery voltage sensor on FMU v1.
 *
 * FMUv1 standalone: 1/(10 / (47+10)) * (3.3 / 4095) = 0.00459340659
 * FMUv1 with PX4IO: 0.00459340659
 * FMUv1 with PX4IOAR: (3.3f * 52.0f / 5.0f / 4095.0f) = 0.00838095238
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_SCALING, 0.00459340659f);
#endif

/**
 * Scaling factor for battery current sensor.
 *
 * @group Battery Calibration
 */
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
 * Failsafe channel mapping.
 *
 * The RC mapping index indicates which channel is used for failsafe
 * If 0, whichever channel is mapped to throttle is used
 * otherwise the value indicates the specific rc channel to use
 *
 * @min 0
 * @max 18
 *
 *
 */
PARAM_DEFINE_INT32(RC_MAP_FAILSAFE, 0);  //Default to throttle function

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

/**
 * Return switch channel mapping.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_RETURN_SW, 0);

/**
 * Posctl switch channel mapping.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_POSCTL_SW, 0);

/**
 * Loiter switch channel mapping.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_LOITER_SW, 0);

/**
 * Acro switch channel mapping.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_ACRO_SW, 0);

/**
 * Offboard switch channel mapping.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_OFFB_SW, 0);

/**
 * Flaps channel mapping.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_FLAPS, 0);

/**
 * Auxiliary switch 1 channel mapping.
 *
 * Default function: Camera pitch
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_AUX1, 0);

/**
 * Auxiliary switch 2 channel mapping.
 *
 * Default function: Camera roll
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_AUX2, 0);	/**< default function: camera roll */

/**
 * Auxiliary switch 3 channel mapping.
 *
 * Default function: Camera azimuth / yaw
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_AUX3, 0);

/**
 * Channel which changes a parameter
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 1st parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM1, 0);

/**
 * Channel which changes a parameter
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 2nd parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM2, 0);

/**
 * Channel which changes a parameter
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 3th parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM3, 0);

/**
 * Failsafe channel PWM threshold.
 *
 * @min 800
 * @max 2200
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_FAILS_THR, 0);

/**
 * Threshold for selecting assist mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_ASSIST_TH, 0.25f);

/**
 * Threshold for selecting auto mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_AUTO_TH, 0.75f);

/**
 * Threshold for selecting posctl mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_POSCTL_TH, 0.5f);

/**
 * Threshold for selecting return to launch mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_RETURN_TH, 0.5f);

/**
 * Threshold for selecting loiter mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_LOITER_TH, 0.5f);

/**
 * Threshold for selecting acro mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_ACRO_TH, 0.5f);


/**
 * Threshold for selecting offboard mode
 *
 * min:-1
 * max:+1
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 */
PARAM_DEFINE_FLOAT(RC_OFFB_TH, 0.5f);
