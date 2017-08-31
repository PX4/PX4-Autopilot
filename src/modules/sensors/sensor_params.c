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
 * @file sensor_params.c
 *
 * Parameters defined by the sensors task.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

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
 * Rotation of magnetometer 0 relative to airframe.
 *
 * An internal magnetometer will force a value of -1, so a GCS
 * should only attempt to configure the rotation if the value is
 * greater than or equal to zero.
 *
 * @value -1 Internal mag
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 *
 * @min -1
 * @max 30
 * @reboot_required true
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG0_ROT, -1);

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
 * Rotation of magnetometer 1 relative to airframe.
 *
 * An internal magnetometer will force a value of -1, so a GCS
 * should only attempt to configure the rotation if the value is
 * greater than or equal to zero.
 *
 * @value -1 Internal mag
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 *
 * @min -1
 * @max 30
 * @reboot_required true
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG1_ROT, -1);

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
 * Rotation of magnetometer 2 relative to airframe.
 *
 * An internal magnetometer will force a value of -1, so a GCS
 * should only attempt to configure the rotation if the value is
 * greater than or equal to zero.
 *
 * @value -1 Internal mag
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 *
 * @min -1
 * @max 30
 * @reboot_required true
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG2_ROT, -1);

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
 * ID of Magnetometer the calibration is for.
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG3_ID, 0);

/**
 * Rotation of magnetometer 2 relative to airframe.
 *
 * An internal magnetometer will force a value of -1, so a GCS
 * should only attempt to configure the rotation if the value is
 * greater than or equal to zero.
 *
 * @value -1 Internal mag
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 *
 * @min -1
 * @max 30
 * @reboot_required true
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG3_ROT, -1);

/**
 * Magnetometer X-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_XOFF, 0.0f);

/**
 * Magnetometer Y-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YOFF, 0.0f);

/**
 * Magnetometer Z-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ZOFF, 0.0f);

/**
 * Magnetometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_XSCALE, 1.0f);

/**
 * Magnetometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_YSCALE, 1.0f);

/**
 * Magnetometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG3_ZSCALE, 1.0f);


/**
 * Primary accel ID
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_ACC_PRIME, 0);

/**
 * Primary gyro ID
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_GYRO_PRIME, 0);

/**
 * Primary mag ID
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG_PRIME, 0);

/**
 * Bitfield selecting mag sides for calibration
 *
 * DETECT_ORIENTATION_TAIL_DOWN = 1
 * DETECT_ORIENTATION_NOSE_DOWN = 2
 * DETECT_ORIENTATION_LEFT = 4
 * DETECT_ORIENTATION_RIGHT = 8
 * DETECT_ORIENTATION_UPSIDE_DOWN = 16
 * DETECT_ORIENTATION_RIGHTSIDE_UP = 32
 *
 * @min 34
 * @max 63
 * @value 34 Two side calibration
 * @value 38 Three side calibration
 * @value 63 Six side calibration
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG_SIDES, 63);

/**
 * Primary baro ID
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_BARO_PRIME, 0);

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
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 *
 * @reboot_required true
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_BOARD_ROT, 0);

/**
 * PX4Flow board rotation
 *
 * This parameter defines the yaw rotation of the PX4FLOW board relative to the vehicle body frame.
 * Zero rotation is defined as X on flow board pointing towards front of vehicle.
 * The recommneded installation default for the PX4FLOW board is with the Y axis forward (270 deg yaw).
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 *
 * @reboot_required true
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_FLOW_ROT, 6);

/**
 * Board rotation Y (Pitch) offset
 *
 * This parameter defines a rotational offset in degrees around the Y (Pitch) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_BOARD_Y_OFF, 0.0f);

/**
 * Board rotation X (Roll) offset
 *
 * This parameter defines a rotational offset in degrees around the X (Roll) axis It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_BOARD_X_OFF, 0.0f);

/**
 * Board rotation Z (YAW) offset
 *
 * This parameter defines a rotational offset in degrees around the Z (Yaw) axis. It allows the user
 * to fine tune the board offset in the event of misalignment.
 *
 * @unit deg
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_BOARD_Z_OFF, 0.0f);

/**
 * Select primary magnetometer.
 * DEPRECATED, only used on V1 hardware
 *
 * @min 0
 * @max 2
 * @value 0 Auto-select Mag
 * @value 1 External is primary Mag
 * @value 2 Internal is primary Mag
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_EXT_MAG, 0);

/**
 * Threshold (of RMS) to warn about high vibration levels
 *
 * @group Sensor Calibration
 * @min 0.01
 * @max 10
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ATT_VIBE_THRESH, 0.2f);

/**
 * Scaling factor for battery voltage sensor on PX4IO.
 *
 * @min 1
 * @max 100000
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_V_SCALE_IO, 10000);

/**
 * Scaling from ADC counts to volt on the ADC input (battery voltage)
 *
 * This is not the battery voltage, but the intermediate ADC voltage.
 * A value of -1 signifies that the board defaults are used, which is
 * highly recommended.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_CNT_V_VOLT, -1.0f);

/**
 * Scaling from ADC counts to volt on the ADC input (battery current)
 *
 * This is not the battery current, but the intermediate ADC voltage.
 * A value of -1 signifies that the board defaults are used, which is
 * highly recommended.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_CNT_V_CURR, -1.0);

/**
 * Offset in volt as seen by the ADC input of the current sensor.
 *
 * This offset will be subtracted before calculating the battery
 * current based on the voltage.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_V_OFFS_CURR, 0.0);

/**
 * Battery voltage divider (V divider)
 *
 * This is the divider from battery voltage to 3.3V ADC voltage.
 * If using e.g. Mauch power modules the value from the datasheet
 * can be applied straight here. A value of -1 means to use
 * the board default.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_V_DIV, -1.0);

/**
 * Battery current per volt (A/V)
 *
 * The voltage seen by the 3.3V ADC multiplied by this factor
 * will determine the battery current. A value of -1 means to use
 * the board default.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_A_PER_V, -1.0);

/**
 * Battery monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 *
 * @min 0
 * @max 1
 * @value 0 Power Module
 * @value 1 External
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_SOURCE, 0);

/**
 * Lidar-Lite (LL40LS)
 *
 * @reboot_required true
 * @min 0
 * @max 2
 * @group Sensor Enable
 * @value 0 Disabled
 * @value 1 PWM
 * @value 2 I2C
 */
PARAM_DEFINE_INT32(SENS_EN_LL40LS, 0);

/**
 * Lightware laser rangefinder (serial)
 *
 * @reboot_required true
 * @min 0
 * @max 4
 * @group Sensor Enable
 * @value 0 Disabled
 * @value 1 SF02
 * @value 2 SF10/a
 * @value 3 SF10/b
 * @value 4 SF10/c
 * @value 5 SF11/c
 */
PARAM_DEFINE_INT32(SENS_EN_SF0X, 0);

/**
 * Maxbotix Soanr (mb12xx)
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensor Enable
 */
PARAM_DEFINE_INT32(SENS_EN_MB12XX, 0);

/**
 * TeraRanger One (trone)
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensor Enable
 */
PARAM_DEFINE_INT32(SENS_EN_TRONE, 0);

/**
 * Lightware SF1xx laser rangefinder (i2c)
 *
 * @reboot_required true
 * @min 0
 * @max 4
 * @group Sensor Enable
 * @value 0 Disabled
 * @value 1 SF10/a
 * @value 2 SF10/b
 * @value 3 SF10/c
 * @value 4 SF11/c
 */
PARAM_DEFINE_INT32(SENS_EN_SF1XX, 0);

/**
 * Thermal control of sensor temperature
 *
 * @value -1 Thermal control unavailable
 * @value 0 Thermal control off
 * @group Sensor Enable
 */
PARAM_DEFINE_INT32(SENS_EN_THERMAL, -1);
