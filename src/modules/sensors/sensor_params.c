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
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_BOARD_ROT, 0);

/**
 * PX4Flow board rotation
 *
 * This parameter defines the rotation of the PX4FLOW board relative to the platform.
 * Zero rotation is defined as Y on flow board pointing towards front of vehicle
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
PARAM_DEFINE_INT32(SENS_FLOW_ROT, 0);

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
 * External magnetometer rotation
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
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(SENS_EXT_MAG_ROT, 0);

/**
 * Select primary magnetometer
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
 * RC Channel 1 Minimum
 *
 * Minimum value for RC channel 1
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
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
 * @unit us
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
 * @unit us
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
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC1_DZ, 10.0f);

/**
 * RC Channel 2 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_MIN, 1000.0f);

/**
 * RC Channel 2 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_TRIM, 1500.0f);

/**
 * RC Channel 2 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
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
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC2_DZ, 10.0f);

/**
 * RC Channel 3 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MIN, 1000);

/**
 * RC Channel 3 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_TRIM, 1500);

/**
 * RC Channel 3 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_MAX, 2000);

/**
 * RC Channel 3 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_REV, 1.0f);
/**
 * RC Channel 3 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC3_DZ, 10.0f);

/**
 * RC Channel 4 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MIN, 1000);

/**
 * RC Channel 4 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_TRIM, 1500);

/**
 * RC Channel 4 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_MAX, 2000);

/**
 * RC Channel 4 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_REV, 1.0f);

/**
 * RC Channel 4 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC4_DZ, 10.0f);

/**
 * RC Channel 5 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MIN, 1000);

/**
 * RC Channel 5 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_TRIM, 1500);

/**
 * RC Channel 5 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_MAX, 2000);

/**
 * RC Channel 5 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_REV, 1.0f);

/**
 * RC Channel 5 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC5_DZ,  10.0f);

/**
 * RC Channel 6 Minimum
 *
 * Minimum value for this channel.
 *
 * @unit us
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MIN, 1000);

/**
 * RC Channel 6 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_TRIM, 1500);

/**
 * RC Channel 6 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_MAX, 2000);

/**
 * RC Channel 6 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_REV, 1.0f);

/**
 * RC Channel 6 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC6_DZ, 10.0f);

/**
 * RC Channel 7 Minimum
 *
 * Minimum value for this channel.
 *
 * @unit us
 * @min 800.0
 * @max 1500.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MIN, 1000);

/**
 * RC Channel 7 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_TRIM, 1500);

/**
 * RC Channel 7 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_MAX, 2000);

/**
 * RC Channel 7 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_REV, 1.0f);

/**
 * RC Channel 7 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC7_DZ, 10.0f);

/**
 * RC Channel 8 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MIN, 1000);

/**
 * RC Channel 8 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_TRIM, 1500);

/**
 * RC Channel 8 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_MAX, 2000);

/**
 * RC Channel 8 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_REV, 1.0f);

/**
 * RC Channel 8 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC8_DZ, 10.0f);

/**
 * RC Channel 9 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MIN, 1000);

/**
 * RC Channel 9 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_TRIM, 1500);

/**
 * RC Channel 9 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_MAX, 2000);

/**
 * RC Channel 9 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_REV, 1.0f);

/**
 * RC Channel 9 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC9_DZ, 0.0f);

/**
 * RC Channel 10 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MIN, 1000);

/**
 * RC Channel 10 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_TRIM, 1500);

/**
 * RC Channel 10 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_MAX, 2000);

/**
 * RC Channel 10 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_REV, 1.0f);

/**
 * RC Channel 10 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC10_DZ, 0.0f);

/**
 * RC Channel 11 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MIN, 1000);

/**
 * RC Channel 11 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_TRIM, 1500);

/**
 * RC Channel 11 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_MAX, 2000);

/**
 * RC Channel 11 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_REV, 1.0f);

/**
 * RC Channel 11 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC11_DZ, 0.0f);

/**
 * RC Channel 12 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MIN, 1000);

/**
 * RC Channel 12 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_TRIM, 1500);

/**
 * RC Channel 12 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_MAX, 2000);

/**
 * RC Channel 12 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_REV, 1.0f);

/**
 * RC Channel 12 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC12_DZ, 0.0f);

/**
 * RC Channel 13 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MIN, 1000);

/**
 * RC Channel 13 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_TRIM, 1500);

/**
 * RC Channel 13 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_MAX, 2000);

/**
 * RC Channel 13 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_REV, 1.0f);

/**
 * RC Channel 13 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC13_DZ, 0.0f);

/**
 * RC Channel 14 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MIN, 1000);

/**
 * RC Channel 14 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_TRIM, 1500);

/**
 * RC Channel 14 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_MAX, 2000);

/**
 * RC Channel 14 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_REV, 1.0f);

/**
 * RC Channel 14 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC14_DZ, 0.0f);

/**
 * RC Channel 15 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MIN, 1000);

/**
 * RC Channel 15 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_TRIM, 1500);

/**
 * RC Channel 15 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_MAX, 2000);

/**
 * RC Channel 15 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_REV, 1.0f);

/**
 * RC Channel 15 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC15_DZ, 0.0f);

/**
 * RC Channel 16 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MIN, 1000);

/**
 * RC Channel 16 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_TRIM, 1500);

/**
 * RC Channel 16 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_MAX, 2000);

/**
 * RC Channel 16 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_REV, 1.0f);

/**
 * RC Channel 16 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC16_DZ, 0.0f);

/**
 * RC Channel 17 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MIN, 1000);

/**
 * RC Channel 17 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_TRIM, 1500);

/**
 * RC Channel 17 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_MAX, 2000);

/**
 * RC Channel 17 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_REV, 1.0f);

/**
 * RC Channel 17 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC17_DZ, 0.0f);

/**
 * RC Channel 18 Minimum
 *
 * Minimum value for this channel.
 *
 * @min 800.0
 * @max 1500.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MIN, 1000);

/**
 * RC Channel 18 Trim
 *
 * Mid point value (has to be set to the same as min for throttle channel).
 *
 * @min 800.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_TRIM, 1500);

/**
 * RC Channel 18 Maximum
 *
 * Maximum value for this channel.
 *
 * @min 1500.0
 * @max 2200.0
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_MAX, 2000);

/**
 * RC Channel 18 Reverse
 *
 * Set to -1 to reverse channel.
 *
 * @min -1.0
 * @max 1.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_REV, 1.0f);

/**
 * RC Channel 18 dead zone
 *
 * The +- range of this value around the trim value will be considered as zero.
 *
 * @min 0.0
 * @max 100.0
 * @group Radio Calibration
 */
PARAM_DEFINE_FLOAT(RC18_DZ, 0.0f);

/**
 * Enable relay control of relay 1 mapped to the Spektrum receiver power supply
 *
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Relay controls DSM power
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_RL1_DSM_VCC, 0); /* Relay 1 controls DSM VCC */

/**
 * DSM binding trigger.
 *
 * @value -1 Inactive
 * @value 0 Start DSM2 bind
 * @value 1 Start DSMX bind
 * @min -1
 * @max 1
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_DSM_BIND, -1);

/**
 * Scaling factor for battery voltage sensor on PX4IO.
 *
 * @min 1
 * @max 100000
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_V_SCALE_IO, 10000);

/**
 * Scaling factor for battery voltage sensor on FMU v2.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_V_SCALING, -1.0f);

/**
 * Scaling factor for battery current sensor.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_C_SCALING, -1.0);

/**
 * Offset for battery current sensor.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_C_OFFSET, -1.0);


/**
 * RC channel count
 *
 * This parameter is used by Ground Station software to save the number
 * of channels which were used during RC calibration. It is only meant
 * for ground station use.
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 */

PARAM_DEFINE_INT32(RC_CHAN_CNT, 0);

/**
 * RC mode switch threshold automatic distribution
 *
 * This parameter is used by Ground Station software to specify whether
 * the threshold values for flight mode switches were automatically calculated.
 * 0 indicates that the threshold values were set by the user. Any other value
 * indicates that the threshold value where automatically set by the ground
 * station software. It is only meant for ground station use.
 *
 * @boolean
 * @group Radio Calibration
 */

PARAM_DEFINE_INT32(RC_TH_USER, 1);

/**
 * Roll control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading roll inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_ROLL, 0);

/**
 * Pitch control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading pitch inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_PITCH, 0);

/**
 * Failsafe channel mapping.
 *
 * The RC mapping index indicates which channel is used for failsafe
 * If 0, whichever channel is mapped to throttle is used
 * otherwise the value indicates the specific rc channel to use
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
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
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 0);

/**
 * Yaw control channel mapping.
 *
 * The channel index (starting from 1 for channel 1) indicates
 * which channel should be used for reading yaw inputs from.
 * A value of zero indicates the switch is not assigned.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_MAP_YAW, 0);

/**
 * Single channel flight mode selection
 *
 * If this parameter is non-zero, flight modes are only selected
 * by this channel and are assigned to six slots.
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_FLTMODE, 0);

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
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_MODE_SW, 0);

/**
 * Return switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_RETURN_SW, 0);

/**
 * Rattitude switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_RATT_SW, 0);

/**
 * Position Control switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_POSCTL_SW, 0);

/**
 * Loiter switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_LOITER_SW, 0);

/**
 * Acro switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_ACRO_SW, 0);

/**
 * Offboard switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_OFFB_SW, 0);

/**
 * Kill switch channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_KILL_SW, 0);

/**
 * Flaps channel
 *
 * @min 0
 * @max 18
 * @group Radio Switches
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_FLAPS, 0);

/**
 * AUX1 channel
 *
 * Default function: Camera pitch
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX1, 0);

/**
 * AUX2 channel
 *
 * Default function: Camera roll
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX2, 0);

/**
 * AUX3 Channel
 *
 * Default function: Camera azimuth / yaw
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_AUX3, 0);

/**
 * PARAM1 tuning channel
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 1st parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM1, 0);

/**
 * PARAM2 tuning channel
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 2nd parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM2, 0);

/**
 * PARAM3 tuning channel
 *
 * Can be used for parameter tuning with the RC. This one is further referenced as the 3th parameter channel.
 * Set to 0 to deactivate *
 *
 * @min 0
 * @max 18
 * @group Radio Calibration
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 */
PARAM_DEFINE_INT32(RC_MAP_PARAM3, 0);

/**
 * Failsafe channel PWM threshold.
 *
 * Set to a value slightly above the PWM value assumed by throttle in a failsafe event,
 * but ensure it is below the PWM value assumed by throttle during normal operation.
 *
 * @min 0
 * @max 2200
 * @unit us
 * @group Radio Calibration
 */
PARAM_DEFINE_INT32(RC_FAILS_THR, 0);

/**
 * Threshold for selecting assist mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_ASSIST_TH, 0.25f);

/**
 * Threshold for selecting auto mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_AUTO_TH, 0.75f);

/**
 * Threshold for selecting rattitude mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 */
PARAM_DEFINE_FLOAT(RC_RATT_TH, 0.5f);

/**
 * Threshold for selecting posctl mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 */
PARAM_DEFINE_FLOAT(RC_POSCTL_TH, 0.5f);

/**
 * Threshold for selecting return to launch mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_RETURN_TH, 0.5f);

/**
 * Threshold for selecting loiter mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_LOITER_TH, 0.5f);

/**
 * Threshold for selecting acro mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_ACRO_TH, 0.5f);


/**
 * Threshold for selecting offboard mode
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_OFFB_TH, 0.5f);


/**
 * Threshold for the kill switch
 *
 * 0-1 indicate where in the full channel range the threshold sits
 * 		0 : min
 * 		1 : max
 * sign indicates polarity of comparison
 * 		positive : true when channel>th
 * 		negative : true when channel<th
 *
 * @min -1
 * @max 1
 * @group Radio Switches
 *
 *
 */
PARAM_DEFINE_FLOAT(RC_KILLSWITCH_TH, 0.25f);

/**
 * PWM input channel that provides RSSI.
 *
 * 0: do not read RSSI from input channel
 * 1-18: read RSSI from specified input channel
 *
 * Specify the range for RSSI input with RC_RSSI_PWM_MIN and RC_RSSI_PWM_MAX parameters.
 *
 * @min 0
 * @max 18
 * @value 0 Unassigned
 * @value 1 Channel 1
 * @value 2 Channel 2
 * @value 3 Channel 3
 * @value 4 Channel 4
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @value 17 Channel 17
 * @value 18 Channel 18
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_CHAN, 0);

/**
 * Max input value for RSSI reading.
 *
 * Only used if RC_RSSI_PWM_CHAN > 0
 *
 * @min 0
 * @max 2000
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_MAX, 1000);

/**
 * Min input value for RSSI reading.
 *
 * Only used if RC_RSSI_PWM_CHAN > 0
 *
 * @min 0
 * @max 2000
 * @group Radio Calibration
 *
 */
PARAM_DEFINE_INT32(RC_RSSI_PWM_MIN, 2000);

/**
 * Enable Lidar-Lite (LL40LS) pwm driver
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensor Enable
 */
PARAM_DEFINE_INT32(SENS_EN_LL40LS, 0);

/**
 * Enable sf0x driver
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensor Enable
 */
PARAM_DEFINE_INT32(SENS_EN_SF0X, 0);

/**
 * Set the minimum PWM for the MAIN outputs
 *
 * IMPORTANT: CHANGING THIS PARAMETER REQUIRES A COMPLETE SYSTEM
 * REBOOT IN ORDER TO APPLY THE CHANGES. COMPLETELY POWER-CYCLE
 * THE SYSTEM TO PUT CHANGES INTO EFFECT.
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
 * Set the maximum PWM for the MAIN outputs
 *
 * IMPORTANT: CHANGING THIS PARAMETER REQUIRES A COMPLETE SYSTEM
 * REBOOT IN ORDER TO APPLY THE CHANGES. COMPLETELY POWER-CYCLE
 * THE SYSTEM TO PUT CHANGES INTO EFFECT.
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
 * Set the disarmed PWM for MAIN outputs
 *
 * IMPORTANT: CHANGING THIS PARAMETER REQUIRES A COMPLETE SYSTEM
 * REBOOT IN ORDER TO APPLY THE CHANGES. COMPLETELY POWER-CYCLE
 * THE SYSTEM TO PUT CHANGES INTO EFFECT.
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
PARAM_DEFINE_INT32(PWM_DISARMED, 0);

/**
 * Set the minimum PWM for the MAIN outputs
 *
 * IMPORTANT: CHANGING THIS PARAMETER REQUIRES A COMPLETE SYSTEM
 * REBOOT IN ORDER TO APPLY THE CHANGES. COMPLETELY POWER-CYCLE
 * THE SYSTEM TO PUT CHANGES INTO EFFECT.
 *
 * Set to 1000 for default or 900 to increase servo travel
 *
 * @reboot_required true
 *
 * @min 800
 * @max 1400
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_AUX_MIN, 1000);

/**
 * Set the maximum PWM for the MAIN outputs
 *
 * IMPORTANT: CHANGING THIS PARAMETER REQUIRES A COMPLETE SYSTEM
 * REBOOT IN ORDER TO APPLY THE CHANGES. COMPLETELY POWER-CYCLE
 * THE SYSTEM TO PUT CHANGES INTO EFFECT.
 *
 * Set to 2000 for default or 2100 to increase servo travel
 *
 * @reboot_required true
 *
 * @min 1600
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_AUX_MAX, 2000);

/**
 * Set the disarmed PWM for AUX outputs
 *
 * IMPORTANT: CHANGING THIS PARAMETER REQUIRES A COMPLETE SYSTEM
 * REBOOT IN ORDER TO APPLY THE CHANGES. COMPLETELY POWER-CYCLE
 * THE SYSTEM TO PUT CHANGES INTO EFFECT.
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
PARAM_DEFINE_INT32(PWM_AUX_DISARMED, 1000);
