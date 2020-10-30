/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * Dump GPS communication to a file.
 *
 * If this is set to 1, all GPS communication data will be published via uORB,
 * and written to the log file as gps_dump message.
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group GPS
 */
PARAM_DEFINE_INT32(GPS_DUMP_COMM, 0);

/**
 * u-blox GPS dynamic platform model
 *
 * u-blox receivers support different dynamic platform models to adjust the navigation engine to
 * the expected application environment.
 *
 * @min 0
 * @max 9
 * @value 2 stationary
 * @value 4 automotive
 * @value 6 airborne with <1g acceleration
 * @value 7 airborne with <2g acceleration
 * @value 8 airborne with <4g acceleration
 *
 * @reboot_required true
 *
 * @group GPS
 */
PARAM_DEFINE_INT32(GPS_UBX_DYNMODEL, 7);

/**
 * Heading/Yaw offset for dual antenna GPS
 *
 * Heading offset angle for dual antenna GPS setups that support heading estimation.
 * (currently only for the Trimble MB-Two).
 *
 * Set this to 0 if the antennas are parallel to the forward-facing direction of the vehicle and the first antenna is in
 * front. The offset angle increases counterclockwise.
 *
 * Set this to 90 if the first antenna is placed on the right side and the second on the left side of the vehicle.
 *
 * @min 0
 * @max 360
 * @unit deg
 * @reboot_required true
 * @decimal 0
 *
 * @group GPS
 */
PARAM_DEFINE_FLOAT(GPS_YAW_OFFSET, 0.f);


/**
 * NMEA GPS baudrate
 *
 * This parameter is used to set the NMEA GPS driver's baudrate
 *
 * @min 0
 * @max 6460800
 *
 * @reboot_required true
 *
 * @group GPS
 */
PARAM_DEFINE_INT32(GPS_NMEA_BAUD, 38400);

/**
* GPS1 ptotocol type
*
* This parameter is used to set if use manual gps mode or auto gps mode
*
* @min 0
* @max 5
* @value 0 set gps mode to GPS_DRIVER_MODE_AUTO
* @value 1 set gps mode to GPS_DRIVER_MODE_UBX
* @value 2 set gps mode to GPS_DRIVER_MODE_MTK
* @value 3 set gps mode to GPS_DRIVER_MODE_ASHTECH
* @value 4 set gps mode to GPS_DRIVER_MODE_EMLIDREACH
* @value 5 set gps mode to GPS_DRIVER_MODE_NMEA
* @reboot_required true
*
* @group GPS
*/
PARAM_DEFINE_INT32(GPS_1_PROTOCOL, 0);

/**
* GPS2 ptotocol type
*
* This parameter is used to set if use manual gps mode or auto gps mode
*
* @min 0
* @max 5
* @value 0 set gps mode to GPS_DRIVER_MODE_AUTO
* @value 1 set gps mode to GPS_DRIVER_MODE_UBX
* @value 2 set gps mode to GPS_DRIVER_MODE_MTK
* @value 3 set gps mode to GPS_DRIVER_MODE_ASHTECH
* @value 4 set gps mode to GPS_DRIVER_MODE_EMLIDREACH
* @value 5 set gps mode to GPS_DRIVER_MODE_NMEA
* @reboot_required true
*
* @group GPS
*/
PARAM_DEFINE_INT32(GPS_2_PROTOCOL, 0);
