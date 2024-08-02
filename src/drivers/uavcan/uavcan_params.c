/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/**
 * UAVCAN mode
 *
 *  0 - UAVCAN disabled.
 *  1 - Enables support for UAVCAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for UAVCAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enables support for UAVCAN sensors and actuators with dynamic node ID allocation and firmware update. Also sets the motor control outputs to UAVCAN.
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Sensors and Actuators (ESCs) Automatic Config
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ENABLE, 0);

/**
 * UAVCAN Node ID.
 *
 * Read the specs at http://uavcan.org to learn more about Node ID.
 *
 * @min 1
 * @max 125
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_NODE_ID, 1);

/**
 * UAVCAN CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_BITRATE, 1000000);

/**
 * UAVCAN ESC will spin at idle throttle when armed, even if the mixer outputs zero setpoints.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ESC_IDLT, 1);

/**
 * UAVCAN rangefinder minimum range
 *
 * This parameter defines the minimum valid range for a rangefinder connected via UAVCAN.
 *
 * @unit m
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_RNG_MIN, 0.3f);

/**
 * UAVCAN rangefinder maximum range
 *
 * This parameter defines the maximum valid range for a rangefinder connected via UAVCAN.
 *
 * @unit m
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_RNG_MAX, 200.0f);

/**
 * UAVCAN ANTI_COLLISION light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the ANTI_COLLISION lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_ANTCL, 2);

/**
 * UAVCAN STROBE light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the STROBE lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_STROB, 1);

/**
 * UAVCAN RIGHT_OF_WAY light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the RIGHT_OF_WAY lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_NAV, 3);

/**
 * UAVCAN LIGHT_ID_LANDING light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the LIGHT_ID_LANDING lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_LAND, 0);

/**
 * publish RTCM stream
 *
 * Enable UAVCAN RTCM stream publication
 *  uavcan::equipment::gnss::RTCMStream
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_RTCM, 0);

/**
 * publish moving baseline data RTCM stream
 *
 * Enable UAVCAN RTCM stream publication
 *  ardupilot::gnss::MovingBaselineData
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_PUB_MBD, 0);

/**
 * subscription airspeed
 *
 * Enable UAVCAN airspeed subscriptions.
 *  uavcan::equipment::air_data::IndicatedAirspeed
 *  uavcan::equipment::air_data::TrueAirspeed
 *  uavcan::equipment::air_data::StaticTemperature
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_ASPD, 0);

/**
 * subscription barometer
 *
 * Enable UAVCAN barometer subscription.
 *  uavcan::equipment::air_data::StaticPressure
 *  uavcan::equipment::air_data::StaticTemperature
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_BARO, 0);

/**
 * subscription battery
 *
 * Enable UAVCAN battery subscription.
 *  uavcan::equipment::power::BatteryInfo
 *  ardupilot::equipment::power::BatteryInfoAux
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_BAT, 0);

/**
 * subscription differential pressure
 *
 * Enable UAVCAN differential pressure subscription.
 *  uavcan::equipment::air_data::RawAirData
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_DPRES, 0);

/**
 * subscription flow
 *
 * Enable UAVCAN optical flow subscription.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_FLOW, 0);

/**
 * subscription GPS
 *
 * Enable UAVCAN GPS subscriptions.
 *  uavcan::equipment::gnss::Fix
 *  uavcan::equipment::gnss::Fix2
 *  uavcan::equipment::gnss::Auxiliary
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_GPS, 1);

/**
 * subscription hygrometer
 *
 * Enable UAVCAN hygrometer subscriptions.
 *  dronecan::sensors::hygrometer::Hygrometer
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_HYGRO, 0);

/**
 * subscription ICE
 *
 * Enable UAVCAN internal combusion engine (ICE) subscription.
 *  uavcan::equipment::ice::reciprocating::Status
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_ICE, 0);

/**
 * subscription IMU
 *
 * Enable UAVCAN IMU subscription.
 *  uavcan::equipment::ahrs::RawIMU
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_IMU, 0);

/**
 * subscription magnetometer
 *
 * Enable UAVCAN mag subscription.
 *  uavcan::equipment::ahrs::MagneticFieldStrength
 *  uavcan::equipment::ahrs::MagneticFieldStrength2
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_MAG, 1);

/**
 * subscription range finder
 *
 * Enable UAVCAN range finder subscription.
 *  uavcan::equipment::range_sensor::Measurement
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_RNG, 0);

/**
 * subscription button
 *
 * Enable UAVCAN button subscription.
 *  ardupilot::indication::Button
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_BTN, 0);

/**
 * forwarding of moving baseline data
 *
 * Subscribe to moving baseline messages on UAVCAN, and broadcast
 * any incoming messages to all UAVCAN interfaces.
 * This is useful for operating a GPS-for-yaw system with GPSes on
 * different CAN buses.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_FWD_MBD, 0);

/**
 * logging verbosity
 *
 * Log messages received on uavcan if they have given severity level or higher.
 * Note: Messages with lower severity than general logging verbosity will never be logged.
 *
 * @min -1
 * @max 3
 * @value -1 None
 * @value 0 Error
 * @value 1 Warning
 * @value 2 Info
 * @value 3 Debug
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LOG_LEVEL, 1);

/**
 * UAVCAN ID to use as GPS1
 *
 * If > 0, UAVCAN GNSS bridge will allocate this UAVCAN ID as the first GPS instance.
 *
 * @min 0
 * @max 127
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_GPS1_ID, 0);
