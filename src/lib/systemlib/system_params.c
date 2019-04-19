/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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

/*
 * @file system_params.c
 *
 * System wide parameters
 */

/**
 * Auto-start script index.
 *
 * CHANGING THIS VALUE REQUIRES A RESTART. Defines the auto-start script used to bootstrap the system.
 *
 * @reboot_required true
 * @min 0
 * @max 99999
 * @group System
 */
PARAM_DEFINE_INT32(SYS_AUTOSTART, 0);

/**
 * Automatically configure default values.
 *
 * Set to 1 to reset parameters on next system startup (setting defaults).
 * Platform-specific values are used if available.
 * RC* parameters are preserved.
 *
 * @value 0 Keep parameters
 * @value 1 Reset parameters
 * @value 2 Reload airframe parameters
 * @group System
 */
PARAM_DEFINE_INT32(SYS_AUTOCONFIG, 0);

/**
 * Enable HITL/SIH mode on next boot
 *
 * While enabled the system will boot in Hardware-In-The-Loop (HITL)
 * or Simulation-In-Hardware (SIH) mode and not enable all sensors and checks.
 * When disabled the same vehicle can be flown normally.
 *
 * @value 0 HITL and SIH disabled
 * @value 1 HITL enabled
 * @value 2 SIH enabled
 * @reboot_required true
 *
 * @group System
 */
PARAM_DEFINE_INT32(SYS_HITL, 0);

/**
 * Set restart type
 *
 * Set by px4io to indicate type of restart
 *
 * @min 0
 * @max 2
 * @value 0 Data survives resets
 * @value 1 Data survives in-flight resets only
 * @value 2 Data does not survive reset
 * @category system
 * @volatile
 * @group System
 */
PARAM_DEFINE_INT32(SYS_RESTART_TYPE, 2);

/**
 * Set multicopter estimator group
 *
 * Set the group of estimators used for multicopters and VTOLs
 *
 * @value 1 local_position_estimator, attitude_estimator_q
 * @value 2 ekf2
 *
 * @min 1
 * @max 2
 * @reboot_required true
 * @group System
 */
PARAM_DEFINE_INT32(SYS_MC_EST_GROUP, 2);

/**
 * TELEM2 as companion computer link (deprecated)
 *
 * This parameter is deprecated and will be removed after 1.9.0. Use the generic serial
 * configuration parameters instead (e.g. MAV_0_CONFIG, MAV_0_MODE, etc.).
 *
 * @value 0 Disabled
 * @value 10 FrSky Telemetry
 * @value 20 Crazyflie (Syslink)
 * @value 921600 Companion Link (921600 baud, 8N1)
 * @value 57600 Companion Link (57600 baud, 8N1)
 * @value 1500000 Companion Link (1500000 baud, 8N1)
 * @value 157600 OSD (57600 baud, 8N1)
 * @value 257600 Command Receiver (57600 baud, 8N1)
 * @value 319200 Normal Telemetry (19200 baud, 8N1)
 * @value 338400 Normal Telemetry (38400 baud, 8N1)
 * @value 357600 Normal Telemetry (57600 baud, 8N1)
 * @value 3115200 Normal Telemetry (115200 baud, 8N1)
 * @value 4115200 Iridium Telemetry (115200 baud, 8N1)
 * @value 519200 Minimal Telemetry (19200 baud, 8N1)
 * @value 538400 Minimal Telemetry (38400 baud, 8N1)
 * @value 557600 Minimal Telemetry (57600 baud, 8N1)
 * @value 5115200 Minimal Telemetry (115200 baud, 8N1)
 * @value 6460800 RTPS Client (460800 baud)
 * @value 1921600 ESP8266 (921600 baud, 8N1)
 *
 * @min 0
 * @max 6460800
 * @reboot_required true
 * @group System
 */
PARAM_DEFINE_INT32(SYS_COMPANION, 0);

/**
 * Parameter version
 *
 * This is used internally only: an airframe configuration might set an expected
 * parameter version value via PARAM_DEFAULTS_VER. This is checked on bootup
 * against SYS_PARAM_VER, and if they do not match, parameters from the airframe
 * configuration are reloaded.
 *
 * @min 0
 * @group System
 */
PARAM_DEFINE_INT32(SYS_PARAM_VER, 1);

/**
 * Enable auto start of rate gyro thermal calibration at the next power up.
 *
 * 0 : Set to 0 to do nothing
 * 1 : Set to 1 to start a calibration at next boot
 * This parameter is reset to zero when the the temperature calibration starts.
 *
 * default (0, no calibration)
 *
 * @group System
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(SYS_CAL_GYRO, 0);

/**
 * Enable auto start of accelerometer thermal calibration at the next power up.
 *
 * 0 : Set to 0 to do nothing
 * 1 : Set to 1 to start a calibration at next boot
 * This parameter is reset to zero when the the temperature calibration starts.
 *
 * default (0, no calibration)
 *
 * @group System
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(SYS_CAL_ACCEL, 0);

/**
 * Enable auto start of barometer thermal calibration at the next power up.
 *
 * 0 : Set to 0 to do nothing
 * 1 : Set to 1 to start a calibration at next boot
 * This parameter is reset to zero when the the temperature calibration starts.
 *
 * default (0, no calibration)
 *
 * @group System
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(SYS_CAL_BARO, 0);

/**
 * Required temperature rise during thermal calibration
 *
 * A temperature increase greater than this value is required during calibration.
 * Calibration will complete for each sensor when the temperature increase above the starting temeprature exceeds the value set by SYS_CAL_TDEL.
 * If the temperature rise is insufficient, the calibration will continue indefinitely and the board will need to be repowered to exit.
 *
 * @unit deg C
 * @min 10
 * @group System
 */
PARAM_DEFINE_INT32(SYS_CAL_TDEL, 24);

/**
 * Minimum starting temperature for thermal calibration
 *
 * Temperature calibration for each sensor will ignore data if the temperature is lower than the value set by SYS_CAL_TMIN.
 *
 * @unit deg C
 * @group System
 */
PARAM_DEFINE_INT32(SYS_CAL_TMIN, 5);

/**
 * Maximum starting temperature for thermal calibration
 *
 * Temperature calibration will not start if the temperature of any sensor is higher than the value set by SYS_CAL_TMAX.
 *
 * @unit deg C
 * @group System
 */
PARAM_DEFINE_INT32(SYS_CAL_TMAX, 10);

/**
 * Control if the vehicle has a magnetometer
 *
 * Disable this if the board has no magnetometer, such as the Omnibus F4 SD.
 * If disabled, the preflight checks will not check for the presence of a
 * magnetometer.
 *
 * @boolean
 * @reboot_required true
 *
 * @group System
 */
PARAM_DEFINE_INT32(SYS_HAS_MAG, 1);

/**
 * Control if the vehicle has a barometer
 *
 * Disable this if the board has no barometer, such as some of the the Omnibus
 * F4 SD variants.
 * If disabled, the preflight checks will not check for the presence of a
 * barometer.
 *
 * @boolean
 * @reboot_required true
 *
 * @group System
 */
PARAM_DEFINE_INT32(SYS_HAS_BARO, 1);

/**
 * Bootloader update
 *
 * If enabled, update the bootloader on the next boot.
 *
 * WARNING: do not cut the power during an update process, otherwise you will
 * have to recover using some alternative method (e.g. JTAG).
 *
 * Instructions:
 * - Insert an SD card
 * - Enable this parameter
 * - Reboot the board (plug the power or send a reboot command)
 * - Wait until the board comes back up (or at least 2 minutes)
 * - If it does not come back, check the file bootlog.txt on the SD card
 *
 * @boolean
 * @reboot_required true
 *
 * @group System
 */
PARAM_DEFINE_INT32(SYS_BL_UPDATE, 0);
