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
 * Airspeed sensor compensation model for the SDP3x
 *
 * Model with Pitot
 * 		CAL_AIR_TUBED_MM: Not used, 1.5 mm tubes assumed.
 * 		CAL_AIR_TUBELEN: Length of the tubes connecting the pitot to the sensor.
 * Model without Pitot (1.5 mm tubes)
 * 		CAL_AIR_TUBED_MM: Not used, 1.5 mm tubes assumed.
 * 		CAL_AIR_TUBELEN: Length of the tubes connecting the pitot to the sensor.
 * Tube Pressure Drop
 * 		CAL_AIR_TUBED_MM: Diameter in mm of the pitot and tubes, must have the same diameter.
 * 		CAL_AIR_TUBELEN: Length of the tubes connecting the pitot to the sensor and the static + dynamic port length of the pitot.
 *
 * @value 0 Model with Pitot
 * @value 1 Model without Pitot (1.5 mm tubes)
 * @value 2 Tube Pressure Drop
 *
 * @group Sensors
 */
PARAM_DEFINE_INT32(CAL_AIR_CMODEL, 0);

/**
 * Airspeed sensor tube length.
 *
 * See the CAL_AIR_CMODEL explanation on how this parameter should be set.
 *
 * @min 0.01
 * @max 2.00
 * @unit m
 *
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(CAL_AIR_TUBELEN, 0.2f);

/**
 * Airspeed sensor tube diameter. Only used for the Tube Pressure Drop Compensation.
 *
 * @min 0.1
 * @max 100
 * @unit mm
 *
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(CAL_AIR_TUBED_MM, 1.5f);

/**
 * Differential pressure sensor offset
 *
 * The offset (zero-reading) in Pascal
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_DPRES_OFF, 0.0f);

/**
 * Sensors hub differential pressure mode
 *
 * @value 0 Publish all airspeeds
 * @value 1 Publish primary airspeed
 *
 * @category system
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_DPRES_MODE, 1);

/**
 * Differential pressure (airspeed) max rate.
 *
 * Airspeed data maximum publication rate. This is an upper bound,
 * actual differential pressure data rate is still dependant on the sensor.
 *
 * @min 1
 * @max 100
 * @group Sensors
 * @unit Hz
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_FLOAT(SENS_DPRES_RATE, 10.0f);
