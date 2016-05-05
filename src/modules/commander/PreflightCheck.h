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
 * @file PreflightCheck.h
 *
 * Preflight check for main system components
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#pragma once

namespace Commander
{
/**
* Runs a preflight check on all sensors to see if they are properly calibrated and healthy
*
* The function won't fail the test if optional sensors are not found, however,
* it will fail the test if optional sensors are found but not in working condition.
*
* @param mavlink_log_pub
*   Mavlink output orb handle reference for feedback when a sensor fails
* @param checkMag
*   true if the magneteometer should be checked
* @param checkAcc
*   true if the accelerometers should be checked
* @param checkGyro
*   true if the gyroscopes should be checked
* @param checkBaro
*   true if the barometer should be checked
* @param checkAirspeed
*   true if the airspeed sensor should be checked
* @param checkRC
*   true if the Remote Controller should be checked
* @param checkGNSS
*   true if the GNSS receiver should be checked
**/
bool preflightCheck(orb_advert_t *mavlink_log_pub, bool checkMag, bool checkAcc,
    bool checkGyro, bool checkBaro, bool checkAirspeed, bool checkRC, bool checkGNSS, bool checkDynamic, bool reportFailures = false);

const unsigned max_mandatory_gyro_count = 1;
const unsigned max_optional_gyro_count = 3;

const unsigned max_mandatory_accel_count = 1;
const unsigned max_optional_accel_count = 3;

const unsigned max_mandatory_mag_count = 1;
const unsigned max_optional_mag_count = 3;

const unsigned max_mandatory_baro_count = 1;
const unsigned max_optional_baro_count = 1;

}
