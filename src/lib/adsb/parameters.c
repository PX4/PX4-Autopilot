/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * ADSB-Out squawk code configuration
 *
 * This parameter defines the squawk code. Value should be between 0000 and 7777.
 *
 * @reboot_required false
 * @min 0
 * @max 7777
 * @group ADSB
 *
 */
PARAM_DEFINE_INT32(ADSB_SQUAWK, 1200);

/**
 * ADSB-Out Ident Configuration
 *
 * Enable Identification of Position feature
 *
 * @boolean
 * @reboot_required false
 * @group ADSB
 */
PARAM_DEFINE_INT32(ADSB_IDENT, 0);

/**
 * ADSB-In Vehicle List Size
 *
 * Change number of targets to track
 *
 * @min 0
 * @max 50
 * @reboot_required true
 * @group ADSB
 */
PARAM_DEFINE_INT32(ADSB_LIST_MAX, 25);

/**
 * ADSB-Out ICAO configuration
 *
 * Defines the ICAO ID of the vehicle
 *
 * @reboot_required true
 * @min -1
 * @max 16777215
 * @group ADSB
 *
 */
PARAM_DEFINE_INT32(ADSB_ICAO_ID, 1194684);

/**
 * ADSB-Out Vehicle Size Configuration
 *
 * Report the length and width of the vehicle in meters. In most cases, use '1' for the smallest vehicle size.
 *
 * @reboot_required true
 * @min 0
 * @max 15
 * @group ADSB
 *
 * @value 0 SizeUnknown
 * @value 1 Len15_Wid23
 * @value 2 Len25_Wid28
 * @value 3 Len25_Wid34
 * @value 4 Len35_Wid33
 * @value 5 Len35_Wid38
 * @value 6 Len45_Wid39
 * @value 7 Len45_Wid45
 * @value 8 Len55_Wid45
 * @value 9 Len55_Wid52
 * @value 10 Len65_Wid59
 * @value 11 Len65_Wid67
 * @value 12 Len75_Wid72
 * @value 13 Len75_Wid80
 * @value 14 Len85_Wid80
 * @value 15 Len85_Wid90
 */
PARAM_DEFINE_INT32(ADSB_LEN_WIDTH, 1);

/**
 * ADSB-Out Vehicle Emitter Type
 *
 * Configure the emitter type of the vehicle.
 *
 * @reboot_required true
 * @min 0
 * @max 15
 * @group ADSB
 *
 * @value 0 Unknown
 * @value 1 Light
 * @value 2 Small
 * @value 3 Large
 * @value 4 HighVortex
 * @value 5 Heavy
 * @value 6 Performance
 * @value 7 Rotorcraft
 * @value 8 RESERVED
 * @value 9 Glider
 * @value 10 LightAir
 * @value 11 Parachute
 * @value 12 UltraLight
 * @value 13 RESERVED
 * @value 14 UAV
 * @value 15 Space
 * @value 16 RESERVED
 * @value 17 EmergencySurf
 * @value 18 ServiceSurf
 * @value 19 PointObstacle
 */
PARAM_DEFINE_INT32(ADSB_EMIT_TYPE, 14);

/**
 * ADSB-Out Vehicle Max Speed
 *
 * Informs ADSB vehicles of this vehicle's max speed capability
 *
 * @reboot_required true
 * @min 0
 * @max 6
 * @value 0 UnknownMaxSpeed
 * @value 1 75Kts
 * @value 2 150Kts
 * @value 3 300Kts
 * @value 4 600Kts
 * @value 5 1200Kts
 * @value 6 Over1200Kts
 * @group ADSB
 */
PARAM_DEFINE_INT32(ADSB_MAX_SPEED, 0);

/**
 * ADSB-In Special ICAO configuration
 *
 * This vehicle is always tracked. Use 0 to disable.
 *
 * @reboot_required false
 * @min 0
 * @max 16777215
 * @group ADSB
 *
 */
PARAM_DEFINE_INT32(ADSB_ICAO_SPECL, 0);

/**
 * ADSB-Out Emergency State
 *
 * Sets the vehicle emergency state
 *
 * @reboot_required false
 * @min 0
 * @max 6
 * @value 0 NoEmergency
 * @value 1 General
 * @value 2 Medical
 * @value 3 LowFuel
 * @value 4 NoCommunications
 * @value 5 Interference
 * @value 6 Downed
 * @group ADSB
 */
PARAM_DEFINE_INT32(ADSB_EMERGC, 0);

/**
 * ADSB-Out GPS Offset lat
 *
 * Sets GPS lataral offset encoding
 *
 * @reboot_required false
 * @min 0
 * @max 7
 * @value 0 NoData
 * @value 1 LatLeft2M
 * @value 2 LatLeft4M
 * @value 3 LatLeft6M
 * @value 4 LatRight0M
 * @value 5 LatRight2M
 * @value 6 LatRight4M
 * @value 7 LatRight6M
 * @group ADSB
 */
PARAM_DEFINE_INT32(ADSB_GPS_OFF_LAT, 0);

/**
 * ADSB-Out GPS Offset lon
 *
 * Sets GPS longitudinal offset encoding
 *
 * @reboot_required false
 * @min 0
 * @max 1
 * @value 0 NoData
 * @value 1 AppliedBySensor
 * @group ADSB
 */
PARAM_DEFINE_INT32(ADSB_GPS_OFF_LON, 0);

/**
 * First 4 characters of CALLSIGN
 *
 * Sets first 4 characters of a total of 8. Valid characters are A-Z, 0-9, " ". Example "PX4 " -> 1347957792
 * For CALLSIGN shorter than 8 characters use the null terminator at the end '\0'.
 *
 * @reboot_required true
 * @group ADSB
 */

PARAM_DEFINE_INT32(ADSB_CALLSIGN_1, 0);

/**
* Second 4 characters of CALLSIGN
*
* Sets second 4 characters of a total of 8. Valid characters are A-Z, 0-9, " " only. Example "TEST" -> 1413829460
* For CALLSIGN shorter than 8 characters use the null terminator at the end '\0'.
*
* @reboot_required true
* @group ADSB
*/
PARAM_DEFINE_INT32(ADSB_CALLSIGN_2, 0);
