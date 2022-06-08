/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * parameters.c
 *
 * Sagetech MXS transponder custom parameters
 * @author Megan McCormick megan.mccormick@sagetech.com
 * @author Check Faber chuck.faber@sagetech.com
 */

/**
 * ADSB squawk code configuration
 *
 * This parameter defines the squawk code. Value should be between 0000 and 7777.
 *
 * @reboot_required false
 * @min 0
 * @max 7777
 * @group Transponder
 *
 */
PARAM_DEFINE_INT32(ADSB_SQUAWK, 1200);

/**
 * ADSB Ident Configuration
 *
 * Enable Identification of Position feature
 *
 * @boolean
 * @reboot_required false
 * @group Transponder
 */
PARAM_DEFINE_INT32(ADSB_IDENT, 0);

/**
 * ADSB Vehicle List Size
 *
 * Adjust the number of participants in the target request message
 * Adjustment to this value will send a new target request
 *
 * @min 0
 * @max 50
 * @reboot_required false
 * @group Transponder
 */
PARAM_DEFINE_INT32(ADSB_LIST_MAX, 25);

/**
 * ADSB ICAO configuration
 *
 * Defines the ICAO ID of the vehicle
 *
 * @reboot_required true
 * @min -1
 * @max 16777215
 * @group Transponder
 *
 */
PARAM_DEFINE_INT32(ADSB_ICAO_ID, 0);

/**
 * ADSB Vehicle Size Configuration
 *
 * Configure the size of the vehicle.
 *
 * @reboot_required true
 * @min 0
 * @max 15
 * @group Transponder
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
 * ADSB Vehicle Emitter Type
 *
 * Configure the emitter type of the vehicle.
 *
 * @reboot_required true
 * @min 0
 * @max 15
 * @group Transponder
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
 * ADSB Special ICAO configuration
 *
 * This vehicle is always tracked. Use 0 to disable.
 *
 * @reboot_required false
 * @min 0
 * @max 16777215
 * @group Transponder
 *
 */
PARAM_DEFINE_INT32(ADSB_ICAO_SPECL, 0);


/**
 * Sagetech MXS mode configuration
 *
 * This parameter defines the operating mode of the MXS
 *
 * @reboot_required false
 * @min 0
 * @max 3
 * @group Transponder
 *
 * @value 0 Off
 * @value 1 On
 * @value 2 Standby
 * @value 3 Alt
 */
PARAM_DEFINE_INT32(MXS_MODE, 0);

/**
 * Sagetech MXS Participant Configuration
 *
 * The MXS communication port to receive Target data from
 *
 * @min 0
 * @max 2
 * @reboot_required false
 * @group Transponder
 *
 * @value 0 Auto
 * @value 1 COM0
 * @value 2 COM1
 */
PARAM_DEFINE_INT32(MXS_TARG_PORT, 1);

/**
 * Sagetech MXS COM0 Baud configuration
 *
 * This parameter defines the baud rate for the MXS COM 1 Port
 *
 * @reboot_required true
 * @min 0
 * @max 10
 * @value 0 38400
 * @value 1 600
 * @value 2 4800
 * @value 3 9600
 * @value 4 28800
 * @value 5 57600
 * @value 6 115200
 * @value 7 230400
 * @value 8 19200
 * @value 9 460800
 * @value 10 921600
 * @group Transponder
 */
PARAM_DEFINE_INT32(MXS_COM0_BAUD, 7);

/**
 * Sagetech MXS COM1 Baud configuration
 *
 * This parameter defines the baud rate for the MXS COM 0 Port
 *
 * @reboot_required true
 * @min 0
 * @max 10
 * @value 0 38400
 * @value 1 600
 * @value 2 4800
 * @value 3 9600
 * @value 4 28800
 * @value 5 57600
 * @value 6 115200
 * @value 7 230400
 * @value 8 19200
 * @value 9 460800
 * @value 10 921600
 * @group Transponder
 */
PARAM_DEFINE_INT32(MXS_COM1_BAUD, 5);


/**
 * Sagetech External Configuration Mode
 *
 * This parameter defines the baud rate for the MXS
 *
 * @reboot_required false
 * @boolean
 * @group Transponder
 */
PARAM_DEFINE_INT32(MXS_EXT_CFG, 0);
