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
 * @reboot_required false
 * @min -1
 * @max 16777215
 * @group Transponder
 *
 */
PARAM_DEFINE_INT32(ADSB_ICAO_ID, 0);

/**
 * ADSB Vehicle Size Configuration
 *
 * This vehicle is always tracked. Use 0 to disable.
 *
 * @reboot_required false
 * @min 0
 * @max 15
 * @group Transponder
 *
 * @value 0 SizeUnknown
 * @value 1 sizeL15W23
 * @value 2 sizeL25W28
 * @value 3 sizeL25W34
 * @value 4 sizeL35W33
 * @value 5 sizeL35W38
 * @value 6 sizeL45W39
 * @value 7 sizeL45W45
 * @value 8 sizeL55W45
 * @value 9 sizeL55W52
 * @value 10 sizeL65W59
 * @value 11 sizeL65W67
 * @value 12 sizeL75W72
 * @value 13 sizeL75W80
 * @value 14 sizeL85W80
 * @value 15 sizeL85W90
 */
PARAM_DEFINE_INT32(ADSB_LEN_WIDTH, 1);

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
 * This parameters defines what port to send the target data to
 * Adjustment to this value will send a new target request
 * Note: Auto is the port where the target request is received.
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
PARAM_DEFINE_INT32(MXS_TARG_OUT, 1);

/**
 * Sagetech MXS Baud configuration
 *
 * This parameter defines the baud rate for the MXS
 *
 * @reboot_required false
 * @min 0
 * @max 230400
 * @group Transponder
 */
PARAM_DEFINE_INT32(MXS_BAUD, 0);


/**
 * Sagetech External Configuration Mode
 *
 * This parameter defines the baud rate for the MXS
 *
 * @reboot_required false
 * @boolean
 * @group Transponder
 */
PARAM_DEFINE_INT32(MXS_EXT_CONFIG, 0);