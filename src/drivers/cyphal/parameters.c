/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * Cyphal
 *
 *  0 - Cyphal disabled.
 *  1 - Enables Cyphal
 *
 * @boolean
 * @reboot_required true
 * @group Cyphal
 */
PARAM_DEFINE_INT32(CYPHAL_ENABLE, 1);

/**
 * Cyphal Node ID.
 *
 * Read the specs at https://opencyphal.org/ to learn more about Node ID.
 *
 * @min -1
 * @max 125
 * @reboot_required true
 * @group Cyphal
 */
PARAM_DEFINE_INT32(CYPHAL_ID, 1);

/**
 * UAVCAN/CAN v1 bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group Cyphal
 */
PARAM_DEFINE_INT32(CYPHAL_BAUD, 1000000);

/* Subscription port ID, -1 will be treated as unset */

/**
 * ESC 0 subscription port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_ESC0_SUB, -1);

/**
 * GPS 0 subscription port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_GPS0_SUB, -1);

/**
 * GPS 1 subscription port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_GPS1_SUB, -1);

/**
 * UDRAL battery energy source subscription port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_BMS_ES_SUB, -1);

/**
 * UDRAL battery status subscription port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_BMS_BS_SUB, -1);

/**
 * UDRAL battery parameters subscription  port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_BMS_BP_SUB, -1);

/**
 * Cyphal legacy battery port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_LG_BMS_SUB, -1);


/**
 * sensor_gps uORB over Cyphal subscription port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_UORB_GPS, -1);


/**
 * sensor_gps uORB over Cyphal publication port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_UORB_GPS_P, -1);

// Publication Port IDs

/**
 * Cyphal ESC publication port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_ESC_PUB, -1);

/**
 * Cyphal ESC readiness port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_READ_PUB, -1);

/**
 * Cyphal ESC 0 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB0_SUB, -1);

/**
 * Cyphal ESC 1 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB1_SUB, -1);

/**
 * Cyphal ESC 2 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB2_SUB, -1);

/**
 * Cyphal ESC 3 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB3_SUB, -1);

/**
 * Cyphal ESC 4 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB4_SUB, -1);

/**
 * Cyphal ESC 5 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB5_SUB, -1);

/**
 * Cyphal ESC 6 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB6_SUB, -1);

/**
 * Cyphal ESC 7 zubax feedback port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_FB7_SUB, -1);

/**
 * Cyphal GPS publication port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_GPS_PUB, -1);

/**
 * Cyphal Servo publication port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_SERVO_PUB, -1);

/**
 * actuator_outputs uORB over Cyphal publication port ID.
 *
 * @min -1
 * @max 6143
 * @group Cyphal
 */
PARAM_DEFINE_INT32(UCAN1_ACTR_PUB, -1);
