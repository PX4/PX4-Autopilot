/****************************************************************************
 *
 *   Copyright (C) 2014-2020 PX4 Development Team. All rights reserved.
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
 * UAVCAN CAN bus bitrate.
 *
 * @min 20000
 * @max 1000000
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(CANNODE_BITRATE, 1000000);

/**
 * CAN built-in bus termination
 *
 * @boolean
 * @max 1
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(CANNODE_TERM, 0);

/**
 * Enable MovingBaselineData subscription
 *
 * @boolean
 * @max 1
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(CANNODE_SUB_MBD, 0);

/**
 * Enable RTCM subscription
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(CANNODE_SUB_RTCM, 0);

/**
 * Enable MovingBaselineData publication
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(CANNODE_PUB_MBD, 0);

/**
 * Enable RawIMU pub
 *
 * @boolean
 * @max 1
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(CANNODE_PUB_IMU, 0);
