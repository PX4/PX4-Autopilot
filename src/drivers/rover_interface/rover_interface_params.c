/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Rover type.
 *
 * @value -1 Not selected
 * @value 0 Scout Mini
 * @value 1 Scout
 * @value 2 Scout Pro
 * @value 3 Scout 2
 * @value 4 Scout 2 Pro
 * @value 5 Bunker
 * @value 6 Bunker Mini
 * @group Rover Interface
 */
PARAM_DEFINE_INT32(RI_ROVER_TYPE, -1);

/**
 * Rover interface CAN bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group Rover Interface
 */
PARAM_DEFINE_INT32(RI_CAN_BITRATE, 500000);


/**
 * Rover interface manual control throttle max.
 *
 * @unit m/s
 * @min 1.0
 * @max 3.0
 * @reboot_required true
 * @group Rover Interface
 */
PARAM_DEFINE_FLOAT(RI_MAN_THR_MAX, 1.0);


/**
 * Rover interface mission control throttle max.
 *
 * @unit m/s
 * @min 1.0
 * @max 3.0
 * @reboot_required true
 * @group Rover Interface
 */
PARAM_DEFINE_FLOAT(RI_MIS_THR_MAX, 1.0);
