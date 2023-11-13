/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * Encoder counts per revolution
 *
 * Number of encoder counts for one revolution. The roboclaw treats analog encoders (potentiometers) as having 2047
 * counts per rev. The default value of 1200 corresponds to the default configuration of the Aion R1 rover.
 * @min 1
 * @group Roboclaw driver
 */
PARAM_DEFINE_INT32(RBCLW_COUNTS_REV, 1200);

/**
 * Address of the Roboclaw
 *
 * The Roboclaw can be configured to have an address from 0x80 to 0x87, inclusive. It must be configured to match
 * this parameter.
 * @min 128
 * @max 135
 * @value 128 0x80
 * @value 129 0x81
 * @value 130 0x82
 * @value 131 0x83
 * @value 132 0x84
 * @value 133 0x85
 * @value 134 0x86
 * @value 135 0x87
 * @group Roboclaw driver
 */
PARAM_DEFINE_INT32(RBCLW_ADDRESS, 128);
