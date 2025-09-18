/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Enable arm/disarm stick gesture
 *
 * This determines if moving the left stick to the lower right
 * arms and to the lower left disarms the vehicle.
 *
 * @boolean
 * @group Manual Control
 */
PARAM_DEFINE_INT32(MAN_ARM_GESTURE, 1);

/**
 * Trigger time for kill stick gesture
 *
 * The timeout for holding the left stick to the lower left
 * and the right stick to the lower right at the same time until the gesture
 * kills the actuators one-way.
 *
 * A negative value disables the feature.
 *
 * @group Manual Control
 * @unit s
 * @decimal 2
 * @min -1
 * @max 15
 */
PARAM_DEFINE_FLOAT(MAN_KILL_GEST_T, -1.f);

/**
 * Set primary RC input source. This works only if COM_RC_IN_MODE = 2
 *
 * This determines which RC input has the priority
 *
 * @value -1 Source selection disabled (use last valid source)
 * @value 1 RC
 * @value 2 MAVLink 0
 * @value 3 MAVLink 1
 * @value 4 MAVLink 2
 * @value 5 MAVLink 3
 * @value 6 MAVLink 4
 * @value 7 MAVLink 5
 *
 * @group Manual Control
 */
PARAM_DEFINE_INT32(COM_RC_IN_SOURCE, -1);
