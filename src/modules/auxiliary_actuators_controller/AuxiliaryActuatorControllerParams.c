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
 * Flaps setting during take-off
 *
 * Sets a fraction of full flaps during take-off.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_TO_SCL, 0.f);

/**
 * Flaps setting during landing
 *
 * Sets a fraction of full flaps during landing.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_LND_SCL, 1.f);

/**
 * Spoiler landing setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_LND, 0.f);

/**
 * Spoiler descend setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_DESC, 0.f);

/**
 * Spoiler input in manual flight
 *
 * Chose source for manual setting of spoilers in manual flight modes.
 *
 * @value 0 Disabled
 * @value 1 Flaps channel
 * @value 2 Aux1
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_SPOILERS_MAN, 0);

/**
 * Spoiler setting while landing (hover)
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 1
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_SPOILER_MC_LD, 0.f);
