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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_indi_pos_control_params.c
 *
 * Parameters defined by the fixed-wing rate control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * total takeoff mass
 *
 * This is the mass of the aircraft, used for the INDI
 *
 * @unit kg
 * @min 1.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FW_INDI_MASS, 1.4f);

// ========================================================
// =================== CONTROL GAINS ======================
// ========================================================
/**
 * control gain of position PD-controller (body x-direction)
 *
 * @unit
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FW_INDI_KP_X, 1.0f);

/**
* control gain of position PD-controller (body x-direction)
*
* @unit
* @min 0
* @max 100
* @decimal 1
* @increment 0.1
* @group FW DYN SOAR Control
*/
PARAM_DEFINE_FLOAT(FW_INDI_KP_Y, 1.0f);

/**
* control gain of position PD-controller (body x-direction)
*
* @unit
* @min 0
* @max 100
* @decimal 1
* @increment 0.1
* @group FW DYN SOAR Control
*/
PARAM_DEFINE_FLOAT(FW_INDI_KP_Z, 1.0f);

/**
 * control gain of position PD-controller (body x-direction)
 *
 * @unit
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group FW DYN SOAR Control
 */
PARAM_DEFINE_FLOAT(FW_INDI_KV_X, 0.1f);

/**
* control gain of position PD-controller (body x-direction)
*
* @unit
* @min 0
* @max 100
* @decimal 1
* @increment 0.1
* @group FW DYN SOAR Control
*/
PARAM_DEFINE_FLOAT(FW_INDI_KV_Y, 0.1f);

/**
* control gain of position PD-controller (body x-direction)
*
* @unit
* @min 0
* @max 100
* @decimal 1
* @increment 0.1
* @group FW DYN SOAR Control
*/
PARAM_DEFINE_FLOAT(FW_INDI_KV_Z, 0.1f);

/**
* control gain of attitude PD-controller (body roll-direction)
*
* @unit
* @min 0
* @max 100
* @decimal 1
* @increment 0.1
* @group FW DYN SOAR Control
*/
PARAM_DEFINE_FLOAT(FW_INDI_K_ROLL, 7.0f);

/**
* control gain of attitude PD-controller (body roll-direction)
*
* @unit
* @min 0
* @max 100
* @decimal 1
* @increment 0.1
* @group FW DYN SOAR Control
*/
PARAM_DEFINE_FLOAT(FW_INDI_K_PITCH, 2.0f);
