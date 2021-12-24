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
 * Enable INA228 Power Monitor
 *
 * For systems a INA228 Power Monitor, this should be set to true
 *
 * @group Sensors
 * @boolean
 * @reboot_required true
*/
PARAM_DEFINE_BOOL(SENS_EN_INA228, 0);

/**
 * INA228 Power Monitor Config
 *
 * @group Sensors
 * @min 0
 * @max 65535
 * @decimal 1
 * @increment 1
*/
PARAM_DEFINE_INT32(INA228_CONFIG, 63779);

/**
 * INA228 Power Monitor Max Current
 *
 * @group Sensors
 * @min 0.1
 * @max 327.68
 * @decimal 2
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(INA228_CURRENT, 327.68f);

/**
 * INA228 Power Monitor Shunt
 *
 * @group Sensors
 * @min 0.000000001
 * @max 0.1
 * @decimal 10
 * @increment .000000001
 */
PARAM_DEFINE_FLOAT(INA228_SHUNT, 0.0005f);
