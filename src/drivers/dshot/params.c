/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * Configure DShot
 *
 * This enabled/disabled DShot. The different modes define different
 * speeds, for example DShot150 = 150kb/s. Not all ESCs support all modes.
 *
 * Note: this enabled DShot on the FMU outputs. For boards with an IO it is the
 * AUX outputs.
 *
 * @value 0 Disable (use PWM/Oneshot)
 * @value 150 DShot150
 * @value 300 DShot300
 * @value 600 DShot600
 * @value 1200 DShot1200
 * @reboot_required true
 * @group DShot
 */
PARAM_DEFINE_INT32(DSHOT_CONFIG, 0);

/**
 * Minimum DShot Motor Output
 *
 * Minimum Output Value for DShot in percent. The value depends on the ESC. Make
 * sure to set this high enough so that the motors are always spinning while
 * armed.
 *
 * @unit norm
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group DShot
 */
PARAM_DEFINE_FLOAT(DSHOT_MIN, 0.055f);


