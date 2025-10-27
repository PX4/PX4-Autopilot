/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * MUORB keep alive failure blind land enable
 *
 * If this parameter is set and the keep alive between the apps processor
 * and the DSP fails (times out) then the drone will initiate a blind descent /
 * landing regardless of any other conditions (e.g. even if GPS is good)
 *
 * @reboot_required true
 *
 * @group MUORB
 * @value 0 - Disabled
 * @value 1 - Blind land on keep alive failure
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(MUORB_KAF_LAND, 0);

/**
 * MUORB keep alive failure force disarm seconds
 *
 * If MUORB_KAF_LAND is set and this parameter is greater than 0 then it will
 * be used to determine how many seconds after a forced blind land starts that a
 * forced disarm will be issued.
 *
 * @reboot_required true
 *
 * @group MUORB
 * @value 0 - Disabled
 * @min 0
 * @max 100
 */
PARAM_DEFINE_INT32(MUORB_KAF_DSRM, 0);
