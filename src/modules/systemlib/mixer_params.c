/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mixer_params.c
 *
 * Parameters defined for mixer parameter access
 *
 * @author Matthew Coleman <uavflightdirector@gmail.com>
 * @author Julian Oes <julian@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Mixer Group - Device
 *
 * Mixer group for parameter access
 *
 * @group Mixer Parameters
 * @min -1
 * @max 1
 * @value -1 default inactive
 * @value 0 FMU
 * @value 1 PX4io
 * @value 2 UAVCAN (Future)
 */
PARAM_DEFINE_INT32(MIX_GROUP, -1);

/**
 * Mixer Index
 *
 * Index address of the mixer for parameter access
 *
 * @min -1
 * @value -1 default inactive
 *
 * @group Mixer Parameters
 */
PARAM_DEFINE_INT32(MIX_INDEX, -1);

/**
 * Index address of the parameter in the mixer
 *
 * @min -1
 * @value -1 default inactive
 *
 * @group Mixer Parameters
 */
PARAM_DEFINE_INT32(MIX_PARAM_INDEX, -1);

/**
 * Mixer parameter value
 *
 * @min -10.0
 * @max 10.0
 * @increment 0.1
 * @group Mixer Parameters
 */
PARAM_DEFINE_FLOAT(MIX_PARAMETER, 0.0f);

/**
 * Mixer parameter action
 *
 * @min -1
 * @max 1
 * @value -1 actions complete
 * @value 0 write
 * @value 1 read
 * @value 2 store (Future)
 * @group Mixer Parameters
 */
PARAM_DEFINE_INT32(MIX_PARAM_ACTION, -1);
