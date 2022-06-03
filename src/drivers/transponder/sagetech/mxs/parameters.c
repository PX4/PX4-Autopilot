/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/*
 * parameters.c
 *
 * Sagetech MXS transponder custom parameters
 * @author Megan McCormick megan.mccormick@sagetech.com
 * @author Check Faber chuck.faber@sagetech.com
 */

/**
 * Sagetech MXS mode configuration
 *
 * This parameter defines the operating mode of the MXS
 *
 * @reboot_required false
 * @min 0
 * @max 3
 * @group Sagetech MXS
 *
 * @value 0 Off
 * @value 1 On
 * @value 2 Standby
 * @value 3 Alt
 */
PARAM_DEFINE_INT32(MXS_MODE, 0);

/**
 * Sagetech MXS squawk code configuration
 *
 * This parameter defines the squawk code. Value should be between 0000 and 7777.
 *
 * @reboot_required false
 * @min 0
 * @max 7777
 * @group Sagetech MXS
 *
 */
PARAM_DEFINE_INT32(MXS_SQUAWK, 1200);

/**
 * Sagetech MXS Ident Configuration
 *
 * Enable Identification of Position feature
 *
 * @boolean
 * @reboot_required false
 * @group Sagetech MXS
 */
PARAM_DEFINE_INT32(MXS_IDENT, 0);




