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

/**
 * @file camera_capture_params.c
 * Camera capture parameters
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 */
/**
 * Camera strobe delay
 *
 * This parameter sets the delay between image integration start and strobe firing
 *
 * @unit ms
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group Camera Capture
 */
PARAM_DEFINE_FLOAT(CAM_CAP_DELAY, 0.0f);

/**
 * Camera capture feedback
 *
 * Enables camera capture feedback
 *
 * @boolean
 * @group Camera Control
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CAM_CAP_FBACK, 0);

/**
 * Camera capture timestamping mode
 *
 * Change time measurement
 *
 * @value 0 Get absolute timestamp
 * @value 1 Get timestamp of mid exposure (active high)
 * @value 2 Get timestamp of mid exposure (active low)
 *
 * @group Camera Control
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CAM_CAP_MODE, 0);

/**
 * Camera capture edge
 *
 * @value 0 Falling edge
 * @value 1 Rising edge
 *
 * @group Camera Control
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CAM_CAP_EDGE, 0);
