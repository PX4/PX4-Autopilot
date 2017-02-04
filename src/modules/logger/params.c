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
 * UTC offset (unit: min)
 *
 * the difference in hours and minutes from Coordinated
 * Universal Time (UTC) for a your place and date.
 *
 * for example, In case of South Korea(UTC+09:00),
 * UTC offset is 540 min (9*60)
 *
 * refer to https://en.wikipedia.org/wiki/List_of_UTC_time_offsets
 *
 * @unit min
 * @min -1000
 * @max  1000
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_UTC_OFFSET, 0);

/**
 * Logging Mode
 *
 * Determines when to start and stop logging. By default, logging is started
 * when arming the system, and stopped when disarming.
 *
 * This parameter is only for the new logger (SYS_LOGGER=1).
 *
 * @value 0 when armed until disarm (default)
 * @value 1 from boot until disarm
 * @value 2 from boot until shutdown
 * @value 3 from boot until shutdown - IMU and Baro data only (used for thermal calibration)
 *
 * @min 0
 * @max 3
 * @reboot_required true
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_MODE, 0);

/**
 * Log UUID
 *
 * If set to 1, add an ID to the log, which uniquely identifies the vehicle
 *
 * @boolean
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_UUID, 1);
