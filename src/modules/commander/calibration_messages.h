/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file calibration_messages.h
 *
 * Common calibration messages.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef CALIBRATION_MESSAGES_H_
#define CALIBRATION_MESSAGES_H_

#define CAL_STARTED_MSG	"%s calibration: started"
#define CAL_DONE_MSG	"%s calibration: done"
#define CAL_FAILED_MSG	"%s calibration: failed"
#define CAL_PROGRESS_MSG	"%s calibration: progress <%u>"

#define CAL_FAILED_SENSOR_MSG	"ERROR: failed reading sensor"
#define CAL_FAILED_RESET_CAL_MSG	"ERROR: failed to reset calibration"
#define CAL_FAILED_APPLY_CAL_MSG	"ERROR: failed to apply calibration"
#define CAL_FAILED_SET_PARAMS_MSG	"ERROR: failed to set parameters"
#define CAL_FAILED_SAVE_PARAMS_MSG	"ERROR: failed to save parameters"

#endif /* CALIBRATION_MESSAGES_H_ */
