/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file mavlink_parameters.h
 * MAVLink parameter protocol definitions (BSD-relicensed).
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */


#include <v1.0/mavlink_types.h>
#include <stdbool.h>
#include <systemlib/param/param.h>

/**
 * Handle parameter related messages. 
 */
void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg);

/**
 * Send all parameters at once.
 *
 * This function blocks until all parameters have been sent.
 * it delays each parameter by the passed amount of microseconds.
 *
 * @param delay		The delay in us between sending all parameters.
 */
void mavlink_pm_send_all_params(unsigned int delay);

/**
 * Send one parameter.
 *
 * @param param		The parameter id to send.
 * @return		zero on success, nonzero on failure.
 */
int mavlink_pm_send_param(param_t param);

/**
 * Send one parameter identified by index.
 *
 * @param index		The index of the parameter to send.
 * @return		zero on success, nonzero else.
 */
int mavlink_pm_send_param_for_index(uint16_t index);

/**
 * Send one parameter identified by name.
 *
 * @param name		The index of the parameter to send.
 * @return		zero on success, nonzero else.
 */
int mavlink_pm_send_param_for_name(const char* name);

/**
 * Send a queue of parameters, one parameter per function call.
 *
 * @return		zero on success, nonzero on failure
 */
 int mavlink_pm_queued_send(void);

/**
 * Start sending the parameter queue.
 *
 * This function will not directly send parameters, but instead
 * activate the sending of one parameter on each call of
 * mavlink_pm_queued_send().
 * @see 		mavlink_pm_queued_send()
 */
void mavlink_pm_start_queued_send(void);
