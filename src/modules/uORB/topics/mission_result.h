/****************************************************************************
 *
 *   Copyright (C) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file mission_result.h
 * Mission results that navigator needs to pass on to commander and mavlink.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Ban Siesta <bansiesta@gmail.com>
 */

#ifndef TOPIC_MISSION_RESULT_H
#define TOPIC_MISSION_RESULT_H

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct mission_result_s
{
	unsigned seq_reached;		/**< Sequence of the mission item which has been reached */
	unsigned seq_current;		/**< Sequence of the current mission item				 */
	bool reached;			/**< true if mission has been reached					 */
	bool finished;			/**< true if mission has been completed					 */
	bool stay_in_failsafe;		/**< true if the commander should not switch out of the failsafe mode*/
	bool flight_termination;	/**< true if the navigator demands a flight termination from the commander app */
	bool item_do_jump_changed;	/**< true if the number of do jumps remaining has changed */
	unsigned item_changed_index;	/**< indicate which item has changed */
	unsigned item_do_jump_remaining;/**< set to the number of do jumps remaining for that item */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(mission_result);

#endif
