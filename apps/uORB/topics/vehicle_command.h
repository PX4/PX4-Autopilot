/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file vehicle_command.h
 * Definition of the vehicle command uORB topic.
 */

#ifndef TOPIC_VEHICLE_COMMAND_H_
#define TOPIC_VEHICLE_COMMAND_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

enum PX4_CMD {
	PX4_CMD_CONTROLLER_SELECTION = 1000,
};

struct vehicle_command_s
{
	float param1;			/**< Parameter 1, as defined by MAVLink MAV_CMD enum.   */
	float param2;			/**< Parameter 2, as defined by MAVLink MAV_CMD enum.   */
	float param3;			/**< Parameter 3, as defined by MAVLink MAV_CMD enum.   */
	float param4;			/**< Parameter 4, as defined by MAVLink MAV_CMD enum.   */
	float param5;			/**< Parameter 5, as defined by MAVLink MAV_CMD enum.   */
	float param6;			/**< Parameter 6, as defined by MAVLink MAV_CMD enum.   */
	float param7;			/**< Parameter 7, as defined by MAVLink MAV_CMD enum.   */
	uint16_t command;		/**< Command ID, as defined MAVLink by MAV_CMD enum.   */
	uint8_t target_system;		/**< System which should execute the command  */
	uint8_t target_component;	/**< Component which should execute the command, 0 for all components  */
	uint8_t source_system;		/**< System sending the command  */
	uint8_t source_component;	/**< Component sending the command  */
	uint8_t confirmation;		/**< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) */
}; /**< command sent to vehicle */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_command);



#endif
