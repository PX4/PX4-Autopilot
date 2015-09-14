/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

#pragma once

#include <uORB/topics/vehicle_status.h>

/**
 * The possible events, stimulus, for the
 * finite state machine.
 */
typedef enum {
	EVENT_MANUAL, /* request for manual */
	EVENT_AUTO, /* request for auto */
	EVENT_GUIDED, /* request for guided */
	EVENT_NUMBER, /* number of events */
	EVENT_UNINIT /* unitialized */
} commanderEvent_t;

/**
 * The states within the finite state machine,
 * this should be limited to states in which
 * the vehicle should react differently to
 * outside stimulus and the number should
 * be kept as small as possible.
 */
typedef enum {
	STATE_MANUAL, /* manual mode */
	STATE_AUTO, /* auto mode */
	STATE_GUIDED, /* guided mode */
	STATE_NUMBER, /* number of states */
	STATE_UNINIT /* uninitialized */
} commanderState_t;

/**
 * Typedefs for commander data structure
 * and callbacks.
 */
typedef struct Commander_t *CommanderPtr;
typedef struct Commander_t Commander;
typedef void (*CallbackManual)(CommanderPtr);
typedef void (*CallbackAuto)(CommanderPtr);
typedef void (*CallbackGuided)(CommanderPtr);

/**
 * Commander data structure.
 *
 * This contains callbacks and an identifier
 * of the current state just for
 * use when debugging.
 *
 * Do not use the state variable
 * for switch statements, that violates
 * the purpose of the "State Pattern" approach
 * to finite state machines. Instead, use
 * the callbacks.
 */
struct Commander_t {
	CallbackManual callbackManual;
	CallbackAuto callbackAuto;
	CallbackGuided callbackGuided;
	int state;
	struct vehicle_status_s status;
};

/**
 * Initializes a commander data structure.
 * Does not allocate.
 */
void commanderInit(CommanderPtr instance);

/**
 * Updates a commander based on the event type.
 */
void commanderUpdate(CommanderPtr instance,
		     commanderEvent_t event);


/**
 * Serves as a default for state transitions.
 */
void commanderDefaultTransition(CommanderPtr instance);


/* vim: set noet fenc=utf-8 ff=unix : */
