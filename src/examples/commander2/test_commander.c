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

#include "fsm_commander/Commander.h"

/*#include <stdlib.h>*/
/*#include <stdio.h>*/
#include <assert.h>
/*#include <string.h>*/

#include "cbmc/cbmc_stub.h"
#include "cbmc/common.h"

int main(int argc, char **argv)
{
	Commander cmdr;
	UNUSED(argc);
	UNUSED(argv);
	commanderInit(&cmdr);

	for(int i=0; i<10; i++) {
		commanderEvent_t event = nondet_uint() % EVENT_NUMBER;
		__CPROVER_assume(event >= 0 && event < EVENT_NUMBER);
		/*printf("state: %d\tevent: %d\t", cmdr.state, event);*/
		commanderUpdate(&cmdr, event);
		/*printf("new state: %d\n", cmdr.state);*/
		assert(cmdr.state >= 0 && cmdr.state < STATE_NUMBER);
	}

	commanderUpdate(&cmdr, EVENT_MANUAL);
	__CPROVER_assert(cmdr.state == STATE_MANUAL, "transiton to manual always works");

	commanderUpdate(&cmdr, EVENT_AUTO);
	__CPROVER_assert(cmdr.state == STATE_AUTO, "transition to auto from manual works");

	__CPROVER_assert(true, "reached end");

	return RET_OK;
}

/* vim: set noet fenc=utf-8 ff=unix : */
