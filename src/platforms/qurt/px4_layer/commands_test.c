/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file commands_default.c
 * Commands to run for the "qurt_default" config
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

const char *get_commands()
{
	static const char *commands =
		"hello start\n"
		"uorb start\n"
		"simulator start -s\n"
		"barosim start\n"
		"adcsim start\n"
		"accelsim start\n"
		"gyrosim start\n"
		"list_devices\n"
		"list_topics\n"
		"list_tasks\n"
		"param show *\n"
		"rgbled start\n"
#if 0
		"sensors start\n"
		"param set CAL_GYRO0_ID 2293760\n"
		"param set CAL_ACC0_ID 1310720\n"
		"hil mode_pwm"
		"param set CAL_ACC1_ID 1376256\n"
		"param set CAL_MAG0_ID 196608\n"
		"mavlink start -d /tmp/ttyS0\n"
		"commander start\n"
#endif
		;

	return commands;
}
