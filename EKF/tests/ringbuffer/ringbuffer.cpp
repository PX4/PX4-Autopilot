/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ringbuffer.cpp
 * Test routines for the EKF ringbuffer
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include <stdint.h>
#include <cassert>
#include <EKF/RingBuffer.h>

struct sample {
	uint64_t time_us;
	float data[3];
};

int main(int argc, char *argv[])
{
	(void)argc; // unused
	(void)argv; // unused

	sample x;
	x.time_us = 1000000;
	x.data[0] = x.data[1] = x.data[2] = 1.5f;

	sample y;
	y.time_us = 2000000;
	y.data[0] = y.data[1] = y.data[2] = 3.0f;

	sample z;
	z.time_us = 3000000;
	z.data[0] = z.data[1] = z.data[2] = 4.0f;

	// Test1: false buffer allocation
	bool initialised = false;
	RingBuffer<sample> buffer;

	//initialised = buffer.allocate(-1);
	//assert(initialised == false);
	//initialised = buffer.allocate(0);
	//assert(initialised == false);

	// Test2: good initialisation
	initialised = buffer.allocate(3);
	assert(initialised == true);

	// Test3: pushing data into ringbuffer
	buffer.push(x);
	assert(buffer.get_newest().time_us == x.time_us);
	//assert(buffer.get_oldest().time_us == 0);
	buffer.push(y);
	buffer.push(z);
	assert(buffer.get_newest().time_us == z.time_us);
	assert(buffer.get_oldest().time_us == x.time_us);

	// Test3: Retreiving data from the buffer
	sample pop = {};
	// this should return false, because there is no data older than t = 0
	assert(buffer.pop_first_older_than(0, &pop) == false);

	assert(buffer.pop_first_older_than(x.time_us + 1, &pop) == true);
	assert(pop.time_us == x.time_us);
	assert(buffer.pop_first_older_than(y.time_us + 100, &pop) == true);
	assert(pop.time_us == y.time_us);
	assert(buffer.pop_first_older_than(z.time_us + 100, &pop) == true);
	assert(pop.time_us == z.time_us);

	// Test 4: Bigger buffer, redo Test3
	buffer.allocate(10);
	buffer.push(x);
	assert(buffer.get_newest().time_us == x.time_us);
	//assert(buffer.get_oldest().time_us == 0);
	buffer.push(y);
	buffer.push(z);
	assert(buffer.get_newest().time_us == z.time_us);

	// this should return false, because the diff to 0 is too large
	// buffer will not accept measurement will is older than 0.5 seconds
	assert(buffer.pop_first_older_than(600000, &pop) == false);

	assert(buffer.pop_first_older_than(x.time_us + 1, &pop) == true);
	assert(pop.time_us == x.time_us);
	assert(buffer.pop_first_older_than(y.time_us + 100, &pop) == true);
	assert(pop.time_us == y.time_us);
	assert(buffer.pop_first_older_than(z.time_us + 100, &pop) == true);
	assert(pop.time_us == z.time_us);

	return 0;
}
