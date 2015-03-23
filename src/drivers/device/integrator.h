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

/**
 * @file integrator.h
 *
 * A resettable integrator
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

 #pragma once

class Integrator {
public:
	Integrator(uint64_t auto_reset_interval = 0);
	virtual ~Integrator();

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if putting the item triggered an integral reset
	 *			and the integral should be published
	 */
	bool			put(uint64_t time, float val);

private:
	uint64_t _auto_reset_interval;		/**< the interval after which the content will be published and the integrator reset */
	float _integral_auto;			/**< the integrated value which auto-resets after _auto_reset_interval */
	float _integral_last_read;		/**< the integrated value since the last read */

	/* we don't want this class to be copied */
	Integrator(const Integrator&);
	Integrator operator=(const Integrator&);
};

bool
Integrator::put(uint64_t time, float val)
{

}
